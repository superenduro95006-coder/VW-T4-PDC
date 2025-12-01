// =============================================================
//   Version P+
//   4 Parksensoren hinten, optional 4 vorn, optional 2 Totwinkel
//   - MQB-PDC-Sensoren (5,5 V) über Pololu 2595
//   - VW-ähnliches Timing für Parkmodus
//   - Distanzentzerrung (aus deinen Messpunkten)
//   - Zonen + Buzzer hinten & vorn
//   - Totwinkel-Sensoren nur im "Fahrmodus" (Stub)
// =============================================================

#include <Arduino.h>
#include "MedianFilter.h"

// ---------------- Konfiguration ----------------

#define SENS_RL_PIN 2
#define SENS_RLI_PIN 3
#define SENS_RRI_PIN 4
#define SENS_RR_PIN 5

#define SENS_FL_PIN 6
#define SENS_FLI_PIN 7
#define SENS_FRI_PIN 8
#define SENS_FR_PIN 9

#define SENS_LBL_PIN 10
#define SENS_RBL_PIN 16
// Optional: LED-Ausgänge für optisches Signal (eine pro Seite)
#define LED_LBL_PIN A0
#define LED_RBL_PIN A1

// Buzzer-Pins (Transistor/MOSFET steuert 8E0 919 279)
#define BUZZ_R_PIN 14
#define BUZZ_F_PIN 15

#define PARK_ENABLE_PIN 15

#define RX_LED_PIN 17


const unsigned int BUZZER_FREQ_HZ = 3000;  // ca. PDC-Tonhöhe

// Schwellen für Totwinkel-Entscheidung (in korrigierten cm!)
const float BLIND_ENTER_CM = 300.0;  // ab hier: "im Totwinkel" (rein)
const float BLIND_EXIT_CM  = 350.0;  // erst ab hier wieder frei (raus)

// Hysteresezählung, um Flackern zu vermeiden
const uint8_t BLIND_HIT_COUNT_ON   = 3;  // 3 Treffer -> aktiv
const uint8_t BLIND_MISS_COUNT_OFF = 5;  // 5 Fehlmessungen -> aus


// Pulsdauer -> grobe Roh-cm (vor Entzerrung)
const float DIVISOR = 25.6;

// pulseIn-Timing
const unsigned long PULSE_TIMEOUT_US = 16000;  // max. 20 ms
const unsigned long RING_DOWN_US     = 200;    // ~12 cm (alles darunter = Ring-Down)
const unsigned long MAX_ECHO_US      = 10000;  // >2 m -> ungültig
// Wie oft darf ein Sensor maximal "schießen"?
// Guard-Zeit pro Sensor in Mikrosekunden (Ausschwingzeit + Reserve)
const unsigned long SENSOR_GUARD_US = 12000;  // 15 ms als Startwert
const unsigned long SENSOR_DWELL_US = 10000;  // 12 ms als Startwert



// Median Messparameter
const uint8_t N_MEAS = 5;   // Einzelmessungen pro Sensor im Parkmodus

// Grund-Glättungsparameter
const float ALPHA_BASE    = 0.7;
const float MAX_STEP_BASE = 30.0;

// Nichtlineare Distanzentzerrung (aus deinen 3 Messpunkten)
/*
const float POLY_A = -0.002303;
const float POLY_B =  2.0655;
const float POLY_C = -9.542;
*/ 
// Polynom aus den neuen Messwerten
const float POLY_A = 0.00127600683f;
const float POLY_B = 0.733338949f;
const float POLY_C = 16.4056609f;


enum Zone {
  ZONE_FREE = 0,   // >150 cm oder ungültig
  ZONE_FAR  = 1,   // 80–150 cm
  ZONE_MID  = 2,   // 40–80 cm
  ZONE_NEAR = 3    // 20–40 cm
};

// Zonen-Definition
const float ZONE_NEAR_MIN = 20.0;   // kleinste Zone beginnt bei 10 cm
const float ZONE_NEAR_MAX = 40.0;
const float ZONE_MID_MAX  = 80.0;
const float ZONE_FAR_MAX  = 150.0;

struct USSensor {
  uint8_t      pin; // Arduino-Pin für diesen Sensor
  float        filtered;    // zuletzt gefilterter/distanz-korrigierter Wert in cm
  unsigned long lastPingUs; // Zeit des letzten Triggerpulses (Guard-Zeit)
  MedianFilter filter;
};

struct PdcSensor {
  USSensor sensor;
  Zone         zone;        // zuletzt berechnete Zone
  const char*  name;        // optional: Label fürs Debugging ("RH0", "VF2" etc.)
};

struct BlindSensor {
  USSensor sensor;
  // interner Totwinkel-Zustand
  bool     blindActive;
  uint8_t  blindHitCount;
  uint8_t  blindMissCount;
  uint8_t  LEDpin;
  const char*  name;        // optional: Label fürs Debugging ("RH0", "VF2" etc.)
};

struct Buzzer {
  uint8_t      pin; // Arduino-Pin für diesen Buzzer
  unsigned long lastChangeMs;
  bool          phaseOn;
  Zone          lastZone;
};

//const uint8_t NUM_REAR  = 4;
//const uint8_t NUM_FRONT = 4;

struct PDC {
  uint8_t numSensors;
  PdcSensor sensors[4];
  Buzzer buzzer;
  Zone zone;
  unsigned long lastSensorPingUs; // Zeit der letzten Sensorenabrfrage - macht 
};


PDC rearPDC = PDC {
  4,
  {
  {{SENS_RL_PIN, -1.0f, 0, MedianFilter(5, 100)}, ZONE_FREE, "RL"},
  {{SENS_RLI_PIN, -1.0f, 0, MedianFilter(5, 100)}, ZONE_FREE, "RLI"},
  {{SENS_RRI_PIN, -1.0f, 0, MedianFilter(5, 100)}, ZONE_FREE, "RRI"},
  {{SENS_RR_PIN, -1.0f, 0, MedianFilter(5, 100)}, ZONE_FREE, "RR"}
  },
  {BUZZ_R_PIN, 0, false, ZONE_FREE},
  ZONE_FREE,
  0
};

const bool    ENABLE_FRONT = false;
PDC frontPDC = PDC {
  4,
  {
  {{SENS_FL_PIN,  -1.0f, 0, MedianFilter(5, 100)}, ZONE_FREE, "FL"},
  {{SENS_FLI_PIN,  -1.0f, 0, MedianFilter(5, 100)}, ZONE_FREE, "FLI"},
  {{SENS_FRI_PIN,  -1.0f, 0, MedianFilter(5, 100)}, ZONE_FREE, "FRI"},
  {{SENS_FR_PIN, -1.0f, 0, MedianFilter(5, 100)}, ZONE_FREE, "FR"}
  },
  {BUZZ_F_PIN, 0, false, ZONE_FREE},
  ZONE_FREE,
  0
};


// Blindspot / Totwinkel (nur Fahrmodus)
const bool    ENABLE_BLINDSPOT = false;
const uint8_t NUM_BLIND = 2;
BlindSensor blindSensors[NUM_BLIND] = {
  {{SENS_LBL_PIN,  -1.0f, 0, MedianFilter(5, 100)}, false, 0, 0, LED_LBL_PIN, "LBL"},
  {{SENS_RBL_PIN, -1.0f, 0, MedianFilter(5, 100)}, false, 0, 0, LED_RBL_PIN, "RBL"}
};

// -------------------------------------------------------------
// Platzhalter: Parkmodus/Fahrmodus
// -> später durch echte Signale (Rückwärtsgang, Geschwindigkeit) ersetzen
// -------------------------------------------------------------
bool isParkMode() {
  // TODO: Rückwärtsgang / Geschwindigkeit einlesen
  return true;  //digitalRead(PARK_ENABLE_PIN);   // aktuell immer Parkmodus aktiv
}

bool isDriveMode() {
  return !isParkMode();
}

// -------------------------------------------------------------
// Buzzer-Steuerung für eine Zone
// - ZONE_FREE: aus
// - ZONE_FAR : langsam
// - ZONE_MID : mittel
// - ZONE_NEAR: fast Dauerton
// -------------------------------------------------------------
void updateBuzzerForZone(Zone z,
                         unsigned long nowMs,
                         Buzzer &buzzer
                         ) {
  if (z != buzzer.lastZone) {
    buzzer.lastZone    = z;
    buzzer.phaseOn     = false;
    buzzer.lastChangeMs = nowMs;
    noTone(buzzer.pin);
  }

  if (z == ZONE_FREE) {
    noTone(buzzer.pin);
    return;
  }

  unsigned long onTime, offTime;
  switch (z) {
    case ZONE_FAR:
      onTime  = 100;
      offTime = 900;
      break;
    case ZONE_MID:
      onTime  = 150;
      offTime = 350;
      break;
    case ZONE_NEAR:
      onTime  = 600;
      offTime = 100;
      break;
    default:
      onTime  = 0;
      offTime = 0;
      break;
  }

  if (buzzer.phaseOn) {
    if (nowMs - buzzer.lastChangeMs >= onTime) {
      buzzer.phaseOn      = false;
      buzzer.lastChangeMs = nowMs;
      noTone(buzzer.pin);
    }
  } else {
    if (nowMs - buzzer.lastChangeMs >= offTime) {
      buzzer.phaseOn      = true;
      buzzer.lastChangeMs = nowMs;
      tone(buzzer.pin, BUZZER_FREQ_HZ);
    }
  }
}

void muteBuzzer(Buzzer buzzer){
  noTone(buzzer.pin);
}

// -------------------------------------------------------------
// Trigger-Ping für einen Sensor
// -------------------------------------------------------------
void triggerSensorAtPin(uint8_t pin) {
  pinMode(pin, INPUT);
  delayMicroseconds(20);

  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(75);   // dein bewährter Triggerpuls
  pinMode(pin, INPUT);     // Leitung freigeben
}

float readSensorAtPin(uint8_t pin) {
  triggerSensorAtPin(pin);

  unsigned long dur = pulseIn(pin, HIGH, PULSE_TIMEOUT_US);
  if (dur == 0) return -1.0;

  if (dur < RING_DOWN_US) return -1.0;
  if (dur > MAX_ECHO_US)  return -1.0;

  float cm = (dur / 2.0) / DIVISOR;
  if (cm < 0.0 || cm > 300.0) return -1.0;

  return cm;
}

// -------------------------------------------------------------
// Einzelmessung: Pulsdauer -> "Roh-cm"
// -------------------------------------------------------------
float measureDistance(USSensor &s) {
  unsigned long nowUs = micros();

  // Guard-Zeit prüfen: ist die Ausschwingzeit schon vorbei?
  if (nowUs - s.lastPingUs < SENSOR_GUARD_US) {
    // zu früh: Sensor noch am Ausschwingen → keine neue Messung
    return -1.0;
  }

  // Ab hier: wir dürfen messen → Zeitpunkt des Pings merken
   s.lastPingUs = nowUs;

  return readSensorAtPin(s.pin);
}

float measureBlockedDistance(USSensor &s) {
  unsigned long nowUs = micros();

  // Guard-Zeit prüfen: ist die Ausschwingzeit schon vorbei?
  if (nowUs - s.lastPingUs < SENSOR_GUARD_US) {
    // zu früh: warten bis zu neuen Messung
    delayMicroseconds(SENSOR_GUARD_US - (nowUs - s.lastPingUs)+1);
    nowUs = micros();
  }

  // Ab hier: wir dürfen messen → Zeitpunkt des Pings merken
   s.lastPingUs = nowUs;

  return readSensorAtPin(s.pin);
}

// -------------------------------------------------------------
// Median über N_MEAS Messungen
// -------------------------------------------------------------
float medianDistance(USSensor &s) {
  /*
  float vals[N_MEAS];
  uint8_t valid = 0;

  for (uint8_t i = 0; i < N_MEAS; i++) {
    float d = measureDistance(s);
    if (d > 0.0) vals[valid++] = d;
  }

  if (valid == 0) return -1.0;

  // sortieren
  for (uint8_t i = 0; i < valid - 1; i++) {
    for (uint8_t j = i + 1; j < valid; j++) {
      if (vals[j] < vals[i]) {
        float t = vals[i];
        vals[i] = vals[j];
        vals[j] = t;
      }
    }
  }
  return vals[valid / 2];
  */

  float d = measureDistance(s);
  
  if (d < 0.0) return -1.0;

  s.filter.in((int)(d+.5));

  return float(s.filter.out());

}

//measure twice - measurement is valid only if both value are equal
//otherwise return -1
float verifiedDistance(USSensor &s){
  uint8_t valid = 1;

  float d1 = measureBlockedDistance(s);
  float d2 = measureBlockedDistance(s);

  if(abs(d1-d2) > 100){
    return -1;
  }
  
  return d1;

}

// -------------------------------------------------------------
// Nichtlineare Distanzkorrektur (aus deinen Messpunkten)
// -------------------------------------------------------------
float applyPolynomialCorrection(float raw) {
  if (raw < 0.0) return raw;
  return POLY_A * raw * raw + POLY_B * raw + POLY_C;
}

// -------------------------------------------------------------
// Zonenabhängige Zusatzkorrektur + Clamping
// - alles <20 cm -> 20 cm
// - Nahbereich leicht quantisiert
// -------------------------------------------------------------
float applyZoneCorrection(float d) {
  if (d < 0.0) return d;

  if (d < ZONE_NEAR_MIN) {
    d = ZONE_NEAR_MIN;
  }

  if (d >= ZONE_NEAR_MIN && d <= ZONE_NEAR_MAX) {
    d = roundf(d * 1.0f) / 1.0f;  // Ganze cm
  }

  return d;
}

// -------------------------------------------------------------
// Glättung & Schrittbegrenzung (PDC-Sensoren)
// -------------------------------------------------------------
float filterPdcValue(float rawCm, float filteredValue) {
  if (rawCm < 0.0) {
    return filteredValue;
  }

  // 1. Kennlinie
  float d = applyPolynomialCorrection(rawCm);

  // 2. Zonenkorrektur
  d = applyZoneCorrection(d);

  // 3. Zonenabhängige Filterparameter
  float alphaLocal   = ALPHA_BASE;
  float maxStepLocal = MAX_STEP_BASE;

  //zone des letzten bekannten wertes
  Zone lastZone = getZoneFromDistance(filteredValue);
  Zone zone = getZoneFromDistance(d);

  //each zone has to be passed - jumps in zones are false readings (nearby noise)
  if(lastZone == ZONE_NEAR && abs(zone-lastZone) > 1)
    return filteredValue;

  if (zone == ZONE_NEAR_MAX) {
    alphaLocal   = 0.3;
    maxStepLocal = 7.0;
  } else if (zone == ZONE_MID_MAX) {
    alphaLocal   = 0.45;
    maxStepLocal = 15.0;
  } else {
    alphaLocal   = 0.55;
    maxStepLocal = 25.0;
  }

  // 4. Initialisierung
  if (filteredValue < 0.0) {
    filteredValue = d;
    return d;
  }

  // 5. Schrittbegrenzung + Glättung
  float diff = d - filteredValue;

  if (diff >  maxStepLocal) diff =  maxStepLocal;
  if (diff < -maxStepLocal) diff = -maxStepLocal;

  filteredValue += alphaLocal * diff;

  return filteredValue;
}

//reads distance and applies filter to readings
void updatePdcSensor(PdcSensor &s) {
  float raw = medianDistance(s.sensor); //verifiedDistance(s.sensor);
  s.sensor.filtered   = filterPdcValue(raw, s.sensor.filtered);
  s.zone   = getZoneFromDistance(s.sensor.filtered);
}

void updatePdc(PDC &pdc) {
  Zone zOverall  = ZONE_FREE;

  for (uint8_t i = 0; i < pdc.numSensors; i++) {
    //wait for next Sensor read
    if(micros() - pdc.lastSensorPingUs < SENSOR_DWELL_US) {
      delayMicroseconds(SENSOR_DWELL_US - (millis() - pdc.lastSensorPingUs));
    }

    PdcSensor &sensor = pdc.sensors[i];
    updatePdcSensor(sensor);
    pdc.lastSensorPingUs = sensor.sensor.lastPingUs;

    if (sensor.zone > zOverall) zOverall = sensor.zone;

    Serial.print(sensor.name);
      Serial.print("(");
      Serial.print((int)sensor.zone);
      Serial.print("): ");
    if (sensor.sensor.filtered < 0.0) Serial.print(F("----"));
    else { Serial.print(sensor.sensor.filtered, 1); Serial.print(F(" cm")); }
    Serial.print(F("   "));
  }
  pdc.zone = zOverall;
  // Buzzer nach jeweils "schlimmster" Zone steuern
  updateBuzzerForZone(pdc.zone,
                      millis(),
                      pdc.buzzer);


}

// -------------------------------------------------------------
// Zone aus Distanz ableiten
// -------------------------------------------------------------
Zone getZoneFromDistance(float d) {
  if (d < 0.0 || d > ZONE_FAR_MAX) {
    return ZONE_FREE;
  } else if (d <= ZONE_NEAR_MAX) {
    return ZONE_NEAR;
  } else if (d <= ZONE_MID_MAX) {
    return ZONE_MID;
  } else {
    return ZONE_FAR;
  }
}



// -------------------------------------------------------------
// Parkmodus: PDC vorne + hinten mit Zonen und Buzzern
// VW-ähnliches Timing: wir gehen senkrecht durch die Sensoren,
// mit kleinen Delays (~10–14ms) dazwischen.
// -------------------------------------------------------------
void loopParkMode() {
  unsigned long now = millis();

  // ----- Hinten (4 Sensoren) -----
  updatePdc(rearPDC);
  
  // ----- Vorn (optional 4 Sensoren) -----
  if (ENABLE_FRONT) {
     updatePdc(frontPDC);
  }

  Serial.println();
}

// -------------------------------------------------------------
// Fahrmodus: einfache Totwinkelerkennung (ja/nein pro Seite)
// - misst Abstand
// - wendet die gleiche Kennlinie an
// - vergleicht mit BLIND_ENTER_CM / BLIND_EXIT_CM
// - zählt Treffer/Fehlmessungen für Hysterese
// - setzt blindActive[0/1] und LED
// -------------------------------------------------------------
void loopDriveMode() {
  if (!ENABLE_BLINDSPOT) {
    return;
  }

  Serial.print(F("DriveMode BLIND: "));

  for (uint8_t i = 0; i < NUM_BLIND; i++) {
    // einfache Einzelmessung reicht hier (Schnappschuss)
    float raw = measureDistance(blindSensors[i].sensor);
    float d   = applyPolynomialCorrection(raw);  // gleiche Entzerrung wie PDC

    bool inRange = false;

    if (d > 0.0) {
      // Objekt in sinnvoller Reichweite für Totwinkel?
      if (d <= BLIND_ENTER_CM) {
        inRange = true;
      }
    }

    // Hysterese-Logik
    if (inRange) {
      if (blindSensors[i].blindHitCount < 255) blindSensors[i].blindHitCount++;
      blindSensors[i].blindMissCount = 0;

      if (!blindSensors[i].blindActive && blindSensors[i].blindHitCount >= BLIND_HIT_COUNT_ON) {
        blindSensors[i].blindActive = true;  // Zustand wird "gemerkt"
      }
    } else {
      if (blindSensors[i].blindMissCount < 255) blindSensors[i].blindMissCount++;
        blindSensors[i].blindHitCount = 0;

      if (blindSensors[i].blindActive && blindSensors[i].blindMissCount >= BLIND_MISS_COUNT_OFF) {
        blindSensors[i].blindActive = false; // erst nach mehreren "frei" wieder aus
      }
    }

    // LED-Ausgabe (optisches Signal)
    digitalWrite(blindSensors[i].LEDpin, blindSensors[i].blindActive ? HIGH : LOW);

    // Debug-Ausgabe
    Serial.print(blindSensors[i].name);
    Serial.print(F(": "));
    if (d < 0.0) {
      Serial.print(F("----"));
    } else {
      Serial.print(blindSensors[i].sensor.filtered, 1);
      Serial.print(F(" cm"));
    }
    Serial.print(F("  state="));
    Serial.print(blindSensors[i].blindActive ? F("1") : F("0"));
    Serial.print(F("  "));
  }

  Serial.println();
}

void resetPDC(PDC &pdc){
  pdc.zone = ZONE_FREE;
  pdc.buzzer.lastZone = ZONE_FREE;
  muteBuzzer(pdc.buzzer);
}

void resetBlind() {
  for (uint8_t i = 0; i < NUM_BLIND; i++) {
    blindSensors[i].blindMissCount = 0;
    blindSensors[i].blindHitCount = 0;
    blindSensors[i].blindActive = false;
    digitalWrite(blindSensors[i].LEDpin, blindSensors[i].blindActive ? HIGH : LOW);
  }
}

// -------------------------------------------------------------
// Setup
// -------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  for (uint8_t i = 0; i < rearPDC.numSensors; i++) {
    pinMode(rearPDC.sensors[i].sensor.pin, INPUT);
  }


  if (ENABLE_FRONT) {
    for (uint8_t i = 0; i < frontPDC.numSensors; i++) {
      pinMode(frontPDC.sensors[i].sensor.pin, INPUT);
    }
  }
  if (ENABLE_BLINDSPOT) {
  for (uint8_t i = 0; i < NUM_BLIND; i++) {
    pinMode(blindSensors[i].sensor.pin, INPUT);
    pinMode(blindSensors[i].LEDpin, OUTPUT);
    digitalWrite(blindSensors[i].LEDpin, LOW);
  }
}

  pinMode(PARK_ENABLE_PIN, INPUT);
  pinMode(rearPDC.buzzer.pin, OUTPUT);
  pinMode(frontPDC.buzzer.pin, OUTPUT);
  muteBuzzer(rearPDC.buzzer);
  muteBuzzer(frontPDC.buzzer);

  Serial.println(F("Version P+ gestartet (4x hinten, optional 4x vorn, optional 2x Totwinkel)"));
}


long getIntervallFromDZone(Zone z) {
  switch (z) {
    case ZONE_FAR:
      return 1000;
    case ZONE_MID:
      return 300;
    case ZONE_NEAR:
      return 50;
    default:
      return 1000000;
  }
}

// Variables will change:
int ledState = LOW;  // ledState used to set the LED
unsigned long previousMillis = 0; 
// -------------------------------------------------------------
// Hauptloop: Moduswahl
// -------------------------------------------------------------
void loop() {
  if (isParkMode()) {
    resetBlind();
    //Serial.println("loopParkMode");
    loopParkMode();
  } else {
    resetPDC(rearPDC);
    resetPDC(frontPDC);
    loopDriveMode();
  }
  delayMicroseconds(10000);

  long interval = getIntervallFromDZone(rearPDC.zone);

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(RX_LED_PIN, ledState);
  }
}

