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

// ---------------- Konfiguration ----------------

// Rear PDC
const uint8_t NUM_REAR  = 4;
const uint8_t REAR_PINS[NUM_REAR] = {2, 3, 4, 5};   // HL, HLI, HRI, HR

// Front PDC (optional: auf false setzen, wenn noch nicht vorhanden)
const bool    ENABLE_FRONT = true;
const uint8_t NUM_FRONT = 4;
const uint8_t FRONT_PINS[NUM_FRONT] = {6, 7, 8, 10}; // VL, VLI, VRI, VR

// Blindspot / Totwinkel (nur Fahrmodus)
const bool    ENABLE_BLINDSPOT = true;
const uint8_t NUM_BLIND = 2;
const uint8_t BLIND_PINS[NUM_BLIND] = {11, 12}; // TL, TR

// Optional: LED-Ausgänge für optisches Signal (eine pro Seite)
const uint8_t BLIND_LED_PINS[NUM_BLIND] = {A0, A1}; // anpassen wie gewünscht

// Schwellen für Totwinkel-Entscheidung (in korrigierten cm!)
const float BLIND_ENTER_CM = 300.0;  // ab hier: "im Totwinkel" (rein)
const float BLIND_EXIT_CM  = 350.0;  // erst ab hier wieder frei (raus)

// Hysteresezählung, um Flackern zu vermeiden
const uint8_t BLIND_HIT_COUNT_ON   = 3;  // 3 Treffer -> aktiv
const uint8_t BLIND_MISS_COUNT_OFF = 5;  // 5 Fehlmessungen -> aus

// interner Totwinkel-Zustand
bool     blindActive[NUM_BLIND]   = {false, false};
uint8_t  blindHitCount[NUM_BLIND] = {0, 0};
uint8_t  blindMissCount[NUM_BLIND]= {0, 0};


// Buzzer-Pins (Transistor/MOSFET steuert 8E0 919 279)
const uint8_t BUZZER_REAR_PIN  = 9;
const uint8_t BUZZER_FRONT_PIN = 13;

// pulseIn-Timing
const unsigned long PULSE_TIMEOUT_US = 20000;  // max. 20 ms
const unsigned long RING_DOWN_US     = 700;    // ~12 cm (alles darunter = Ring-Down)
const unsigned long MAX_ECHO_US      = 18000;  // >3 m -> ungültig

// Messparameter
const uint8_t N_MEAS = 3;   // Einzelmessungen pro Sensor im Parkmodus

// Grund-Glättungsparameter
const float ALPHA_BASE    = 0.25;
const float MAX_STEP_BASE = 15.0;

// Pulsdauer -> grobe Roh-cm (vor Entzerrung)
const float DIVISOR = 25.6;

// Nichtlineare Distanzentzerrung (aus deinen 3 Messpunkten)
const float POLY_A = -0.002303;
const float POLY_B =  2.0655;
const float POLY_C = -9.542;

// Zonen-Definition
const float ZONE_NEAR_MIN = 20.0;   // kleinste Zone beginnt bei 20 cm
const float ZONE_NEAR_MAX = 40.0;
const float ZONE_MID_MAX  = 80.0;
const float ZONE_FAR_MAX  = 150.0;


enum Zone {
  ZONE_FREE = 0,   // >150 cm oder ungültig
  ZONE_FAR  = 1,   // 80–150 cm
  ZONE_MID  = 2,   // 40–80 cm
  ZONE_NEAR = 3    // 20–40 cm
};

// Gefilterte Werte
float filteredRear[NUM_REAR]   = {-1, -1, -1, -1};
float filteredFront[NUM_FRONT] = {-1, -1, -1, -1};
float filteredBlind[NUM_BLIND] = {-1, -1};

// Buzzer-Zustand
unsigned long buzzerRearLastChangeMs  = 0;
bool          buzzerRearPhaseOn       = false;
Zone          buzzerRearLastZone      = ZONE_FREE;

unsigned long buzzerFrontLastChangeMs = 0;
bool          buzzerFrontPhaseOn      = false;
Zone          buzzerFrontLastZone     = ZONE_FREE;

const unsigned int BUZZER_FREQ_HZ = 3000;  // ca. PDC-Tonhöhe

// -------------------------------------------------------------
// Platzhalter: Parkmodus/Fahrmodus
// -> später durch echte Signale (Rückwärtsgang, Geschwindigkeit) ersetzen
// -------------------------------------------------------------
bool isParkMode() {
  // TODO: Rückwärtsgang / Geschwindigkeit einlesen
  return true;   // aktuell immer Parkmodus aktiv
}

bool isDriveMode() {
  return !isParkMode();
}

// -------------------------------------------------------------
// Trigger-Ping für einen Sensor
// -------------------------------------------------------------
void triggerSensor(uint8_t pin) {
  pinMode(pin, INPUT);
  delayMicroseconds(20);

  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(75);   // dein bewährter Triggerpuls
  pinMode(pin, INPUT);     // Leitung freigeben
}

// -------------------------------------------------------------
// Einzelmessung: Pulsdauer -> "Roh-cm"
// -------------------------------------------------------------
float singlePingCm(uint8_t pin) {
  triggerSensor(pin);

  unsigned long dur = pulseIn(pin, HIGH, PULSE_TIMEOUT_US);
  if (dur == 0) return -1.0;

  if (dur < RING_DOWN_US) return -1.0;
  if (dur > MAX_ECHO_US)  return -1.0;

  float cm = (dur / 2.0) / DIVISOR;
  if (cm < 0.0 || cm > 300.0) return -1.0;

  return cm;
}

// -------------------------------------------------------------
// Median über N_MEAS Messungen
// -------------------------------------------------------------
float medianDistance(uint8_t pin) {
  float vals[N_MEAS];
  uint8_t valid = 0;

  for (uint8_t i = 0; i < N_MEAS; i++) {
    float d = singlePingCm(pin);
    if (d > 0.0) vals[valid++] = d;
    delay(3);
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
float filterPdcValue(float &filteredValue, float rawCm) {
  if (rawCm < 0.0) {
    return -1.0;
  }

  // 1. Kennlinie
  float d = applyPolynomialCorrection(rawCm);

  // 2. Zonenkorrektur
  d = applyZoneCorrection(d);

  // 3. Zonenabhängige Filterparameter
  float alphaLocal   = ALPHA_BASE;
  float maxStepLocal = MAX_STEP_BASE;

  if (d <= ZONE_NEAR_MAX) {
    alphaLocal   = 0.15;
    maxStepLocal = 5.0;
  } else if (d <= ZONE_MID_MAX) {
    alphaLocal   = 0.25;
    maxStepLocal = 10.0;
  } else {
    alphaLocal   = 0.35;
    maxStepLocal = 20.0;
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
// Buzzer-Steuerung für eine Zone
// - ZONE_FREE: aus
// - ZONE_FAR : langsam
// - ZONE_MID : mittel
// - ZONE_NEAR: fast Dauerton
// -------------------------------------------------------------
void updateBuzzerForZone(Zone z,
                         uint8_t buzzerPin,
                         unsigned long nowMs,
                         Zone &lastZone,
                         bool &phaseOn,
                         unsigned long &lastChangeMs) {
  if (z != lastZone) {
    lastZone    = z;
    phaseOn     = false;
    lastChangeMs = nowMs;
    noTone(buzzerPin);
  }

  if (z == ZONE_FREE) {
    noTone(buzzerPin);
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

  if (phaseOn) {
    if (nowMs - lastChangeMs >= onTime) {
      phaseOn      = false;
      lastChangeMs = nowMs;
      noTone(buzzerPin);
    }
  } else {
    if (nowMs - lastChangeMs >= offTime) {
      phaseOn      = true;
      lastChangeMs = nowMs;
      tone(buzzerPin, BUZZER_FREQ_HZ);
    }
  }
}

// -------------------------------------------------------------
// Setup
// -------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  for (uint8_t i = 0; i < NUM_REAR; i++) {
    pinMode(REAR_PINS[i], INPUT);
  }
  if (ENABLE_FRONT) {
    for (uint8_t i = 0; i < NUM_FRONT; i++) {
      pinMode(FRONT_PINS[i], INPUT);
    }
  }
  if (ENABLE_BLINDSPOT) {
  for (uint8_t i = 0; i < NUM_BLIND; i++) {
    pinMode(BLIND_PINS[i], INPUT);
    pinMode(BLIND_LED_PINS[i], OUTPUT);
    digitalWrite(BLIND_LED_PINS[i], LOW);
  }
}

  pinMode(BUZZER_REAR_PIN, OUTPUT);
  pinMode(BUZZER_FRONT_PIN, OUTPUT);
  noTone(BUZZER_REAR_PIN);
  noTone(BUZZER_FRONT_PIN);

  Serial.println(F("Version P+ gestartet (4x hinten, optional 4x vorn, optional 2x Totwinkel)"));
}

// -------------------------------------------------------------
// Parkmodus: PDC vorne + hinten mit Zonen und Buzzern
// VW-ähnliches Timing: wir gehen senkrecht durch die Sensoren,
// mit kleinen Delays (~10–14ms) dazwischen.
// -------------------------------------------------------------
void loopParkMode() {
  unsigned long now = millis();

  Zone zRearOverall  = ZONE_FREE;
  Zone zFrontOverall = ZONE_FREE;

  // ----- Hinten (4 Sensoren) -----
  for (uint8_t i = 0; i < NUM_REAR; i++) {
    float raw = medianDistance(REAR_PINS[i]);
    float d   = filterPdcValue(filteredRear[i], raw);
    Zone  z   = getZoneFromDistance(d);

    if (z > zRearOverall) zRearOverall = z;

    Serial.print(F("RH"));
    Serial.print(i);
    Serial.print(F("("));
    Serial.print((int)z);
    Serial.print(F("): "));
    if (d < 0.0) Serial.print(F("----"));
    else { Serial.print(d, 1); Serial.print(F(" cm")); }
    Serial.print(F("   "));

    delay(10);  // Abstand zum nächsten Sensor
  }

  // ----- Vorn (optional 4 Sensoren) -----
  if (ENABLE_FRONT) {
    for (uint8_t i = 0; i < NUM_FRONT; i++) {
      float raw = medianDistance(FRONT_PINS[i]);
      float d   = filterPdcValue(filteredFront[i], raw);
      Zone  z   = getZoneFromDistance(d);

      if (z > zFrontOverall) zFrontOverall = z;

      Serial.print(F("VF"));
      Serial.print(i);
      Serial.print(F("("));
      Serial.print((int)z);
      Serial.print(F("): "));
      if (d < 0.0) Serial.print(F("----"));
      else { Serial.print(d, 1); Serial.print(F(" cm")); }
      Serial.print(F("   "));

      delay(10);
    }
  }

  Serial.println();

  now = millis();
  // Buzzer nach jeweils "schlimmster" Zone steuern
  updateBuzzerForZone(zRearOverall,
                      BUZZER_REAR_PIN,
                      now,
                      buzzerRearLastZone,
                      buzzerRearPhaseOn,
                      buzzerRearLastChangeMs);

  updateBuzzerForZone(zFrontOverall,
                      BUZZER_FRONT_PIN,
                      now,
                      buzzerFrontLastZone,
                      buzzerFrontPhaseOn,
                      buzzerFrontLastChangeMs);
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
    delay(50);
    return;
  }

  Serial.print(F("DriveMode BLIND: "));

  for (uint8_t i = 0; i < NUM_BLIND; i++) {
    // einfache Einzelmessung reicht hier (Schnappschuss)
    float raw = singlePingCm(BLIND_PINS[i]);
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
      if (blindHitCount[i] < 255) blindHitCount[i]++;
      blindMissCount[i] = 0;

      if (!blindActive[i] && blindHitCount[i] >= BLIND_HIT_COUNT_ON) {
        blindActive[i] = true;  // Zustand wird "gemerkt"
      }
    } else {
      if (blindMissCount[i] < 255) blindMissCount[i]++;
      blindHitCount[i] = 0;

      if (blindActive[i] && blindMissCount[i] >= BLIND_MISS_COUNT_OFF) {
        blindActive[i] = false; // erst nach mehreren "frei" wieder aus
      }
    }

    // LED-Ausgabe (optisches Signal)
    digitalWrite(BLIND_LED_PINS[i], blindActive[i] ? HIGH : LOW);

    // Debug-Ausgabe
    Serial.print(F("B"));
    Serial.print(i);
    Serial.print(F(": "));
    if (d < 0.0) {
      Serial.print(F("----"));
    } else {
      Serial.print(d, 1);
      Serial.print(F(" cm"));
    }
    Serial.print(F("  state="));
    Serial.print(blindActive[i] ? F("1") : F("0"));
    Serial.print(F("  "));
    delay(10);
  }

  Serial.println();
  delay(50);  // Totwinkel-Update-Rate ~20 Hz
}

// -------------------------------------------------------------
// Hauptloop: Moduswahl
// -------------------------------------------------------------
void loop() {
  if (isParkMode()) {
    loopParkMode();
  } else {
    loopDriveMode();
  }
}

