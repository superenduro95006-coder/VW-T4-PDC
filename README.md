# VAG-Ultraschallsensor-Interface (PDC + Totwinkel) für Arduino

Dieses Projekt ermöglicht es, originale **VAG-Ultraschallsensoren** (Volkswagen, Audi, Skoda, Seat – MQB & PQ35/PQ46 Generation) mit einem **Arduino-kompatiblen Board** auszulesen und wie in OEM-Fahrzeugen zu verwenden.

## 🚗 Projektziel

Ziel des Projekts ist es, originale VAG-Ultraschallsensoren ohne das originale PDC-Steuergerät direkt mit einem Arduino auszulesen und daraus:

1. **Entfernungen pro Sensor** zu bestimmen  
2. diese Entfernungen über eine **nichtlineare Korrekturkennlinie** in realistische cm umzusetzen  
3. **Zonen** (Nah / Mittel / Fern) automatisch zu erkennen  
4. **originale VAG-PDC-Tongeber** anzusteuern (8E0 919 279)  
5. optional eine **Totwinkel-Erkennung** mit einfacher Ja/Nein-Logik zu liefern

Das System eignet sich für Fahrzeug-Nachrüstungen, Forschungsprojekte, Showcars oder eigene Fahrzeug-Elektronik.

---

## 🧠 Projektübersicht

### Funktionsprinzip

Alle VAG-Ultraschallsensoren der PDC-Systeme arbeiten **nicht über LIN**, sondern über eine **pulsbasierte 1-Draht-Triggerleitung**:

1. Arduino erzeugt einen **Triggerpuls**  
2. Sensor sendet Ultraschall  
3. Sensor liefert ein **Hochpegel-Echo** zurück  
4. Arduino misst Pulsdauer (`pulseIn()`)  
5. über Korrekturkennlinie → realistische Entfernung  
6. Einteilung in Zonen → PDC-Warnlogik  
7. bei Bedarf → PDC-Buzzer

### Sensor-Generationen

Getestet u. a.:

- **5Q0/5Q0C**-Serien (MQB)
- **1Z0**, **1K0**, **3C0**, **7H0** (PQ35 / PQ46)

---

## 🔧 Hardwarekomponenten

### 1. Arduino (empfohlen: ATmega32u4)
- sauberes USB-Serial  
- separater UART  
- stabile Timer-Abmessungen für präzise Pulszeiten  

### 2. Pegelwandlung: **Pololu 2595**
- wandelt Sensor-Logikpegel (5.5 V) auf 5 V  
- bidirektional  
- pro Sensor 1 Kanal

### 3. Sensor-Versorgung
- 5.5–6.0 V über Step-Down  
- **niemals direkt 12 V** an die Sensoren  

### 4. PDC-Buzzer (VAG 8E0 919 279)
- 12-V-Lautsprecher  
- über Transistor/MOSFET schalten  
- gleicher Klang wie OEM-PDC  

### 5. LEDs für Totwinkel (optional)
- je 1 LED pro Seite  
- zeigt Objekt im Totwinkel (Ja/Nein)

---

## 🛠 Verschaltung (Beispiel-Pinbelegung)

```
REAR SENSORS:
  HL  → D2
  HLI → D3
  HRI → D4
  HR  → D5

FRONT SENSORS (optional):
  VL  → D6
  VLI → D7
  VRI → D8
  VR  → D10

BLIND SPOT (optional):
  TL → D11
  TR → D12

PDC BUZZERS:
  Rear  → D9   (über MOSFET)
  Front → D13  (über MOSFET)

BLIND SPOT LED:
  TL LED → A0
  TR LED → A1

Power:
  Sensoren + Pegelwandler HV → +5.5 V
  Arduino + Pegelwandler LV  → +5 V
  GND → gemeinsam
```

---

## 🧮 Softwaremechanismen

### 1. Trigger & Echo
- definierter Triggerpuls (75 µs LOW → Release)
- Sensor erzeugt Echo-HIGH
- Pulsbreite = Entfernung (über Schalllaufzeit)

### 2. Nichtlineare Entzerrung
Basierend auf deinen Messdaten (z. B. 50cm→36 raw):

```
dist = a * raw^2 + b * raw + c
```

### 3. Filterung
- Median aus mehreren Messungen  
- Zonenabhängige Glättung  
- Schrittbegrenzung  
- hartes Minimum 20 cm (OEM-Verhalten)

### 4. Zonen (OEM-ähnlich)

| Zone | Abstand | Bedeutung |
|------|---------|----------|
| 3 | 20–40 cm | Nah |
| 2 | 40–80 cm | Mittel |
| 1 | 80–150 cm | Fern |
| 0 | >150 cm | frei |

### 5. Buzzer-Logik
Zonen → unterschiedliche Piepintervalle (VW-Stil):

- Fern: 100 ms an, 900 ms aus  
- Mittel: 150 ms an, 350 ms aus  
- Nah: 600 ms an, 100 ms aus  

### 6. Totwinkel („Blindspot“)
- Ja/Nein pro Seite  
- Distanzschwellen (z. B. <300 cm → Objekt erkannt)  
- Hysterese:
  - 3 Treffer → aktiv  
  - 5 Fehlmessungen → deaktiviert  
- Ausgabe über LEDs  

---

## 🔄 Betriebsmodi

### PARKMODUS
- hintere PDC-Sensoren immer aktiv  
- vordere PDC-Sensoren optional  
- Buzzer vorne & hinten  
- VW-ähnlicher Zeitplan (~50–60ms)

### FAHRMODUS
- nur Totwinkel-Sensoren  
- Ja/Nein-Auswertung  
- keine PDC-Pieptöne  

Die Umschaltung erfolgt später über Rückwärtsgang + Geschwindigkeit (CAN oder separater Pin).

---

## 📁 Projektstruktur

```
/src
   └── main.ino          → Hauptprogramm (Version P+)
/hardware
   └── wiring_diagram/    → Schaltpläne, Fotos
/docs
   └── algorithms.md      → Zonen, Filter, Totwinkel
README.md
```

---

## 📜 Lizenz
MIT-Lizenz oder frei anpassbar.

