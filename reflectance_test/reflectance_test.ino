#include <QTRSensors.h>

// Definiera vilka analoga pins som används
const uint8_t sensorPins[] = {A0, A1, A2, A3, A4, A5}; // 6 sensorer
const uint8_t numSensors = sizeof(sensorPins) / sizeof(sensorPins[0]); // Antal sensorer

// Skapa ett QTRSensors-objekt
QTRSensors qtr;

void setup() {
  // Ställ in 3.3V som referensspänning
  analogReference(EXTERNAL); // Använd extern referens (3.3V kopplad till AREF)

  // Konfigurera QTR-sensorn
  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, numSensors); // Tilldela sensorpins
  //qtr.setEmitterPin(QTR_NO_EMITTER_PIN);     // Ingen emitter används

  // Starta seriell kommunikation
  Serial.begin(9600);
  Serial.println("Startar kalibrering...");

  // Kalibrera sensorerna
  for (int i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(20); // Vänta 20 ms mellan varje kalibreringsmätning
  }
  Serial.println("Kalibrering klar!");
}

void loop() {
  // Array för att hålla sensordata
  uint16_t sensorValues[numSensors];

  // Läs kalibrerade värden
  qtr.readCalibrated(sensorValues);

  // Skriv ut sensordata till Serial Monitor
  Serial.print("Sensorvärden: ");
  for (uint8_t i = 0; i < numSensors; i++) {
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }
  Serial.println(); // Gå till nästa rad i Serial Monitor

  delay(100); // Vänta lite mellan varje mätning
}