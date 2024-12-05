#include "CytronMotorDriver.h"
#include <Encoder.h>
#include <QTRSensors.h>

// Motorstyrning för MDD3A
CytronMD motorLeft(PWM_PWM, 9, 10);   // Vänster motor: PWM_A = Pin 9, PWM_B = Pin 10
CytronMD motorRight(PWM_PWM, 3, 5);   // Höger motor: PWM_A = Pin 3, PWM_B = Pin 5

// Encoder-konfiguration
Encoder encoderLeft(2, 4);  // Encoder A och B för vänster motor
Encoder encoderRight(6, 7); // Encoder A och B för höger motor

// QTR-sensorer
const uint8_t sensorPins[] = {A0, A1, A2, A3, A4, A5}; // 6 sensorer
const uint8_t numSensors = sizeof(sensorPins) / sizeof(sensorPins[0]); // Antal sensorer
//QTRSensorsAnalog qtr(sensorPins, numSensors); // Använd analog QTR-sensor
QTRSensors qtr;
// Variabler för linjeföljning
uint16_t sensorValues[numSensors];
int maxSpeed = 255;    // Maxhastighet för motorerna
int baseSpeed = 255;   // Basfart när roboten kör rakt fram (full hastighet)
int turnAdjustment = 50; // Justeringsvärde för svängar (mindre värde)

void setup() {
  // Ställ in 3.3V som referensspänning
  analogReference(EXTERNAL); // Använd extern referens (3.3V kopplad till AREF)

  // Konfigurera QTR-sensorn
  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, numSensors); // Tilldela sensorpins

  Serial.begin(9600);

  // Stoppa motorerna initialt
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);

  // Nollställ encoders
  encoderLeft.write(0);
  encoderRight.write(0);

  // Kalibrera QTR-sensorerna
  Serial.println("Startar kalibrering...");
  for (int i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(20);
  }
  Serial.println("Kalibrering klar!");
}

void loop() {
  // Läs kalibrerade värden
  qtr.readCalibrated(sensorValues);

  // Variabler för att bestämma riktning
  bool leftMost = sensorValues[5] < 750;
  bool leftMid = sensorValues[4] < 750;
  bool centerLeft = sensorValues[3] < 750;
  bool centerRight = sensorValues[2] < 750;
  bool rightMid = sensorValues[1] < 750;
  bool rightMost = sensorValues[0] < 750;

  int leftSpeed = baseSpeed;
  int rightSpeed = baseSpeed;

  if (centerLeft || centerRight) {
    // Linjen är i mitten, kör rakt fram
    leftSpeed = baseSpeed;
    rightSpeed = baseSpeed;
  } else if (leftMid || leftMost) {
    // Linjen är till vänster, sväng vänster genom att minska hastigheten på vänster motor
    leftSpeed = baseSpeed - turnAdjustment;
    rightSpeed = baseSpeed;
  } else if (rightMid || rightMost) {
    // Linjen är till höger, sväng höger genom att minska hastigheten på höger motor
    leftSpeed = baseSpeed;
    rightSpeed = baseSpeed - turnAdjustment;
  } else {
    // Linjen är förlorad, fortsätt rakt fram eller stanna
    leftSpeed = baseSpeed;
    rightSpeed = baseSpeed;
    // Alternativt kan du stanna motorerna här om du vill
     leftSpeed = 0;
     rightSpeed = 0;
  }

  // Begränsa hastigheterna till tillåtna värden
  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  // Sätt motorhastigheter
  motorLeft.setSpeed(leftSpeed);
  motorRight.setSpeed(-rightSpeed); // Negativ eftersom motorerna är spegelvända

  // Utskrift för felsökning
  Serial.print("Sensorvärden: ");
  for (uint8_t i = 0; i < numSensors; i++) {
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }
  Serial.print(" | Vänster hastighet: ");
  Serial.print(leftSpeed);
  Serial.print(" Höger hastighet: ");
  Serial.println(rightSpeed);

  delay(10); // Kort paus för stabilitet
}