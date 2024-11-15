#include "CytronMotorDriver.h"
#include <Encoder.h>

// Motorstyrning för MDD3A
CytronMD motorLeft(PWM_PWM, 3, 5);   // Vänster motor: PWM_A = Pin 3, PWM_B = Pin 5
CytronMD motorRight(PWM_PWM, 9, 10); // Höger motor: PWM_A = Pin 9, PWM_B = Pin 10

// Encoder-konfiguration
Encoder encoderLeft(2, 4);  // Encoder A och B för vänster motor
Encoder encoderRight(6, 7); // Encoder A och B för höger motor

// Konstanter
#define PULSES_PER_REV 960        // Antal pulser per varv
#define DEGREES_PER_PULSE (360.0 / PULSES_PER_REV) // Grader per puls
#define TARGET_DEGREES 180        // Hur många grader varje hjul ska snurra

void setup() {
  Serial.begin(9600);

  // Stoppa motorerna initialt
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);

  // Nollställ encoders
  encoderLeft.write(0);
  encoderRight.write(0);
}

void loop() {
  // Kör framåt
  driveForward(255); // Full hastighet
  delay(1000);       // Kör i 1 sekund
  stopMotors();      // Stoppa

  // Sväng höger baserat på encoderpulser
  turnRight(255, TARGET_DEGREES); // Sväng höger 180 grader
  stopMotors();      // Stoppa

  // Fortsätt i loopen
}

// Funktion för att köra framåt
void driveForward(int pwm) {
  encoderLeft.write(0);  // Nollställ encoders
  encoderRight.write(0);

  motorLeft.setSpeed(pwm);  // Vänster motor framåt
  motorRight.setSpeed(pwm); // Höger motor framåt
}

// Funktion för att svänga höger
void turnRight(int pwm, float degrees) {
  encoderLeft.write(0);  // Nollställ encoders
  encoderRight.write(0);

  motorLeft.setSpeed(pwm);   // Vänster motor framåt
  motorRight.setSpeed(-pwm); // Höger motor bakåt

  // Räkna antal pulser som motsvarar den önskade vinkeln
  long targetPulses = degrees / DEGREES_PER_PULSE;

  // Vänta tills encodern registrerar önskat antal pulser
  while (abs(encoderLeft.read()) < targetPulses && abs(encoderRight.read()) < targetPulses) {
    Serial.print("Encoder Left Pulses: ");
    Serial.print(encoderLeft.read());
    Serial.print(" | Encoder Right Pulses: ");
    Serial.println(encoderRight.read());
  }
}

// Funktion för att stoppa motorerna
void stopMotors() {
  motorLeft.setSpeed(0);  // Stäng av vänster motor
  motorRight.setSpeed(0); // Stäng av höger motor
  delay(500);             // Kort paus för stabilitet
}
