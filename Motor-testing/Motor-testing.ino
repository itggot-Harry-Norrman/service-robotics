#include <QTRSensors.h>
#include "CytronMotorDriver.h"

// Define analog pins used
const uint8_t sensorPins[] = {A0, A1, A2, A3, A4, A5}; // 6 sensors
const uint8_t numSensors = sizeof(sensorPins) / sizeof(sensorPins[0]); // Number of sensors

// Create a QTRSensors object
QTRSensors qtr;

// Motor control for MDD3A
CytronMD motorLeft(PWM_PWM, 9, 10);   // Left motor: PWM_A = Pin 9, PWM_B = Pin 10
CytronMD motorRight(PWM_PWM, 3, 5);   // Right motor: PWM_A = Pin 3, PWM_B = Pin 5

// PD Controller constants
const double KP = 0.03;   // Proportional gain (adjust as needed)
const double KD = 0.0475;   // Derivative gain (adjust as needed)
double lastError = 0;    // To store the previous error for derivative calculation

// Maximum motor speed
const int maxSpeed = 250;

void setup() {
  // Set 3.3V as reference voltage
  analogReference(EXTERNAL); // Use external reference (3.3V connected to AREF)

  // Configure QTR sensor
  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, numSensors); // Assign sensor pins

  // Start serial communication
  Serial.begin(9600);
  Serial.println("Startar kalibrering...");

  // Calibrate sensors
  for (int i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(10); // Wait 10 ms between each calibration reading
  }
  Serial.println("Kalibrering klar!");

  // Stop motors initially
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
}

void loop() {
  // Array to hold sensor data
  uint16_t sensorValues[numSensors];

  // Read line position with readLineBlack()
  int position = qtr.readLineBlack(sensorValues); // Returns 0–5000

  // Print sensor data and line position to Serial Monitor
  Serial.print("Sensorvärden: ");
  for (uint8_t i = 0; i < numSensors; i++) {
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }
  Serial.print("| Linjeposition: ");
  Serial.println(position); // 0–5000 where 2500 is middle

  // Check if all sensors are below threshold (line lost)
  const uint16_t stopThreshold = 600; // Threshold to determine if line is lost
  bool allBelowThreshold = true;
  for (uint8_t i = 0; i < numSensors; i++) {
    if (sensorValues[i] >= stopThreshold) {
      allBelowThreshold = false;
      break;
    }
  }

  if (allBelowThreshold) {
    // Stop motors if all sensors are below threshold
    motorLeft.setSpeed(0);
    motorRight.setSpeed(0);
    Serial.println("Alla sensorer under 500, stannar...");
    delay(100); // Wait a bit before next check
    return;
  }

  // PD Control
  int error = position - 2500; // Error from middle position
  double derivative = error - lastError;
  double adjustment = KP * error + KD * derivative;
  lastError = error;

  // Compute motor speeds
  int leftSpeed = maxSpeed + adjustment;
  int rightSpeed = maxSpeed - adjustment;

  // Constrain motor speeds to maxSpeed
  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  // Set motor speeds (corrected polarity)
  motorLeft.setSpeed(-leftSpeed);  // Negative to correct direction
  motorRight.setSpeed(rightSpeed); // Positive for correct direction

  // Print debug information
  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(" | Adjustment: ");
  Serial.print(adjustment);
  Serial.print(" | Left Speed: ");
  Serial.print(leftSpeed);
  Serial.print(" | Right Speed: ");
  Serial.println(rightSpeed);

  delay(10); // Short delay to avoid overload
}
