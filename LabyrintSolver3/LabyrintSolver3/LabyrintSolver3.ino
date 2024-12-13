#include <QTRSensors.h>
#include "CytronMotorDriver.h"
#include <Servo.h>
#include <NewPing.h>

// *** Sensor- och motorinställningar *** //
const uint8_t sensorPins[] = {A0, A1, A2, A3, A4, A5}; 
const uint8_t numSensors = 6; 
QTRSensors qtr;


// Motorer (Cytron MDD3A)
CytronMD motorLeft(PWM_PWM, 6, 11);
CytronMD motorRight(PWM_PWM, 3, 5);

// *** PID-variabler (endast PD) *** //
float Kp = 0.2;
float Kd = 0.1;
int lastError = 0;

int maxSpeed = 200;
int baseSpeed; // maxSpeed / 2

// *** Ultraljud för cylinder *** //
#define TRIGGER_PIN  13
#define ECHO_PIN     12
#define MAX_DISTANCE 200
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
long cylinderLimit = 10; 

// *** Servon *** //
Servo myServoLarge;
Servo myServoSmall;
int currentLargePos = 75;  
int currentSmallPos = 165; 

// *** Faser *** //
// 0: Följ linjen
// 2: Plocka upp cylinder
int phase = 0;

// *** Path-sekvens för korsningar *** //
//char path[] = {'R', 'L', 'L', 'R', 'L', 'R', 'R', 'L'}; 
// char path[] = {'R','r','L','L','d','S','S','L','l','S','R','L','L','d','R','R','L','R','R','R','d','L','L','R','l','S'};
char path[] = {'L','l','L'};
int pathLength = sizeof(path) / sizeof(path[0]);
int pathIndex = 0; 

// Tröskel för svart linje
// originally 500
const uint16_t blackThreshold = 500;

// Tider
unsigned long calibrationEndTime;
unsigned long lastPickupTime = 0; 
unsigned long lastTurnTime = 0;  // Ny variabel för att undvika korsningsdetektering direkt efter sväng

void setup() {
  analogReference(EXTERNAL);
  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, numSensors);

  Serial.begin(9600);
  Serial.println("Startar kalibrering...");
  calibrateSensors();
  Serial.println("Kalibrering klar!");
  alignWithLine();

  // Servoinit
  myServoLarge.attach(10);
  myServoSmall.attach(9);
  myServoLarge.write(currentLargePos);
  myServoSmall.write(currentSmallPos);

  // Motorinit
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
  baseSpeed = maxSpeed / 2;
}

void loop() {
  uint16_t sensorValues[numSensors];
  int position = qtr.readLineBlack(sensorValues);
  int error = position - 2500;
  
  int derivative = error - lastError;
  int correction = Kp * error + Kd * derivative;
  // Serial.println(String(correction));
  if(correction > 250 || correction < -250){
    correction = 0;
  }
  lastError = error;

  switch (phase) {
    case 0: // Linjeföljning
      lineFollow(sensorValues, correction);
      break;
    case 2: // Upphämtning av cylinder
      pickupCylinder();
      break;
  }

  delay(1);
}

// Linjeföljning: justera motorer, kolla cylinder & korsningar
void lineFollow(uint16_t *sensorValues, int correction) {
  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  // Kolla korsning
  if (isIntersection(sensorValues)) {
    motorLeft.setSpeed(0);
    motorRight.setSpeed(0);

    if (pathIndex < pathLength) {
      char action = path[pathIndex++];
      handleIntersection(action);
    } else {
      Serial.println("Inga fler instruktioner. Fortsätter rakt fram.");
    }
    return;
  }

  motorLeft.setSpeed(leftSpeed);
  motorRight.setSpeed(-rightSpeed);

  // Kolla cylinder
  if (checkForCylinder() && (millis() - lastPickupTime > 2000)) {
    motorLeft.setSpeed(-50);
    motorRight.setSpeed(-50);
    //old batteries 200 5.3v
    delay(150);
    phase = 2;
    return;
  }


}

// Upphämtning cylinder
void pickupCylinder() {
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);

  delay(500);
  moveLargeServo(120);
  delay(500);
  moveSmallServo(90);
  moveLargeServo(180);
  delay(500);
  moveSmallServo(170);
  delay(500);
  moveLargeServo(74);
  moveSmallServo(50);
  delay(500);
  moveSmallServo(175);
  delay(500);

  lastPickupTime = millis();
  phase = 0;
}

// Servorörelser
void moveLargeServo(int pos) {
  int startPos = currentLargePos;
  if (startPos < pos) {
    for (int i = startPos; i < pos; i++) {
      myServoLarge.write(i);
      currentLargePos = i;
      delay(15);
    }
  } else {
    for (int i = startPos; i > pos; i--) {
      myServoLarge.write(i);
      currentLargePos = i;
      delay(15);
    }
  }
}

void moveSmallServo(int pos) {
  int startPos = currentSmallPos;
  if (startPos < pos) {
    for (int i = startPos; i < pos; i++) {
      myServoSmall.write(i);
      currentSmallPos = i;
      delay(15);
    }
  } else {
    for (int i = startPos; i > pos; i--) {
      myServoSmall.write(i);
      currentSmallPos = i;
      delay(15);
    }
  }
}

// Kalibrera sensorer
void calibrateSensors() {
  for (int i = 0; i < 400; i++) {
    qtr.calibrate();
    motorLeft.setSpeed(100);
    motorRight.setSpeed(100);
    delay(1);
  }
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
  calibrationEndTime = millis();
}

// Kolla cylinder
bool checkForCylinder() {
  if (millis() - calibrationEndTime < 2000) {
    return false;
  }
  long distance = sonar.ping_cm();
  // Serial.print("Distans: ");
  // Serial.println(distance);
  if (distance > 0 && distance < cylinderLimit) {
    Serial.println("Cylinder detected!");
    return true;
  }
  return false;
}

// Kolla korsning
bool isIntersection(uint16_t *sensorValues) {
  // Ingen korsningsdetektering första 2 sek efter kalibrering
  if (millis() - calibrationEndTime < 2000) {
    return false;
  }

  // Ingen korsningsdetektering 3 sek efter upphämtning
  /*if (millis() - lastPickupTime < 3000) {
    return false;
  }*/

  // Ingen korsningsdetektering 2 sek efter senaste sväng
  if (millis() - lastTurnTime < 1000) {
    return false;
  }

  bool leftIntersection = (sensorValues[0] > blackThreshold && sensorValues[1] > blackThreshold && sensorValues[5] < blackThreshold);
  bool rightIntersection = (sensorValues[4] > blackThreshold && sensorValues[5] > blackThreshold && sensorValues[0] < blackThreshold);
  bool tIntersection = (sensorValues[0] > blackThreshold && sensorValues[5] > blackThreshold);
  bool missingLine = (sensorValues[0] < blackThreshold && sensorValues[1] < blackThreshold && sensorValues[2] < blackThreshold && sensorValues[3] < blackThreshold && sensorValues[4] < blackThreshold && sensorValues[5] < blackThreshold);
  if(leftIntersection){
    Serial.println("left inter");
  }
  if(rightIntersection){
    Serial.println("right inter");
  }
  if(tIntersection){
    Serial.println("t inter");
  }
  if(missingLine) {
    Serial.println("missing line");
  }
  return (leftIntersection || rightIntersection || tIntersection || missingLine);
}

// Hantera sväng
void handleIntersection(char action) {
  if (millis() - calibrationEndTime < 2000) {
    return;
  }
  
  Serial.print("Hanterar korsning: ");
  // Serial.println(action);

  if (action == 'L') {
    turnLeft();
  } else if (action == 'R') {
    turnRight();
  } else if (action == 'S') {
    Serial.println("kör rakt fram");
    continueStraight();
    
  } else if(action == 'r'){
    Serial.println("Svänger höger utan linje");
    rightWithoutLine();
    
  } else if(action == 'l'){
    Serial.println("Svänger vänster utan linje");
    leftWithoutLine();
  } else if(action == 'd'){
    deadEnd();
  }
}

// Sväng vänster
void turnLeft() {
  Serial.println("Svänger vänster");
  motorLeft.setSpeed(maxSpeed / 2);
  motorRight.setSpeed(-maxSpeed / 2);
  delay(250);
  while (true) {
    motorLeft.setSpeed(maxSpeed / 2);
    motorRight.setSpeed(maxSpeed / 2);
  // delay(500);

    uint16_t sensorValues[numSensors];

    qtr.read(sensorValues);
    if (lineFound(sensorValues)) {
      break;
    }
    delay(10);
  }
  // delay(50);
  lastError = 0;
  lastTurnTime = millis();
}

// Sväng höger
void turnRight() {
  Serial.println("Svänger höger");
  motorLeft.setSpeed(maxSpeed / 2);
  motorRight.setSpeed(-maxSpeed / 2);
  delay(250);
  while (true) {
    motorLeft.setSpeed(-maxSpeed / 2);
    motorRight.setSpeed(-maxSpeed / 2);
  // delay(500);

    uint16_t sensorValues[numSensors];

    qtr.read(sensorValues);
    if (lineFound(sensorValues)) {
      break;
    }
    delay(10);
  }

  lastError = 0;

  // motorLeft.setSpeed(maxSpeed / 2);
  // motorRight.setSpeed(-maxSpeed / 2);
  // delay(150);

  // Registrera tidpunkt för avslutad sväng
  lastTurnTime = millis();
}

void continueStraight() {
  int error;
  int position;
  long startTime = millis();
  long duration = 500;
  while(millis() - startTime < duration){
    uint16_t sensorValues[numSensors];
    position = qtr.readLineBlack(sensorValues);
    error = position - 2500;
    
    int derivative = error - lastError;
    int correction = Kp * error + Kd * derivative;
    lastError = error;

    int leftSpeed = baseSpeed + correction;
    int rightSpeed = baseSpeed - correction;

    leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
    rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

    motorLeft.setSpeed(leftSpeed);
    motorRight.setSpeed(-rightSpeed);
    delay(1);
  }
  // delay(250);
  lastTurnTime = millis();
}

void rightWithoutLine() {
  motorLeft.setSpeed(maxSpeed / 2);
  motorRight.setSpeed(-maxSpeed / 2);
  // old batteries 600
  delay(1000);
  motorLeft.setSpeed(-maxSpeed / 2);
  motorRight.setSpeed(-maxSpeed / 2);
  // old batteries 1000
  delay(800);
  motorLeft.setSpeed(maxSpeed / 2);
  motorRight.setSpeed(-maxSpeed / 2);
    uint16_t sensorValues[numSensors];
  while (true) {
    qtr.read(sensorValues);
    if (lineFound(sensorValues)) {
      break;
    }
    delay(10);
  }
}

void leftWithoutLine() {
  motorLeft.setSpeed(maxSpeed / 2);
  motorRight.setSpeed(-maxSpeed / 2);
  // old batteries 600
  delay(1500);
  motorLeft.setSpeed(maxSpeed / 2);
  motorRight.setSpeed(maxSpeed / 2);
  // old batteries 1000
  delay(750);
  motorLeft.setSpeed(maxSpeed / 2);
  motorRight.setSpeed(-maxSpeed / 2);
    uint16_t sensorValues[numSensors];
  while (true) {
    qtr.read(sensorValues);
    if (lineFound(sensorValues)) {
      break;
    }
    delay(10);
  }
}

void deadEnd() {
//  Serial.println("Svänger höger 180 grader");

  motorLeft.setSpeed(-maxSpeed / 2);
  motorRight.setSpeed(-maxSpeed / 2);
  //delay(1000);

  uint16_t sensorValues[numSensors];
  while (true) {
    qtr.read(sensorValues);
    if (lineFound(sensorValues)) {
      break;
    }
    delay(10);
  }

  lastError = 0;

  motorLeft.setSpeed(maxSpeed / 2);
  motorRight.setSpeed(-maxSpeed / 2);
  delay(150);

  // Registrera tidpunkt för avslutad sväng
  lastTurnTime = millis();
}

void alignWithLine(){
  motorLeft.setSpeed(maxSpeed / 2);
  motorRight.setSpeed(maxSpeed / 2);
  uint16_t sensorValues[numSensors];
  while (true) {
    qtr.read(sensorValues);
    if (lineFound(sensorValues)) {
      break;
    }
    delay(1);
  }
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
}

// Linje hittad igen efter sväng
bool lineFound(uint16_t *sensorValues) {
  bool centerSensors = (sensorValues[2] > blackThreshold) || (sensorValues[3] > blackThreshold);
  bool outerSensors = (sensorValues[0] < blackThreshold) && (sensorValues[5] < blackThreshold);
  return (centerSensors && outerSensors);
}
