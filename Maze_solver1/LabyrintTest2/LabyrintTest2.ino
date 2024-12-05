#include <QTRSensors.h>
#include "CytronMotorDriver.h"
#include <NewPing.h>

// Definiera vilka analoga pins som används
const uint8_t sensorPins[] = {A0, A1, A2, A3, A4, A5}; // 6 sensorer
const uint8_t numSensors = 6; // Antal sensorer

// Skapa ett QTRSensors-objekt
QTRSensors qtr;

// Motorstyrning för MDD3A
CytronMD motorLeft(PWM_PWM, 9, 10);   // Vänster motor: PWM_A = Pin 9, PWM_B = Pin 10
CytronMD motorRight(PWM_PWM, 3, 5);   // Höger motor: PWM_A = Pin 3, PWM_B = Pin 5

//Ultrasonic 
#define TRIGGER_PIN  11  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     12  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
long cylinderLimit = 5;
bool isCylinderFound = false;


NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

//Phases
//0: Following line, 1: Navigate missing line, 2: cylinder pickup
int phase = 0;
unsigned long timeSince;
unsigned long timeLimit = 100;

// PID-variabler
float Kp = 0.2;  // Justera efter behov
float Ki = 0;    // Bör ej behöva användas
float Kd = 0.1;

int lastError = 0;
float integral = 0;

// Maximal hastighet
int maxSpeed = 200;
int baseSpeed;

// Definiera svängsekvensen vid korsningar ('L' för vänster, 'R' för höger, 'S' för rakt fram)
char path[] = {'R', 'L', 'L', 'R'}; // Exempel på sekvens
int pathLength = sizeof(path) / sizeof(path[0]);
int pathIndex = 0; // Index för aktuell svänginstruktion

int deadEndFound = 0;

char missingLinePath[] = {'R','L'};
int missingPathLength = sizeof(missingLinePath) / sizeof(missingLinePath[0]);
int missingPathIndex = 0;

// Tröskelvärde för att detektera svart linje (justera baserat på dina sensorvärden)
const uint16_t blackThreshold = 500; // Justera efter behov

// *** Nytt: Variabler för kalibreringsfördröjning ***
unsigned long calibrationEndTime;
const unsigned long calibrationDelay = 2000; // 1 sekund
// *** Slut på nytt ***

void setup() {
  // Ställ in 3.3V som referensspänning
  analogReference(EXTERNAL); // Använd extern referens (3.3V kopplad till AREF)

  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, numSensors); // Tilldela sensorpins

  // Starta seriell kommunikation
  Serial.begin(9600);
  Serial.println("Startar kalibrering...");

  // Kalibrera sensorerna
  calibrateSensors();
  Serial.println("Kalibrering klar!");

  // *** Nytt: Spara tiden när kalibreringen är klar ***
  calibrationEndTime = millis();
  // *** Slut på nytt ***

  // Stoppa motorerna initialt
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
  baseSpeed = maxSpeed / 2;
  timeSince = millis();
}

void loop() {
  uint16_t sensorValues[numSensors];

  // Läs av linjens position med readLineBlack()
  int position = qtr.readLineBlack(sensorValues); // Returnerar 0–5000

  // Beräkna felet från mitten
  int error = position - 2500;

  // PID-beräkningar
  integral += error;
  int derivative = error - lastError;
  int correction = Kp * error + Ki * integral + Kd * derivative;

  lastError = error;
  
  switch(phase) {
    case 0:
      //checkForCylinder();
      Serial.println("line follow");
      lineFollow(sensorValues, correction);
      break;
    case 1:
      //checkForCylinder();
      Serial.println("missing line");
      navMissingLine();
      break;
    case 2:
      Serial.println("approach cyl");
      approachCylinder(correction);
    case 3:
      Serial.println("pickup cyl");
      pickupCylinder();
      break;
  }

  delay(1); // Kort fördröjning för att undvika överbelastning
}

void lineFollow(uint16_t sensorValues, int correction) {
// Beräkna motorhastigheter
  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;
  if(millis()-timeSince > timeLimit) {
    timeSince = millis();
    if(checkForCylinder()){
      phase = 2;
    }
  }
  // Begränsa hastigheterna till tillåtna värden
  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  // Sätt motorhastigheter (korrigerad polaritet vid behov)
  motorLeft.setSpeed(leftSpeed);
  motorRight.setSpeed(-rightSpeed);

  // Kontrollera om vi ska börja söka efter korsningar efter kalibrering
  if (millis() - calibrationEndTime > calibrationDelay) {
    // Kontrollera om korsning upptäcks
    if (isIntersection(sensorValues)) {
      // Stoppa motorerna
      motorLeft.setSpeed(0);
      motorRight.setSpeed(0);

      // Om det finns fler instruktioner i path[], utför nästa sväng
      if (pathIndex < pathLength) {
        char action = path[pathIndex++];
        handleIntersection(action);
      } else {
        // Inga fler instruktioner; fortsätt att följa linjen
        Serial.println("Inga fler instruktioner. Fortsätter rakt fram.");
      }
    }
  }
  
}
void navMissingLine() {
  //if line found
  //Move to phase 0
  //else
  //navigate missing path
  //use cytron encoder to read rpm and turn
  uint16_t sensorValues[numSensors];
  qtr.read(sensorValues);
  if(lineFound(sensorValues)){
    phase = 0;
  } else{

  }

}
void approachCylinder(int correction){
  long dist = sonar.ping_cm();
  int leftSpeed = (float)dist/(float)cylinderLimit * (baseSpeed + correction);
  int rightSpeed = (float)dist/(float)cylinderLimit * (baseSpeed - correction);

  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  motorLeft.setSpeed(leftSpeed);
  motorRight.setSpeed(-rightSpeed);
  if(dist < 1){
    phase = 3;
  }
}
void pickupCylinder() {
  //move servo0 down
  //wait for this to execute

  //close servo1
  //wait for this to execute

  //move servo0 up
  //wait for this to execute

  //open servo1 up

  // Hope for cylinder being picked up
  Serial.println("pickup cyl");
  while(true) {
      motorLeft.setSpeed(100);
      motorRight.setSpeed(100);
  }
  //phase = 0;
}

void calibrateSensors() {
  // Rotera roboten och kalibrera sensorerna
  for (int i = 0; i < 400; i++) {
    qtr.calibrate();

    // Flytta roboten lite för att exponera olika ytor (rotera)
    motorLeft.setSpeed(100);
    motorRight.setSpeed(100);

    delay(1); // Vänta 1 ms mellan varje kalibreringsavläsning
  }

  // Stoppa motorerna efter kalibrering
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
}

// Funktion för att detektera korsning
bool isIntersection(uint16_t *sensorValues) {
  // Detektera korsning när de två vänstra eller två högra sensorerna detekterar svart linje men inte motsatta sidan

  // Vänster korsning: två vänstra sensorer > blackThreshold, högra sensorn < blackThreshold
  bool leftIntersection = (sensorValues[0] > blackThreshold && sensorValues[1] > blackThreshold && sensorValues[5] < blackThreshold);

  // Höger korsning: två högra sensorer > blackThreshold, vänstra sensorn < blackThreshold
  bool rightIntersection = (sensorValues[4] > blackThreshold && sensorValues[5] > blackThreshold && sensorValues[0] < blackThreshold);

  // T-korsning: både vänstra och högra yttersta sensorer > blackThreshold
  bool tIntersection = (sensorValues[0] > blackThreshold && sensorValues[5] > blackThreshold);

  // Returnera true om någon typ av korsning detekteras
  return (leftIntersection || rightIntersection || tIntersection);
}

// Hantera sväng vid korsning
void handleIntersection(char action) {
  Serial.print("Hanterar korsning: ");
  Serial.println(action);

  if (action == 'L') {
    motorLeft.setSpeed(100);
  motorRight.setSpeed(-100);
  delay(200);
    turnLeft();
  } else if (action == 'R') {
     motorLeft.setSpeed(100);
  motorRight.setSpeed(-100);
  delay(200);
    turnRight();
  } else if (action == 'S') {
    // Fortsätt rakt fram, inget behöver göras
    Serial.println("Fortsätter rakt fram.");
  }
}

// Funktion för att svänga vänster
void turnLeft() {
  Serial.println("Svänger vänster");

  // Börja svänga vänster genom att rotera på stället
  motorLeft.setSpeed(maxSpeed / 2);
  motorRight.setSpeed(maxSpeed / 2);

  // Ignorera linjeavläsningar under en kort period för att lämna korsningen
  delay(500); // Justera tiden efter behov

  // Fortsätt svänga tills linjen hittas igen
  uint16_t sensorValues[numSensors];
  while (true) {
    qtr.read(sensorValues);
    if (lineFound(sensorValues)) {
      break;
    }
    delay(10);
  }

  // Återställ PID-variabler
  lastError = 0;
  integral = 0;

  // Fortsätt framåt en kort stund för att stabilisera roboten
  motorLeft.setSpeed(maxSpeed / 2);
  motorRight.setSpeed(-maxSpeed / 2);
  delay(100); // Justera tiden efter behov
}

// Funktion för att svänga höger
void turnRight() {
  Serial.println("Svänger höger");

  // Börja svänga höger genom att rotera på stället
  motorLeft.setSpeed(-maxSpeed / 2);
  motorRight.setSpeed(-maxSpeed / 2);

  // Ignorera linjeavläsningar under en kort period för att lämna korsningen
  delay(500); // Justera tiden efter behov

  // Fortsätt svänga tills linjen hittas igen
  uint16_t sensorValues[numSensors];
  while (true) {
    qtr.read(sensorValues);
    if (lineFound(sensorValues)) {
      break;
    }
    delay(10);
  }

  // Återställ PID-variabler
  lastError = 0;
  integral = 0;

  // Fortsätt framåt en kort stund för att stabilisera roboten
  motorLeft.setSpeed(maxSpeed / 2);
  motorRight.setSpeed(-maxSpeed / 2);
  delay(100); // Justera tiden efter behov
}

// Funktion för att detektera om linjen hittas igen efter sväng
bool lineFound(uint16_t *sensorValues) {
  // Kolla om centrala sensorer detekterar svart linje och yttre sensorer inte gör det
  bool centerSensors = (sensorValues[2] > blackThreshold) || (sensorValues[3] > blackThreshold);
  bool outerSensors = (sensorValues[0] < blackThreshold) && (sensorValues[5] < blackThreshold);

  if (centerSensors && outerSensors) {
    return true;
  } else {
    return false;
  }
}
bool checkForCylinder() {
  // long DistanceSum = 0;
  // for(int i = 0; i<5; i++) {
  //   DistanceSum += sonar.ping_cm();
  // }
  long distance = sonar.ping_cm();
  
  Serial.println(distance); // Send ping, get distance in cm and print result (0 = outside set distance range)
  if(distance < cylinderLimit){
    Serial.println("Cylinder within 5cm");
    return true;
  } else {
    Serial.println("Cylinder not within 5cm");
    return false;
  }
  
}

