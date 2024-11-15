// TODO:
// Better naming for motorpins
#include <QTRSensors.h>
#include <NewPing.h>
#include <Servo.h>

int servo1pin = 10; // servograb
int servo2pin = 9; //servolift

Servo servolift;
Servo servograb;

//Motor Pins
int motor1pin1 = 5; //blue left m1
int motor1pin2 = 6; //orange
int motor2pin1 = 11; //orange right m2
int motor2pin2 = 3; //blue

int motorSpeed = 50;   //regular motorspeed. The whole code works for =50

// Sonar: Pins , Max distance and Ping Timers
  #define TRIGGER_PIN 13 //trigger
  #define ECHO_PIN 12 //brown
  #define MAX_DISTANCE 100

  unsigned int pingSpeed = 500; // How often to ping (ms) 
  unsigned long pingTimer;  // Time for next ping
  NewPing sonar(TRIGGER_PIN,ECHO_PIN,MAX_DISTANCE);



// Sensors
  const uint8_t SensorCount = 6; 
  uint16_t sensors[SensorCount];
  static uint16_t lastError = 0;
  int llv = 600;
  QTRSensors qtr;

  int counter = 0;
  int counterDE = 0;
  int counterZy= 0;

void setup(){

  //servo
  servograb.attach(servo1pin);
  servolift.attach(servo2pin);

  //servolift.write(170);
  //delay(1000);
  servograb.write(80);
  servolift.write(70);



  setupIRSensors();
  setupLocomotion();
}

void loop(){
  int16_t position = qtr.readLineBlack(sensors);
  int tt = 535;

  position = qtr.readLineBlack(sensors);
    
  if(sensors[0] > llv && sensors[1] > llv || sensors[4] > llv && sensors[5] > llv){
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(sensors[i]);
      Serial.print(' ');
    }
    Serial.print("Junction or turn");
    

    counter++;

    Serial.print(counter);

    if (counter == 2 || counter == 19 || counter == 23 || counter == 31 || counter == 40  || counter == 52  ){ //right turns
     setMotors(1.4*motorSpeed, -0.8*motorSpeed);
     delay(tt);
     while(sensors[2] < llv && sensors [3] <llv){
       int16_t position = qtr.readLineBlack(sensors);
      }
    }else if (counter ==  4 || counter == 22 || counter == 45 || counter == 46 ){ //left turns
    setMotors(-0.8*motorSpeed, 1.4*motorSpeed);
     delay(tt);
     while(sensors[2] < llv && sensors [3] <llv){
       int16_t position = qtr.readLineBlack(sensors);
      }
    }else if (counter == 5 || counter == 6 || counter == 24|| counter == 33 || counter == 34 || counter == 43 || counter == 51){ //straight
    setMotors(motorSpeed, motorSpeed);
     delay(tt);
     while(sensors[2] < llv){
       int16_t position = qtr.readLineBlack(sensors);
      }  
    } else if(sensors[0] > llv && sensors[1] > llv){
     Serial.print(sensors[0]);
     Serial.print(" LEFT\n");

     setMotors(-0.8*motorSpeed, 1.4*motorSpeed);
     delay(tt);
     while(sensors[2] < llv && sensors [3] <llv){
        int16_t position = qtr.readLineBlack(sensors);
      }  
    } else if(sensors[4] > llv && sensors[5] > llv){
      Serial.print(sensors[5]);
      Serial.print(" RIGHT\n");
      setMotors(1.4*motorSpeed, -0.8*motorSpeed);
      delay(tt);
      while(sensors[2] < llv && sensors [3] <llv){
        int16_t position = qtr.readLineBlack(sensors);
      }   
    }

  }


 if(sensors[0] < llv && sensors[1] < llv && sensors[2] < llv && sensors[3] < llv && sensors[4] < llv && sensors[5] < llv){
    Serial.print("DEAD_END\n");

    counterDE++;

    setMotors(0, 0);
    delay(tt);
    unsigned int uS = sonar.ping(); //Send a ping
    unsigned int distance = sonar.convert_cm(uS); //Convert ping
    position = qtr.readLineBlack(sensors);
    
    
   if(counterDE == 6){  // straight
      setMotors(motorSpeed, motorSpeed);

      while(sensors[0] < llv && sensors[1] < llv && sensors[2] < llv && sensors[3] < llv && sensors[4] < llv && sensors[5] < llv) {
        position =  qtr.readLineBlack(sensors);
      }
      
    } else if (counterDE== 3 || counterDE == 5)  { //straight with optimized speed
      setMotors(motorSpeed, motorSpeed);

      while(sensors[0] < llv && sensors[1] < llv && sensors[2] < llv && sensors[3] < llv && sensors[4] < llv && sensors[5] < llv) {
        position =  qtr.readLineBlack(sensors);
      }  

    } else if (distance < 30 && sensors[0] < llv && sensors[1] < llv && sensors[2] < llv && sensors[3] < llv && sensors[4] < llv && sensors[5] < llv){ //180 degree turns
      setMotors(-20, 20);    // maybe we can increase these values
      while(sensors[2] < llv && sensors[3] < llv) {
        position =  qtr.readLineBlack(sensors);
     }
    } else if (counterDE == 9){       //end action
      setMotors(0, 0);
      delay(10000);
    } 

  }    
    
  

  position = qtr.readLineBlack(sensors);


  unsigned int uS = sonar.ping(); //Send a ping
  unsigned int distance = sonar.convert_cm(uS); //Convert ping
  Serial.print(sonar.convert_cm(uS)); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
  Serial.println("cm");


   if (distance == 10) {
   setMotors(0,0);
   counterZy++;
   delay(1000);
   servolift.write(160);
   delay(1000);
   servograb.write(140);
   delay(1000);
   servolift.write(70);
   delay(1000);
   servograb.write(80);
   delay(1000);
   unsigned int uS = sonar.ping();
   Serial.print("grab");
   
 
}
 else if (distance == 10 && counterZy==2) {
   setMotors(0,0);
   counterZy++;
   delay(1000);
   servolift.write(160);
   delay(1000);
   servograb.write(140);
   delay(1000);
   servolift.write(70);
   delay(1000);
   unsigned int uS = sonar.ping();
   Serial.print("grab");
  }
 


  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensors[i]);
    Serial.print(' ');
  }
  //Serial.print("STRAIGHT\n");
  Serial.println();
  int16_t error = position - 2500;

  float P = error;
  float I = I + error;
  float D = error - lastError;

  float KP = 0.035;
  float KI = 0;
  float KD = 0;

  int16_t correction = (KP * P) + (KI * I) + (KD * D);

  int16_t m1Speed = motorSpeed + correction;
  int16_t m2Speed = motorSpeed - correction;

  //Only positive speeds
  if(m1Speed < 0) { m1Speed = 0; }
  if(m2Speed < 0) { m2Speed = 0; }

  //Speed limit at 50
  if(m1Speed > 50) { m1Speed = 50;}
  if(m2Speed > 50) { m2Speed = 50;}

  setMotors(m1Speed, m2Speed);

  lastError = error; 
}

void setupIRSensors(){
  // Set line sensor to Analog and set 
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);

  // Calibrate
  for (uint8_t i = 0; i < 250; i++)
  {
    qtr.calibrate();
    delay(2);
  }
}

void setupLocomotion(){
  // Set motor pins to output
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
}



bool checkSensors(){
  bool lostLine;
  unsigned int lowestLineValue = 20;
  int16_t position = qtr.readLineBlack(sensors);
  for(int i = 0; i < SensorCount; i++){
    if(sensors[i] > lowestLineValue){
      lostLine = false; 
      break;
    }
  }

  return lostLine;
}

//Function to run the motors, the Speeds can be set between -100 and 100
void setMotors(int leftSpeed,int rightSpeed){ 
  if(rightSpeed>=0 and rightSpeed != 0){
    int Speed = map(rightSpeed, 0, 100, 0, 255);
    analogWrite(motor1pin1, Speed);
    digitalWrite(motor1pin2, LOW);    
  }
  if(rightSpeed<=0 and rightSpeed != 0){
    int Speed = map(rightSpeed, -100, 0, 255, 0);
    digitalWrite(motor1pin1, LOW);
    analogWrite(motor1pin2, Speed);
  }
  if(rightSpeed == 0){
   digitalWrite(motor1pin1, LOW);
   digitalWrite(motor1pin2, LOW);
  }
  if(leftSpeed>=0 and leftSpeed != 0){
    int Speed = map(leftSpeed, 0, 100, 0, 255);
    analogWrite(motor2pin1, Speed);
    digitalWrite(motor2pin2, LOW); 
  }
  if(leftSpeed<=0 and leftSpeed != 0){
    int Speed = map(leftSpeed, -100, 0, 255, 0);
    digitalWrite(motor2pin1, LOW);
    analogWrite(motor2pin2, Speed);
  }
  if(leftSpeed == 0){
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, LOW);   
  }  
}