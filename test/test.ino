#include "Robojax_L298N_DC_motor.h" // library for easy interface with motor drivers 
#include "ICM_20948.h"
#include "Wire.h"

ICM_20948_I2C IMU; 

const int MAX_IMU_ERROR = 6; 

// designates the pins for driver 1 (LHS motor set)
#define DR1_ENA 2
#define DR1_IN1 3 
#define DR1_IN2 4

// which motor is this
#define DR1_IN3 5
#define DR1_IN4 6
#define DR1_ENB 7

// designates the pins for driver 2 (RHS motor set)
#define DR2_ENA 8
#define DR2_IN1 9
#define DR2_IN2 10

// which motor is this
#define DR2_IN3 11
#define DR2_IN4 12
#define DR2_ENB 13

#define motor1 1
#define motor2 2

const int FWD = 1;
const int BWD = 2; 

// sampling timer setup:
unsigned long previousMillis = 0; // Store last time updated
const long interval = 10; // Interval [milliseconds]

Robojax_L298N_DC_motor motorLeft(DR1_IN1, DR1_IN2, DR1_ENA, DR1_IN3, DR1_IN4, DR1_ENB, true); // the boolean argument enables debugging when true
Robojax_L298N_DC_motor motorRight(DR2_IN1, DR2_IN2, DR2_ENA, DR2_IN3, DR2_IN4, DR2_ENB, true);

void configureMotors() {
  motorLeft.begin();
  motorRight.begin();
}

void goStraight() {
  motorLeft.rotate(motor1, 100, FWD); 
  motorLeft.rotate(motor2, 100, FWD);
  motorRight.rotate(motor1, 100, FWD); 
  motorRight.rotate(motor2, 100, FWD);
}

void turnRight() {
  motorLeft.rotate(motor1, 70, FWD); 
  motorLeft.rotate(motor2, 70, FWD);
  motorRight.rotate(motor1, 70, BWD); 
  motorRight.rotate(motor2, 70, BWD);
}

void stopCar() {
//  motorLeft.rotate(motor1, 0, FWD); 
//  motorLeft.rotate(motor2, 0, FWD);
//  motorRight.rotate(motor1, 0, FWD); 
//  motorRight.rotate(motor2, 0, FWD);
  // TODO: investigate brake function
   motorLeft.brake(motor1);
   motorLeft.brake(motor2);
   motorRight.brake(motor1);
   motorRight.brake(motor2);
}

void setup() {
  Serial.begin(115200);
  
  // put your setup code here, to run once:
  configureMotors();

  Wire.begin(); // start I2C bus
  Wire.setClock(400000); // set clock speed

  bool initialized = false;
  while (!initialized) {
    IMU.begin(Wire, 0); // initialize IMU
    if (IMU.status == ICM_20948_Stat_Ok) {
      initialized = true;
    }
    Serial.print(IMU.statusString());
    delay(500);
  }
}

float yawRate() {
  IMU.getAGMT();
  return IMU.gyrY();
}

float yawAngle = 0; 

void turnCorner() {
    float yawAngle = 0;
    float startTime = millis();
    Serial.print("HELLO!");
    
    motorLeft.rotate(motor1, 70, FWD); 
    motorLeft.rotate(motor2, 70, FWD);
    motorRight.rotate(motor1, 70, BWD); 
    motorRight.rotate(motor2, 70, BWD);
    
    while (yawAngle <= 90) {
      float thisTime = millis();
      if (yawRate() > MAX_IMU_ERROR) {
        yawAngle += yawRate() * interval;
      }
      if (thisTime - startTime >= 500) {
        startTime = millis();
        Serial.print("The car has turned: ");
        Serial.print(yawAngle);
        Serial.println(" degrees");
      }
    }
    stopCar();
}

void loop() {
  turnCorner();
}
