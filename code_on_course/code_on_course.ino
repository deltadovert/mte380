#include <Wire.h>
#include <VL53L0X.h>
#include "Robojax_L298N_DC_motor.h"
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "SR04.h"
#include "ArduPID.h"
#define TRIG_FRONT 28
#define ECHO_FRONT 26
#define TRIG_LEFT 32
#define ECHO_LEFT 30

SR04 UltrasonicLeft = SR04(ECHO_LEFT, TRIG_LEFT); 
SR04 UltrasonicFront = SR04(ECHO_FRONT, TRIG_FRONT);

ICM_20948_I2C IMU;

// front left motor
#define DR1_ENA 13
#define DR1_IN1 12
#define DR1_IN2 11

// back left motor
#define DR1_IN3 10
#define DR1_IN4 9
#define DR1_ENB 8

// back right
#define DR2_ENA 2
#define DR2_IN1 3
#define DR2_IN2 4

// front right
#define DR2_IN3 5
#define DR2_IN4 6
#define DR2_ENB 7

#define motor1 1
#define motor2 2

#define TRIG 50
#define ECHO 52

const int FWD = 2;
const int BWD = 1; 

Robojax_L298N_DC_motor motorLeft(DR1_IN1, DR1_IN2, DR1_ENA, DR1_IN3, DR1_IN4, DR1_ENB, false); // the boolean argument enables debugging when true
Robojax_L298N_DC_motor motorRight(DR2_IN1, DR2_IN2, DR2_ENA, DR2_IN3, DR2_IN4, DR2_ENB, false);

int left_motor_power = 50;
int right_motor_power = 50;

// PID Controllers
ArduPID adjustController;

double adjust_setpoint = 5.08;
double adjust_input;
double adjust_output;
double adjust_p = 500;
double adjust_i = 1000;
double adjust_d = 10000;

void setup() {
  adjustController.begin(&adjust_input, &adjust_output, &adjust_setpoint, adjust_p, adjust_i, adjust_d);
  adjustController.setOutputLimits(-50, 50);
  adjustController.setWindUpLimits(-10, 10);
  adjustController.start();

  delay(500);
  Wire.begin();

  Serial.begin (115200);

  motorLeft.begin();
  motorRight.begin();

  bool initialized = false;
  while (!initialized) {
    IMU.begin(Wire, 1);

    //Serial.print(F("Initialization of the sensor returned: "));
    //Serial.println(IMU.statusString());
    if (IMU.status != ICM_20948_Stat_Ok)
    {
      //Serial.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
}

float yawAngle = 0;

float prevTime = millis();


void readGyro() { 
  // read ToF

  // read gyro
  if (IMU.dataReady()) {
    IMU.getAGMT();
    if (abs(IMU.gyrZ()) > 6) {
      yawAngle += IMU.gyrZ() * (millis() - prevTime) / 1000;
    }
    prevTime = millis(); 
  }
}

void turnRight() {
  motorLeft.rotate(motor1, 100, FWD);
  motorLeft.rotate(motor2, 100, FWD);
  motorRight.rotate(motor1, 100, BWD);
  motorRight.rotate(motor2, 100, BWD);
}

void motorStart(int speedLeft, int speedRight) {
  motorLeft.rotate(motor1, speedLeft, FWD);
  motorLeft.rotate(motor2, speedLeft, FWD);
  motorRight.rotate(motor1, speedRight, FWD);
  motorRight.rotate(motor2, speedRight, FWD);
}

void motorStop() {
  motorLeft.rotate(motor1, 0, FWD);
  motorLeft.rotate(motor2, 0, FWD);
  motorRight.rotate(motor1, 0, FWD);
  motorRight.rotate(motor2, 0, FWD);
}

float initialAngle = 0;

bool startUp = true; 
bool startedTurn = false; 
bool turning = false; 

float timeStarted = 0;

bool stoponeseconds = false;

bool readFront = true;

int stopDistances[6] = {25, 25, 25, 47, 47, 47}; // these distances might be fucked 
int sideSetpoints[6] = {5.08, 5.08, 5.08, 35.56, 35.56}; // 66.04
int turn = 0;

void straighten() {
  adjust_input = UltrasonicLeft.Distance();
  adjustController.compute();
  left_motor_power = 100;
  right_motor_power = 100;
  left_motor_power += adjust_output;  
  right_motor_power -= adjust_output;
  if (left_motor_power < 0) {
    left_motor_power = 0;
  }
  if (left_motor_power > 100) {
    left_motor_power = 100;
  }
  if (right_motor_power < 0) {
    right_motor_power = 0;
  }
  if (right_motor_power > 100) {
    right_motor_power = 100;
  }
  motorStart(left_motor_power, right_motor_power);
}

void loop() {
  if (readFront) {
    straighten();
  }
  readGyro();

  if (UltrasonicFront.Distance() <= stopDistances[turn] && UltrasonicFront.Distance() > 0 && readFront) {
    motorStop();
    startedTurn = true; 
    readFront = false;
  }

  if (startedTurn) {
    initialAngle = yawAngle;
    timeStarted = millis();
    startedTurn = false;
    stoponeseconds = true;
  }

  if (stoponeseconds && (millis() - timeStarted >= 1000)) {
    turnRight();
    turning = true;
    stoponeseconds = false;
  }

  if (turning) {
    if (abs(yawAngle - initialAngle) >= 80) {
       motorStop();
       //reInitTOF(FrontTOF, 28, 0x33);
       turning = false; 
       startUp = true;
       readFront = true;
       turn++;
       adjust_setpoint = sideSetpoints[turn];
       delay(500);
    }
  }
}
