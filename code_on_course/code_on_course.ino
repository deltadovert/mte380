#include <Wire.h>
#include <VL53L0X.h>
#include "Robojax_L298N_DC_motor.h"
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

VL53L0X LeftFrontTOF;
VL53L0X LeftBackTOF; 

ICM_20948_I2C IMU;


// front left motor
#define DR1_ENA 7
#define DR1_IN1 6 
#define DR1_IN2 5

// back left motor
#define DR1_IN3 4
#define DR1_IN4 3
#define DR1_ENB 2

// back right
#define DR2_ENA 8
#define DR2_IN1 9
#define DR2_IN2 10

// front right
#define DR2_IN3 11
#define DR2_IN4 12
#define DR2_ENB 13

#define motor1 1
#define motor2 2

#define TRIG 50
#define ECHO 52

const int FWD = 2;
const int BWD = 1; 

Robojax_L298N_DC_motor motorLeft(DR1_IN1, DR1_IN2, DR1_ENA, DR1_IN3, DR1_IN4, DR1_ENB, false); // the boolean argument enables debugging when true
Robojax_L298N_DC_motor motorRight(DR2_IN1, DR2_IN2, DR2_ENA, DR2_IN3, DR2_IN4, DR2_ENB, false);

void setup()
{
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(32, OUTPUT);
  pinMode(30, OUTPUT);
  digitalWrite(32, LOW);
  digitalWrite(30, LOW);

  delay(500);
  Wire.begin();

  Serial.begin (115200);

  pinMode(30, INPUT);
  delay(150);
  LeftBackTOF.init(true);
  delay(100);
  LeftBackTOF.setAddress(0x70);

  pinMode(32, INPUT);
  delay(150); 
  LeftFrontTOF.init(true);
  delay(100);
  LeftFrontTOF.setAddress(0x45); 

  motorLeft.begin();
  motorRight.begin();

  bool initialized = false;
  while (!initialized)
  {
    IMU.begin(Wire, 1);

    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(IMU.statusString());
    if (IMU.status != ICM_20948_Stat_Ok)
    {
      Serial.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  LeftFrontTOF.startContinuous();
  LeftBackTOF.startContinuous();
}

//float angleWrtWall() {
////  return ()
//}

float yawAngle = 0;

float prevTime = millis();

float totalYawRate = 0;
int iterYaw = 0;
float averageYaw = 0;

void readGyro() { 
  // read ToF

  // read gyro
  if (IMU.dataReady()) {
    IMU.getAGMT();
    if (abs(IMU.gyrZ()) > 6) {
      yawAngle += IMU.gyrZ() * (millis() - prevTime) / 1000;
    }
    prevTime = millis(); 
    //Serial.println(yawAngle);
  }
}

long duration;
float frontDistance;

void usReading() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(ECHO, HIGH);
  // Calculating the distance
  frontDistance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(frontDistance);

}

void correctLeft() {
  motorLeft.rotate(motor1, 80, BWD);
  motorLeft.rotate(motor2, 80, BWD);
  motorRight.rotate(motor1, 0, FWD);
  motorRight.rotate(motor2, 0, FWD);
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

float tof_front_dist_filtered = 0;
float tof_left_dist_filtered = 0;

int LB_OFFSET = 62; // 35
int LF_OFFSET = 35; //62
int FRONT_OFFSET = 50;
int LF_BACK = 20; 

float lf_dist = 0;
float lb_dist = 0;

float initialAngle = 0;

void processData() {
  lf_dist = LeftFrontTOF.readRangeContinuousMillimeters();
  lb_dist = LeftBackTOF.readRangeContinuousMillimeters();

  if (lf_dist - LF_OFFSET < 0) {
    lf_dist = 0;
  } else {
    lf_dist = lf_dist - LF_OFFSET - LF_BACK;
  }
  if (lb_dist - LB_OFFSET < 0) {
    lb_dist = 0;
  } else {
    lb_dist = lb_dist - LB_OFFSET;
  }
}

void reInitTOF(VL53L0X &TOF, int XSHUT, int ADDRESS) {
  pinMode(XSHUT, OUTPUT);
  digitalWrite(XSHUT, LOW);
  pinMode(XSHUT, INPUT);
  delay(150);
  TOF.init(true);
  TOF.setAddress(ADDRESS);
  TOF.startContinuous();
}

bool startUp = true; 
bool startedTurn = false; 
bool turning = false; 

float timeStarted = 0;

bool stoponeseconds = false;

bool readFront = true;

int stopDistances[6] = {500, 250, 250, 450, 450, 450};
int turn = 0;

void loop()
{
  if (startUp) {
    motorStart(100, 100);
    delay(500);
    motorStart(55, 100); 
    startUp = false;
  }

  usReading();
  processData();
  readGyro();

  //Serial.print("front reading: ");
  //Serial.println(frontDistance);
  //Serial.print("Left back reading: ");
  //Serial.println(lb_dist);
  //Serial.println(" ");

 // Serial.println(average);
  if (frontDistance <= stopDistances[turn] && frontDistance > 0 && readFront) {
    motorStop();
    startedTurn = true; 
    readFront = false;
  }

  if (startedTurn) {
    initialAngle = yawAngle;
    timeStarted = millis();
    Serial.println("Starting turn");
    startedTurn = false;
    stoponeseconds = true;
  }

  if (stoponeseconds && (millis() - timeStarted >= 1000)) {
    turnRight();
    turning = true;
    stoponeseconds = false;
  }

  if (turning) {
    //Serial.println("In turning");
    //Serial.println(yawAngle - initialAngle);
    if (abs(yawAngle - initialAngle) >= 85) {
       motorStop();
       //reInitTOF(FrontTOF, 28, 0x33);
       turning = false; 
       startUp = true;
       readFront = true;
       turn++;
       delay(500);
    }
  }
}
