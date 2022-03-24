// TODO: confirm distances for tofDistanceArray

#include "ICM_20948.h" // this library makes working with IMU a whole lot easier
#include "Robojax_L298N_DC_motor.h" // library for easy interface with motor drivers 

ICM_20948_I2C IMU; // interface for IMU

// initialize vars 
float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, yaw;
float deltat; 

#define AD0_VAL 1 

// designates the pins for driver 1 (LHS motor set)
#define DR1_IN1 2 
#define DR1_IN2 3
#define DR1_IN3 4
#define DR1_IN4 5
#define DR1_ENA 6
#define DR1_ENB 7

// designates the pins for driver 2 (RHS motor set)
#define DR2_IN1 8
#define DR2_IN2 9
#define DR2_IN3 10
#define DR2_IN4 11
#define DR2_ENA 12
#define DR2_ENB 13

#define motor1 1
#define motor2 2

const int CW = 1;
const int CCW = 2; 

Robojax_L298N_DC_motor motorLeft(DR1_IN1, DR1_IN2, DR1_ENA, DR1_IN3, DR1_IN4, DR1_ENB, true); // the boolean argument enables debugging when true
Robojax_L298N_DC_motor motorRight(DR2_IN1, DR2_IN2, DR2_ENA, DR2_IN3, DR2_IN4, DR2_ENB, true);

// distances 
const float tofDistanceArray[11] = [50.8, 50.8, 50.8, 355.6, 355.6, 355.6, 355.6, 660.4, 660.4, 660.4, 660.4]; 

void setup() {
  Serial.begin(115200); // for displaying data 
  
  Wire.begin(); // start I2C bus
  Wire.setClock(400000); // set clock speed

  bool initialized = false;
  while (!initialized) {
    IMU.begin(Wire, AD0_VAL); // initialize IMU
    if (IMU.status != ICM_20948_Stat_Ok) {
      Serial.println("Trying again...");
    }
    else {
      initialized = true; 
    }
  }

  // initialize motors
  motorLeft.begin();
  motorRight.begin();
}

float timestep = 2; 
float yaw = 0;

void getYaw() {
  IMU.getAGMT();
  if (abs(IMU.gyrZ()) > 6) {
    yaw += IMU.gyrZ() * timestep / 1000;
  }
  return yaw; 
}

void goForward() {
  motorLeft.rotate(motor1, 100, CW);
  motorLeft.rotate(motor2, 100, CW);
  motorRight.rotate(motor1, 100, CW);
  motorRight.rotate(motor2, 100, CW);
}

void rotateCW90() {
  updateAngles();
  const initialYaw = yaw;
  const currentYaw = initialYaw;
  motorLeft.rotate(motor1, 100, CW);
  motorLeft.rotate(motor2, 100, CW);
  motorRight.rotate(motor1, 100, CCW); // need the right wheels spinning backwards to turn CW in place
  motorRight.rotate(motor2, 100, CCW);
  while (abs(currentYaw - initialYaw)) {
    updateAngles(); 
    currentYaw = yaw;
  }
  motorRight.rotate(motor1, 100, CW); 
  motorRight.rotate(motor2, 100, CW);
}

void loop() {
  if (IMU.dataReady()) {
    updateValues(); 
  }
}
