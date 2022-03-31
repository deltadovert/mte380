#include "Robojax_L298N_DC_motor.h" // library for easy interface with motor drivers 
#include "Wire.h"
#include "DFRobot_VL53L0X.h"

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

const int FWD = 2;
const int BWD = 1; 

Robojax_L298N_DC_motor motorLeft(DR1_IN1, DR1_IN2, DR1_ENA, DR1_IN3, DR1_IN4, DR1_ENB, true); // the boolean argument enables debugging when true
Robojax_L298N_DC_motor motorRight(DR2_IN1, DR2_IN2, DR2_ENA, DR2_IN3, DR2_IN4, DR2_ENB, true);

#define SERIAL_PORT Serial
#define WIRE_PORT Wire 

DFRobot_VL53L0X TOF_Front;

int interval = 10; 

void setup()
{
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT)
  {
  };

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  motorLeft.begin();
  motorRight.begin();

  TOF_Front.begin(0x50); // Set I2C device address
  TOF_Front.setMode(DFRobot_VL53L0X::eContinuous, DFRobot_VL53L0X::eLow); 
  TOF_Front.start();
}

float prevTime = millis();

bool started = false; 

void readData(float &frontDistance) { 
  // read ToF
  if (millis() - prevTime >= interval) {
    frontDistance = TOF_Front.getDistance() - 60;
    prevTime = millis(); 
  }
}

void startMotors() {
  motorLeft.rotate(motor1, 55, FWD);
  motorLeft.rotate(motor2, 55, FWD);
  motorRight.rotate(motor1, 100, FWD);
  motorRight.rotate(motor2, 100, FWD);
}

void stopMotors() {
  motorLeft.rotate(motor1, 0, FWD);
  motorLeft.rotate(motor2, 0, FWD);
  motorRight.rotate(motor1, 0, FWD);
  motorRight.rotate(motor2, 0, FWD);
}

float target = 300; 
bool firstTime = true;
float frontDistance = 1000;

void loop()
{
  delay(100);  
  // if distance to wall <= TARGET
  // set turn to true
  
  if (!started) {
    started = true;
    startMotors();
  }

  else {
    if (firstTime) {
      delay(500);
      firstTime = false; 
    }
    //readData(frontDistance);
    //Serial.print("Distance: ");
    //Serial.println(frontDistance);
    delay(6000);
    stopMotors();
    
  }
  
}
