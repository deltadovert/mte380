#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "ArduPID.h"

#include <VL53L0X.h>
#include "Robojax_L298N_DC_motor.h"
#include "Wire.h"

VL53L0X FrontTOF;
VL53L0X LeftFrontTOF;
VL53L0X LeftBackTOF; 

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

Robojax_L298N_DC_motor motorLeft(DR1_IN1, DR1_IN2, DR1_ENA, DR1_IN3, DR1_IN4, DR1_ENB, false); // the boolean argument enables debugging when true
Robojax_L298N_DC_motor motorRight(DR2_IN1, DR2_IN2, DR2_ENA, DR2_IN3, DR2_IN4, DR2_ENB, false);

int left_motor_power = 50;
int right_motor_power = 50;

// PID Controllers
ArduPID adjustController;

double adjust_setpoint = 100;
double adjust_input;
double adjust_output;
double adjust_p = 3;
double adjust_i = 1;
double adjust_d = 1.5;


void setup() {
  adjustController.begin(&adjust_input, &adjust_output, &adjust_setpoint, adjust_p, adjust_i, adjust_d);
  adjustController.setOutputLimits(-255, 255);
  adjustController.setWindUpLimits(-10, 10);
  adjustController.start();
  
  pinMode(32, OUTPUT);
  pinMode(30, OUTPUT);
  pinMode(28, OUTPUT);
  digitalWrite(32, LOW);
  digitalWrite(30, LOW);
  digitalWrite(28, LOW);

  delay(500);
  Wire.begin();

  Serial.begin (115200);

  pinMode(30, INPUT);
  delay(150);
  LeftBackTOF.init(true);
  delay(100);
  LeftBackTOF.setAddress(0x70);

  pinMode(28, INPUT);
  delay(150);
  FrontTOF.init(true);
  delay(100);
  FrontTOF.setAddress(0x33);

  pinMode(32, INPUT);
  delay(150); 
  LeftFrontTOF.init(true);
  delay(100);
  LeftFrontTOF.setAddress(0x45); 

  motorLeft.begin();
  motorRight.begin();

  FrontTOF.startContinuous();
  LeftFrontTOF.startContinuous();
  LeftBackTOF.startContinuous();
}

int normalizedControllerOutput() {
  return map(adjust_output, -255, 255, -100, 100);
}


void motorStart(int speedLeft, int speedRight) {
  motorLeft.rotate(motor1, speedLeft, FWD);
  motorLeft.rotate(motor2, speedLeft, FWD);
  motorRight.rotate(motor1, speedRight, FWD);
  motorRight.rotate(motor2, speedRight, FWD);
}
  
void loop() {
  adjust_input = LeftBackTOF.readRangeSingleMillimeters() - 62;
  Serial.println("tof val");
  Serial.println(adjust_input);
  adjustController.compute();
  Serial.println("pid output: ");
  Serial.print(adjust_output);
  Serial.print(" ");
  Serial.print(adjust_input);

  left_motor_power = 65;
  right_motor_power = 65;

  
 //   Serial.println("Pitch: ");
    //Serial.println(imu.getRoll());
  float PID_output = adjust_output / 255.0;
  left_motor_power += 20*PID_output;  
  right_motor_power -= 20*PID_output;
  motorStart(left_motor_power, right_motor_power);
  // normalize PID output
  float mapped_controller_output = normalizedControllerOutput();
  
}
