#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "ArduPID.h"

#include "Robojax_L298N_DC_motor.h"
#include "Wire.h"
#include "SR04.h"

#define TRIG_FRONT 28
#define ECHO_FRONT 26
#define TRIG_LEFT 32
#define ECHO_LEFT 30

SR04 UltrasonicLeft = SR04(ECHO_LEFT, TRIG_LEFT); 
SR04 UltrasonicFront = SR04(ECHO_FRONT, TRIG_FRONT);

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

const int FWD = 2;
const int BWD = 1; 

Robojax_L298N_DC_motor motorLeft(DR1_IN1, DR1_IN2, DR1_ENA, DR1_IN3, DR1_IN4, DR1_ENB, false); // the boolean argument enables debugging when true
Robojax_L298N_DC_motor motorRight(DR2_IN1, DR2_IN2, DR2_ENA, DR2_IN3, DR2_IN4, DR2_ENB, false);

int left_motor_power = 50;
int right_motor_power = 50;

// PID Controllers
ArduPID adjustController;

double adjust_setpoint = 30;
double adjust_input;
double adjust_output;
double adjust_p = 100;
double adjust_i = 0;
double adjust_d = 0;


void setup() {
  adjustController.begin(&adjust_input, &adjust_output, &adjust_setpoint, adjust_p, adjust_i, adjust_d);
  adjustController.setOutputLimits(-50, 50);
  adjustController.setWindUpLimits(-10, 10);
  adjustController.start();

  delay(500);

  Serial.begin (115200);

  motorLeft.begin();
  motorRight.begin();
}

void motorStart(int speedLeft, int speedRight) {
  motorLeft.rotate(motor1, speedLeft, FWD);
  motorLeft.rotate(motor2, speedLeft, FWD);
  motorRight.rotate(motor1, speedRight, FWD);
  motorRight.rotate(motor2, speedRight, FWD);
}
  
void loop() {
  adjust_input = UltrasonicLeft.Distance();
  Serial.print("Input: ");
  Serial.println(adjust_input);
  adjustController.compute();
  Serial.print("pid output: ");
  Serial.println(adjust_output);

  left_motor_power = 50;
  right_motor_power = 50;

  
 //   Serial.println("Pitch: ");
    //Serial.println(imu.getRoll());
  float PID_output = adjust_output;
  left_motor_power += PID_output;  
  right_motor_power -= PID_output;
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
  // normalize PID output
  
}
