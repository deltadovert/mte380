#include "Robojax_L298N_DC_motor.h" // library for easy interface with motor drivers 

// designates the pins for driver 1 (LHS motor set)
#define DR1_ENA 7
#define DR1_IN1 6 
#define DR1_IN2 5

// which motor is this
#define DR1_IN3 4
#define DR1_IN4 3
#define DR1_ENB 2

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
  motorLeft.rotate(motor1, 100, FWD); 
  motorLeft.rotate(motor2, 100, FWD);
  motorRight.rotate(motor1, 100, BWD); 
  motorRight.rotate(motor2, 100, BWD);
}

void motorStop() {
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
  // put your setup code here, to run once:
  configureMotors();
}

void loop() {
    // put your main code here, to run repeatedly:
//  goStraight();
//  delay(2000);
  turnRight();
  delay(2000);
  motorStop();
  delay(2000);
}
