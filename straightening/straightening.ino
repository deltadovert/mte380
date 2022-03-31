#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "Robojax_L298N_DC_motor.h" // library for easy interface with motor drivers 

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

#define SERIAL_PORT Serial
#define WIRE_PORT Wire 
#define AD0_VAL 1      

ICM_20948_I2C IMU;

void setup()
{
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT)
  {
  };

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  //motorLeft.begin();
  //motorRight.begin();

  bool initialized = false;
  while (!initialized)
  {
    IMU.begin(WIRE_PORT, 1);

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(IMU.statusString());
    if (IMU.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
}

float yawAngle = 0; // global
float prevTime = millis();

void readData() { 
  // read ToF

  // read gyro
  if (IMU.dataReady()) {
    IMU.getAGMT();
    if (abs(IMU.gyrZ()) > 6) {
      Serial.println(IMU.gyrZ());
      yawAngle += IMU.gyrZ() * (millis() - prevTime) / 1000; // clockwise positive 
    }
  }
}

void startTurn() {
  motorLeft.rotate(motor1, 100, FWD);
  motorLeft.rotate(motor2, 100, FWD);
  motorRight.rotate(motor1, 100, BWD);
  motorRight.rotate(motor2, 100, BWD);
}

void stopMotors() {
  motorLeft.rotate(motor1, 0, FWD);
  motorLeft.rotate(motor2, 0, FWD);
  motorRight.rotate(motor1, 0, FWD);
  motorRight.rotate(motor2, 0, FWD);
}

// turn 

bool turn = true; 
bool turnInitiated = false; 
float initialAngle = 0;

void loop()
{
  readData(); 
  
 
  
}
