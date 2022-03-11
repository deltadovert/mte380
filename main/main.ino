#include "ICM_20948.h" // this library makes working with IMU a whole lot easier

ICM_20948_I2C IMU; // create IMU object

#define AD0_VAL 1 

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
  
}

float yawAngle = 0;
int timestep = 10; // timestep

void loop() {
  if (IMU.dataReady()) {
    IMU.getAGMT(); // update IMU readings 
    float yawRateCurr = IMU.gyrZ();
    yawAngle += yawRateCurr * timestep / 1000;
    if (yawAngle > 360)
      yawAngle -= 360;
    if (yawAngle < -360)
      yawAngle += 360;
    Serial.print("Current angle: ");
    Serial.println(yawAngle);
  }
  delay(timestep); 
}
