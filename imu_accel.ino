// i2c_motion.py : https://github.com/EIGSEP/casperfpga/blob/py38/src/i2c_motion.py#L96
// blocks.py     : https://github.com/EIGSEP/eigsep_corr/blob/main/eigsep_corr/blocks.py#L1527
// i2c communication info: https://gitlab.developers.cam.ac.uk/phy/ra/shc44/hera-rx/-/tree/main/i2c-comms

  /*
  Readout interpretation
  ----------------------
  • ±Z value indicates that the FEM is oriented upwards or downwards, respectively.
  • theta = 0˚  FEM is oriented horizontally, top side up.
  • theta = 90˚ FEM is oriented vertically, cw or ccw around x-axis.
  • theta = 180˚ FEM is horizontal, upside down.
  • phi   = 0˚ FEM is level along x-axis (no tilt).
  • phi   = 0˚ -> 90˚ FEM is tilted backwards (top side tilting up, bottom side tilting down).
  • phi   = 0˚ -> -90˚ FEM tilted forward (top down, bottom up).
  • phi   = ±90˚ FEM vertical, x-axis up or down.
  • phi   = ±180˚ FEM level but inverted. 
  */

#include <Wire.h>
#include <math.h>

const byte imu_addr = 0x69;
bool stop = false; 

void initIMU() {
  
  Wire.beginTransmission(imu_addr);
  Wire.write(0x6b); // Power management register | line 280 in i2c_motion.py
  Wire.write(0x00); // Set to zero (wakes up the MPU-9250).
  Wire.endTransmission();

  // Set accelerometer configuration.
  Wire.beginTransmission(imu_addr);
  Wire.write(0x1c); // Accelerometer configuration register | line 128 i2c_motion.py
  Wire.write(0x00); // Set accelerometer range to ±2g | line 105 i2c_motion.py
  Wire.endTransmission();

  // Set gyro configuration.
  Wire.beginTransmission(imu_addr);
  Wire.write(0x1b); // Gyro configuration register | line 122 in i2c_motion.py
  Wire.write(0x00); // Set gyro range to ±250 degrees/sec
  Wire.endTransmission();
}

void readAccelerometer(int &x, int &y, int &z) {
  // Step 1: Write the register address where accelerometer data starts.
  Wire.beginTransmission(imu_addr);
  Wire.write(0x3b); // Starting register for accelerometer data | line 220 in i2c_motion.py
  Wire.endTransmission(false); // Send a restart signal.

  // Reads the data from registers.
  Wire.requestFrom((uint8_t)imu_addr, (uint8_t)6); // Request 6 bytes of data.

  if (Wire.available() == 6) {
    x = Wire.read() << 8 | Wire.read();
    y = Wire.read() << 8 | Wire.read();
    z = Wire.read() << 8 | Wire.read();
  }
}

// Function to calculate theta and phi of FEM in soherical coordinate system in degrees.
void calculatePose(int x, int y, int z, float &theta, float &phi) {
  float ax = x / 16384.0; // Assuming the accelerometer is set to ±2g, this gives us a range between ±19.6 m/s^2
                          // where ±1g translates to 16384 LSB/g (least significant bits per g). 
  float ay = y / 16384.0; // Adjust the denominator if the range is different.
  float az = z / 16384.0; // 16384 = 2^14, because the accelerometer data is 16-bit.
  
  theta = atan2(ay, az) * 180.0 / PI; 
  phi   = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI; 
}

void setup() {
  Serial.begin(9600); 

  // Initializing i2c communications as Master.
  Wire.begin();

  // Initializing IMU on the FEM.
  initIMU();

  Serial.println("Type 's' to stop the loop.");
}

void loop() {
  // Check for serial input.
  if (Serial.available() > 0) {
    char input = Serial.read();
    if (input == 's') {
      stop = true;
    }
  }

  if (stop) {
    Serial.println("Stopped reading position.");
    while (true) {
      // Infinite loop to halt further operations.
    }
  }

  int x, y, z;
  readAccelerometer(x, y, z);

  float theta, phi;
  calculatePose(x, y, z, theta, phi);

  Serial.print("X: "); Serial.print(x);
  Serial.print(" Y: "); Serial.print(y);
  Serial.print(" Z: "); Serial.print(z);
  Serial.print(" Theta: "); Serial.print(theta); // degrees
  Serial.print(" Phi: "); Serial.println(phi);   // degrees

  delay(2000); // 2 second delay, i.e. 2000 ms. 

}

