#include <Wire.h>

// imu sensor device address
const int mpu_addr = 0x68;

// Accelerometer data
float acc_x, acc_y, acc_z;

/* ============================== setup ============================== */
void setup()
{
  // initializing serial and wire library
  Serial.begin(9600);
  Wire.begin();

  // start communication with sensor
  Wire.beginTransmission(mpu_addr);
    Wire.write(0x6B);
    Wire.write(0x00);
   Wire.endTransmission(true);
}

/* ============================== loop ============================== */
void loop()
{
  
  /* ----- read accelerometer data ----- */

  // Start with register 0x3B
  Wire.beginTransmission(mpu_addr);
    Wire.write(0x3B);
  Wire.endTransmission(false);
  // Read 6 registers total
  Wire.requestFrom(mpu_addr, 6, true);
  acc_x = (Wire.read() << 8 | Wire.read()) / 16384.0;
  acc_y = (Wire.read() << 8 | Wire.read()) / 16384.0;
  acc_z = (Wire.read() << 8 | Wire.read()) / 16384.0;

  // printing to serial monitor
  Serial.print("\nAcceleration x: ");
  Serial.print(acc_x);
  Serial.print("\tAcceleration y: ");
  Serial.print(acc_y);
  Serial.print("\tAcceleration z: ");
  Serial.print(acc_z);
}
