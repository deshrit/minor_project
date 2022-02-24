#include<Wire.h>

// sensor device address
const uint8_t mpu_addr = 0x68;

// sensor values
float acc_x, acc_y;

/* ---- setup ------- */
void setup()
{
  // baud rate for serial communication
  Serial.begin(9600);
  Wire.begin();
  // wake up mpu from sleep
  // writing 0x00 to 0x6B register
  Wire.beginTransmission(mpu_addr);
    Wire.write(0x6B);
    Wire.write(0x00);
  Wire.endTransmission(true);
}

void loop()
{
  read_imu_data(mpu_addr);

  // printing to serial monitor
  Serial.print("\nAcceleration x: ");
  Serial.print(acc_x);
  Serial.print("\tAcceleration y: ");
  Serial.print(acc_y);

//  delay(500);
}

void read_imu_data(const uint8_t addr)
{
  // read imu data
  // starting from 0x3B register
  Wire.beginTransmission(mpu_addr);
    Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(addr, 4, true);
  acc_x = (Wire.read() << 8 | Wire.read()) / 16384.0;
  acc_y = (Wire.read() << 8 | Wire.read()) / 16384.0;

  if (acc_x >= 1.0) acc_x = 1.0;
  if (acc_x <= -1.0) acc_x = -1.0;
  acc_x = (float)round(acc_x*10.0)/10.0;

  
  if (acc_y >= 1.0) acc_y = 1.0;
  if (acc_y <= -1.0) acc_y = -1.0;
  acc_y = (float)round(acc_y*10.0)/10.0;

}
