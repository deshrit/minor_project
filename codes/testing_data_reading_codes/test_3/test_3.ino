#include <Wire.h>

/* -------------------- imu varibales -------------------- */

// imu sensor device address
const uint8_t mpu_addr = 0x68;

// Accelerometer data
float acc_x = 0, acc_y = 0;


/* -------------------- motor dirver variables -------------------- */

// pins for base wheel motor
const uint8_t base_motor_ena = 3;
const uint8_t base_motor_in1 = 4;
const uint8_t base_motor_in2 = 5;

/* -------------------- motor class -------------------- */
struct Motor
{
  // variables;
  uint8_t motor_enable, motor_in1, motor_in2, motor_speed, motor_dir;

  // constructor
  Motor(const uint8_t enable, const uint8_t in1, const uint8_t in2, uint8_t sped = 150, bool dir = false)
  {
    this->motor_enable = enable;
    this->motor_in1 = in1;
    this->motor_in2 = in2;
    this->motor_speed = sped;
    this->motor_dir = dir;
  }
};

// BASE MOTOR OBJECT
Motor base_motor(base_motor_ena, base_motor_in1, base_motor_in2);


/* -------------------- pid calculation variables -------------------- */

// ----- gain constants to be adjusted
int kp_acc_x = 100, ki_acc_x = 0, kd_acc_x = 0;
// ----- gain constants to be adjusted

// set points, errors and pids
float set_point_acc_x = 0.0;
float error_acc_x = 0.0, previous_error_acc_x = 0.0;
float pid_p_acc_x = 0.0, pid_i_acc_x = 0.0, pid_d_acc_x = 0.0, pid_acc_x = 0.0;

// time data
unsigned long current_time, previous_time;
double elapsed_time;


/* ======================================== setup ======================================== */
void setup()
{
  /* ---------- imu setup ---------- */
  
  Serial.begin(9600);
  // initiate wire library
  Wire.begin();
  // start communication with sensor
  // power up register
  Wire.beginTransmission(mpu_addr);
      Wire.write(0x6B);
      Wire.write(0x00);
   Wire.endTransmission(true);


  /* ---------- motor driver setup ---------- */
   
  //for base wheel
  pinMode(base_motor.motor_enable, OUTPUT);
  pinMode(base_motor.motor_in1, OUTPUT);
  pinMode(base_motor.motor_in2, OUTPUT);

  // time in milli second
  current_time = millis();
}




/* ======================================== loop ======================================== */
void loop()
{
  
  /* ---------- read accelerometer and gyro data ----------*/
//  read_imu_data(mpu_addr);
  
//  Serial.print("\n\nacc_x: ");
//  Serial.print(acc_x);

  
  // time calculation
//  previous_time = current_time;
//  current_time = millis();
//  elapsed_time = (current_time - previous_time);
//
//  Serial.print("\telapsed_time: ");
//  Serial.print(elapsed_time);

  base_motor.motor_dir = false;
  rotate_motor(base_motor);
  delay(1000);

  stop_motor(base_motor);
  delay(1000);
  
  base_motor.motor_dir = true;
  rotate_motor(base_motor);
  delay(1000);

  stop_motor(base_motor);
  delay(1000);
}



/* ---------- read data from the mpu-6050 module ---------- */

// passing i2c device address as function parameter
void read_imu_data(const uint8_t addr)
{
   // Start with register 0x3B
  Wire.beginTransmission(mpu_addr);
      Wire.write(0x3B);
  Wire.endTransmission(false);
  // Read 4 registers total
  Wire.requestFrom(mpu_addr, 4, true);
  acc_x = (Wire.read() << 8 | Wire.read()) / 16384.0;
  acc_y = (Wire.read() << 8 | Wire.read()) / 16384.0;

  // filtering the pitch data i.e acc_x
  if (acc_x >= 1.0) acc_x = 1.0;
  if (acc_x <= -1.0) acc_x = -1.0;
  acc_x = (float)round(acc_x*10.0)/10.0;


  // filtering the roll data i.e acc_y
  if (acc_y >= 1.0) acc_y = 1.0;
  if (acc_y <= -1.0) acc_y = -1.0;
  acc_y = (float)round(acc_y*10.0)/10.0;
}


/* ---------- motor rotating function ---------- */

// motor object and direction - dir as parameter
void rotate_motor(Motor motor) // true => clockwise, false => anticlockwise
{ 
  if(motor.motor_dir) {
    digitalWrite(motor.motor_in1, HIGH);
    digitalWrite(motor.motor_in2, LOW);
    analogWrite(motor.motor_enable, motor.motor_speed);
    return;
  }

  digitalWrite(motor.motor_in1, LOW);
  digitalWrite(motor.motor_in2, HIGH);
  analogWrite(motor.motor_enable, motor.motor_speed);
}

// motor object
void stop_motor(Motor motor)
{
  digitalWrite(motor.motor_in1, LOW);
  digitalWrite(motor.motor_in2, LOW);
  analogWrite(motor.motor_enable, 0);
}


/* ---------- calculate pid for acc_x ---------- */

void calculate_pid_acc_x()
{
  // pid calculation
  error_acc_x = set_point_acc_x - acc_x;
  pid_p_acc_x = kp_acc_x * error_acc_x;
  pid_i_acc_x = pid_i_acc_x + ki_acc_x * error_acc_x;
  pid_d_acc_x = kd_acc_x * ((error_acc_x - previous_error_acc_x) / elapsed_time);
  pid_acc_x = pid_p_acc_x + pid_i_acc_x + pid_d_acc_x;

  Serial.print("\n\npid_p: ");
  Serial.print(pid_p_acc_x);

  Serial.print("\tpid_i: ");
  Serial.print(pid_i_acc_x);

  Serial.print("\tpid_d: ");
  Serial.print(pid_d_acc_x);
  
  Serial.print("\tPID_X: ");
  Serial.print(pid_acc_x);
  
  previous_error_acc_x = error_acc_x;
  
  // give the pid value to system accordingly
  if(error_acc_x > 0) base_motor.motor_dir = true;
  if(error_acc_x < 0) base_motor.motor_dir = false;
  base_motor.motor_speed = abs((int)pid_acc_x); // speed is the pid value

}
