#include <Wire.h>

/* -------------------- imu varibales -------------------- */

// imu sensor device address
const uint8_t mpu_addr = 0x68;

// Accelerometer data
float acc_x, acc_y;


/* -------------------- motor dirver variables -------------------- */

// pins for reaction wheel motor
const uint8_t reaction_motor_ena = 3;
const uint8_t reaction_motor_in1 = 4;
const uint8_t reaction_motor_in2 = 5;


// pins for base wheel motor
const uint8_t base_motor_ena = 6;
const uint8_t base_motor_in1 = 7;
const uint8_t base_motor_in2 = 8;



/* ---------- Motor class ---------- */
struct Motor
{
  // variables;
  uint8_t motor_enable, motor_in1, motor_in2, motor_speed;
  bool motor_dir;

  // constructor
  Motor(const uint8_t enable, const uint8_t in1, const uint8_t in2, uint8_t sped = 0, bool dir = false)
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

// REACTION MOTOR OBJECT
Motor reaction_motor(reaction_motor_ena, reaction_motor_in1, reaction_motor_in2);


/* -------------------- pid calculation variables -------------------- */

// ----- gain constants to be adjusted
int kp_acc_x = 300, ki_acc_x = 0, kd_acc_x = 0;
int kp_acc_y = 500, ki_acc_y = 0, kd_acc_y = 0;
// ----- gain constants to be adjusted

// set points, errors and pids
float set_point_acc_x = 0.0, set_point_acc_y = 0.0;
float error_acc_x = 0.0, previous_error_acc_x = 0.0;
float error_acc_y = 0.0, previous_error_acc_y = 0.0;
float pid_p_acc_x = 0.0, pid_i_acc_x = 0.0, pid_d_acc_x = 0.0, pid_acc_x = 0.0; 
float pid_p_acc_y = 0.0, pid_i_acc_y = 0.0, pid_d_acc_y = 0.0, pid_acc_y = 0.0; 

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

  //for reaction wheel
  pinMode(reaction_motor.motor_enable, OUTPUT);
  pinMode(reaction_motor.motor_in1, OUTPUT);
  pinMode(reaction_motor.motor_in2, OUTPUT);
     
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
  read_imu_data(mpu_addr);

  Serial.print("\n\nacc_x: "); // acc_y is the roll axis(reaction wheel) accelerometer data
  Serial.print(acc_x);
  Serial.print("\n\nacc_y: "); // acc_y is the pitch axis(base wheel) accelerometer data
  Serial.print(acc_y);

  // elapsed time calculation
  previous_time = current_time;
  current_time = millis();
  elapsed_time = (current_time - previous_time); // time in milisec

  //  /* ---------- calculate pid pitch axis ----------*/
  calculate_pid_acc_y();
  rotate_motor(base_motor);

  /* ---------- calculate pid roll axis ----------*/
  calculate_pid_acc_x();
  rotate_motor(reaction_motor);
}




/* ---------- read data from the mpu-6050 module ---------- */

// passing i2c device address as function parameter
void read_imu_data(const uint8_t addr)
{
   // Start with register 0x3B
  Wire.beginTransmission(addr);
      Wire.write(0x3B);
  Wire.endTransmission(false);
  // Read 4 registers total
  Wire.requestFrom(addr, 4, true);
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
void rotate_motor(Motor motor) // false => clockwise, true=> anticlockwise
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


/* ---------- calculate pid for acc_x(pitch axis - reaction wheel) ---------- */

void calculate_pid_acc_x()
{
  // pid calculation
  error_acc_x = set_point_acc_x - acc_x;
  pid_p_acc_x = kp_acc_x * error_acc_x;
  pid_i_acc_x = pid_i_acc_x + ki_acc_x * error_acc_x;
  pid_d_acc_x = kd_acc_x * (error_acc_x - previous_error_acc_x);
  pid_acc_x = pid_p_acc_x + pid_i_acc_x + pid_d_acc_x;

//  pid_acc_x = pid_p_acc_x;

  previous_error_acc_x = error_acc_x;
  
  Serial.print("\tpid_x: ");
  Serial.print(pid_acc_x);
  
  // give the pid value to system accordingly
  if(error_acc_x > 0) base_motor.motor_dir = true;
  if(error_acc_x < 0) base_motor.motor_dir = false;
  base_motor.motor_speed = abs((int)pid_acc_x); // speed is the pid value

}



/* ---------- calculate pid for acc_y(roll axis - base wheel) ---------- */

void calculate_pid_acc_y()
{
  // pid calculation
  error_acc_y = set_point_acc_y - acc_y;
  pid_p_acc_y = kp_acc_y * error_acc_y;
  pid_i_acc_y = pid_i_acc_y + ki_acc_y * error_acc_y;
  pid_d_acc_y = kd_acc_y * (error_acc_y - previous_error_acc_y);
  pid_acc_y = pid_p_acc_y + pid_i_acc_y + pid_d_acc_y;
//  pid_acc_y = pid_p_acc_y;

  previous_error_acc_y = error_acc_y;
  
  Serial.print("\tpid_y: ");
  Serial.print(pid_acc_y);
  
  
  // give the pid value to system accordingly
  if(error_acc_y > 0) base_motor.motor_dir = true;
  if(error_acc_y < 0) base_motor.motor_dir = false;
  base_motor.motor_speed = abs((int)pid_acc_y); // speed is the pid value

}
