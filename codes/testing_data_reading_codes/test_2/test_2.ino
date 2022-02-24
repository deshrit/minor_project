// pins for base wheel motor
const uint8_t base_motor_ena = 3;
const uint8_t base_motor_in1 = 4;
const uint8_t base_motor_in2 = 5;


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


void setup()
{
  pinMode(base_motor_in1, OUTPUT);
  pinMode(base_motor_in2, OUTPUT);
  pinMode(base_motor_ena, OUTPUT);
}

void loop()
{
  base_motor.motor_dir = false;
  rotate_motor(base_motor);
  delay(2000);
  
  stop_motor(base_motor);
  delay(2000);
  
  base_motor.motor_dir = true;
  rotate_motor(base_motor);
  delay(2000);
}

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
