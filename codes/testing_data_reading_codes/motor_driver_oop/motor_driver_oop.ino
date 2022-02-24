// pins for base wheel motor
const uint8_t base_motor_ena = 3;
const uint8_t base_motor_in1 = 4;
const uint8_t base_motor_in2 = 5;
// base motor spped
const uint8_t base_motor_speed = 127;



// pins for reaction wheel motor
const uint8_t reaction_motor_ena = 6;
const uint8_t reaction_motor_in1 = 7;
const uint8_t reaction_motor_in2 = 8;
// reaction motor speed
const uint8_t reaction_motor_speed = 127;

/* ---------- Motor class ---------- */
struct Motor
{
  // variables;
  uint8_t motor_enable, motor_in1, motor_in2, motor_speed;

  // constructor
  Motor(const uint8_t enable, const uint8_t in1, const uint8_t in2, const uint8_t sped)
  {
    this->motor_enable = enable;
    this->motor_in1 = in1;
    this->motor_in2 = in2;
    this->motor_speed = sped;
  }
};

// BASE MOTOR OBJECT
struct Motor base_motor(base_motor_ena, base_motor_in1, base_motor_in2, base_motor_speed);

// REACTION MOTOR OBJECT
struct Motor reaction_motor(reaction_motor_ena, reaction_motor_in1, reaction_motor_in2, reaction_motor_speed);

/* ============================== setup ============================== */
void setup()
{
  //for base wheel
  pinMode(base_motor.motor_enable, OUTPUT);
  pinMode(base_motor.motor_in1, OUTPUT);
  pinMode(base_motor.motor_in2, OUTPUT);

  //for reaction wheel
  pinMode(reaction_motor.motor_enable, OUTPUT);
  pinMode(reaction_motor.motor_in1, OUTPUT);
  pinMode(reaction_motor.motor_in2, OUTPUT);
}

/* ============================== loop ============================== */
void loop()
{
  // base motor clockwise
  rotate_motor(base_motor,true);
  rotate_motor(reaction_motor,true);
  delay(1000);
  // base motor anticlockwise

  // reaction motor clockwise
  rotate_motor(base_motor,false);
  rotate_motor(reaction_motor,false);
  delay(1000);
}



/* ----- motor rotation ----- */

void rotate_motor(struct Motor motor, bool dir)
{
  if(dir) {
    digitalWrite(motor.motor_in1, HIGH);
    digitalWrite(motor.motor_in2, LOW);
    analogWrite(motor.motor_enable, motor.motor_speed);
    return;
  }

  digitalWrite(motor.motor_in1, LOW);
  digitalWrite(motor.motor_in2, HIGH);
  analogWrite(motor.motor_enable, motor.motor_speed);
}
