// pins for base wheel motor
const uint8_t base_motor_ena = 3;
const uint8_t base_motor_in1 = 4;
const uint8_t base_motor_in2 = 5;

// pins for reaction wheel motor
const uint8_t reaction_motor_ena = 6;
const uint8_t reaction_motor_in1 = 7;
const uint8_t reaction_motor_in2 = 8;

// motor rotation speed
const uint8_t motor_speed = 127;

/* ============================== setup ============================== */
void setup()
{
  //for base wheel
  pinMode(base_motor_ena, OUTPUT);
  pinMode(base_motor_in1, OUTPUT);
  pinMode(base_motor_in2, OUTPUT);

  //for reaction wheel
  pinMode(reaction_motor_ena, OUTPUT);
  pinMode(reaction_motor_in1, OUTPUT);
  pinMode(reaction_motor_in2, OUTPUT);
}

/* ============================== loop ============================== */
void loop()
{
  // base motor clockwise
  motor_rotate(base_motor_ena, base_motor_in1, base_motor_in2, true);
  // base motor anticlockwise
  motor_rotate(reaction_motor_ena, reaction_motor_in1, reaction_motor_in2, true);
  delay(1000);
  
  // reaction motor clockwise
  
  // reaction motor anticlockwise
  motor_rotate(base_motor_ena, base_motor_in1, base_motor_in2, false);
  motor_rotate(reaction_motor_ena, reaction_motor_in1, reaction_motor_in2, false);
  delay(1000);
}



/* ----- motor rotation ----- */

// dir = true -> clockwise
// else -> anticlockwise
void motor_rotate(const uint8_t motor_ena, const uint8_t motor_in1, const uint8_t motor_in2, bool dir)
{
  if(dir) {
    digitalWrite(motor_in1, HIGH);
    digitalWrite(motor_in2, LOW);
    analogWrite(motor_ena, motor_speed);
    return;
  }
  digitalWrite(motor_in1, LOW);
  digitalWrite(motor_in2, HIGH);
  analogWrite(motor_ena, motor_speed);
}
