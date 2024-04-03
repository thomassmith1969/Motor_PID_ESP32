#include "Motor_PID.h"


double positions[10];
int b_pins[10];
int _currentMotorIndex=-1;

static void step0(){
  if(digitalRead(b_pins[0])==0)positions[0]++;else positions[0]--; 
}
static void step1(){
  if(digitalRead(b_pins[1])==0)positions[1]++;else positions[1]--; 
}
static void step2(){
  if(digitalRead(b_pins[2])==0)positions[2]++;else positions[2]--; 
}
static void step3(){
  if(digitalRead(b_pins[3])==0)positions[3]++;else positions[3]--; 
}
static void step4(){
  if(digitalRead(b_pins[4])==0)positions[4]++;else positions[4]--; 
}
static void step5(){
  if(digitalRead(b_pins[5])==0)positions[5]++;else positions[5]--; 
}
static void step6(){
  if(digitalRead(b_pins[6])==0)positions[6]++;else positions[6]--; 
}
static void step7(){
  if(digitalRead(b_pins[7])==0)positions[7]++;else positions[7]--; 
}
static void step8(){
  if(digitalRead(b_pins[8])==0)positions[8]++;else positions[8]--; 
}
static void step9(){
  if(digitalRead(b_pins[9])==0)positions[9]++;else positions[9]--; 
}

Motor::Motor(uint8_t enca, uint8_t encb, uint8_t in1, uint8_t in2, uint8_t pwmpin, int lower_limit, int upper_limit)
{
  _currentMotorIndex++;
  this->_instanceIndex=_currentMotorIndex;
  if(this->_instanceIndex>9)throw "Can only have up to 10 instances of motor created";
  this->enca = enca;
  this->encb = encb;
  this->attachEncoders();
  this->in1 = in1;
  this->in2 = in2;
  this->pwmpin = pwmpin;
  this->upper_limit = upper_limit;
  this->lower_limit = lower_limit;
}
void Motor::attachEncoders(){
  pinMode(enca, INPUT);
  pinMode(encb, INPUT);

  switch (this->_instanceIndex)
  {
  case 0:
    b_pins[0]=this->encb;
    attachInterrupt(digitalPinToInterrupt(this->enca),step0,RISING);
    break;
  case 1:
    b_pins[1]=this->encb;
    attachInterrupt(digitalPinToInterrupt(this->enca),step1,RISING);
    break;
  case 2:
    b_pins[2]=this->encb;
    attachInterrupt(digitalPinToInterrupt(this->enca),step2,RISING);
    break;
  case 3:
    b_pins[3]=this->encb;
    attachInterrupt(digitalPinToInterrupt(this->enca),step3,RISING);
    break;
  case 4:
    b_pins[4]=this->encb;
    attachInterrupt(digitalPinToInterrupt(this->enca),step4,RISING);
    break;
  case 5:
    b_pins[5]=this->encb;
    attachInterrupt(digitalPinToInterrupt(this->enca),step5,RISING);
    break;
  case 6:
    b_pins[6]=this->encb;
    attachInterrupt(digitalPinToInterrupt(this->enca),step6,RISING);
    break;
  case 7:
    b_pins[7]=this->encb;
    attachInterrupt(digitalPinToInterrupt(this->enca),step7,RISING);
    break;
  case 8:
    b_pins[8]=this->encb;
    attachInterrupt(digitalPinToInterrupt(this->enca),step8,RISING);
    break;
  case 9:
    b_pins[9]=this->encb;
    attachInterrupt(digitalPinToInterrupt(this->enca),step9,RISING);
    break;
  
  default:
    throw "invalid instanceIndex";
  }
}
void Motor::init(double kp, double ki, double kd)
{
  this->kp = kp;
  this->kd = kd;
  this->ki = ki;
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  if (pwmpin != 0){
    pinMode(pwmpin, OUTPUT);
    pwmpwm.attachPin(pwmpin,1000,10);
    }
  else{
    in1pwm.attachPin(in1,1000,10);
    in2pwm.attachPin(in2,1000,10);
  }
  motor_state = 1;
}

void Motor::set_motor(int dir, int pwm_val)
{
  pwm_val = constrain(pwm_val, lower_limit, upper_limit);
  if (pwmpin != 0)
    pwmpwm.write(pwm_val);
  if (dir == 1 && motor_state == 1)
  {
    (pwmpin != 0) ? digitalWrite(in1, 1) : in1pwm.write(pwm_val);
    (pwmpin != 0) ? digitalWrite(in2, 0) : in2pwm.write(0);
  }
  else if (dir == -1 && motor_state == 1)
  {
    (pwmpin != 0) ? digitalWrite(in1, 0) : in1pwm.write(0);
    (pwmpin != 0) ? digitalWrite(in2, 1) : in2pwm.write(pwm_val);
  }
  else
  {
    (pwmpin != 0) ? digitalWrite(in1, 0) : in1pwm.write(0);
    (pwmpin != 0) ? digitalWrite(in2, 0) : in2pwm.write(0);
  }
}

void Motor::start()
{
  long curr_t = micros();
//  rising_interrupt();

  double delta_t = ((double)(curr_t - prev_t)) / (1.0e6);
  prev_t = curr_t;

  long e = positions[this->_instanceIndex] - target;

  double dedt = ((double)e - eprev) / (delta_t);

  eintegral = eintegral + e * delta_t;

  double u = kp * (double)e + kd * dedt + ki * eintegral;

  double pwr = fabs(u);
  if (pwr > 255)
  {
    pwr = 255;
  }

  int dir = 1;
  if (u < 0)
  {
    dir = -1;
  }
  if (e == 0)
  {
    turn_off();
    target_is_reached = true;
  }
  else
  {
    turn_on();
    target_is_reached = false;
  }

  set_motor(dir, pwr);

  eprev = e;
}

void Motor::turn_on()
{
  motor_state = 1;
}

void Motor::turn_off()
{
  motor_state = 0;
  if (pwmpin != 0)
    analogWrite(pwmpin, 0);
  (pwmpin != 0) ? digitalWrite(in1, 0) : analogWrite(in1, 0);
  (pwmpin != 0) ? digitalWrite(in2, 0) : analogWrite(in2, 0);
}

void Motor::set_position(double position)
{
  positions[this->_instanceIndex] = (long)round(position);
}

long Motor::get_position()
{
  return positions[this->_instanceIndex];
}

void Motor::set_target(double target)
{
  this->target = (long)round(target);
}

long Motor::get_target()
{
  return target;
}

void Motor::limit(int lower_limit, int upper_limit)
{
  this->lower_limit = lower_limit;
  this->upper_limit = upper_limit;
}

bool Motor::target_reached(bool reset)
{
  if (reset)
    target_is_reached = false;
  return target_is_reached;
}
