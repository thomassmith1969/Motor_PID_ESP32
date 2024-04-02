#include "Motor_PID.h"


static double positions[9];
static bool a_vals[9];
static bool b_vals[9];
static int _currentMotorIndex=-1;

static void changeA0(){
  a_vals[0]=!a_vals[0];
  positions[0]+=b_vals[0]?1:-1;
}
static void changeB0(){
  b_vals[0]=!b_vals[0];
  positions[0]+=a_vals[0]?-1:1;
}
static void changeA1(){
  a_vals[1]=!a_vals[1];
  positions[1]+=b_vals[1]?1:-1;
}
static void changeB1(){
  b_vals[1]=!b_vals[1];
  positions[1]+=a_vals[1]?-1:1;
}
static void changeA2(){
  a_vals[2]=!a_vals[2];
  positions[2]+=b_vals[2]?1:-1;
}
static void changeB2(){
  b_vals[2]=!b_vals[2];
  positions[2]+=a_vals[2]?-1:1;
}
static void changeA3(){
  a_vals[3]=!a_vals[3];
  positions[3]+=b_vals[3]?1:-1;
}
static void changeB3(){
  b_vals[3]=!b_vals[3];
  positions[3]+=a_vals[3]?-1:1;
}
static void changeA4(){
  a_vals[4]=!a_vals[4];
  positions[4]+=b_vals[4]?1:-1;
}
static void changeB4(){
  b_vals[4]=!b_vals[4];
  positions[4]+=a_vals[4]?-1:1;
}
static void changeA5(){
  a_vals[5]=!a_vals[5];
  positions[5]+=b_vals[5]?1:-1;
}
static void changeB5(){
  b_vals[5]=!b_vals[5];
  positions[5]+=a_vals[5]?-1:1;
}
static void changeA6(){
  a_vals[6]=!a_vals[6];
  positions[6]+=b_vals[6]?1:-1;
}
static void changeB6(){
  b_vals[6]=!b_vals[6];
  positions[6]+=a_vals[6]?-1:1;
}
static void changeA7(){
  a_vals[7]=!a_vals[7];
  positions[7]+=b_vals[7]?1:-1;
}
static void changeB7(){
  b_vals[7]=!b_vals[7];
  positions[7]+=a_vals[7]?-1:1;
}
static void changeA8(){
  a_vals[8]=!a_vals[8];
  positions[8]+=b_vals[8]?1:-1;
}
static void changeB8(){
  b_vals[8]=!b_vals[8];
  positions[8]+=a_vals[8]?-1:1;
}


Motor::Motor(uint8_t enca, uint8_t encb, uint8_t in1, uint8_t in2, uint8_t pwmpin, int lower_limit, int upper_limit)
{
  this->_instanceIndex=_currentMotorIndex++;
  if(this->_instanceIndex>8)throw "Can only have up to 9 instances of motor created";
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
    attachInterrupt(digitalPinToInterrupt(this->enca),changeA0,CHANGE);
    attachInterrupt(digitalPinToInterrupt(this->encb),changeB0,CHANGE);
    break;
  case 1:
    attachInterrupt(digitalPinToInterrupt(this->enca),changeA1,CHANGE);
    attachInterrupt(digitalPinToInterrupt(this->encb),changeB1,CHANGE);
    break;
  case 2:
    attachInterrupt(digitalPinToInterrupt(this->enca),changeA2,CHANGE);
    attachInterrupt(digitalPinToInterrupt(this->encb),changeB2,CHANGE);
    break;
  case 3:
    attachInterrupt(digitalPinToInterrupt(this->enca),changeA3,CHANGE);
    attachInterrupt(digitalPinToInterrupt(this->encb),changeB3,CHANGE);
    break;
  case 4:
    attachInterrupt(digitalPinToInterrupt(this->enca),changeA4,CHANGE);
    attachInterrupt(digitalPinToInterrupt(this->encb),changeB4,CHANGE);
    break;
  case 5:
    attachInterrupt(digitalPinToInterrupt(this->enca),changeA5,CHANGE);
    attachInterrupt(digitalPinToInterrupt(this->encb),changeB5,CHANGE);
    break;
  case 6:
    attachInterrupt(digitalPinToInterrupt(this->enca),changeA6,CHANGE);
    attachInterrupt(digitalPinToInterrupt(this->encb),changeB6,CHANGE);
    break;
  case 7:
    attachInterrupt(digitalPinToInterrupt(this->enca),changeA7,CHANGE);
    attachInterrupt(digitalPinToInterrupt(this->encb),changeB7,CHANGE);
    break;
  case 8:
    attachInterrupt(digitalPinToInterrupt(this->enca),changeA8,CHANGE);
    attachInterrupt(digitalPinToInterrupt(this->encb),changeB8,CHANGE);
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
