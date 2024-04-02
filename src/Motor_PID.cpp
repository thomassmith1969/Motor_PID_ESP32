#include "Motor_PID.h"



Motor::Motor(uint8_t enca, uint8_t encb, uint8_t in1, uint8_t in2, uint8_t pwmpin, int lower_limit, int upper_limit)
{
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
  this->encoder.attachHalfQuad(enca,encb);
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

  long e = encoder.getCount() - target;

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
  encoder.setCount((int64_t)round(position));
}

long Motor::get_position()
{
  return encoder.getCount();
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
