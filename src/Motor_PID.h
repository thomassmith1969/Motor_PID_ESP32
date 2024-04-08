#pragma once
// author:: Natan :Lisowski
//  this library let you control motor with pwm signal
// this works with HG7881 Hbridge  L9110S
// link to repository
//
//  library inspired by https://github.com/curiores/ArduinoTutorials
//   encoder motor control
#include <Arduino.h>
#include <math.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>


class Motor
{
public:
	double kp = 0.0f, kd = 0.0f, ki = 0.0f;  // your pid variables
	bool motor_state{ false };                 // this variable let the pwm signal go
	Motor(uint8_t enca, uint8_t encb, uint8_t in1, uint8_t in2, uint8_t pwmpin = 0, int lower_limit = 50, int upper_limit = 255); // constructor
	~Motor() {};
	void start();           // place this in loop without delays
	void init(double kp, double ki, double kd); // this function initializes pid regulator
	void turn_on();         // changes motor_state variable to true
	void turn_off();        // changes motor_state variable to false
	void set_position(double posi = 0);
	long get_position();
	void set_target(double target);
	long get_target();
	void limit(int lower_limit, int upper_limit);
	bool target_reached(bool reset = false); // check if target position is reached by motor

private:
	uint8_t enca, encb, in2, in1;
	int _instanceIndex;
	ESP32PWM in1pwm;
	ESP32PWM in2pwm;
	ESP32PWM pwmpwm;
	ESP32Encoder encoder;
	void set_motor(int dir, int pwmVal);
	void attachEncoders();
	bool buffer{ false };
	uint8_t pwmpin = 0;
	int upper_limit = 0, lower_limit = 0;
	long prev_t{ 0 };
	long target{ 0 };
	double eprev = 0, eintegral = 0;
	bool target_is_reached = false;
};
