#include "PID.h"





PID::PID(float kp,float ki,float kd,float filter_bandwith,float dt)
{
	this->error = 0;
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
	this->dt = dt;
	this->filter_bandwith = filter_bandwith;
	this->integral_max = 2.0;
	this->integral = 0;
	this->derivative = 0;
	this->filter_past = 0;
	this->filter = 0;
}

float PID::updatePID(float error)
{
	this->error = error;

	float p_value = this->kp * this->error; //p term

	this->integral += this->error * this->dt;
	this->integral = constrain(this->integral,-this->integral_max,this->integral_max);

	float i_value = this->integral * this->ki; //i term

	//low pass filter
	this->filter = this->filter_past + this->dt * (this->filter_bandwith * (this->error - this->filter_past));
	this->filter_past = this->filter;
	float d_value = this->kd * ((this->filter - this->derivative) / this->dt); //d term
	this->derivative = this->filter;
	
	
	float output = p_value + d_value + i_value;

	return output;
}

void PID::set_ki(float ki)
{
	this->ki = ki;
}

void PID::set_kd(float kd)
{
	this->kd = kd;
}

void PID::set_kp(float kp)
{
	this->kp = kp;
}

void PID::set_dt(float dt)
{
	this->dt = dt;
}

float PID::get_dt()
{
	return this->dt;
}

void PID::reset_pid()
{
	this->integral = 0;	
}

