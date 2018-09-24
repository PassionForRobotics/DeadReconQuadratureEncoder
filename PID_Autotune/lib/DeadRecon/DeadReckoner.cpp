
#include "DeadReckoner.h"
#include <Arduino.h>

#define UNSIGNED_LONG_MAX 4294967295

DeadReckoner::DeadReckoner(volatile long long int *_left, volatile long long int *_right, double _tpr, double _r, double _l)
{
	this->left_ticks_ = _left;
	this->right_ticks_ = _right;
	this->ticks_per_rev_ = _tpr;
	this->radius_ = _r;
	this->length_ = _l;
}

long double DeadReckoner::getX()
{
	return this->xc_;
}

long double DeadReckoner::getY()
{
	return this->yc_;
}

long double * DeadReckoner::getWl()
{
	return &this->wl_;
}

long double * DeadReckoner::getWr()
{
	return &this->wr_;
}

long double DeadReckoner::getTheta()
{
	return this->theta_;
}

void DeadReckoner::computeAngularVelocities()
{
	// Time elapsed after computing the angular velocity previously.
	unsigned long dt_omega = micros() - this->prev_wheel_compute_time_; // in microseconds
	if (dt_omega < 0) {
		// micros() has overflowed and reset to 0
		dt_omega = UNSIGNED_LONG_MAX - this->prev_wheel_compute_time_ + micros();
	}

	long double c = 2 * PI / (this->ticks_per_rev_ * dt_omega / 1000000.0); // ticks to rad/s conversion factor
	this->wl_ = ((*this->left_ticks_) - this->left_ticks_prev_) * c;
	this->wr_ = ((*this->right_ticks_) - this->right_ticks_prev_) * c;

	this->left_ticks_prev_ = (*this->left_ticks_);
	this->right_ticks_prev_ = (*this->right_ticks_);

	this->prev_wheel_compute_time_ = micros();
}

void DeadReckoner::computePosition()
{
	computeAngularVelocities();
	// Time elapsed after the previous position has been integrated.
	unsigned long dt_integration = micros() - this->prev_integration_time_;
	if (dt_integration < 0) {
		// micros() has overflowed and has reset to 0
		dt_integration = UNSIGNED_LONG_MAX - this->prev_integration_time_ + micros();
	}

	long double dt = dt_integration / 1000000.0; // convert to seconds

	// Dead reckoning equations

	long double Vl = this->wl_ * this->radius_;
	long double  Vr = this->wr_ * this->radius_;
	long double  v = (Vr + Vl) / 2.0;
	long double  w = (Vr - Vl) / this->length_;
	// Uses 4th order Runge-Kutta to integrate numerically to find position.
	long double  xNext = this->xc_ + dt * v*(2 + cos(dt*w / 2))*cos(this->theta_ + dt * w / 2) / 3;
	long double  yNext = this->yc_ + dt * v*(2 + cos(dt*w / 2))*sin(this->theta_ + dt * w / 2) / 3;
	long double  thetaNext = this->theta_ + dt * w;

	this->xc_ = xNext;
	this->yc_ = yNext;
	this->theta_ = thetaNext;

	this->prev_integration_time_ = micros();
}

void DeadReckoner::reset(double _x, double _y, double _th)
{
	this->xc_ = _x;
	this->yc_ = _y;
	this->theta_ = _th;
	this->wl_ = 0;
	this->wr_ = 0;
	this->left_ticks_prev_ = 0;
	this->right_ticks_prev_ = 0;
	this->prev_wheel_compute_time_ = micros();
}
