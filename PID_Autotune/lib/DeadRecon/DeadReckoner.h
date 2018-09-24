
#ifndef DEAD_RECONER_H_
#define DEAD_RECONER_H_

// By https://github.com/jaean123/DeadReckoning-library

#include <Arduino.h>

class DeadReckoner
{

public:
	DeadReckoner(volatile long long int *, volatile long long int *, double, double, double);
	void computePosition();
	long double getX();
	long double getY();
	long double *getWl();
	long double *getWr();
	long double getTheta();
	void reset(double x, double y, double th);

private:
	void computeAngularVelocities();
	volatile long long int *left_ticks_, *right_ticks_; // Number of total wheel encoder tick counts for left and right wheels.
	int left_ticks_prev_, right_ticks_prev_; // Number of total wheel encoder tick counts at time computeAngularVelocities() is called.
	long double xc_, yc_; // Latest position coordinates in ticks.
	long double wl_, wr_; // Latest left and right angular velocity of the wheels in radians per second.
	long double ticks_per_rev_; // Number of tick registers per second of the encoder.
	long double length_; // Length from left wheel to right wheel.
	long double radius_; // Radius of the wheel.
	long double theta_;
	unsigned long prev_integration_time_;
	unsigned long prev_wheel_compute_time_;

};

#endif
