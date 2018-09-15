
#ifndef _DeadReckoner_h
#define _DeadReckoner_h

#include <Arduino.h>

class DeadReckoner {

public:
	DeadReckoner(volatile long long int *, volatile long long int *, double, double, double);
	void computePosition();
	double getX();
	double getY();
	double getWl();
	double getWr();
	double getTheta();

private:
	void computeAngularVelocities();
	volatile long long int *left_ticks_, *right_ticks_; // Number of total wheel encoder tick counts for left and right wheels.
	int left_ticks_prev_, right_ticks_prev_; // Number of total wheel encoder tick counts at time computeAngularVelocities() is called.
	double xc_, yc_; // Latest position coordinates in ticks.
	double wl_, wr_; // Latest left and right angular velocity of the wheels in radians per second.
	double ticks_per_rev_; // Number of tick registers per second of the encoder.
	double length_; // Length from left wheel to right wheel.
	double radius_; // Radius of the wheel.
	double theta_;
	unsigned long prev_integration_time_;
	unsigned long prev_wheel_compute_time_;

};

#endif
