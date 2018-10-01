#ifndef DIFF_DRIVE_H_
#define DIFF_DRIVE_H_

#include <Arduino.h>
#include <PID_v1.h>

// ENCODER PINS
// Left Encoder
#define ENCODER_LEFTA_PIN 2
#define ENCODER_LEFTB_PIN 3
// Right Encoder
#define ENCODER_RIGHTA_PIN 18
#define ENCODER_RIGHTB_PIN 19

// Motor PINS
// Left Motor
#define MOTOR_2_PIN_1 11
#define MOTOR_2_PIN_2 12
// Right Motor
#define MOTOR_1_PIN_1 9
#define MOTOR_1_PIN_2 10


#define RADIUS 35.0 // wheel radius in mm
#define LENGTH 150.0 // wheel base length in mm
#define TICKS_PER_REV 334.0

#define UPDATE_TIME 10000 // 10ms


typedef void (*isr_cb)(void);

class Motor
{
public:
	Motor ();
  void begin(int _pin_A, int _pin_B);
	void setPWM (int _pwm);
private:
	int pwm_;
	int pin_1_;
	int pin_2_;
};

class QuadEncoder
{
public:
  QuadEncoder();
	QuadEncoder(int _pin_A, int _pin_B);
	void begin( isr_cb isr_cb_A , isr_cb isr_cb_B );
	long int getTick ();
	void reset ();
  volatile void pulseA(void);
  volatile void pulseB(void);
private:
  volatile long int ticks_;
  volatile bool pastA_;
	volatile bool pastB_;
	int pin_1_;
	int pin_2_;
};

class Controller
{
public:
	void begin (Motor _motor,
	            QuadEncoder _quad_encoder,
	            float _wheel_radius);
	void setPID (float _P,
	             float _I,
	             float _D);
	void update ();
	void setPIDMode (int mode=0);
protected:
	Motor *motor_;
	QuadEncoder *quad_encoder_;
	float wheel_radius_;
	PID *pid_;
	float pid_output_;
	float pid_setpoint_;
	volatile unsigned long int last_update_time_;

};

class SpeedController : public Controller
{
public:
	void begin (Motor *_motor,
	            QuadEncoder *_quad_encoder,
	            float _wheel_radius);
	void setSpeed (float speed);
	void setPID (float _P,
	             float _I,
	             float _D);
	void update ();
	float getRadialDist();
	void calculate();
	void setPIDMode (int mode=0);
  void testSpeed(float _speed);
 private:
	long int ticks_;
	long int last_ticks_;
	float radial_dist_;
	float last_radial_dist_;
	float radial_speed_;
	float last_radial_speed_;
	float radial_acceleration_;

// 	Motor motor_;
// 	QuadEncoder quad_encoder_;
// 	float wheel_diameter_;
// 	PID pid;
};



/*(NULL)*/
class PositionController : public Controller
{
public:
	void begin (Motor *_motor,
	            QuadEncoder *_quad_encoder,
	            float _wheel_diameter);
	void setPosition (float _dist);
	void setPID (float _P,
	             float _I,
	             float _D);
	void update ();
	void setPIDMode (int mode=0);
  void testPosition(float _dist);
	void testSpeed(float _speed);
private:
 	//float updateRadialDist();
 	float radial_dist_;
	// Motor motor_;
	// QuadEncoder quad_encoder_;
	// float wheel_diameter_;
	// PID pid_;
	SpeedController *speed_controller_;
};

/*(NULL)*/
class Kinematics
{
public:
	Kinematics (float _wheel_radius,
	            float _wheel_base,
	            float _update_duration);
	void update ();
	void calculateNextPos (float * _x,
	                   float * _y,
	                   float * _th);
private:
	void calculateNextPos(float * _x,
	                    float * _y,
	                    float * _th,
	                    float _speed,
	                    float _duration);
	float getAcceleration ();
	float wheel_radius_;
	float wheel_base;
	float update_duration_;
	long int last_update_timestamp_;
};

/*(NULL)*/
class DiffDrive
{
public:
	DiffDrive (float _wheel_radius,
	           float _wheel_base,
	           float _update_duration);
  //~DiffDrive(); // will never need it
  //void initDiffDrive(Motor &M1, Motor &M2, QuadEncoder &Q1, QuadEncoder &Q2);
  //void initMotors(int _m1p1, int _m1p2, int _m2p1, int _m2p2);
	void testDist(float _dist);
	void testSpeed(float _speed);
  void update ();
	void drive (float  _l, float  _r);
  //void drive (float _d, float _a);

private:
  Motor * Ml_, *Mr_;
  QuadEncoder * Ql_, *Qr_;
	float wheel_radius_;
	float wheel_base_;
	float update_duration_;
	long int last_update_timestamp_;
	PositionController *position_controller_left_;
	PositionController *position_controller_right_;
	Kinematics *kinematics_;
	void initQuadEncoders();
	static void ql_pulseA();
	static void ql_pulseB();
	static void qr_pulseA();
	static void qr_pulseB();
};

#endif
