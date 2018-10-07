#ifndef DIFF_DRIVE_H_
#define DIFF_DRIVE_H_

#include <Arduino.h>
#include <PID_v1.h>

#include <SerialCommand.h>

// ENCODER PINS
// Left Encoder
#define ENCODER_LEFTA_PIN (2)
#define ENCODER_LEFTB_PIN (3)
// Right Encoder
#define ENCODER_RIGHTA_PIN (18)
#define ENCODER_RIGHTB_PIN (19)

// Motor PINS
// Left Motor
#define MOTOR_2_PIN_1 (11)
#define MOTOR_2_PIN_2 (12)
// Right Motor
#define MOTOR_1_PIN_1 (9)
#define MOTOR_1_PIN_2 (10)

//wheel detail
#define RADIUS (35.0) // wheel radius in mm
#define LENGTH (150.0) // wheel base length in mm
#define TICKS_PER_REV (334.0)

// time in micro seconds
#define UPDATE_TIME (10000) //10000us // 10ms

// Just for ease in debugging
#define LEFT (0)
#define RIGHT (1)

// The given motors were having stall below 20 and max speed saturation at 180
// selected 20 at low speed will be ok enough to get the bot accelerated
// selected 150 as max speed saturation as well as jitter at top speed 180
// Can go till 160
#define MIN_SPEED (20)
#define MAX_SPEED (150)

// tested speed pid 15.0 120.0 0.2
#define PID_SPEED_P (15.0) //(0)
#define PID_SPEED_I (120.0)//(11.0)
#define PID_SPEED_D (0.2) //(0)

// to test
#define PID_POSITION_P (300.0) //(1.0)
#define PID_POSITION_I (2307.7) // (0)
#define PID_POSITION_D (5.75)  //(0)

#define CONTROLLER_SPEED (0)
#define CONTROLLER_POSITION (1)

#define POSITION_ATTANED_THRESH (0.01)
#define SPEED_ATTANED_THRESH (2.5)

// typedef for quadrature encoder ISR callbacks
typedef void (*isr_cb)(void);

class Motor
{
public:
void begin(int _lr, int _pin_A, int _pin_B);
void setPWM (int _pwm);
int getLR(){
        return this->lr_;
}
private:
int pwm_;
int pin_1_;
int pin_2_;
int lr_;
};

class QuadEncoder
{
public:
QuadEncoder(int _lr, int _pin_A, int _pin_B);
void begin( isr_cb isr_cb_A, isr_cb isr_cb_B );
long int getTick ();
void reset ();
volatile void pulseA(void);
volatile void pulseB(void);
int getLR(){
        return this->lr_;
}
private:
volatile long int ticks_;
volatile bool pastA_;
volatile bool pastB_;
int pin_1_;
int pin_2_;
int lr_;

};

class Controller
{
public:
void begin (Motor _motor,
            QuadEncoder _quad_encoder,
            float _wheel_radius);
void setPositionPID (float _P,
                     float _I,
                     float _D);
void update (int _mode);
void setPIDMode (int mode=0);
protected:
Motor *motor_;
QuadEncoder *quad_encoder_;
float wheel_radius_;
PID *pid_;
float pid_output_;
float pid_setpoint_;
float last_error_;
float accumulated_error_;
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
void update (int _mode);
float * getPositionPIDIn();
float getRadialDist();
float getLinearDist();
float getRevolutions();
float getRadialSpeed();
float getLinearSpeed();
float getRadialAcceleration();
float getLinearAcceleration();
void calculate();
void reset();
void setPIDMode (int mode=0);
void testSpeed(float _speed);
long int getTicks()
{
        return this->ticks_;
}
bool hasSpeedAttained()
{
        return this->has_speed_attained_;
}
private:
bool  twiddle();
float * getPIDIn();
long int ticks_;
long int last_ticks_;
long int delta_ticks_;

float radial_dist_; // unit = radians
float linear_dist_; // unit = unit of RADIUS (mm)
float revolutions_; // unit = revolutions

float radial_speed_; // rad/seconds
float radial_acceleration_; // unit = rad/s/s

float rev_speed_; // revolution/seconds
float rev_acceleration_; // revolution/seconds/seconds

float linear_speed_; // unit = mm/seconds
float linear_acceleration_; // unit = mm/s/s

bool has_speed_attained_;
int lr_;

//  Motor motor_;
//  QuadEncoder quad_encoder_;
//  float wheel_diameter_;
//  PID pid;
};



/*(NULL)*/
class PositionController : public Controller
{
public:
void begin (Motor *_motor,
            QuadEncoder *_quad_encoder,
            float _wheel_diameter);
void setPosition (float _dist);
void setPositionPID (float _P,
                     float _I,
                     float _D);
void setSpeedPID (float _P,
                  float _I,
                  float _D);
bool hasPositionAttained()
{
        return this->has_position_attained_;
}
bool hasSpeedAttained()
{
        return this->speed_controller_->hasSpeedAttained();
}
void update (int _mode);
void setPIDMode (int mode=0);
void testPosition(float _dist);
float getPosition();
void testSpeed(float _speed);
private:
float position_;
bool has_position_attained_;

int lr_;
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
void testDist(float _dist);
void testSpeed(float _speed);
void update ();
void drive (float _l, float _r);
void setControllerMode(int _mode)
{
        this->controller_mode_ = _mode;
}
bool hasAttained(int _l_or_r, int _sp_or_pos);
//void drive (float _d, float _a);

private:
int controller_mode_;
bool enable_wheel_left_,  enable_wheel_right_;
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

static void testSpeed();
static void parseAndSetPositionPIDValues();
static void parseAndSetSpeedPIDValues();
static void testPosition();
static void setControllerMode();
static void setWheelStatus();
};

#endif
