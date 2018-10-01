#include <Arduino.h>
#include <DiffDrive.h>


/*
class SpeedController : public Controller
{
public:
	void begin (Motor *_motor,
	            QuadEncoder *_quad_encoder,
	            float _wheel_diameter);
	void setSpeed (float speed);
	void setPID (float _P,
	             float _I,
	             float _D);
	void update ();
	void setPIDMode (int mode=0);
  void testSpeed(float _speed);
// private:
// 	Motor motor_;
// 	QuadEncoder quad_encoder_;
// 	float wheel_diameter_;
// 	PID pid;
};
*/

void SpeedController::begin(Motor *_motor, QuadEncoder *_quad_encoder, float _wheel_radius)
{
  this->motor_ = _motor;
  this->quad_encoder_ = _quad_encoder;
  this->wheel_radius_ = _wheel_radius;

  this->pid_ = new PID((double *)&this->radial_speed_, (double *)&this->pid_output_, (double *)(&this->pid_setpoint_),0.1,0.5,0.000010,P_ON_M, DIRECT);
}


void SpeedController::calculate()
{
  this->ticks_ = this->quad_encoder_->getTick();
  this->radial_dist_ = ( 2.0 * PI * (this->ticks_ )) / TICKS_PER_REV ; // arc dist
  this->radial_speed_ = ( (this->radial_dist_-this->last_radial_dist_)*60000.0) / UPDATE_TIME; // dist/t_us = dist*60/t_us = rpm
  this->radial_acceleration_ = ( (this->radial_speed_-this->last_radial_speed_)*60000.0) / UPDATE_TIME; // speed/t_us = speed*60/t_us = rpmm

  this->last_ticks_ = this->ticks_;
  this->last_radial_dist_ = this->radial_dist_;
  this->last_radial_speed_ = this->radial_speed_;
}

void SpeedController::update ()
{
  unsigned long int curr_time = (unsigned long int)micros();
  if(curr_time - this->last_update_time_ >= UPDATE_TIME ) // 10ms
  {
    this->last_update_time_ = curr_time;

    this->pid_->Compute(); // another class should call calculate first or calculate should be called before pid.compute

    this->motor_->setPWM(this->pid_output_);

  }
}

float SpeedController::getRadialDist()
{
  return this->radial_dist_;
}


void SpeedController::setPID(float _P, float _I, float _D)
{
  this->pid_->SetTunings(_P, _I, _D, P_ON_M);
}

void SpeedController::setSpeed(float _speed)
{
  this->pid_setpoint_ = _speed;
}
