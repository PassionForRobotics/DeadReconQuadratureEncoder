#include <Arduino.h>
#include <DiffDrive.h>

void SpeedController::begin(Motor *_motor, QuadEncoder *_quad_encoder, float _wheel_radius)
{
        this->motor_ = _motor;
        this->quad_encoder_ = _quad_encoder;
        this->wheel_radius_ = _wheel_radius;

        this->lr_ = this->motor_->getLR();

        this->pid_ = new PID((double *)&this->radial_speed_
                             , (double *)&this->pid_output_, (double *)(&this->pid_setpoint_),0.0,11.0,0.0,P_ON_M, DIRECT);
        this->pid_->SetSampleTime(UPDATE_TIME/1000.0);
        this->pid_->SetMode(AUTOMATIC);
        this->pid_->SetOutputLimits(-255, 255);


}

void SpeedController::calculate()
{
        this->ticks_ = this->quad_encoder_->getTick();
        this->radial_dist_ += ( 2.0 * PI * (this->ticks_ - this->last_ticks_ )) / TICKS_PER_REV; // arc dist
        this->radial_speed_ = ( (this->radial_dist_ - this->last_radial_dist_)*1000.0*1000.0) / UPDATE_TIME; // dist/t_us = dist/t_us = rps
        this->radial_acceleration_ = ( (this->radial_speed_-this->last_radial_speed_)*1000.0*1000.0) / UPDATE_TIME; // speed/t_us = speed/t_us = rpss

        // Serial.print((int)this, HEX);Serial.print(" sc.ticks: ");Serial.println(this->ticks_);
        // //Serial.print("radial_dist: ");Serial.println(this->radial_dist_);
        // Serial.print((int)this, HEX);Serial.print(" sc.radial_speed: ");Serial.println(this->radial_speed_);

        this->last_ticks_ = this->ticks_;
        this->last_radial_dist_ = this->radial_dist_;
        this->last_radial_speed_ = this->radial_speed_;
}

void SpeedController::update ()
{
        unsigned long int curr_time = (unsigned long int)micros();

        // Following statement is making the speed and position loop unsynced - nyquist issue
        //if(curr_time - this->last_update_time_ >= UPDATE_TIME ) // 10ms
        {
                this->last_update_time_ = curr_time;

                // if(100000<curr_time)
                // {
                //   while(1);
                // }

                //Serial.print((int)this->lr_, HEX);Serial.print(" sc.ticks: ");Serial.println(this->ticks_);
                //Serial.print((int)this->lr_, HEX);Serial.print(" radial_dist: ");Serial.println(this->radial_dist_);
                //Serial.print((int)this->lr_, HEX);Serial.print(" sc.radial_speed: ");Serial.println(this->radial_speed_);

                this->pid_->Compute(); // another class should call calculate first or calculate should be called before pid.compute

                //Serial.print((int)this->lr_, HEX);Serial.print(" sc.speed pid out: ");Serial.println(this->pid_output_);
                //Serial.print((int)this->lr_, HEX);Serial.print(" sc.speed setpoint: ");Serial.println(this->pid_setpoint_);
                //Serial.print((int)this->lr_, HEX);Serial.print(" sc.speed - setpoint: ");Serial.println(this->radial_speed_ - this->pid_setpoint_);

                Serial.print((int)this->lr_, HEX);
                Serial.print(","); Serial.print(this->radial_speed_);
                Serial.print(","); Serial.print(this->pid_setpoint_);
                Serial.print(","); Serial.println(this->pid_output_);

                this->motor_->setPWM(this->pid_output_);

                //Serial.print((int)this->lr_, HEX);Serial.print(" PID: ");Serial.print(this->pid_->GetKp(),6);
                //Serial.print(" ");Serial.print(this->pid_->GetKi(),6);
                //Serial.print(" ");Serial.println(this->pid_->GetKd(),6);

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
        this->radial_dist_ = 0;
        this->last_radial_dist_ = 0;

        float speed = constrain(abs(_speed), 20.0, 150.0);
        int sign =  _speed > 0 ? 1 : -1;

        this->pid_setpoint_ = sign*speed;

//  Serial.print((int)this->lr_);Serial.print(" sc._speed: ");Serial.println(_speed);

}
