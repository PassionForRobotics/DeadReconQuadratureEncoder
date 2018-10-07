#include <Arduino.h>
#include <DiffDrive.h>

void SpeedController::begin(Motor *_motor, QuadEncoder *_quad_encoder, float _wheel_radius)
{
        this->motor_ = _motor;
        this->quad_encoder_ = _quad_encoder;
        this->wheel_radius_ = _wheel_radius;

        this->lr_ = this->motor_->getLR();

        this->pid_ = new PID((double *)this->getPIDIn()
                             , (double *)&this->pid_output_
                             , (double *)(&this->pid_setpoint_)
                             , PID_SPEED_P, PID_SPEED_I, PID_SPEED_D,P_ON_M, DIRECT);
        this->pid_->SetSampleTime(UPDATE_TIME/1000.0);
        this->pid_->SetMode(AUTOMATIC);
        this->pid_->SetOutputLimits(-255, 255);


}

void SpeedController::calculate()
{
        this->ticks_ = this->quad_encoder_->getTick();
        this->delta_ticks_ = this->ticks_ - this->last_ticks_;

        float delta_revolutions = ( (this->delta_ticks_ )) / TICKS_PER_REV; // delta // unit = revolution
        float delta_angle = delta_revolutions * 360; // delta // unit = degrees
        float delta_readians = delta_revolutions * 2.0 * PI;  // delta // unit = radians // arc dist

        this->radial_dist_ += delta_readians; // unit = radians
        this->revolutions_ += delta_revolutions; // unit = revolutions
        this->linear_dist_ = this->radial_dist_ * RADIUS; // unit = unit of RADIUS (mm)

        this->radial_speed_ = ( (delta_readians)*1000.0*1000.0) / UPDATE_TIME; // rad/seconds
        this->radial_acceleration_ = ( (this->radial_speed_)*1000.0*1000.0) / UPDATE_TIME; // unit = rad/s/s

        this->rev_speed_ = ((delta_revolutions)*1000.0*1000.0) / UPDATE_TIME; // revolution/seconds
        this->rev_acceleration_ = ((this->rev_speed_)*1000.0*1000.0) / UPDATE_TIME; // revolution/seconds/seconds

        this->linear_speed_ = this->radial_speed_ * RADIUS; // unit = mm/seconds
        this->linear_acceleration_ = ( (this->linear_speed_)*1000.0*1000.0) / UPDATE_TIME; // unit = mm/s/s

        // Serial.print((int)this, HEX);Serial.print(" sc.ticks: ");Serial.println(this->ticks_);
        // //Serial.print("radial_dist: ");Serial.println(this->radial_dist_);
        // Serial.print((int)this, HEX);Serial.print(" sc.radial_speed: ");Serial.println(this->radial_speed_);

        this->last_ticks_ = this->ticks_;
        //this->last_radial_dist_ = this->radial_dist_;
        //this->last_radial_speed_ = this->radial_speed_;
}



void SpeedController::update (int _mode)
{
        unsigned long int curr_time = (unsigned long int)micros();

        // Following statement is making the speed and position loop unsynced - nyquist issue
        //if(curr_time - this->last_update_time_ >= UPDATE_TIME ) // 10ms
        {
                this->last_update_time_ = curr_time;
                float error = this->pid_setpoint_ - (*this->getPIDIn());
                // if(100000<curr_time)
                // {
                //   while(1);
                // }

                //Serial.print((int)this->lr_, HEX);Serial.print(" sc.ticks: ");Serial.println(this->ticks_);
                //Serial.print((int)this->lr_, HEX);Serial.print(" radial_dist: ");Serial.println(this->radial_dist_);
                //Serial.print((int)this->lr_, HEX);Serial.print(" sc.radial_speed: ");Serial.println(this->radial_speed_);

                this->pid_->Compute(); // another class should call calculate first or calculate should be called before pid.compute
                //this->twiddle();
                //Serial.print((int)this->lr_, HEX);Serial.print(" sc.speed pid out: ");Serial.println(this->pid_output_);
                //Serial.print((int)this->lr_, HEX);Serial.print(" sc.speed setpoint: ");Serial.println(this->pid_setpoint_);
                //Serial.print((int)this->lr_, HEX);Serial.print(" sc.speed - setpoint: ");Serial.println(this->radial_speed_ - this->pid_setpoint_);

                if(SPEED_ATTANED_THRESH>abs(error))
                {
                        // position attained
                        this->has_speed_attained_ = true;

                }
                //Serial.print(","); Serial.print(this->radial_dist_); // continue printing in position controller

                if(CONTROLLER_SPEED == _mode)
                {
                        this->motor_->setPWM(this->pid_output_);

                        Serial.print((int)this->lr_, HEX);
                        Serial.print(","); Serial.print(*this->getPIDIn());
                        Serial.print(","); Serial.print(this->pid_setpoint_);
                        Serial.print(","); Serial.print(this->pid_output_);
                        Serial.print(","); Serial.print(this->has_speed_attained_*200);
                        Serial.print(","); Serial.println(this->linear_speed_);
                }
                //Serial.print((int)this->lr_, HEX);Serial.print(" PID: ");Serial.print(this->pid_->GetKp(),6);
                //Serial.print(" ");Serial.print(this->pid_->GetKi(),6);
                //Serial.print(" ");Serial.println(this->pid_->GetKd(),6);

        }
}


float * SpeedController::getPositionPIDIn()
{
        // Which unit needs to be here
        // rad
        // mm
        // rev
        return &this->radial_dist_;
        //return &this->revolutions_;
        //return &this->linear_dist_;
}

float * SpeedController::getPIDIn()
{
        // Which unit needs to be here
        // rad/sec
        // mm/sec
        // rev/sec
        return &this->radial_speed_;
        //return &this->rev_speed_;
        //return &this->linear_speed_;
}

float SpeedController::getRadialDist()
{
        return this->radial_dist_;
}

float SpeedController::getLinearDist()
{
        return this->linear_dist_;
}

float SpeedController::getRevolutions()
{
        return this->revolutions_;
}

float SpeedController::getRadialSpeed()
{
        return this->radial_speed_;
}

float SpeedController::getLinearSpeed()
{
        return this->linear_speed_;
}

float SpeedController::getRadialAcceleration()
{
        return this->radial_acceleration_;
}

float SpeedController::getLinearAcceleration()
{
        return this->linear_acceleration_;
}



void SpeedController::setPID(float _P, float _I, float _D)
{
        this->pid_->SetTunings(_P, _I, _D, P_ON_M);
}

void SpeedController::setSpeed(float _speed)
{
        //this->radial_dist_ = 0;
        //this->last_radial_dist_ = 0;

        float speed = abs(_speed); //constrain(abs(_speed), 20.0, 150.0);
        int sign =  _speed > 0 ? 1 : -1;

        this->pid_setpoint_ = sign*speed;

        this->has_speed_attained_ = false;

//  Serial.print((int)this->lr_);Serial.print(" sc._speed: ");Serial.println(_speed);

}

void SpeedController::reset()
{
        this->quad_encoder_->reset();

        this->ticks_=0;
        this->delta_ticks_ =0;

        this->radial_dist_=0.0f; // unit = radians
        this->revolutions_ = 0.0f; // unit = revolutions // unit less may be !
        this->linear_dist_=0.0f; // unit = unit of RADIUS (mm)

        this->radial_speed_=0.0f; // rad/seconds
        this->radial_acceleration_=0.0f; // unit = rad/s/s

        this->rev_speed_=0.0f; // revolution/seconds
        this->rev_acceleration_=0.0f; // revolution/seconds/seconds

        this->linear_speed_=0.0f; // unit = mm/seconds
        this->linear_acceleration_=0.0f; // unit = mm/s/s

        this->last_ticks_=0;

}



// Did not understand this ??
// https://github.com/amolgm/AutotunePIDcontroller/blob/master/AutotunePIDcontroller/LF_sum11_4/LF_sum11_4.c#L342
bool SpeedController::twiddle()
{
// Choose an initialization parameter vector
        static float p[3] = {0.0, 0.0, 0.0};
// Define potential changes
        static float dp[3] = {1.0, 1.0, 1.0};
// Calculate the error
        float best_err = this->radial_speed_; //  A is an algorithm that returns an error.

        float threshold = 0.000000000000001;

        int i;

        bool status;

        float err;

        if ((dp[0]+dp[1]+dp[2]) > threshold)
        {
                for( i=0; i<3; i++)
                {
                        p[i] += dp[i];
                        err = this->radial_speed_;

                        if( err < best_err) // There was some improvement
                        {
                                best_err = err;
                                dp[i] *= 1.1;
                        }
                        else
                        {
                                // There was no improvement
                                p[i] -= 2*dp[i];// Go into the other direction
                                err = this->radial_speed_;

                                if (  err < best_err)// There was an improvement
                                {
                                        best_err = err;
                                        dp[i] *= 1.05;
                                }
                                else //  There was no improvement
                                {
                                        p[i] += dp[i];
                                        // As there was no improvement, the step size in either
                                        // direction, the step size might simply be too big.
                                        dp[i] *= 0.95;
                                }
                        }
                }
                status = false;
        }
        else
        {
                //pid calculated

                status = true;

        }

        this->setPID(p[0], p[1], p[2]);

        Serial.print("PID values: P:");
        Serial.print(p[0], 6);

        Serial.print(" ( ");
        Serial.print(dp[0], 6);

        Serial.print(" ) I:");
        Serial.print(p[1], 6);

        Serial.print(" ( ");
        Serial.print(dp[1], 6);

        Serial.print(" ) D:");
        Serial.print(p[2], 6);

        Serial.print(" ( ");
        Serial.print(dp[2], 6);
        Serial.println(" ) ");

        return status;
}
