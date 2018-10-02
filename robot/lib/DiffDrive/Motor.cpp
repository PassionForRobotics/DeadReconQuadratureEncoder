#include <Arduino.h>
#include <DiffDrive.h>

void Motor::begin(int _lr, int _pin_A, int _pin_B)
{

        this->pin_1_ = _pin_A;
        this->pin_2_ = _pin_B;

        this->lr_ = _lr;

        pinMode(this->pin_1_, OUTPUT);
        pinMode(this->pin_2_, OUTPUT);

        this->setPWM(0);
}

static double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
        return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



void Motor:: setPWM(int _s)
{
        //TODO: max_vel and acc check

        this->pwm_ = _s;

        int sign =  _s > 0 ? 1 : -1;

        _s = abs(_s);//+100;

        //Serial.print("pwm "); Serial.print(sign); Serial.print(" "); Serial.println(_s);

        if(sign == 1)
        {
                analogWrite(this->pin_1_, _s );
                analogWrite(this->pin_2_, 0 );
        }
        else
        {
                analogWrite(this->pin_1_, 0 );
                analogWrite(this->pin_2_, _s );
        }

}
