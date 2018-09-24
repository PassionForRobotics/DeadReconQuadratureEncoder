
#include <Arduino.h>

#include "QuadEnc.h"

#include <DeadReckoner.h>

#define SPEED_COMPUTE_INTERVAL (10.0)
#define ENCODER_PULSES_PER_REV (334)
#define SPEED_MUTIPLIER (9000.0) // To match output


  QuadEncoder::QuadEncoder(int _pin_1, int _pin_2)//, float _radius, float _separation, int _ticks_per_rev, int _compute_interval)
  {
    this->pin_1_ = _pin_1;
    this->pin_2_ = _pin_2;

    //this->radius_ = _radius;
    //this->separation_ = _separation;

    //this->ticks_per_rev_ = _ticks_per_rev;

    //this->compute_interval_ = _compute_interval;
  }


volatile void QuadEncoder::pulseA(void)
  {
    // Test transition
    this->pastA_ = digitalRead(this->pin_1_) == HIGH;
    // and adjust counter + if A leads B
    position_ += (this->pastA_ == this->pastB_) ? +1 : -1;
  }

  volatile void QuadEncoder::pulseB(void)
  {
    // Test transition
    this->pastB_ = digitalRead(this->pin_2_) == HIGH;
    // and adjust counter + if A leads B
      position_ += (this->pastA_ == this->pastB_) ? +1 : -1;
  }

  volatile void QuadEncoder::updateRPM(void)
  {

    	if (millis() - this->prev_speed_compute_time_ > SPEED_COMPUTE_INTERVAL)
      {
    		this->prev_speed_compute_time_ = millis();

        long long int ticks = (double) this->position_;

        double delta_pos = ((double)ticks - (double)this->last_position_)/((double)ENCODER_PULSES_PER_REV);

        this->last_position_ = ticks;

        this->meas_rps_ = ticks%ENCODER_PULSES_PER_REV;

        this->rp10millis_ = (double)delta_pos/(double)SPEED_COMPUTE_INTERVAL;

        this->rps_ = this->rp10millis_*100.0;

        this->rpm_ = this->rps_*60.0;
      }

  }

  void QuadEncoder::begin( void (*isr_cb_A)(void), void (*isr_cb_B)(void)  )
  {

    pinMode(this->pin_1_, INPUT);
    //turn on pullup resistor
    //digitalWrite(encoder0PinA, HIGH); //ONLY FOR SOME ENCODER(MAGNETIC)!!!!
    pinMode(this->pin_2_, INPUT);
    //turn on pullup resistor
    //digitalWrite(encoder0PinB, HIGH); //ONLY FOR SOME ENCODER(MAGNETIC)!!!!
    //pastA_ = (bool)digitalRead(this->pin_1_); //initial value of channel A;
    //pastB_ = (bool)digitalRead(this->pin_2_); //and channel B

    //To speed up even more, you may define manually the ISRs
    // encoder A channel on interrupt 0 (Arduino's pin 2)
    attachInterrupt(digitalPinToInterrupt(this->pin_1_), isr_cb_A, RISING);
    // encoder B channel pin on interrupt 1 (Arduino's pin 3)
    attachInterrupt(digitalPinToInterrupt(this->pin_2_), isr_cb_B, CHANGE);
  }



  void QuadEncoder::reset(int _v)
  {
    position_ = _v;
  }
