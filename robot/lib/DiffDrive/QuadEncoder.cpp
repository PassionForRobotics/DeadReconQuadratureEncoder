#include <Arduino.h>
#include <DiffDrive.h>

  //QuadEncoder();
    QuadEncoder::QuadEncoder(int _pin_A, int _pin_B)
    {
      this->pin_1_ = _pin_A;
      this->pin_2_ = _pin_B;
    }

    void QuadEncoder::begin( isr_cb isr_cb_A , isr_cb isr_cb_B )
    {
      attachInterrupt(digitalPinToInterrupt(this->pin_1_), isr_cb_A,  RISING);
      attachInterrupt(digitalPinToInterrupt(this->pin_1_), isr_cb_B,  CHANGE);
    }

    long int QuadEncoder::getTick ()
    {
        return this->ticks_;
    }

    void QuadEncoder::reset ()
    {
      this->ticks_ = 0;
    }



    volatile void QuadEncoder::pulseA(void)
      {
        // Test transition
        this->pastA_ = digitalRead(this->pin_1_) == HIGH;
        // and adjust counter + if A leads B
        this->ticks_ += (this->pastA_ == this->pastB_) ? +1 : -1;
      }

      volatile void QuadEncoder::pulseB(void)
      {
        // Test transition
        this->pastB_ = digitalRead(this->pin_2_) == HIGH;
        // and adjust counter + if A leads B
        this->ticks_ += (this->pastA_ == this->pastB_) ? +1 : -1;
      }
//private:
//  volatile long int ticks_;
//  volatile bool pastA_;
//	volatile bool pastB_;
//	void (*isr_cb_A)(void);
//	void (*isr_cb_B)(void);
