#ifndef QUAD_ENC_H_
#define QUAD_ENC_H_

#include <Arduino.h>

class QuadEncoder
{
private:
  int pin_1_, pin_2_;
  volatile long long int position_;
  //float radius_, separation_;
  int ticks_per_rev_;
  volatile bool pastA_, pastB_;
  //int compute_interval_;

public:
  void begin( void (*isr_cb_A)(void), void (*isr_cb_B)(void)  );
  QuadEncoder(int _pin_1, int _pin_2);//, float _radius, float _separation, int _ticks_per_rev, int _compute_interval);
  volatile void pulseA(void);
  volatile void pulseB(void);
  inline volatile long long int * getPosition(){return &position_;}


};

#endif
