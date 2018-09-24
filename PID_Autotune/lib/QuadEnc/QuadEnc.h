#ifndef QUAD_ENC_H_
#define QUAD_ENC_H_

#include <Arduino.h>

class QuadEncoder
{
private:
  int pin_1_, pin_2_;
  volatile long long int position_;
  long long int last_position_;
  long int prev_speed_compute_time_;
  double rp10millis_, rps_, rpm_, meas_rps_;
  //float radius_, separation_;
  int ticks_per_rev_;
  volatile bool pastA_, pastB_;
  //int compute_interval_;

public:
  void begin( void (*isr_cb_A)(void), void (*isr_cb_B)(void)  );
  QuadEncoder(int _pin_1, int _pin_2);//, float _radius, float _separation, int _ticks_per_rev, int _compute_interval);
  volatile void pulseA(void);
  volatile void pulseB(void);
  volatile void updateRPM(void);
  double *getRPM(void) {return &rpm_;}
  void reset(int _v);
  inline volatile long long int * getPosition(void){return &position_;}


};

#endif
