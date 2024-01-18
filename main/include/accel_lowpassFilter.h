#ifndef ACCEL_LOWPASSFILTER_H_
#define ACCEL_LOWPASSFILTER_H_

/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 100 Hz

* 0 Hz - 20 Hz
  gain = 1
  desired ripple = 0.2 dB
  actual ripple = 0.11582101203038833 dB

* 25 Hz - 50 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -42.116505691297874 dB

*/

#define ACCEL_LOWPASSFILTER_TAP_NUM 43

typedef struct {
  double history[ACCEL_LOWPASSFILTER_TAP_NUM];
  unsigned int last_index;
} accel_lowpassFilter;

void accel_lowpassFilter_init(accel_lowpassFilter* f);
void accel_lowpassFilter_put(accel_lowpassFilter* f, double input);
double accel_lowpassFilter_get(accel_lowpassFilter* f);

#endif