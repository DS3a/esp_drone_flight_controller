#ifndef GYRO_HIGHPASSFILTER_H_
#define GYRO_HIGHPASSFILTER_H_

/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 1000 Hz

* 0 Hz - 200 Hz
  gain = 0
  desired attenuation = -20 dB
  actual attenuation = -25.043791557656363 dB

* 300 Hz - 500 Hz
  gain = 1
  desired ripple = 0.2 dB
  actual ripple = 0.08268370078713436 dB

*/

#define GYRO_HIGHPASSFILTER_TAP_NUM 17

typedef struct {
  double history[GYRO_HIGHPASSFILTER_TAP_NUM];
  unsigned int last_index;
} gyro_highpassFilter;

void gyro_highpassFilter_init(gyro_highpassFilter* f);
void gyro_highpassFilter_put(gyro_highpassFilter* f, double input);
double gyro_highpassFilter_get(gyro_highpassFilter* f);

#endif