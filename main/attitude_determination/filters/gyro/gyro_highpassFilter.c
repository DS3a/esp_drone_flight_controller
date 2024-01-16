#include "gyro_highpassFilter.h"

static double filter_taps[GYRO_HIGHPASSFILTER_TAP_NUM] = {
  0.01527281790347584,
  0.018335327574133232,
  -0.020580654132941974,
  -0.05048576799202502,
  0.01162976932022604,
  0.08798199765093072,
  -0.024440617685040387,
  -0.3181830547427437,
  0.5162939946488528,
  -0.3181830547427437,
  -0.024440617685040387,
  0.08798199765093072,
  0.01162976932022604,
  -0.05048576799202502,
  -0.020580654132941974,
  0.018335327574133232,
  0.01527281790347584
};

void gyro_highpassFilter_init(gyro_highpassFilter* f) {
  int i;
  for(i = 0; i < GYRO_HIGHPASSFILTER_TAP_NUM; ++i)
    f->history[i] = 0;
  f->last_index = 0;
}

void gyro_highpassFilter_put(gyro_highpassFilter* f, double input) {
  f->history[f->last_index++] = input;
  if(f->last_index == GYRO_HIGHPASSFILTER_TAP_NUM)
    f->last_index = 0;
}

double gyro_highpassFilter_get(gyro_highpassFilter* f) {
  double acc = 0;
  int index = f->last_index, i;
  for(i = 0; i < GYRO_HIGHPASSFILTER_TAP_NUM; ++i) {
    index = index != 0 ? index-1 : GYRO_HIGHPASSFILTER_TAP_NUM-1;
    acc += f->history[index] * filter_taps[i];
  };
  return acc;
}
