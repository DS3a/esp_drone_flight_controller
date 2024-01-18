#include "gyro_highpassFilter.h"

static double filter_taps[GYRO_HIGHPASSFILTER_TAP_NUM] = {
  -0.01174166224278905,
  -0.029045150868676206,
  -0.055973139516344375,
  -0.08843168719950603,
  -0.12000989153159465,
  -0.14310530975742314,
  0.8484130805223907,
  -0.14310530975742314,
  -0.12000989153159465,
  -0.08843168719950603,
  -0.055973139516344375,
  -0.029045150868676206,
  -0.01174166224278905
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
