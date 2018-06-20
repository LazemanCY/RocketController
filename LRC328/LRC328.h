#ifndef LRC328_H_
#define LRC328_H_

#include "Arduino.h"

typedef struct {
  int16_t  accSmooth[3];
  int16_t  gyroData[3];
  int16_t  magADC[3];
  int16_t  gyroADC[3];
  int16_t  accADC[3];
} imu_t;

enum rc {
  ROLL,
  PITCH,
  YAW
};

#endif /* LRC328_H_ */
