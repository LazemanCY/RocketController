#ifndef CONFIG_H_
#define CONFIG_H_

//#define DEBUG

#define GYRO_DLPF_CFG   0 //Default settings LPF 256Hz/8000Hz sample

/*default for GY-87 installed flat
#define ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  = -X; imu.accADC[PITCH]  = -Y; imu.accADC[YAW]  =  Z;}
#define GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[ROLL] =  Y; imu.gyroADC[PITCH] = -X; imu.gyroADC[YAW] = -Z;}
#define MAG_ORIENTATION(X, Y, Z)  {imu.magADC[ROLL]  =  X; imu.magADC[PITCH]  =  Y; imu.magADC[YAW]  = -Z;}
*/

#define ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  = -X; imu.accADC[PITCH]  = -Z; imu.accADC[YAW]  =  Y;}
#define GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[ROLL] =  Y; imu.gyroADC[PITCH] = -X; imu.gyroADC[YAW] = -Z;}

#define ACC_Triggle   1500 //1G is 512, 3G is 1536

//#define TimerDelay  140 //20 is 1 second
#define TimerDelay  54  //around 2.7s

#define SwitchOnTime  1000

#define LED_RED   6
#define LED_BLUE  13
#define PIN_FIRE  4
#define VOT_PIN   A7
#define KEY       2

#endif /* CONFIG_H_ */
