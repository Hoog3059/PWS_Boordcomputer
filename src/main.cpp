//#define ACCELGYRO_CALIBRATE
//#define SENSORTEST
#define FINAL

#ifdef ACCELGYRO_CALIBRATE
#include "./v2/accelgyro_calibrate.h"
#elif defined(SENSORTEST)
#include "./v2/sensor_test.h"
#elif defined(FINAL)
#include "./v2/final.h"
#endif