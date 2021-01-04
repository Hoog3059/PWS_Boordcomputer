//#define ACCELGYRO_CALIBRATE
//#define SENSORTEST
#define FINAL

#ifdef ACCELGYRO_CALIBRATE
#include "accelgyro_calibrate.h"
#elif defined(SENSORTEST)
#include "sensor_test.h"
#elif defined(FINAL)
#include "final.h"
#endif