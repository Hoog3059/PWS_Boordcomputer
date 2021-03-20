//#define ACCELGYRO_CALIBRATE
//#define SENSORTEST
#define FINAL

#ifdef ACCELGYRO_CALIBRATE
#include "accelgyro_calibrate.cpp"
#elif defined(SENSORTEST)
#include "sensor_test.cpp"
#elif defined(FINAL)
#include "final.cpp"
#endif