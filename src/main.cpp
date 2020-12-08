//#define ACCELGYRO_CALIBRATE
#define FINAL
//#define TEST

#ifdef ACCELGYRO_CALIBRATE
#include "accelgyro_calibrate.h"
#elif defined(FINAL)
#include "final.h"
#elif defined(TEST)
#include "accelgyro.h"
#endif