#define ACCELGYRO
//#define ACCELGYRO_CALIBRATE
//#define ALTIMETER
//#define ALTIMETER_TEST
//#define FINAL

#ifdef ACCELGYRO
#include "accelgyro.h"
#elif defined(ACCELGYRO_CALIBRATE)
#include "accelgyro_calibrate.h"
#elif defined(ALTIMETER)
#include "altimeter.h"
#elif defined(ALTIMETER_TEST)
#include "altimeter_test.h"
#elif defined(FINAL)
#include "final.h"
#endif
