//#define ACCELGYRO
//#define ACCELGYRO_CALIBRATE
//#define DMP_TEST
//#define ALTIMETER
//#define ALTIMETER_TEST
#define FINAL
//#define TEST

#ifdef ACCELGYRO
#include "accelgyro.h"
#elif defined(ACCELGYRO_CALIBRATE)
#include "accelgyro_calibrate.h"
#elif defined(DMP_TEST)
#include "dmp_test.h"
#elif defined(ALTIMETER)
#include "altimeter.h"
#elif defined(ALTIMETER_TEST)
#include "altimeter_test.h"
#elif defined(FINAL)
#include "final.h"
#elif defined(TEST)
#include "test.h"
#endif
