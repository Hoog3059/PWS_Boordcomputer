#define ACCELGYRO
//#define ALTIMETER
//#define ALTIMETER_TEST

#ifdef ACCELGYRO
#include "accelgyro.h"
#elif defined(ALTIMETER)
#include "altimeter.h"
#elif defined(ALTIMETER_TEST)
#include "altimeter_test.h"
#endif
