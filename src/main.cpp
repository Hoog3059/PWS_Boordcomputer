#define ACCELGYRO
//#define ALTIMETER
//#define ALTIMETER_TEST
//#define FINAL

#ifdef ACCELGYRO
#include "accelgyro.h"
#elif defined(ALTIMETER)
#include "altimeter.h"
#elif defined(ALTIMETER_TEST)
#include "altimeter_test.h"
#elif defined(FINAL)
#include "final.h"
#endif
