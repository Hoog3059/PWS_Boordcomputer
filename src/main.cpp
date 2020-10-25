//#define ACCELGYRO
#define ALTIMETER

#ifdef ACCELGYRO
#include "accelgyro.h"
#elif defined(ALTIMETER)
#include "altimeter.h"
#endif
