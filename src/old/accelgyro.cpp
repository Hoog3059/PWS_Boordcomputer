#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include "Wire.h"

/** AccelGyro options **/
#pragma region AccelGyro

MPU6050 accelgyro;
bool accelgyroError = false;

#define GRAVITY 9.806

#define ACCEL_X_AXIS_DOWN
//#define ACCEL_Y_AXIS_DOWN
//#define ACCEL_Z_AXIS_DOWN

/* Accel range
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 */
#define ACCEL_RANGE 3
#define GYRO_RANGE 1

#define ACCELGYRO_SAMPLE_MEAN 20

// Variables to store accelgyro data in.
int16_t ax, ay, az;
int16_t gx, gy, gz;

int16_t *av;

#pragma endregion

/** Velocity in m/s */
double velocity = 0;

unsigned long time;
unsigned long previousTime;

void calibrateAccelGyro();
void AccelGyroMeanReadings();
void takeSensorReadings();

#define NO_MEAN

void setup()
{
    // Define vertical acceleration vector.
#ifdef ACCEL_X_AXIS_DOWN
    av = &ax;
#elif defined(ACCEL_Y_AXIS_DOWN)
    av = &ay;
#elif defined(ACCEL_Z_AXIS_DOWN)
    av = &az;
#endif

    Wire.begin();
    Wire.setWireTimeout(2500);
    Serial.begin(115200);
    accelgyro.initialize();

    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // AccelGyro calibration
    accelgyro.setXAccelOffset(-2995);
    accelgyro.setYAccelOffset(1561);
    accelgyro.setZAccelOffset(1256);
    accelgyro.setXGyroOffset(51);
    accelgyro.setYGyroOffset(26);
    accelgyro.setZGyroOffset(-76);

    accelgyro.setFullScaleAccelRange(ACCEL_RANGE);
    accelgyro.setFullScaleGyroRange(GYRO_RANGE);

    previousTime = millis();
}

void loop()
{
    takeSensorReadings();

    time = millis();

    Serial.print(millis() - previousTime);

    // In cm/s
    velocity = velocity + (*av) * double(time - previousTime) / 1000.0;

    previousTime = time;

    Serial.print(" : ");
    Serial.print(velocity);
    Serial.print(" cm/s    ");

    Serial.print("    a/g:    ");
    Serial.print(ax);
    Serial.print("    ");
    Serial.print(ay);
    Serial.print("    ");
    Serial.print(az);
    Serial.print(" cm/s^2    ");
    Serial.print(gx);
    Serial.print("    ");
    Serial.print(gy);
    Serial.print("    ");
    Serial.println(gz);
}

void takeSensorReadings()
{ // Accelgyro

    accelgyroError = !accelgyro.testConnection();

    if(accelgyroError){
        accelgyro.initialize();
    }

    AccelGyroMeanReadings();

    // Correct for gravity
    (*av) = int16_t((*av) - (GRAVITY * 100));
}

#ifdef NO_MEAN
void AccelGyroMeanReadings()
{
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    /** Convert raw acceleration to cm/s^2 **/
    ax = int16_t(ax / (16348.0 / pow(2, ACCEL_RANGE)) * GRAVITY * 100);
    ay = int16_t(ay / (16348.0 / pow(2, ACCEL_RANGE)) * GRAVITY * 100);
    az = int16_t(az / (16348.0 / pow(2, ACCEL_RANGE)) * GRAVITY * 100);

    // Angular velocity in °/s
    gx = int16_t(gx / (131.0 / pow(2, GYRO_RANGE)));
    gy = int16_t(gy / (131.0 / pow(2, GYRO_RANGE)));
    gz = int16_t(gz / (131.0 / pow(2, GYRO_RANGE)));

    delay(10);
}
#else
void AccelGyroMeanReadings()
{
    long buff_ax = 0;
    long buff_ay = 0;
    long buff_az = 0;
    long buff_gx = 0;
    long buff_gy = 0;
    long buff_gz = 0;

    for (size_t i = 0; i < ACCELGYRO_SAMPLE_MEAN; i++)
    {
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        /** Convert raw acceleration to cm/s^2 **/
        ax = int16_t(ax / (16348.0 / pow(2, ACCEL_RANGE)) * GRAVITY * 100);
        ay = int16_t(ay / (16348.0 / pow(2, ACCEL_RANGE)) * GRAVITY * 100);
        az = int16_t(az / (16348.0 / pow(2, ACCEL_RANGE)) * GRAVITY * 100);

        buff_ax += ax;
        buff_ay += ay;
        buff_az += az;

        // Angular velocity in °/s
        gx = int16_t(gx / (131.0 / pow(2, GYRO_RANGE)));
        gy = int16_t(gy / (131.0 / pow(2, GYRO_RANGE)));
        gz = int16_t(gz / (131.0 / pow(2, GYRO_RANGE)));

        buff_gx += gx;
        buff_gy += gy;
        buff_gz += gz;

        delay(2);
    }

    ax = buff_ax / ACCELGYRO_SAMPLE_MEAN;
    ay = buff_ay / ACCELGYRO_SAMPLE_MEAN;
    az = buff_az / ACCELGYRO_SAMPLE_MEAN;

    gx = buff_gx / ACCELGYRO_SAMPLE_MEAN;
    gy = buff_gy / ACCELGYRO_SAMPLE_MEAN;
    gz = buff_gz / ACCELGYRO_SAMPLE_MEAN;
}
#endif