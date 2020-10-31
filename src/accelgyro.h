#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include "Wire.h"

/** AccelGyro options **/
#pragma region AccelGyro

MPU6050 accelgyro;

#define GRAVITY 9.806

/* Accel range
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 */
#define ACCEL_RANGE 3
#define GYRO_RANGE 3

/** Number of measurements to mean for more accurate measurement. */
#define ACCELGYRO_SAMPLE_MEAN 20

#define ACCELGYRO_CALIBRATION_BUFFERSIZE 1000
#define ACCEL_CALIBRATION_DEADZONE 8
#define GYRO_CALIBRATION_DEADZONE 1

/** Defines which axis forms a right angle with the ground */
#define ACCEL_X_AXIS_DOWN
//#define ACCEL_Y_AXIS_DOWN
//#define ACCEL_Z_AXIS_DOWN

/** If defined, skip calibration. */
#define ACCELGYRO_NO_CALIBRATION
/** If defined, calibration offsets will be printed to Serial port */
#define VERBOSE_CALIBRATION

// Variables to store accelgyro data in.
int16_t ax, ay, az;
int16_t gx, gy, gz;

#pragma endregion

/** Velocity in m/s */
double velocity = 0;
/** Displacement in m */
double displacement = 0;

unsigned long time;
unsigned long previousTime;

void calibrateAccelGyro();
void accelgyroMeanSensors(long *mean_ax, long *mean_ay, long *mean_az, long *mean_gx, long *mean_gy, long *mean_gz);

void setup()
{
    Wire.begin();
    Serial.begin(115200);
    accelgyro.initialize();

    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

#ifdef ACCELGYRO_NO_CALIBRATION
    accelgyro.setXAccelOffset(62515);
    accelgyro.setYAccelOffset(1530);
    accelgyro.setZAccelOffset(1522);
    accelgyro.setXGyroOffset(73);
    accelgyro.setYGyroOffset(38);
    accelgyro.setZGyroOffset(65456);
#else
    accelgyro.setFullScaleAccelRange(0);
    accelgyro.setFullScaleGyroRange(0);

    calibrateAccelGyro();
#endif

    accelgyro.setFullScaleAccelRange(ACCEL_RANGE);
    accelgyro.setFullScaleGyroRange(GYRO_RANGE);

    previousTime = millis();
}

void loop()
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

        /** Convert raw acceleration to m/s^2
         * 
         */
        ax = int16_t(ax / (16348.0 / pow(2, ACCEL_RANGE)) * GRAVITY);
        ay = int16_t(ay / (16348.0 / pow(2, ACCEL_RANGE)) * GRAVITY);
        az = int16_t(az / (16348.0 / pow(2, ACCEL_RANGE)) * GRAVITY);

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

    // Correct for gravity
    ax = int16_t(ax - GRAVITY);

    time = millis();

    // In m/s
    velocity = velocity + ax * double(time - previousTime) / 1000.0;

    // In m
    displacement = displacement + velocity * double(time - previousTime) / 1000.0;

    previousTime = time;

    Serial.print(" : ");
    Serial.print(velocity);
    Serial.print(" m/s    ");
    Serial.print(displacement);
    Serial.print(" m");

    Serial.print("    a/g:    ");
    Serial.print(ax);
    Serial.print("    ");
    Serial.print(ay);
    Serial.print("    ");
    Serial.print(az);
    Serial.print(" dm/s^2    ");
    Serial.print(gx);
    Serial.print("    ");
    Serial.print(gy);
    Serial.print("    ");
    Serial.println(gz);
}

#pragma region Calibration
// Calibration for the MPU6050.
// Algorithm from https://mjwhite8119.github.io/Robots/mpu6050
void calibrateAccelGyro()
{
    Serial.println("Calibrating accelerometer and gyroscope...");

    accelgyro.setXAccelOffset(0);
    accelgyro.setYAccelOffset(0);
    accelgyro.setZAccelOffset(0);
    accelgyro.setXGyroOffset(0);
    accelgyro.setYGyroOffset(0);
    accelgyro.setZGyroOffset(0);

    int ax_offset, ay_offset, az_offset;
    int gx_offset, gy_offset, gz_offset;

    long mean_ax, mean_ay, mean_az;
    long mean_gx, mean_gy, mean_gz;

    accelgyroMeanSensors(&mean_ax, &mean_ay, &mean_az, &mean_gx, &mean_gy, &mean_gz);

    ax_offset = -mean_ax / 8;
    ay_offset = -mean_ay / 8;
    az_offset = -mean_ay / 8;

#ifdef ACCEL_X_AXIS_DOWN
    ax_offset = (16348 - mean_az) / 8;
#elif defined(ACCEL_Y_AXIS_DOWN)
    ay_offset = (16348 - mean_az) / 8;
#elif defined(ACCEL_Z_AXIS_DOWN)
    az_offset = (16348 - mean_az) / 8;
#endif

    gx_offset = -mean_gx / 4;
    gy_offset = -mean_gy / 4;
    gz_offset = -mean_gz / 4;
    while (1)
    {
        Serial.print(".");

        int ready = 0;
        accelgyro.setXAccelOffset(ax_offset);
        accelgyro.setYAccelOffset(ay_offset);
        accelgyro.setZAccelOffset(az_offset);

        accelgyro.setXGyroOffset(gx_offset);
        accelgyro.setYGyroOffset(gy_offset);
        accelgyro.setZGyroOffset(gz_offset);

        // Get the mean values from the sensor
        accelgyroMeanSensors(&mean_ax, &mean_ay, &mean_az, &mean_gx, &mean_gy, &mean_gz);

#ifdef ACCEL_X_AXIS_DOWN
        if (abs(16348 - mean_ax) <= ACCEL_CALIBRATION_DEADZONE)
            ready++;
        else
            ax_offset = ax_offset + (16348 - mean_ax) / ACCEL_CALIBRATION_DEADZONE;

        if (abs(mean_ay) <= ACCEL_CALIBRATION_DEADZONE)
            ready++;
        else
            ay_offset = ay_offset - mean_ay / ACCEL_CALIBRATION_DEADZONE;

        if (abs(mean_az) <= ACCEL_CALIBRATION_DEADZONE)
            ready++;
        else
            az_offset = az_offset - mean_az / ACCEL_CALIBRATION_DEADZONE;
#elif defined(ACCEL_Y_AXIS_DOWN)
        if (abs(mean_ax) <= ACCEL_CALIBRATION_DEADZONE)
            ready++;
        else
            ax_offset = ax_offset - mean_ax / ACCEL_CALIBRATION_DEADZONE;

        if (abs(16348 - mean_ay) <= ACCEL_CALIBRATION_DEADZONE)
            ready++;
        else
            ay_offset = ay_offset + (16348 - mean_ay) / ACCEL_CALIBRATION_DEADZONE;

        if (abs(mean_az) <= ACCEL_CALIBRATION_DEADZONE)
            ready++;
        else
            az_offset = az_offset - mean_az / ACCEL_CALIBRATION_DEADZONE;
#elif defined(ACCEL_Z_AXIS_DOWN)
        if (abs(mean_ax) <= ACCEL_CALIBRATION_DEADZONE)
            ready++;
        else
            ax_offset = ax_offset - mean_ax / ACCEL_CALIBRATION_DEADZONE;

        if (abs(mean_ay) <= ACCEL_CALIBRATION_DEADZONE)
            ready++;
        else
            ay_offset = ay_offset - mean_ay / ACCEL_CALIBRATION_DEADZONE;

        if (abs(16348 - mean_az) <= ACCEL_CALIBRATION_DEADZONE)
            ready++;
        else
            az_offset = az_offset + (16348 - mean_az) / ACCEL_CALIBRATION_DEADZONE;
#endif

        if (abs(mean_gx) <= GYRO_CALIBRATION_DEADZONE)
            ready++;
        else
            gx_offset = gx_offset - mean_gx / (GYRO_CALIBRATION_DEADZONE + 1);

        if (abs(mean_gy) <= GYRO_CALIBRATION_DEADZONE)
            ready++;
        else
            gy_offset = gy_offset - mean_gy / (GYRO_CALIBRATION_DEADZONE + 1);

        if (abs(mean_gz) <= GYRO_CALIBRATION_DEADZONE)
            ready++;
        else
            gz_offset = gz_offset - mean_gz / (GYRO_CALIBRATION_DEADZONE + 1);

        if (ready == 6)
            break;
    }

#ifdef VERBOSE_CALIBRATION
    Serial.println();
    Serial.print("OFFSETS    ");
    Serial.print(ax_offset);
    Serial.print("    ");
    Serial.print(ay_offset);
    Serial.print("    ");
    Serial.print(az_offset);
    Serial.print("    ");
    Serial.print(gx_offset);
    Serial.print("    ");
    Serial.print(gy_offset);
    Serial.print("    ");
    Serial.print(gz_offset);
    Serial.println();

    delay(5000);
#endif
}

void accelgyroMeanSensors(long *mean_ax, long *mean_ay, long *mean_az, long *mean_gx, long *mean_gy, long *mean_gz)
{

    int16_t ax_temp, ay_temp, az_temp;
    int16_t gx_temp, gy_temp, gz_temp;

    long buff_ax, buff_ay, buff_az;
    long buff_gx, buff_gy, buff_gz;

    int i = 0;
    while (i < (ACCELGYRO_CALIBRATION_BUFFERSIZE + 101))
    {
        accelgyro.getMotion6(&ax_temp, &ay_temp, &az_temp, &gx_temp, &gy_temp, &gz_temp);

        if (i > 100 && i <= (ACCELGYRO_CALIBRATION_BUFFERSIZE + 100))
        { //First 100 measures are discarded
            buff_ax = buff_ax + ax_temp;
            buff_ay = buff_ay + ay_temp;
            buff_az = buff_az + az_temp;
            buff_gx = buff_gx + gx_temp;
            buff_gy = buff_gy + gy_temp;
            buff_gz = buff_gz + gz_temp;
        }
        if (i == (ACCELGYRO_CALIBRATION_BUFFERSIZE + 100))
        {
            *mean_ax = buff_ax / ACCELGYRO_CALIBRATION_BUFFERSIZE;
            *mean_ay = buff_ay / ACCELGYRO_CALIBRATION_BUFFERSIZE;
            *mean_az = buff_az / ACCELGYRO_CALIBRATION_BUFFERSIZE;
            *mean_gx = buff_gx / ACCELGYRO_CALIBRATION_BUFFERSIZE;
            *mean_gy = buff_gy / ACCELGYRO_CALIBRATION_BUFFERSIZE;
            *mean_gz = buff_gz / ACCELGYRO_CALIBRATION_BUFFERSIZE;
        }
        i++;
        delay(2); //Needed so we don't get repeated measures
    }
}
#pragma endregion
