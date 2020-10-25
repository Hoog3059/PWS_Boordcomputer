#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include "Wire.h"

#pragma region AccelGyro
#define ACCELGYRO_SCL A5
#define ACCELGYRO_SDA A4

MPU6050 accelgyro;

#define GRAVITY 9.806

/* Accel range
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 */
#define ACCEL_RANGE 0
#define GYRO_RANGE 0
#define ACCELGYRO_CALIBRATION_BUFFERSIZE 1000
#define ACCEL_CALIBRATION_DEADZONE 8
#define GYRO_CALIBRATION_DEADZONE 1
#define ACCEL_X_AXIS_DOWN
//#define ACCEL_Y_AXIS_DOWN
//#define ACCEL_Z_AXIS_DOWN
#define ACCELGYRO_NO_CALIBRATION

// Variables to store accelgyro data in.
int16_t ax, ay, az;
int16_t gx, gy, gz;

long buff_ax, buff_ay, buff_az;
long buff_gx, buff_gy, buff_gz;

long mean_ax, mean_ay, mean_az;
long mean_gx, mean_gy, mean_gz;

uint16_t ax_offset, ay_offset, az_offset;
uint16_t gx_offset, gy_offset, gz_offset;
#pragma endregion

int16_t velocity;
int16_t displacement;

unsigned long time;
unsigned long previousTime;

void calibrateAccelGyro();
void accelgyroMeanSensors();

void setup()
{
    Wire.begin();
    Serial.begin(38400);
    accelgyro.initialize();

    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

#ifdef ACCELGYRO_NO_CALIBRATION
    accelgyro.setXAccelOffset(62521);
    accelgyro.setYAccelOffset(1551);
    accelgyro.setZAccelOffset(1474);
    accelgyro.setXGyroOffset(69);
    accelgyro.setYGyroOffset(41);
    accelgyro.setZGyroOffset(65456);
#else
    accelgyro.setFullScaleAccelRange(0);
    accelgyro.setFullScaleGyroRange(0);

    calibrateAccelGyro();

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

    accelgyro.setFullScaleAccelRange(ACCEL_RANGE);
    accelgyro.setFullScaleGyroRange(GYRO_RANGE);

    previousTime = millis();
}

void loop()
{
    time = millis();
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Acceleration in 10^-3 m/s^2
    ax = int16_t(float(ax) / (16348.0 / pow(2, ACCEL_RANGE)) * GRAVITY * 1000.0);
    ay = int16_t(float(ay) / (16348.0 / pow(2, ACCEL_RANGE)) * GRAVITY * 1000.0);
    az = int16_t(float(az) / (16348.0 / pow(2, ACCEL_RANGE)) * GRAVITY * 1000.0);

    // Angular velocity in 10^-3 °/s
    gx = int16_t(float(gx) / (131.0 / pow(2, GYRO_RANGE)) * 1000.0);
    gy = int16_t(float(gy) / (131.0 / pow(2, GYRO_RANGE)) * 1000.0);
    gz = int16_t(float(gz) / (131.0 / pow(2, GYRO_RANGE)) * 1000.0);

    // Correct for gravity
    ax = int16_t(ax - GRAVITY * 1000);

    velocity = int16_t(velocity + (ax / 1000.0) * (time - previousTime) / 1000.0);
    displacement = int16_t(displacement + velocity * (time - previousTime) / 1000.0);

    previousTime = time;

    Serial.print(velocity);
    Serial.print(" m/s    ");
    Serial.print(displacement);
    Serial.println(" m");

    /*Serial.print("a/g:    ");
    Serial.print(ax);
    Serial.print("    ");
    Serial.print(ay);
    Serial.print("    ");
    Serial.print(az);
    Serial.print("    ");
    Serial.print(gx);
    Serial.print("    ");
    Serial.print(gy);
    Serial.print("    ");
    Serial.println(gz);*/
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

    accelgyroMeanSensors();

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
        accelgyroMeanSensors();

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
}

void accelgyroMeanSensors()
{
    int i = 0;
    buff_ax = 0;
    buff_ay = 0;
    buff_az = 0;
    buff_gx = 0;
    buff_gy = 0;
    buff_gz = 0;

    while (i < (ACCELGYRO_CALIBRATION_BUFFERSIZE + 101))
    {
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        if (i > 100 && i <= (ACCELGYRO_CALIBRATION_BUFFERSIZE + 100))
        { //First 100 measures are discarded
            buff_ax = buff_ax + ax;
            buff_ay = buff_ay + ay;
            buff_az = buff_az + az;
            buff_gx = buff_gx + gx;
            buff_gy = buff_gy + gy;
            buff_gz = buff_gz + gz;
        }
        if (i == (ACCELGYRO_CALIBRATION_BUFFERSIZE + 100))
        {
            mean_ax = buff_ax / ACCELGYRO_CALIBRATION_BUFFERSIZE;
            mean_ay = buff_ay / ACCELGYRO_CALIBRATION_BUFFERSIZE;
            mean_az = buff_az / ACCELGYRO_CALIBRATION_BUFFERSIZE;
            mean_gx = buff_gx / ACCELGYRO_CALIBRATION_BUFFERSIZE;
            mean_gy = buff_gy / ACCELGYRO_CALIBRATION_BUFFERSIZE;
            mean_gz = buff_gz / ACCELGYRO_CALIBRATION_BUFFERSIZE;
        }
        i++;
        delay(2); //Needed so we don't get repeated measures
    }
}
#pragma endregion
