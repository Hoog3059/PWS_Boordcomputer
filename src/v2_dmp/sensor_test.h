#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>                      // https://github.com/arduino-libraries/Servo
#include <I2Cdev.h>                     // https://github.com/jrowberg/i2cdevlib
#include <MPU6050_6Axis_MotionApps20.h> // https://github.com/jrowberg/i2cdevlib
#include <Adafruit_BMP280.h>            // https://github.com/adafruit/Adafruit_BMP280_Library

//#include "accelgyro_calibration.h"

#pragma region LED_AND_BUTTON
/** Operating mode status LED pins **/
#define LED_RED_PIN 7
#define LED_GREEN_PIN 5
#define LED_BLUE_PIN 4

/** Pushbutton **/
#define PUSHBUTTON_PIN 2
#pragma endregion

#pragma region ALTIMETER
/** Barometric altimeter **/
Adafruit_BMP280 bmp;

float bmpSeaLevel; // in hPa

int altitude;
#pragma endregion

#pragma region ACCELGYRO
/** Accelgyro */
MPU6050 accelgyro;

uint16_t accelgyroPacketSize;
uint16_t accelgyroFIFOCount;
uint8_t accelgyroFIFOBuffer[64];

Quaternion accelgyroQuaternion;
VectorFloat accelgyroGravity;

VectorInt16 accelgyroRawAcceleration;  // Raw sensor reading.
VectorInt16 accelgyroRealAcceleration; // Gravity removed.
VectorInt16 accelgyroAcceleration;     // Acceleration with respect to the world.

float accelgyroYawPitchRoll[3];

/* Accel range
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 */
#define ACCEL_RANGE 0
/* Gyro range
 * 0 = +/- 250 °/sec
 * 1 = +/- 500 °/sec
 * 2 = +/- 1000 °/sec
 * 3 = +/- 2000 °/sec
 */
#define GYRO_RANGE 0

/** Defines which axis forms a right angle with the ground */
//#define ACCEL_X_AXIS_DOWN
//#define ACCEL_Y_AXIS_DOWN
#define ACCEL_Z_AXIS_DOWN

unsigned long previousTime;
#pragma endregion

#pragma region SD_CARD
/** SD-Card reader **/
#define SD_CHIP_SELECT_PIN 10

File logFile;
#pragma endregion

#pragma region SERVO
/** Parachute trigger **/
#define PARACHUTE_SERVO_PIN 9

Servo parachuteServo;
#pragma endregion

/** Launch trigger thresholds **/
// In meters
// #define LAUNCH_ALTITUDE_TRIGGER_THRESHOLD 5
// In cm/s^2
#define LAUNCH_ACCELERATION_TRIGGER_THRESHOLD 750

/** Function declarations **/
void changeStatusLed(int red, int green, int blue);
void fetchSensorData();
void accelgyroReadSensors();
void bmpCalibrate();

void setup()
{
    Wire.begin();
    Wire.setClock(400000);
    Wire.setWireTimeout(2500);

    Serial.begin(115200);

    // Pin initialisation.
    pinMode(PUSHBUTTON_PIN, INPUT);

    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_BLUE_PIN, OUTPUT);

    changeStatusLed(HIGH, LOW, HIGH);

    // AccelGyro initialisation
    Serial.println("Initialising MPU6050.");
    delay(10);

    accelgyro.initialize();

    if (!accelgyro.testConnection())
    {
        Serial.println("MPU6050 Connection error.");
    }
    delay(1000);
    Serial.println("Initialising DMP.");
    uint8_t dmpStatus = accelgyro.dmpInitialize();

    Serial.println("Calibrating accelerometer.");

    //accelgyroCalibration(accelgyro);
    accelgyro.setXAccelOffset(-3007);
    accelgyro.setYAccelOffset(1600);
    accelgyro.setZAccelOffset(1683);
    accelgyro.setXGyroOffset(-33);
    accelgyro.setYGyroOffset(96);
    accelgyro.setZGyroOffset(-66);

    accelgyro.PrintActiveOffsets();

    accelgyro.setFullScaleAccelRange(ACCEL_RANGE);
    accelgyro.setFullScaleGyroRange(GYRO_RANGE);

    if (dmpStatus == 0)
    {
        Serial.println("Enabling DMP.");
        accelgyro.setDMPEnabled(true);
        accelgyroPacketSize = accelgyro.dmpGetFIFOPacketSize();
    }
    else
    {
        Serial.println("DMP error");
        Serial.println(dmpStatus);
    }

    Serial.println("Initialising servo.");

    // Servo initialisation
    parachuteServo.attach(PARACHUTE_SERVO_PIN);
    parachuteServo.write(170);

    Serial.println("Initialising barometer.");

    // Barometer initialisation
    if (!bmp.begin(0x76))
    {
        Serial.println("Baro error");
    }

    Serial.println("Initialising SD-cardreader");

    // SD initialisation
    if (!SD.begin(SD_CHIP_SELECT_PIN))
    {
        Serial.println("SD error");
    }

    Serial.println("Starting sensor test.");

    previousTime = millis();
}

void loop()
{
    // Check for button press. Change current mode.
    if (digitalRead(PUSHBUTTON_PIN))
    {
        changeStatusLed(LOW, HIGH, LOW);
    }
    else
    {
        changeStatusLed(LOW, LOW, HIGH);
    }

    fetchSensorData();

    Serial.print(millis() - previousTime);
    Serial.print("    ms    ");
    Serial.print(accelgyroAcceleration.x);
    Serial.print("    ");
    Serial.print(accelgyroAcceleration.y);
    Serial.print("    ");
    Serial.print(accelgyroAcceleration.z);
    Serial.print("    mm/s^2    ");
    Serial.print(accelgyroYawPitchRoll[0]);
    Serial.print("    ");
    Serial.print(accelgyroYawPitchRoll[1]);
    Serial.print("    ");
    Serial.print(accelgyroYawPitchRoll[2]);
    Serial.print("    deg    ");
    Serial.print(bmp.getStatus());
    Serial.print("  (bmpstatus)    ");
    Serial.print(bmp.readPressure());
    Serial.print("    Pa    ");
    /*
    if (accelgyroAcceleration.z >= LAUNCH_ACCELERATION_TRIGGER_THRESHOLD)
    {
        Serial.print("## LAUNCH DETECTED ##");
    }
    */
    Serial.println();
    previousTime = millis();
}

void changeStatusLed(int red, int green, int blue)
{
    digitalWrite(LED_RED_PIN, red);
    digitalWrite(LED_GREEN_PIN, green);
    digitalWrite(LED_BLUE_PIN, blue);
}

void fetchSensorData()
{
    // Barometer
    altitude = bmp.readAltitude(bmpSeaLevel);
    // Accelgyro
    accelgyroReadSensors();
}

void accelgyroReadSensors()
{
    accelgyro.dmpGetCurrentFIFOPacket(accelgyroFIFOBuffer);

    accelgyro.dmpGetQuaternion(&accelgyroQuaternion, accelgyroFIFOBuffer);
    accelgyro.dmpGetGravity(&accelgyroGravity, &accelgyroQuaternion);

    accelgyro.dmpGetAccel(&accelgyroRawAcceleration, accelgyroFIFOBuffer);
    accelgyro.dmpGetLinearAccel(&accelgyroRealAcceleration, &accelgyroRawAcceleration, &accelgyroGravity);
    accelgyro.dmpGetLinearAccelInWorld(&accelgyroAcceleration, &accelgyroRealAcceleration, &accelgyroQuaternion);

    accelgyro.dmpGetYawPitchRoll(accelgyroYawPitchRoll, &accelgyroQuaternion, &accelgyroGravity);
}

// Calibration for the BMP280
void bmpCalibrate()
{
    bmpSeaLevel = bmp.readPressure() / 100;
}
