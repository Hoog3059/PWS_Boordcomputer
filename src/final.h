#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h> // https://github.com/arduino-libraries/Servo
#include <I2Cdev.h> // https://github.com/jrowberg/i2cdevlib
#include <MPU6050.h> // https://github.com/pkourany/I2CDEV_MPU6050
#include <Adafruit_BMP280.h> // https://github.com/adafruit/Adafruit_BMP280_Library

/** Defines different operating modes for the flightcomputer.
 * - NotReady     : The flightcomputer is initializing and is not ready to record flight.
 * - Standby      : The flightcomputer is prepared to start recording.
 * - Ready        : The flightcomputer is recording and waiting for launch.
 * - Flight       : The flightcomputer has recorded launch and is now logging and waiting for apogee.
 * - Error        : An error has occurred and the flightcomputer has stopped recording.
*/
enum OperatingMode
{
    NotReady,
    Standby,
    Ready,
    Flight,
    Error
};

OperatingMode currentMode = OperatingMode::NotReady;

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

// In meters
float maxAltitude = 0;
float altitude;
#pragma endregion

#pragma region ACCELGYRO
/** Accelgyro */
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

/** Defines which axis forms a right angle with the ground */
#define ACCEL_X_AXIS_DOWN
//#define ACCEL_Y_AXIS_DOWN
//#define ACCEL_Z_AXIS_DOWN

// Variables to store accelgyro data in.
int16_t ax, ay, az;
int16_t gx, gy, gz;

/** Velocity in m/s */
double velocity = 0;
double maxVelocity = 0;
/** Displacement in m */
double displacement = 0;
/** Rotation from 90° from the ground */
double rotation = 0;

unsigned long time;
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

bool parachuteDeployed = false;
#pragma endregion

#pragma region TRIGGERS
/** Launch trigger thresholds **/
// In meters
#define LAUNCH_ALTITUDE_TRIGGER_THRESHOLD 5
// In m/s^2
#define LAUNCH_ACCELERATION_TRIGGER_THRESHOLD 2

/** Parachute deploy trigger tresholds */
// In meters
#define PARACHUTE_ALTITUDE_TRIGGER_THRESHOLD 5
// In m/s
#define PARACHUTE_VELOCITY_TRIGGER_THRESHOLD 1
#pragma endregion

/** Mission time **/
unsigned long startTime;

/** Function declarations **/
void changeOperatingMode(OperatingMode mode);
void changeStatusLed(int red, int green, int blue);
void takeSensorReadings();
void AccelGyroMeanReadings();
void bmpCalibrate();

void setup()
{
    // Pin initialisation
    pinMode(PUSHBUTTON_PIN, INPUT);

    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_BLUE_PIN, OUTPUT);

    changeStatusLed(HIGH, LOW, HIGH);

    // AccelGyro initialisation
    Wire.begin();

    accelgyro.initialize();

    if (!accelgyro.testConnection())
    {
        changeOperatingMode(OperatingMode::Error);
    }

    accelgyro.setXAccelOffset(-3013);
    accelgyro.setYAccelOffset(1537);
    accelgyro.setZAccelOffset(1530);
    accelgyro.setXGyroOffset(66);
    accelgyro.setYGyroOffset(35);
    accelgyro.setZGyroOffset(-74);

    accelgyro.setFullScaleAccelRange(ACCEL_RANGE);
    accelgyro.setFullScaleGyroRange(GYRO_RANGE);

    // Servo initialisation
    parachuteServo.attach(PARACHUTE_SERVO_PIN);
    parachuteServo.write(10);

    // Barometer initialisation
    if (!bmp.begin(0x76))
    {
        changeOperatingMode(OperatingMode::Error);
    }

    // SD initialisation
    if (!SD.begin(SD_CHIP_SELECT_PIN))
    {
        changeOperatingMode(OperatingMode::Error);
    }

    if (SD.exists("ROCKET_1.txt"))
    {
        SD.remove("ROCKET_1.txt");
    }

    logFile = SD.open("ROCKET_1.txt", FILE_WRITE);

    if (!logFile)
    {
        changeOperatingMode(OperatingMode::Error);
    }

    // After succesful setup, go into standby.
    changeOperatingMode(OperatingMode::Standby);
}

void loop()
{
    // Check for button press. Change current mode.
    if (digitalRead(PUSHBUTTON_PIN))
    {
        changeStatusLed(LOW, LOW, LOW);

        delay(1000);

        switch (currentMode)
        {
        case Standby:
            changeOperatingMode(OperatingMode::Ready);
            break;
        case Ready:
            changeOperatingMode(OperatingMode::Standby);
            break;
        case Flight:
            changeOperatingMode(OperatingMode::Standby);
            break;
        }
    }

    // Routines for different operating modes.
    switch (currentMode)
    {
    case Standby:
        // Process
        break;
    case Ready:
    case Flight:
        takeSensorReadings();

        time = millis();

        // In m/s
        velocity = velocity + ax * double(time - previousTime) / 1000.0;
        // In m
        // displacement = displacement + velocity * double(time - previousTime) / 1000.0;

        previousTime = time;

        // Log
        // Format:  time;ax;ay;az;gx;gy;gz;velocity;barAlt
        logFile.print(millis() - startTime);
        logFile.print(";");
        logFile.print(ax);
        logFile.print(";");
        logFile.print(ay);
        logFile.print(";");
        logFile.print(az);
        logFile.print(";");
        logFile.print(gx);
        logFile.print(";");
        logFile.print(gy);
        logFile.print(";");
        logFile.print(gz);
        logFile.print(";");
        logFile.print(velocity);
        logFile.print(";");
        logFile.println(altitude);

        if (currentMode == OperatingMode::Ready)
        {
            // Check if launch detected, otherwise break;
            if (altitude > LAUNCH_ALTITUDE_TRIGGER_THRESHOLD || ax >= LAUNCH_ACCELERATION_TRIGGER_THRESHOLD)
            {
                changeOperatingMode(OperatingMode::Flight);
            }
            else
            {
                break;
            }
        }

        if (altitude > maxAltitude)
        {
            maxAltitude = altitude;
        }

        if (velocity > maxVelocity)
        {
            maxVelocity = velocity;
        }

        // Parachute triggers
        if (!parachuteDeployed)
        {
            if (velocity < maxVelocity && velocity < PARACHUTE_VELOCITY_TRIGGER_THRESHOLD)
            {
                parachuteServo.write(170);
                parachuteDeployed = true;
            }

            if (maxAltitude - altitude >= PARACHUTE_ALTITUDE_TRIGGER_THRESHOLD)
            {
                parachuteServo.write(170);
                parachuteDeployed = true;
            }
        }
        break;
    case Error:
        changeOperatingMode(OperatingMode::Error);
        break;
    default:
        changeOperatingMode(OperatingMode::Standby);
        break;
    }
}

// Routines for every operating mode change.
void changeOperatingMode(OperatingMode mode)
{
    currentMode = mode;
    switch (mode)
    {
    case Standby:
        changeStatusLed(LOW, LOW, HIGH);

        logFile.close();

        bmp.setSampling(Adafruit_BMP280::MODE_SLEEP,
                        Adafruit_BMP280::SAMPLING_NONE,
                        Adafruit_BMP280::SAMPLING_NONE,
                        Adafruit_BMP280::FILTER_X16,
                        Adafruit_BMP280::STANDBY_MS_1000);
        break;
    case Ready:
        logFile = SD.open("ROCKET_1.txt", FILE_WRITE);

        if (!logFile)
        {
            changeOperatingMode(OperatingMode::Error);
        }

        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                        Adafruit_BMP280::SAMPLING_NONE,
                        Adafruit_BMP280::SAMPLING_X16,
                        Adafruit_BMP280::FILTER_X16,
                        Adafruit_BMP280::STANDBY_MS_1);

        delay(1000);

        bmpCalibrate();

        changeStatusLed(LOW, HIGH, LOW);
        startTime = millis();
        break;
    case Flight:
        changeStatusLed(LOW, LOW, LOW);
        break;
    case Error:
        changeStatusLed(HIGH, LOW, LOW);
        while (true)
            ;
    }
}

void changeStatusLed(int red, int green, int blue)
{
    digitalWrite(LED_RED_PIN, red);
    digitalWrite(LED_GREEN_PIN, green);
    digitalWrite(LED_BLUE_PIN, blue);
}

void takeSensorReadings()
{
    // Barometer
    altitude = bmp.readAltitude(bmpSeaLevel);

    // Accelgyro
    AccelGyroMeanReadings();

    // Correct for gravity
    ax = int16_t(ax - GRAVITY);
}

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
}

// Calibration for the BMP280
void bmpCalibrate()
{
    bmpSeaLevel = bmp.readPressure() / 100;
}
