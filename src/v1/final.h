#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>           // https://github.com/arduino-libraries/Servo
#include <I2Cdev.h>          // https://github.com/jrowberg/i2cdevlib
#include <MPU6050.h>         // https://github.com/jrowberg/i2cdevlib
#include <Adafruit_BMP280.h> // https://github.com/adafruit/Adafruit_BMP280_Library
#include "flight_settings.h"

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
// LED Pins
#define LED_RED_PIN 7
#define LED_GREEN_PIN 5
#define LED_BLUE_PIN 4

// Pushbutton pin
#define PUSHBUTTON_PIN 2
#pragma endregion

#pragma region ALTIMETER
// Barometric altimeter
Adafruit_BMP280 bmp;

float bmpSeaLevel; // in hPa

// In meters
float maxAltitude = 0;
float altitude;
#pragma endregion

#pragma region ACCELGYRO
// Accelgyro
MPU6050 accelgyro;

// In m/s^2
#define GRAVITY 9.806

// Variables to store accelgyro data in.
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Vertical acceleration (pointer).
int16_t *av;

/** Velocity in cm/s */
double velocity = 0;
double maxVelocity = 0;

// Δt used for integration.
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
// Some triggers are commented out because these are disabled.
// They can be re-enabled by uncommenting the triggers and question and the checks in question on line

/** Launch trigger thresholds **/
// In meters
// #define LAUNCH_ALTITUDE_TRIGGER_THRESHOLD 5
// In cm/s^2
#define LAUNCH_ACCELERATION_TRIGGER_THRESHOLD 750

/** Parachute deploy trigger tresholds */
// In meters
// #define PARACHUTE_ALTITUDE_TRIGGER_THRESHOLD 5
// In cm/s
#define PARACHUTE_VELOCITY_TRIGGER_THRESHOLD 100
#pragma endregion

/** Mission time (in ms from boot of Arduino) - used for documenting t **/
unsigned long startTime;

/** Function declarations **/
void changeOperatingMode(OperatingMode mode);
void changeStatusLed(int red, int green, int blue);
void fetchSensorData();
void accelgyroReadSensors();
void bmpCalibrate();

void setup()
{
    // Pin initialisation
    pinMode(PUSHBUTTON_PIN, INPUT);

    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_BLUE_PIN, OUTPUT);

    changeStatusLed(HIGH, LOW, HIGH); // Purple

    Wire.begin();
    Wire.setWireTimeout(2500);

    // AccelGyro initialisation
    accelgyro.initialize();

    if (!accelgyro.testConnection())
    {
        changeOperatingMode(OperatingMode::Error);
    }

    // AccelGyro calibration
    accelgyro.setXAccelOffset(-3222);
    accelgyro.setYAccelOffset(1581);
    accelgyro.setZAccelOffset(1349);
    accelgyro.setXGyroOffset(-9);
    accelgyro.setYGyroOffset(114);
    accelgyro.setZGyroOffset(-51);

    accelgyro.setFullScaleAccelRange(ACCEL_RANGE);
    accelgyro.setFullScaleGyroRange(GYRO_RANGE);

    // Define vertical acceleration vector.
#ifdef ACCEL_X_AXIS_DOWN
    av = &ax;
#elif defined(ACCEL_Y_AXIS_DOWN)
    av = &ay;
#elif defined(ACCEL_Z_AXIS_DOWN)
    av = &az;
#endif

    // Servo initialisation
    parachuteServo.attach(PARACHUTE_SERVO_PIN);
    parachuteServo.write(170);

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

    /** Defines whether flight computer should assume an accidental reset has occured.
     * If true, recovery sequence shall be initiated on boot. */
    bool recoverAfterReset = true;

    // Check if file exists.
    // If yes, check to see if recovery should be initiated.
    // If no, boot normally.
    if (SD.exists("ROCKET_1.txt"))
    {
        changeStatusLed(HIGH, HIGH, LOW); // Yellow

        int counter = 0;

        // Check if pushbutton is being held for 2000ms.
        // If yes, delete file and boot normally.
        // If no, initiate recovery sequence.
        while (digitalRead(PUSHBUTTON_PIN))
        {
            counter += 1;
            delay(10);

            // If button is held for long enough, delete file.
            if (counter >= 200)
            {
                changeStatusLed(HIGH, LOW, HIGH); // Purple
                delay(2000);
                recoverAfterReset = false;
                SD.remove("ROCKET_1.txt");
                break;
            }
        }
    }
    else
    {
        recoverAfterReset = false;
    }

    logFile = SD.open("ROCKET_1.txt", FILE_WRITE);

    if (!logFile)
    {
        changeOperatingMode(OperatingMode::Error);
    }

    if (recoverAfterReset)
    {
        // Recovery sequence
        logFile.println("######## RECOVER AFTER RESET ########");
        logFile.close();

        changeOperatingMode(OperatingMode::Ready);
        changeOperatingMode(OperatingMode::Flight);
    }
    else
    {
        // Normal boot

        // Data headings
        logFile.println("t;ax;ay;az;gx;gy;gz;v;h;accelStatus;baroStatus");
        logFile.println("ms;cm/s^2;cm/s^2;cm/s^2;deg/s;deg/s;deg/s;cm/s;m;;");

        // After succesful setup, go into standby.
        changeOperatingMode(OperatingMode::Standby);
    }
}

void loop()
{
    int counter = 0;
    // If button is held for 1000ms, change mode.
    while (digitalRead(PUSHBUTTON_PIN))
    {
        counter += 1;
        delay(1);

        if (counter > 1000)
        {
            changeStatusLed(HIGH, LOW, HIGH); // Purple

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
            break;
        }
    }

    // Routines for different operating modes.
    switch (currentMode)
    {
    case Standby:
        // Do nothing
        break;
    case Ready:
    case Flight:
        fetchSensorData();

        time = millis();

        if (currentMode == OperatingMode::Flight)
        {
            // In cm/s
            velocity = velocity + (*av) * double(time - previousTime) / 1000.0;
        }
        else if (currentMode == OperatingMode::Ready)
        {
            // Check if launch detected
            // Disabled check: if (altitude > LAUNCH_ALTITUDE_TRIGGER_THRESHOLD || (*av) >= LAUNCH_ACCELERATION_TRIGGER_THRESHOLD)
            if ((*av) >= LAUNCH_ACCELERATION_TRIGGER_THRESHOLD)
            {
                logFile.println("######## LAUNCH DETECTED ########");
                changeOperatingMode(OperatingMode::Flight);
                velocity = velocity + (*av) * double(time - previousTime) / 1000.0;
            }
        }

        previousTime = time;

        // Log
        // Format:  t;ax;ay;az;gx;gy;gz;v;h;accelStatus;baroStatus
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
        logFile.print(altitude);
        logFile.print(";");
        logFile.print(accelgyro.testConnection() ? "1" : "0");
        logFile.print(";");
        logFile.println(bmp.getStatus() == 12 || bmp.getStatus() == 4 ? "1" : "0");

        logFile.flush();

        // Code below this line is only for "Flight" mode, so break if not in "Flight" mode.
        if (currentMode == OperatingMode::Ready)
            break;

        // Set max altitude measured.
        if (altitude > maxAltitude)
        {
            maxAltitude = altitude;
        }

        // Set max vertical velocity measured.
        if (velocity > maxVelocity)
        {
            maxVelocity = velocity;
        }

        // Parachute triggers
        if (!parachuteDeployed)
        {
            if (velocity < maxVelocity && velocity < PARACHUTE_VELOCITY_TRIGGER_THRESHOLD)
            {
                parachuteServo.write(10);
                parachuteDeployed = true;

                logFile.println("######## PARACHUTE DEPLOYED ########");
            }

            // Allows for parachute trigger on altitude drop measured by barometer.
            // Uncomment to enable.
            /*            
            if (maxAltitude - altitude >= PARACHUTE_ALTITUDE_TRIGGER_THRESHOLD)
            {
                parachuteServo.write(10);
                parachuteDeployed = true;

                logFile.println("######## PARACHUTE DEPLOYED ########");
            }
            */
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
        changeStatusLed(LOW, LOW, HIGH); // Blue

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

        bmpCalibrate();

        changeStatusLed(LOW, HIGH, LOW); // Green
        startTime = millis();
        break;
    case Flight:
        changeStatusLed(LOW, LOW, LOW); // Off
        break;
    case Error:
        changeStatusLed(HIGH, LOW, LOW); // Red
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

void fetchSensorData()
{
    // Barometer
    int barometerStatus = bmp.getStatus();
    // If barometerStatus is not 12 or 4, then the barometer has had an error.
    // The following code tries to restart the barometer.
    if (barometerStatus != 12 && barometerStatus != 4)
    {
        if (bmp.begin(0x76))
        {
            bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                            Adafruit_BMP280::SAMPLING_NONE,
                            Adafruit_BMP280::SAMPLING_X16,
                            Adafruit_BMP280::FILTER_X16,
                            Adafruit_BMP280::STANDBY_MS_1);
        }
    }
    altitude = bmp.readAltitude(bmpSeaLevel);

    // Accelgyro
    accelgyroReadSensors();

    // Correct for gravity
    (*av) = int16_t((*av) - (GRAVITY * 100));
}

void accelgyroReadSensors()
{
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    /** Convert raw acceleration to cm/s^2 **/
    ax = int16_t(ax / (16384.0 / pow(2, ACCEL_RANGE)) * GRAVITY * 100);
    ay = int16_t(ay / (16384.0 / pow(2, ACCEL_RANGE)) * GRAVITY * 100);
    az = int16_t(az / (16384.0 / pow(2, ACCEL_RANGE)) * GRAVITY * 100);

    // Angular velocity in °/s
    gx = int16_t(gx / (131.0 / pow(2, GYRO_RANGE)));
    gy = int16_t(gy / (131.0 / pow(2, GYRO_RANGE)));
    gz = int16_t(gz / (131.0 / pow(2, GYRO_RANGE)));
}

// Calibration for the BMP280
void bmpCalibrate()
{
    bmpSeaLevel = bmp.readPressure() / 100;
}
