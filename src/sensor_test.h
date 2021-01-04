#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>           // https://github.com/arduino-libraries/Servo
#include <I2Cdev.h>          // https://github.com/jrowberg/i2cdevlib
#include <MPU6050.h>         // https://github.com/pkourany/I2CDEV_MPU6050
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

// In m/s^2
#define GRAVITY 9.806

/* Accel range
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 */
#define ACCEL_RANGE 3
/* Gyro range
 * 0 = +/- 250 °/sec
 * 1 = +/- 500 °/sec
 * 2 = +/- 1000 °/sec
 * 3 = +/- 2000 °/sec
 */
#define GYRO_RANGE 3

/** Number of measurements to mean for more accurate measurement. */
#define ACCELGYRO_SAMPLE_MEAN 20

/** Defines which axis forms a right angle with the ground */
//#define ACCEL_X_AXIS_DOWN
//#define ACCEL_Y_AXIS_DOWN
#define ACCEL_Z_AXIS_DOWN

// Variables to store accelgyro data in.
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Vertical acceleration (pointer).
int16_t *av;

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

/** Function declarations **/
void changeOperatingMode(OperatingMode mode);
void changeStatusLed(int red, int green, int blue);
void takeSensorReadings();
void AccelGyroReadSensors();
void bmpCalibrate();

void setup()
{
    Serial.begin(115200);

    // Pin initialisation
    pinMode(PUSHBUTTON_PIN, INPUT);

    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_BLUE_PIN, OUTPUT);

    changeStatusLed(HIGH, LOW, HIGH);

    Wire.begin();
    Wire.setWireTimeout(2500);

    // AccelGyro initialisation
    Serial.println("1");
    delay(10);

    accelgyro.initialize();

    Serial.println("2");

    if (!accelgyro.testConnection())
    {
        changeOperatingMode(OperatingMode::Error);
        Serial.println("Accel error");
    }

    Serial.println("3");

    // AccelGyro calibration
    accelgyro.setXAccelOffset(-2771);
    accelgyro.setYAccelOffset(1622);
    accelgyro.setZAccelOffset(1453);
    accelgyro.setXGyroOffset(48);
    accelgyro.setYGyroOffset(22);
    accelgyro.setZGyroOffset(-70);

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

    Serial.println("4");

    // Barometer initialisation
    if (!bmp.begin(0x76))
    {
        changeOperatingMode(OperatingMode::Error);
        Serial.println("Baro error");
    }

    Serial.println("5");

    // SD initialisation
    if (!SD.begin(SD_CHIP_SELECT_PIN))
    {
        changeOperatingMode(OperatingMode::Error);
        Serial.println("SD error");
    }

    Serial.println("6");

    // After succesful setup, go into standby.
    changeOperatingMode(OperatingMode::Standby);
}

void loop()
{
    // Check for button press. Change current mode.
    if (digitalRead(PUSHBUTTON_PIN))
    {
        changeStatusLed(LOW, HIGH, LOW);
    }else{
        changeStatusLed(LOW, LOW, HIGH);
    }
    
    takeSensorReadings();

    Serial.print(millis() - previousTime);
    Serial.print("   ");
    Serial.print(ax);
    Serial.print("   ");
    Serial.print(ay);
    Serial.print("   ");
    Serial.print(az);
    Serial.print("   cm/s^2   ");
    Serial.print(gx);
    Serial.print("   ");
    Serial.print(gy);
    Serial.print("   ");
    Serial.print(gz);
    Serial.print("   deg/s   ");
    Serial.print(bmp.getStatus());
    Serial.print("      ");
    Serial.print(bmp.readPressure());
    Serial.println("   Pa   ");    
    previousTime = millis();
}

// Routines for every operating mode change.
void changeOperatingMode(OperatingMode mode)
{
    switch (mode)
    {
    case Error:
        changeStatusLed(HIGH, LOW, LOW);
        while (true)
            ;
    default:
        break;
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
    int barometerStatus = bmp.getStatus();
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
    AccelGyroReadSensors();

    // Correct for gravity
    (*av) = int16_t((*av) - (GRAVITY * 100));
}

void AccelGyroReadSensors()
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

// Calibration for the BMP280
void bmpCalibrate()
{
    bmpSeaLevel = bmp.readPressure() / 100;
}
