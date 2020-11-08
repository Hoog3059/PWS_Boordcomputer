#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP280.h>

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

/** Operating mode status LED pins **/
#define LED_RED_PIN 3
#define LED_GREEN_PIN 5
#define LED_BLUE_PIN 6

/** Pushbutton **/
#define PUSHBUTTON_PIN 2

/** Barometric altimeter **/
Adafruit_BMP280 bmp;

float bmpSeaLevel; // in hPa

// In meters
float maxAltitude = 0;
float altitude;

/** SD-Card reader **/
#define SD_CHIP_SELECT_PIN 4

File logFile;

/** Parachute trigger **/
#define PARACHUTE_SERVO_PIN 9

Servo parachuteServo;

/** Trigger thresholds **/
// In meters
#define LAUNCH_ALTITUDE_TRIGGER_THRESHOLD 5
#define PARACHUTE_ALTITUDE_TRIGGER_THRESHOLD 5

/** Mission time **/
unsigned long startTime;

/** Function declarations **/
void changeOperatingMode(OperatingMode mode);
void changeStatusLed(int red, int green, int blue);
void bmpCalibrate();

void setup()
{
    pinMode(PUSHBUTTON_PIN, INPUT);

    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_BLUE_PIN, OUTPUT);

    changeStatusLed(HIGH, LOW, HIGH);

    parachuteServo.attach(PARACHUTE_SERVO_PIN);
    parachuteServo.write(10);

    if (!bmp.begin(0x76))
    {
        changeOperatingMode(OperatingMode::Error);
    }

    if (SD.begin(SD_CHIP_SELECT_PIN))
    {
        if (SD.exists("ROCKET_1.txt"))
        {
            SD.remove("ROCKET_1.txt");
        }

        logFile = SD.open("ROCKET_1.txt", FILE_WRITE);

        if (!logFile)
        {
            changeOperatingMode(OperatingMode::Error);
        }
    }
    else
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
            changeOperatingMode(OperatingMode::Flight);
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
        // Wait for launch
        if(altitude > LAUNCH_ALTITUDE_TRIGGER_THRESHOLD){
            changeOperatingMode(OperatingMode::Flight);
        }
        break;
    case Flight:
        altitude = bmp.readAltitude(bmpSeaLevel);
        if (altitude >= maxAltitude)
        {
            maxAltitude = altitude;
        }
        else if (maxAltitude - altitude >= PARACHUTE_ALTITUDE_TRIGGER_THRESHOLD)
        {
            parachuteServo.write(170);
        }

        // time;barometricAltitude
        // TODO: Log altitude
        logFile.print(millis() - startTime);
        logFile.print(";");
        logFile.println(altitude);
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
        break;
    case Flight:
        changeStatusLed(LOW, LOW, LOW);

        startTime = millis();
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

// Calibration for the BMP280
void bmpCalibrate()
{
    bmpSeaLevel = bmp.readPressure() / 100;
}
