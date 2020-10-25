#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP280.h>

/** Defines different operating modes.
 * NotReady     - The flightcomputer is initializing and is not ready to record flight.
 * Standby      - The flightcomputer is prepared to start recording.
 * Ready        - The flightcomputer is recording and waiting for launch.
 * Flight       - The flightcomputer has recorded launch and is now logging and waiting for apogee.
 * Error        - An error has occurred and the flightcomputer has stopped recording.
**/
enum OperatingMode
{
    NotReady,
    Standby,
    Ready,
    Flight,
    Error
};

OperatingMode currentMode;

/** Operating mode status LED pins. **/
#define LED_RED_PIN 3
#define LED_GREEN_PIN 5
#define LED_BLUE_PIN 6

/** Button **/
#define PUSHBUTTON_PIN 2
#define PUSHBUTTON_MILLIS_THRESHOLD 100

unsigned int buttonStartTime;

int buttonValue = 0;
int previousButtonValue = 0;

/** Barometric altimeter: options **/
Adafruit_BMP280 bmp;

float bmpSeaLevel;

float maxAltitude;
float altitude;

/** SD-Card reader: options **/
#define SD_CHIP_SELECT_PIN 4

File logFile;

/** Parachute trigger: options **/
#define PARACHUTE_SERVO_PIN 9

Servo parachuteServo;

#define PARACHUTE_ALTITUDE_TRIGGER_THRESHOLD 5

/** Function declarations **/
void changeOperatingMode(OperatingMode mode);
void bmpCalibrate();

void setup()
{
    currentMode = OperatingMode::NotReady;

    pinMode(PUSHBUTTON_PIN, INPUT);

    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_BLUE_PIN, OUTPUT);

    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_BLUE_PIN, HIGH);

    parachuteServo.attach(PARACHUTE_SERVO_PIN);
    parachuteServo.write(100);

    /*
    if (bmp.begin())
    {
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                        Adafruit_BMP280::SAMPLING_X1,
                        Adafruit_BMP280::SAMPLING_X16,
                        Adafruit_BMP280::FILTER_X16,
                        Adafruit_BMP280::STANDBY_MS_1000);
    }
    else
    {
        changeOperatingMode(OperatingMode::Error);
    }*/

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
    buttonValue = digitalRead(PUSHBUTTON_PIN);
    if (buttonValue)
    {
        digitalWrite(LED_RED_PIN, LOW);
        digitalWrite(LED_GREEN_PIN, LOW);
        digitalWrite(LED_BLUE_PIN, LOW);

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
        default:
            break;
        }        
    }

    switch (currentMode)
    {
    case NotReady:
        // Process
        break;
    case Standby:
        // Process
        break;
    case Ready:
        // Wait for launch
        break;
    case Flight:
        altitude = bmp.readAltitude(bmpSeaLevel);
        if (altitude >= maxAltitude)
        {
            maxAltitude = altitude;
        }
        else if (maxAltitude - altitude >= PARACHUTE_ALTITUDE_TRIGGER_THRESHOLD)
        {
            // TODO: Trigger parachute
        }

        // TODO: Log altitude
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
    case NotReady:
        digitalWrite(LED_RED_PIN, HIGH);
        digitalWrite(LED_GREEN_PIN, LOW);
        digitalWrite(LED_BLUE_PIN, HIGH);
        /* TODO:
         * - Set altimeter to standby.
         * - Close file.
         */
        break;
    case Standby:
        digitalWrite(LED_RED_PIN, LOW);
        digitalWrite(LED_GREEN_PIN, LOW);
        digitalWrite(LED_BLUE_PIN, HIGH);
        /* TODO:
         * - Set altimeter to standby.
         * - Close file
         */
        break;
    case Ready:
        digitalWrite(LED_RED_PIN, LOW);
        digitalWrite(LED_GREEN_PIN, HIGH);
        digitalWrite(LED_BLUE_PIN, LOW);
        /* TODO:
         * - Set altimeter to high-sampling mode.
         * - Open file.
         */
        break;
    case Flight:
        digitalWrite(LED_RED_PIN, LOW);
        digitalWrite(LED_GREEN_PIN, LOW);
        digitalWrite(LED_BLUE_PIN, LOW);
        /* TODO:
         * - Set altimeter to high-sampling mode.
         */
        break;
    case Error:
        digitalWrite(LED_RED_PIN, HIGH);
        digitalWrite(LED_GREEN_PIN, LOW);
        digitalWrite(LED_BLUE_PIN, LOW);
        while (true)
            ;
    }
}

// Calibration for the BMP280
void bmpCalibrate()
{
    bmpSeaLevel = bmp.readPressure();
}
