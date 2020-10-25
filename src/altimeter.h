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
*/
enum OperatingMode
{
    NotReady,
    Standby,
    Ready,
    Flight,
    Error
};

OperatingMode currentMode;

#pragma region Mode_LED
#define LED_RED_PIN 3
#define LED_GREEN_PIN 5
#define LED_BLUE_PIN 6
#pragma endregion

#pragma region Altimeter
Adafruit_BMP280 bmp;

float bmpSeaLevel;

float maxAltitude;
float altitude;
#pragma endregion

#pragma region SD_Cardreader
#define SD_CHIP_SELECT 4

File logFile;
#pragma endregion
// In meters
#define PARACHUTE_ALTITUDE_TRIGGER_THRESHOLD 5

void changeOperatingMode(OperatingMode mode);
void bmpCalibrate();

void setup()
{
    currentMode = OperatingMode::NotReady;

    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_BLUE_PIN, OUTPUT);
    
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_BLUE_PIN, HIGH);

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
    }

    if (!SD.begin(SD_CHIP_SELECT))
    {
        changeOperatingMode(OperatingMode::Error);
    }*/

    Serial.println("Init complete");

    // After succesful setup, go into standby.
    changeOperatingMode(OperatingMode::Standby);
}

void loop()
{
    // TODO: Check for button presses to change mode.
    Serial.println(currentMode);

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
