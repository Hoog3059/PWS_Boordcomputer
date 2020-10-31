#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

/** Barometric altimeter options **/
Adafruit_BMP280 bmp;

float bmpSeaLevel;

float maxAltitude;
float altitude;

void setup()
{
    Serial.begin(115200);

    if (bmp.begin(0x76))
    {
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                        Adafruit_BMP280::SAMPLING_X1,
                        Adafruit_BMP280::SAMPLING_X16,
                        Adafruit_BMP280::FILTER_X16,
                        Adafruit_BMP280::STANDBY_MS_1);
    }

    delay(2000);

    bmpSeaLevel = bmp.readPressure();
}

void loop()
{
    Serial.print(bmp.readAltitude(bmpSeaLevel / 100)); /* Adjusted to local forecast! */
    Serial.println(" m");
}
