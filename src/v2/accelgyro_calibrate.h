/* ======== MODIFICATIONS MADE BY TIMO HOOGENBOSCH =======
 Certain aspects of this code have been modified and customised to work with our project. The original code
 can be found at https://github.com/Protonerd/DIYino/blob/master/MPU6050_calibration.ino.

 Modifications that have been made:
 - Include Arduino.h (required for use with PlatformIO IDE).
 - Define functions averageSensorReading() and calibration() at the top of the code.
 - Add code to define the "down"-axis to account for gravity properly.
 - Simplified unneccesary code.
*/
#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Wire.h>
#include "flight_settings.h"

int buffersize = 1000;      // Number of readings to average, make it higher to get more precision but sketch will be slower (default:1000).
int discardfirstmeas = 100; // Number of initial measurements to be discarded
int acel_deadzone = 10;     // Accelerometer error allowed, make it lower to get more precision, but sketch may not converge (default:8).
int giro_deadzone = 10;     // Giro error allowed, make it lower to get more precision, but sketch may not converge (default:1).

MPU6050 accelgyro;

int16_t ax, ay, az, gx, gy, gz;

int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

unsigned int loopcount = 0;

void averageSensorReading();
bool accelgyroCalibration();

void setup()
{
    Wire.begin();
    Wire.setClock(400000);
    Wire.setWireTimeout(2500);

    Serial.begin(115200);

    // Initialize accelgyro
    accelgyro.initialize();

    Serial.println("\nMPU6050 Calibration Sketch");
    delay(2000);
#ifdef ACCEL_X_AXIS_DOWN
    Serial.println("\nCurrent mode: ACCEL_X_AXIS_DOWN");
#elif defined(ACCEL_Y_AXIS_DOWN)
    Serial.println("\nCurrent mode: ACCEL_Y_AXIS_DOWN");
#elif defined(ACCEL_Z_AXIS_DOWN)
    Serial.println("\nCurrent mode: ACCEL_Z_AXIS_DOWN");
#endif
    Serial.println("\nDon't touch MPU6050 until you see a finish message.\n");
    delay(3000);

    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    delay(1000);

    Serial.print("Current offsets: ");
    Serial.print(accelgyro.getXAccelOffset());
    Serial.print("    ");
    Serial.print(accelgyro.getYAccelOffset());
    Serial.print("    ");
    Serial.print(accelgyro.getZAccelOffset());
    Serial.print("    ");
    Serial.print(accelgyro.getXGyroOffset());
    Serial.print("    ");
    Serial.print(accelgyro.getYGyroOffset());
    Serial.print("    ");
    Serial.print(accelgyro.getZGyroOffset());
    Serial.println();
    delay(1000);

    accelgyro.setXAccelOffset(0);
    accelgyro.setYAccelOffset(0);
    accelgyro.setZAccelOffset(0);
    accelgyro.setXGyroOffset(0);
    accelgyro.setYGyroOffset(0);
    accelgyro.setZGyroOffset(0);

    accelgyro.setFullScaleGyroRange(0);
    accelgyro.setFullScaleAccelRange(0);
}

void loop()
{
    Serial.println("\nReading sensors for first time...");
    averageSensorReading();

    Serial.println("\nCalculating offsets...");
    if (accelgyroCalibration())
    {
        Serial.println("\nCalibration successful!");        
    }
    else
    {
        Serial.println("\nCalibration failed!");
        Serial.println("Please reset.");
        while(1){
            delay(1000);
        }
    }
    delay(1000);

    averageSensorReading();

    Serial.println("\nFINISHED!");
    Serial.print("\nSensor readings with offsets:  ");
    Serial.print(mean_ax);
    Serial.print("    ");
    Serial.print(mean_ay);
    Serial.print("    ");
    Serial.print(mean_az);
    Serial.print("    ");
    Serial.print(mean_gx);
    Serial.print("    ");
    Serial.print(mean_gy);
    Serial.print("    ");
    Serial.println(mean_gz);
    Serial.print("Your offsets:  ");
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
    Serial.println(gz_offset);
    Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
    Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
    Serial.print("accelgyro.setXAccelOffset(");
    Serial.print(ax_offset);
    Serial.println(");");
    Serial.print("accelgyro.setYAccelOffset(");
    Serial.print(ay_offset);
    Serial.println(");");
    Serial.print("accelgyro.setZAccelOffset(");
    Serial.print(az_offset);
    Serial.println(");");
    Serial.print("accelgyro.setXGyroOffset(");
    Serial.print(gx_offset);
    Serial.println(");");
    Serial.print("accelgyro.setYGyroOffset(");
    Serial.print(gy_offset);
    Serial.println(");");
    Serial.print("accelgyro.setZGyroOffset(");
    Serial.print(gz_offset);
    Serial.println(");");
    accelgyro.PrintActiveOffsets();
}

void averageSensorReading()
{
    long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

    while (i < (buffersize + discardfirstmeas + 1))
    {
        // Read raw accel/gyro data.
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        //First 100 measures are discarded
        if (i > discardfirstmeas && i <= (buffersize + discardfirstmeas))
        {
            buff_ax = buff_ax + ax;
            buff_ay = buff_ay + ay;
            buff_az = buff_az + az;
            buff_gx = buff_gx + gx;
            buff_gy = buff_gy + gy;
            buff_gz = buff_gz + gz;
        }
        if (i == (buffersize + 100))
        {
            mean_ax = buff_ax / buffersize;
            mean_ay = buff_ay / buffersize;
            mean_az = buff_az / buffersize;
            mean_gx = buff_gx / buffersize;
            mean_gy = buff_gy / buffersize;
            mean_gz = buff_gz / buffersize;
        }
        i++;
        delay(2); //Needed so we don't get repeated measures.
    }

    Serial.print("Results of measurements a/g:  ");
    Serial.print(mean_ax);
    Serial.print("    ");
    Serial.print(mean_ay);
    Serial.print("    ");
    Serial.print(mean_az);
    Serial.print("    ");
    Serial.print(mean_gx);
    Serial.print("    ");
    Serial.print(mean_gy);
    Serial.print("    ");
    Serial.println(mean_gz);
}

bool accelgyroCalibration()
{
    ax_offset = -mean_ax / 8;
    ay_offset = -mean_ay / 8;
    az_offset = -mean_az / 8;

#ifdef ACCEL_X_AXIS_DOWN
    ax_offset = (int(16384 / pow(2, ACCEL_RANGE)) - mean_ax) / 8;
#elif defined(ACCEL_Y_AXIS_DOWN)
    ay_offset = (int(16384 / pow(2, ACCEL_RANGE)) - mean_ay) / 8;
#elif defined(ACCEL_Z_AXIS_DOWN)
    az_offset = (int(16384 / pow(2, ACCEL_RANGE)) - mean_az) / 8;
#endif

    gx_offset = -mean_gx / 4;
    gy_offset = -mean_gy / 4;
    gz_offset = -mean_gz / 4;
    while (1)
    {
        int ready = 0;
        accelgyro.setXAccelOffset(ax_offset);
        accelgyro.setYAccelOffset(ay_offset);
        accelgyro.setZAccelOffset(az_offset);

        accelgyro.setXGyroOffset(gx_offset);
        accelgyro.setYGyroOffset(gy_offset);
        accelgyro.setZGyroOffset(gz_offset);

        averageSensorReading();
        Serial.println("...");

#ifdef ACCEL_X_AXIS_DOWN
        if (abs(int(16384 / pow(2, ACCEL_RANGE)) - mean_ax) <= acel_deadzone)
            ready++;
        else
            ax_offset = ax_offset + (int(16384 / pow(2, ACCEL_RANGE)) - mean_ax) / acel_deadzone;

        if (abs(mean_ay) <= acel_deadzone)
            ready++;
        else
            ay_offset = ay_offset - mean_ay / acel_deadzone;

        if (abs(mean_az) <= acel_deadzone)
            ready++;
        else
            az_offset = az_offset - mean_az / acel_deadzone;
#elif defined(ACCEL_Y_AXIS_DOWN)
        if (abs(mean_ax) <= acel_deadzone)
            ready++;
        else
            ax_offset = ax_offset - mean_ax / acel_deadzone;

        if (abs(int(16384 / pow(2, ACCEL_RANGE)) - mean_ay) <= acel_deadzone)
            ready++;
        else
            ay_offset = ay_offset + (int(16384 / pow(2, ACCEL_RANGE)) - mean_ay) / acel_deadzone;

        if (abs(mean_az) <= acel_deadzone)
            ready++;
        else
            az_offset = az_offset - mean_az / acel_deadzone;
#elif defined(ACCEL_Z_AXIS_DOWN)
        if (abs(mean_ax) <= acel_deadzone)
            ready++;
        else
            ax_offset = ax_offset - mean_ax / acel_deadzone;

        if (abs(mean_ay) <= acel_deadzone)
            ready++;
        else
            ay_offset = ay_offset - mean_ay / acel_deadzone;

        if (abs(int(16384 / pow(2, ACCEL_RANGE)) - mean_az) <= acel_deadzone)
            ready++;
        else
            az_offset = az_offset + (int(16384 / pow(2, ACCEL_RANGE)) - mean_az) / acel_deadzone;
#endif

        if (abs(mean_gx) <= giro_deadzone)
            ready++;
        else
            gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

        if (abs(mean_gy) <= giro_deadzone)
            ready++;
        else
            gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

        if (abs(mean_gz) <= giro_deadzone)
            ready++;
        else
            gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);

        if (ready == 6)
        {
            return true;
            break;
        }

        Serial.print("Resulting offset calibration value a/g:  ");
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
        Serial.println(gz_offset);
        loopcount = loopcount + 1;
        Serial.print("Loop Cnt: ");
        Serial.println(loopcount);
        if (loopcount == 20)
        {
            return false;
            break; // Exit the calibration routine if no stable results can be obtained after 20 calibration loops
        }
    }
}