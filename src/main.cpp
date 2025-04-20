#include <Arduino.h>
#include "elrs.h"
#include "imu.h"
#include "config.h"

void setup()
{
    Serial.begin(115200);

    xTaskCreatePinnedToCore(
        crsfTask,        // task function
        "CRSF Task",     // name
        2048,            // stack size
        NULL,            // params
        1,               // priority
        &crsfTaskHandle, // task handle
        1                // core (1 = App Core)
    );
    xTaskCreatePinnedToCore(
        imuPIDTask,      // task function
        "IMU PID Task",  // name
        2048,            // stack size
        NULL,            // params
        1,               // priority
        NULL,            // task handle (not used)
        1                // core (1 = App Core)
    );
    xTaskCreatePinnedToCore(
        wifiTask,        // task function
        "WiFi Task",     // name
        8192,            // stack size
        NULL,            // params
        1,               // priority
        NULL,            // task handle (not used)
        0                // core (0 = Pro CPU)
    );
}

void loop()
{
}

// rc channel[2] is throttle
// rc channel[3] is yaw
// rc channel[1] is pitch
// rc channel[0] is roll
// rc channel[4] is arm
// rc channel[8] is calibration