#ifndef CALYPSOBLE_H
#define CALYPSOBLE_H

#include <ArduinoBLE.h>
//  arduino-libraries/ArduinoBLE @ ^1.4.0
#include "sensesp.h"


namespace sensesp {

typedef struct Calypso_Data {
    float windSpeed;
    int windDirection;
    float windAngle;
    int BatteryLevel;
    int Temperature;
    int Humidity;
    int Pressure;
    int Roll;
    int Pitch;
    int Compass;
    long lastUpdateAtMillis = 0; // Last update time in milliseconds
} Calypso_Data;

typedef struct Calypso_BLE {
    BLEDevice peripheral;
    
    // Calypso Update refresh time millisecond
    const long CALYPSO_DATARATE_REFRESH_TIME = (30*1000);  // 30s
    long Calypso_DataRate_refresh_time = 0;
   

    // WindSpeed...
    // Principal Characteristic:  Notify
    // Characteristic:            UUID: 0x2A39
    BLECharacteristic BLECharacteristicCalypso_2a39;

    // Data Rate Characteristic: Read/Write
    // wind speed output data rate - 1hz,4hz,8hz
    // UUID: 0xA002
    // 0x01-> 1Hz
    // 0x04-> 4Hz   >>> Default
    // 0x08-> 8Hz
    BLECharacteristic BLECharacteristicCalypso_a002;

    // Sensors Characteristic: Read/Write
    // Activate Clinometer and eCompass
    // UUID: 0xA003
    // 0x01-> ON
    // 0x00-> OFF   >>> Default
    BLECharacteristic BLECharacteristicCalypso_a003;
} Calypso_BLE;



class CalypsoBLE {
    private:
        Calypso_BLE BLECalypso;
        byte DataRate = 0;
        void setZeroCalypsoData();

    public:
        Calypso_Data CalypsoData;
        CalypsoBLE();
        bool Calypso_Enable_Compass = false; // Enable eCompass

        void init();
        void Connect();

        void extractData(const unsigned char data[], int length);

        void continuousRead();

};
} // namespace SensESP

#endif