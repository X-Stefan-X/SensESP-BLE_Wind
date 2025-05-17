#include "CalypsoBLE.h"
#include <ArduinoBLE.h>

#include "esp_task_wdt.h"
#include "sensesp.h"

#ifndef CALYPSO_NAME 
#define CALYPSO_NAME "Calypso"
#endif

#ifndef CALYPSO_ADDR 
#define CALYPSO_ADDR "00:00:00:00:00:00"
#endif

#ifndef CALYPSO_ECOMPASS
#define CALYPSO_ECOMPASS false
#endif

//#define MS_TO_KNOT 1.943844492

using namespace SensESP;


CalypsoBLE::CalypsoBLE() {
    CalypsoData.windSpeed = 0;
    CalypsoData.windDirection = 0;
    CalypsoData.BatteryLevel = 0;
    CalypsoData.Temperature = 0;
    CalypsoData.Humidity = 0;
    CalypsoData.Pressure = 0;
    CalypsoData.Roll = 0;
    CalypsoData.Pitch = 0;
    CalypsoData.Compass = 0;
    
    DataRate = 0;
}

void CalypsoBLE::Connect() {
   int count2reset = 0;
    while (!BLECalypso.peripheral.connect()) {
        // Watchdog Reload
        esp_task_wdt_reset();
        debugE("Failed to connect to peripheral, retrying...");
        delay(500);
        if (count2reset > 10) {
            debugE("Failed to connect to peripheral, resetting...");
            esp_restart();
        }
    }
    debugI("Connected to peripheral");
    BLECalypso.peripheral.discoverAttributes();

    BLEService service_180d = BLECalypso.peripheral.service("180D");

    BLECalypso.BLECharacteristicCalypso_2a39 = service_180d.characteristic("2a39");
    BLECalypso.BLECharacteristicCalypso_2a39.subscribe();

    BLECalypso.BLECharacteristicCalypso_a003 = service_180d.characteristic("a003");
    #ifdef CALYPSO_ECOMPASS
        BLECalypso.BLECharacteristicCalypso_a003.writeValue((byte)0x00);
    #else
        BLECalypso.BLECharacteristicCalypso_a003.writeValue((byte)0x01);
    #endif
    BLECalypso.BLECharacteristicCalypso_a002 = service_180d.characteristic("a002");
    BLECalypso.BLECharacteristicCalypso_a002.readValue(DataRate);
    debugI("Connected with Data Rate: %d", DataRate);
}


void CalypsoBLE::extractData(const unsigned char data[], int length)
{
    // Extract data from the received data array
    for (int i = 0; i < length; i++)
    {
        unsigned char byte = data[i];
    }

    CalypsoData.windSpeed = (float)(data[1] * 256 + data[0]) / 100.0;
    CalypsoData.windDirection = (int)(data[3] * 256 + data[2]);
    CalypsoData.BatteryLevel = (int)(data[4] * 10);
    CalypsoData.Temperature = (int)(data[5] - 100);
    //CalypsoData.Humidity = (int)(data[6] * 256 + data[7]);
    //CalypsoData.Pressure = (int)(data[10] * 256 + data[11]);
    
    #ifdef CALYPSO_ECOMPASS
        CalypsoData.Roll = (int)(data[6] - 90);
        CalypsoData.Pitch = (int)(data[7] - 90);
        CalypsoData.Compass = 360 - (data[9] * 256 + data[8]);
        if (CalypsoData.Compass == 360) 
            {
                CalypsoData.Compass = 0;
            }
    #endif
    
   
}

void CalypsoBLE::init() {
    
    if (!BLE.begin()) {
        debugE("starting BLE failed!");
        while (1);  
    }
    debugI("BLE started - scanning for peripherals...");
    BLE.scan();
    char findBLE = 0;
    while (findBLE == 0) {
        // Watchdog Reload
        esp_task_wdt_reset();
        BLECalypso.peripheral = BLE.available();

        if (BLECalypso.peripheral) {
            debugI("BLE peripheral found");
            debugI("Name: %s, Address: %s", BLECalypso.peripheral.localName().c_str(), BLECalypso.peripheral.address().c_str());
            if (BLECalypso.peripheral.localName() == CALYPSO_NAME && BLECalypso.peripheral.address() == CALYPSO_ADDR) {
                debugI("Found Calypso");
                BLE.stopScan();
                CalypsoBLE::Connect();
                findBLE = 1;
            }
        }
    }
    debugI("Calypso connected");
}

    void CalypsoBLE::continuousRead()
    {
        // Watchdog Reload
        esp_task_wdt_reset();
        if (BLECalypso.peripheral.connected()) {
            if (BLECalypso.BLECharacteristicCalypso_2a39.valueUpdated()) {
                BLECalypso.BLECharacteristicCalypso_2a39.read();
                extractData(BLECalypso.BLECharacteristicCalypso_2a39.value(), BLECalypso.BLECharacteristicCalypso_2a39.valueLength());

                // Data Transmission
                debugI("Data Transmission");

            }
        }
        else {
            debugE("Peripheral disconnected");
            CalypsoBLE::Connect();
        }

        if (millis() > BLECalypso.Calypso_DataRate_refresh_time) {
            DataRate = 0;

            BLECalypso.BLECharacteristicCalypso_a002.readValue(DataRate);

            if (CalypsoData.BatteryLevel > 30 && DataRate <= 4) {
                BLECalypso.BLECharacteristicCalypso_a002.writeValue((byte)0x08);
                debugI("Battery Level High - 8 Hz");
            }
            else if (CalypsoData.BatteryLevel <= 30 && DataRate == 8) {
                BLECalypso.BLECharacteristicCalypso_a002.writeValue((byte)0x04);
                debugI("Battery Level Medium - 4Hz");
            }

            BLECalypso.BLECharacteristicCalypso_a002.readValue(DataRate);
            debugI("Data Rate: %d", DataRate);

            BLECalypso.Calypso_DataRate_refresh_time = millis() + BLECalypso.CALYPSO_DATARATE_REFRESH_TIME;
            debugI("Calypso Data Rate refresh time set to %d", BLECalypso.Calypso_DataRate_refresh_time);
        }
    }