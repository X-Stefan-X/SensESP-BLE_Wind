#include "CalypsoBLE.h"
#include <ArduinoBLE.h>

#include "esp_task_wdt.h"
#include "sensesp.h"

#ifndef CALYPSO_NAME 
#define CALYPSO_NAME "ULTRASONIC"
#endif

#ifndef CALYPSO_ADDR 
#define CALYPSO_ADDR "d7:f6:cd:3d:f4:14"
#endif



//#define MS_TO_KNOT 1.943844492

namespace sensesp {

    esp_task_wdt_config_t wdt_config = {
            .timeout_ms = 30000, // 30 seconds
            .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, // All cores
            .trigger_panic = false
        };

    
    void CalypsoBLE::setZeroCalypsoData() {
        CalypsoData.windSpeed = 0.0f;
        CalypsoData.windDirection = 0;
        CalypsoData.windAngle = 0.0f;
        CalypsoData.BatteryLevel = 0;
        CalypsoData.Temperature = 0;
        CalypsoData.Humidity = 0;
        CalypsoData.Pressure = 0.;
        CalypsoData.Roll = 0;
        CalypsoData.Pitch = 0;
        CalypsoData.Compass = 0;
        CalypsoData.lastUpdateAtMillis = 0; // Initialize last update time
        DataRate = 0;
    }
    CalypsoBLE::CalypsoBLE() {
        // Initialize the BLECalypso structure
        this->setZeroCalypsoData();
    }
    void CalypsoBLE::Connect() {
        esp_task_wdt_reset();
        debugI("Connecting to peripheral: %s", BLECalypso.peripheral.localName().c_str());
        while (!BLECalypso.peripheral.connect()) {
            int count2reset = 0;
            // Watchdog Reload
            esp_task_wdt_reset();
            debugE("Failed to connect to peripheral, retrying...");
            delay(500);
            count2reset++;
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
        if (this->Calypso_Enable_Compass) {
            BLECalypso.BLECharacteristicCalypso_a003.writeValue((byte)0x00);
        } else {
            BLECalypso.BLECharacteristicCalypso_a003.writeValue((byte)0x01);
        }
        BLECalypso.BLECharacteristicCalypso_a002 = service_180d.characteristic("a002");
        BLECalypso.BLECharacteristicCalypso_a002.readValue(DataRate);
        debugI("Connected with Data Rate: %d", DataRate);
    }


    void CalypsoBLE::extractData(const unsigned char data[], int length)
    {
        CalypsoData.windSpeed = (float)(data[1] * 256 + data[0]) / 100.0;
        CalypsoData.windDirection = (int)(data[3] * 256 + data[2]);
        CalypsoData.windAngle = ((float)(data[3] * 256 + data[2])) * DEG_TO_RAD;
        CalypsoData.BatteryLevel = (int)(data[4] * 10);
        debugI("Wind Speed: %.2f m/s, Wind Direction: %d degrees, Battery Level: %d %%", CalypsoData.windSpeed, CalypsoData.windDirection, CalypsoData.BatteryLevel);
        //CalypsoData.Temperature = (int)(data[5] - 100);
        //CalypsoData.Humidity = (int)(data[6] * 256 + data[7]);
        //CalypsoData.Pressure = (int)(data[10] * 256 + data[11]);
        
        if (this->Calypso_Enable_Compass)
            CalypsoData.Roll = (int)(data[6] - 90);
            CalypsoData.Pitch = (int)(data[7] - 90);
            debugI("Roll: %d degrees, Pitch: %d degrees", CalypsoData.Roll, CalypsoData.Pitch);
            CalypsoData.Compass = 360 - (data[9] * 256 + data[8]);
            if (CalypsoData.Compass == 360) 
            {
                CalypsoData.Compass = 0;
            }

        CalypsoData.lastUpdateAtMillis = millis(); // Update last update time
    }

    void CalypsoBLE::init() {
        esp_task_wdt_config_t wdt_config = {
            .timeout_ms = 30000, // 30 seconds
            .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, // All cores
            .trigger_panic = false
        };
        esp_task_wdt_init(&wdt_config);
        esp_task_wdt_add(NULL);
        if (!BLE.begin()) {
            debugE("starting BLE failed!");
            while (1);  
        }
        debugI("BLE started - scanning for peripherals...");
        BLE.scan();
        debugI("Scanning for Calypso...");
        char findBLE = 0;
        while (findBLE == 0) {
            // Watchdog Reload
            esp_task_wdt_reset();
            BLECalypso.peripheral = BLE.available();
            if (BLECalypso.peripheral) {
                debugI("BLE peripheral found with Name: %s, Address: %s", BLECalypso.peripheral.localName().c_str(), BLECalypso.peripheral.address().c_str());
                if (BLECalypso.peripheral.address() == CALYPSO_ADDR) { //BLECalypso.peripheral.localName() == CALYPSO_NAME &&
                    debugI("This is Calypso! Yeah!");
                    BLE.stopScan();
                    CalypsoBLE::Connect();
                    findBLE = 1;
                }
            }
        }
        debugI("Calypso connected");
        esp_task_wdt_delete(NULL); // Remove the watchdog for the main task
    }

    void CalypsoBLE::continuousRead()
    {
        esp_task_wdt_add(NULL);
        while (true) {
            // Watchdog Reload
            esp_task_wdt_reset();
            if (!BLECalypso.peripheral.connected()) {
                debugE("Peripheral disconnected");
                this->setZeroCalypsoData();
                this->Connect();
            }
            if (BLECalypso.BLECharacteristicCalypso_2a39.valueUpdated()) {
                BLECalypso.BLECharacteristicCalypso_2a39.read();
                extractData(BLECalypso.BLECharacteristicCalypso_2a39.value(), BLECalypso.BLECharacteristicCalypso_2a39.valueLength());
                // Data Transmission
                debugI("Data Transmission");
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
    }
} // namespace SensESP