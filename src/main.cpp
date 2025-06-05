/**
 * @file async_repeat_sensor.cpp
 * @brief Example of an asynchronous RepeatSensor that reads a digital input.
 *
 * RepeatSensor is a helper class that can be used to wrap any generic Arduino
 * sensor library into a SensESP sensor. This file demonstrates its use with
 * the built-in GPIO digital input, but in an asynchronous manner.
 *
 */

#include "sensesp/signalk/signalk_output.h"
#include "sensesp_app_builder.h"

#include "sensesp/sensors/constant_sensor.h"

#include "CalypsoBLE.h"

using namespace sensesp;

CalypsoBLE* calypso = new CalypsoBLE();
float read_wind_speed() {
      if (calypso->CalypsoData.lastUpdateAtMillis <= millis() - 10000) {
        // If no data has been received in the last second, return 0
        // If no data has been received yet, return 0
        return 0.0f;
      } else {
        return calypso->CalypsoData.windSpeed;
      }
    }
float read_wind_angle() {
      if (calypso->CalypsoData.lastUpdateAtMillis <= millis() - 10000) {
        return 0.0f;
      } else {
        return calypso->CalypsoData.windAngle;
      }
    }
float read_battery_level() {
      return calypso->CalypsoData.BatteryLevel;
    }

// The setup function performs one-time application initialization.
void setup() {
  SetupLogging();

  // Create the global SensESPApp() object.
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("sensesp-Calypso")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    //->set_sk_server("192.168.10.3", 80)
                    //->enable_system_info_sensors()
                    ->enable_ip_address_sensor()
                    ->enable_ota("ThisIsMyPassword!")
                    ->get_app();

  calypso->init();
  calypso->Calypso_Enable_Compass = false; // Disable eCompass

 // Read Calypso in Background
    xTaskCreate(
    [](void* pvParameters) {
      calypso->continuousRead();
    },
    "Calypso Read",
    10000,
    calypso,
    1,
    NULL);

    
    unsigned int calypso_read_interval = 200;

    // Create a RepeatSensor that reads the wind speed every 200 milliseconds
    auto* calypso_wind_speed = new RepeatSensor<float>(calypso_read_interval, read_wind_speed);
    const char* sk_wind_speed_path = "environment.wind.speedApparent";
    SKMetadata* sk_wind_speed_metadata = new SKMetadata();
    sk_wind_speed_metadata->display_name_ = "Apparent Wind Speed";
    sk_wind_speed_metadata->units_ = "m/s";
    sk_wind_speed_metadata->description_ = "Apparent Wind speed measured by Calypso";
    sk_wind_speed_metadata->short_name_ = "AWS";
    sk_wind_speed_metadata->timeout_ = 10.0; // 10 seconds
    calypso_wind_speed->connect_to(
      new SKOutputFloat(sk_wind_speed_path, "/Calypso/WindSpeed/", sk_wind_speed_metadata));

    // Create a RepeatSensor that reads the wind direction every 200 milliseconds
    auto* calypso_wind_direction = new RepeatSensor<float>(calypso_read_interval, read_wind_angle);
    const char* sk_wind_direction_path = "environment.wind.angleApparent";
    SKMetadata* sk_wind_direction_metadata = new SKMetadata();
    sk_wind_direction_metadata->display_name_ = "Apparent Wind Angle";
    sk_wind_direction_metadata->units_ = "rad";
    sk_wind_direction_metadata->description_ = "Apparent Wind Angle measured by Calypso";
    sk_wind_direction_metadata->short_name_ = "AWA";
    sk_wind_direction_metadata->timeout_ = 10.0; // 10 seconds
    calypso_wind_direction->connect_to(
      new SKOutputFloat(sk_wind_direction_path, "/Calypso/WindAngle/", sk_wind_direction_metadata));


    // Create a RepeatSensor that reads the battery level every 10 seconds
    auto* calypso_battery_level = new RepeatSensor<float>(10000, read_battery_level);
    const char* sk_battery_level_path = "electrical.batteries.calypso.capacity.stateOfCharge";
    SKMetadata* sk_battery_level_metadata = new SKMetadata();
    sk_battery_level_metadata->display_name_ = "Calypso Battery Level";
    sk_battery_level_metadata->units_ = "ratio";
    sk_battery_level_metadata->description_ = "Battery level of Calypso";
    sk_battery_level_metadata->short_name_ = "Battery";
    calypso_battery_level->connect_to(
      new SKOutputFloat(sk_battery_level_path, "/Calypso/BatteryLevel/", sk_battery_level_metadata));

    // Create a constant sensor for the battery name and location
    auto* battery_constant_sensor = new StringConstantSensor("Calypso Battery", 300000, "/Calypso/BatteryName/");
    auto battery_constant_sensor_output = new SKOutputString("electrical.batteries.99.name", "", 
                                                            new SKMetadata("string", "Calypso Battery"));
    battery_constant_sensor->connect_to(battery_constant_sensor_output);

      auto* battery_location_constant_sensor = new StringConstantSensor("Masttop", 300000, "/Calypso/BatteryLocation/");
    auto battery_location_constant_sensor_output = new SKOutputString("electrical.batteries.99.location", "", 
                                                            new SKMetadata("string", "Calypso Battery Location"));
    battery_location_constant_sensor->connect_to(battery_location_constant_sensor_output);

}

void loop() {
  event_loop()->tick();
}
