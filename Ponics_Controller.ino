//------------------------ Includes ------------------------
#include <Arduino.h>
#include <esp_sleep.h>
#include <Preferences.h>
#include "Zigbee.h"

//------------------------ GPIO Definition ------------------------
#define INPUT_GPIO    5     // input
#define INPUT_GPIO    5     // input
#define INPUT_GPIO    5     // input
#define INPUT_GPIO    5     // input
#define INPUT_GPIO    5     // input
#define OUTPUT_GPIO_1    5     // input
#define BATTERY_ADC_PIN     0     // Battery voltage measurement
#define BOOT_PIN            9     // Button for Zigbee reset
#define REFERENCE_PIN       4     // Control grnd voltage


//------------------------ Device parameters ------------------------
#define ANALOG_DEVICE_ENDPOINT_NUMBER      1
#define MODEL_NAME          "HYDROPONICS"
#define MANUFACTURER_NAME   "JIMBO"

//------------------------ Endpoints ------------------------
ZigbeeAnalog zbAnalogDevice = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER);
ZigbeeAnalog zbAnalogTemp = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER + 1);
ZigbeeAnalog zbAnalogFan = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER + 2);
ZigbeeAnalog zbAnalogPercent = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER + 3);

//// 'Clusters' that need to be set up in ZigbeeAnalog:
//      -- Input x1: Dht10b water temp sensor
//      -- Input x2: Aht10 air temp and humidity sensor
//      -- Input x1: TDS sensor
//      -- Input x1: pH sensor
//      -- Output x1: PWM fan 1
//      -- Output x1: PWM fan 2
//      -- Output x1: PWM LEDs
//      -- Output x1: Relay controlled water pump
//      -- Output x1: Relay controlled air pump
//      -- Output x1: Relay controlled water heater
//      -- Output x1: Relay controlled dosing pump 1
//      -- Output x1: Relay controlled dosing pump 2
//      -- Output x1: Relay controlled dosing pump 3

//------------------------ Output Callback ------------------------
void onAnalogOutputChange(float analog_output) {
  Serial.printf("Received analog output change: %.1f\r\n", analog_output);
}

//------------------------ Setup ------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(INPUT_GPIO, INPUT);
  pinMode(BOOT_PIN, INPUT_PULLUP);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  /////////////////// Fan A ------------------------
  // Set up analog input
  zbAnalogFan.addAnalogInput();
  zbAnalogFan.setAnalogInputApplication(ESP_ZB_ZCL_AI_RPM_OTHER);
  zbAnalogFan.setAnalogInputDescription("RPM");
  zbAnalogFan.setAnalogInputResolution(1);

  // Set up analog output
  zbAnalogDevice.addAnalogOutput();
  zbAnalogDevice.setAnalogOutputApplication(ESP_ZB_ZCL_AI_RPM_OTHER);
  zbAnalogDevice.setAnalogOutputDescription("Fan Speed (RPM)");

  /////////////////// Fan B ------------------------
  // Set up analog input
  zbAnalogFan.addAnalogInput();
  zbAnalogFan.setAnalogInputApplication(ESP_ZB_ZCL_AI_RPM_OTHER);
  zbAnalogFan.setAnalogInputDescription("RPM");
  zbAnalogFan.setAnalogInputResolution(1);

  // Set up analog output
  zbAnalogDevice.addAnalogOutput();
  zbAnalogDevice.setAnalogOutputApplication(ESP_ZB_ZCL_AI_RPM_OTHER);
  zbAnalogDevice.setAnalogOutputDescription("Fan Speed (RPM)");

  /////////////////// LEDs ------------------------
  // Set up analog input
  zbAnalogFan.addAnalogInput();
  zbAnalogFan.setAnalogInputApplication(ESP_ZB_ZCL_AI_PERCENTAGE_OTHER));
  zbAnalogFan.setAnalogInputDescription("LED Brightness");
  zbAnalogFan.setAnalogInputResolution(1);

  // Set up analog output
  zbAnalogDevice.addAnalogOutput();
  zbAnalogDevice.setAnalogOutputApplication(ESP_ZB_ZCL_AI_PERCENTAGE_OTHER));
  zbAnalogDevice.setAnalogOutputDescription("LED Brightness");

  /////////////////// DHT10b ------------------------
  // Set up analog input
  zbAnalogTemp.addAnalogInput();
  zbAnalogTemp.setAnalogInputApplication(ESP_ZB_ZCL_AI_PERCENTAGE_OTHER));
  zbAnalogTemp.setAnalogInputDescription("Temperature");
  zbAnalogTemp.setAnalogInputResolution(0.1);

  /////////////////// AHT10 ------------------------
  // Set up analog input
  zbAnalogTemp.addAnalogInput();
  zbAnalogTemp.setAnalogInputApplication(ESP_ZB_ZCL_AI_TEMPERATURE_OTHER);
  zbAnalogTemp.setAnalogInputDescription("Temperature");
  zbAnalogTemp.setAnalogInputResolution(0.1);

  // Set up analog input
  zbAnalogPercent.addAnalogInput();
  zbAnalogPercent.setAnalogInputApplication(ESP_ZB_ZCL_AI_PERCENTAGE_OTHER);
  zbAnalogPercent.setAnalogInputDescription("Humidity Percentage");
  zbAnalogPercent.setAnalogInputResolution(0.01);


  
  _ep.setManufacturerAndModel(MANUFACTURER_NAME, MODEL_NAME);
  _ep.setPowerSource(ZB_POWER_SOURCE_MAINS, 75);
  Zigbee.addEndpoint(&_ep);

  Serial.println("Zigbee initializing...");
  // When all EPs are registered, start Zigbee in End Device mode
  if (!Zigbee.begin()) {
    Serial.println("Zigbee failed to start!");
    Serial.println("Rebooting...");
    ESP.restart();
  } else {
    Serial.println("Zigbee started successfully!");
  }
  Serial.println("Connecting to network");
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }
  _ep.setReporting(0, 300, 1.0);
  Serial.println();
}

//------------------------ Main Loop ------------------------
void loop() {
  static uint32_t timeCounter = 0;

  // Report periodically
  // Read ADC value and update the analog value every 2s
  if (!(timeCounter++ % 20)) {  // delaying for 100ms x 20 = 2s
    float analog = (float)analogRead(analogPin);
    Serial.printf("Updating analog input to %.1f\r\n", analog);
    zbAnalogDevice.setAnalogInput(analog);
    zbAnalogTemp.setAnalogInput(analog / 100);
    zbAnalogFan.setAnalogInput(analog);
    zbAnalogPercent.setAnalogInput(analog / 10);

    // Analog input supports reporting
    zbAnalogDevice.reportAnalogInput();
    zbAnalogTemp.reportAnalogInput();
    zbAnalogFan.reportAnalogInput();
    zbAnalogPercent.reportAnalogInput();
  }


  // Zigbee factory reset via long-press
  if (digitalRead(BOOT_PIN) == LOW) {
    delay(50);
    unsigned long t0 = millis();
    while (digitalRead(BOOT_PIN) == LOW) {
      if (millis() - t0 > 3000) {
        Serial.println("Factory resetting Zigbee...");
        Zigbee.factoryReset();
        break;
      }
      delay(50);
    }
  }

  delay(1000);
}
