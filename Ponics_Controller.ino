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
///////// Analog
ZigbeeAnalog zbAnalogDevice = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER);

ZigbeeAnalog zbAnalogFan_a = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER + 1);
ZigbeeAnalog zbAnalogFan_b = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER + 2);
ZigbeeAnalog zbAnalogLights = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER + 3);
ZigbeeAnalog zbAnalogTemp_aht10 = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER + 4);
ZigbeeAnalog zbAnalogHumidity_aht10 = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER + 5);
ZigbeeAnalog zbAnalogTemp_ds12b = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER + 6);
ZigbeeAnalog zbAnalogPh = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER + 13);
ZigbeeAnalog zbAnalogTds = ZigbeeAnalog(ANALOG_DEVICE_ENDPOINT_NUMBER + 14);

///////// Binary
ZigbeeBinary zbBinaryAirPump = ZigbeeBinary(ANALOG_DEVICE_ENDPOINT_NUMBER + 7);
ZigbeeBinary zbBinaryWaterPump = ZigbeeBinary(ANALOG_DEVICE_ENDPOINT_NUMBER + 8);
ZigbeeBinary zbBinaryWaterHeater = ZigbeeBinary(ANALOG_DEVICE_ENDPOINT_NUMBER + 9);
ZigbeeBinary zbBinaryDosePump_a = ZigbeeBinary(ANALOG_DEVICE_ENDPOINT_NUMBER + 10);
ZigbeeBinary zbBinaryDosePump_b = ZigbeeBinary(ANALOG_DEVICE_ENDPOINT_NUMBER + 11);
ZigbeeBinary zbBinaryDosePump_c = ZigbeeBinary(ANALOG_DEVICE_ENDPOINT_NUMBER + 12);



//// 'Clusters' that need to be set up in Zigbee:
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
  zbAnalogFan_a.addAnalogInput();
  zbAnalogFan_a.setAnalogInputApplication(ESP_ZB_ZCL_AI_RPM_OTHER);
  zbAnalogFan_a.setAnalogInputDescription("RPM");
  zbAnalogFan_a.setAnalogInputResolution(1);

  // Set up analog output
  zbAnalogFan_a.addAnalogOutput();
  zbAnalogFan_a.setAnalogOutputApplication(ESP_ZB_ZCL_AI_RPM_OTHER);
  zbAnalogFan_a.setAnalogOutputDescription("Fan Speed (RPM)");

  /////////////////// Fan B ------------------------
  // Set up analog input
  zbAnalogFan_b.addAnalogInput();
  zbAnalogFan_b.setAnalogInputApplication(ESP_ZB_ZCL_AI_RPM_OTHER);
  zbAnalogFan_b.setAnalogInputDescription("RPM");
  zbAnalogFan_b.setAnalogInputResolution(1);

  // Set up analog output
  zbAnalogFan_b.addAnalogOutput();
  zbAnalogFan_b.setAnalogOutputApplication(ESP_ZB_ZCL_AI_RPM_OTHER);
  zbAnalogFan_b.setAnalogOutputDescription("Fan Speed (RPM)");

  /////////////////// LEDs ------------------------
  // Set up analog input
  zbAnalogLights.addAnalogInput();
  zbAnalogLights.setAnalogInputApplication(ESP_ZB_ZCL_AI_PERCENTAGE_OTHER));
  zbAnalogLights.setAnalogInputDescription("LED Brightness");
  zbAnalogLights.setAnalogInputResolution(1);

  // Set up analog output
  zbAnalogLights.addAnalogOutput();
  zbAnalogLights.setAnalogOutputApplication(ESP_ZB_ZCL_AI_PERCENTAGE_OTHER));
  zbAnalogLights.setAnalogOutputDescription("LED Brightness");

  /////////////////// DHT10b ------------------------
  // Set up analog input
  zbAnalogTemp.addAnalogInput();
  zbAnalogTemp.setAnalogInputApplication(ESP_ZB_ZCL_AI_TEMPERATURE_OTHER);
  zbAnalogTemp.setAnalogInputDescription("Water Temperature");
  zbAnalogTemp.setAnalogInputResolution(0.1);

  /////////////////// AHT10 ------------------------
  // Set up analog input
  zbAnalogTemp.addAnalogInput();
  zbAnalogTemp.setAnalogInputApplication(ESP_ZB_ZCL_AI_TEMPERATURE_OTHER);
  zbAnalogTemp.setAnalogInputDescription("Temperature");
  zbAnalogTemp.setAnalogInputResolution(0.1);

  // Set up analog input
  zbAnalogPercent.addAnalogInput();
  zbAnalogPercent.setAnalogInputApplication(ESP_ZB_ZCL_AI_HUMIDITY_OTHER);
  zbAnalogPercent.setAnalogInputDescription("Humidity Percentage");
  zbAnalogPercent.setAnalogInputResolution(0.01);

  /////////////////// Air pump ------------------------
  // Set up binary input
  zbBinaryAirPump.addBinaryInput();
  zbBinaryAirPump.setBinaryInputApplication(0x0078); // Generic Status BI
  zbBinaryAirPump.setBinaryInputDescription("Air Pump Status");

  // Set up binary output
  zbBinaryAirPump.addBinaryOutput();
  zbBinaryAirPump.setBinaryOutputApplication(0xFFFF); // Other Output BO
  zbBinaryAirPump.setBinaryOutputDescription("Air Pump Switch");

  /////////////////// Water pump ------------------------
  // Set up binary input
  zbBinaryWaterPump.addBinaryInput();
  zbBinaryWaterPump.setBinaryInputApplication(0x0078); // Generic Status BI
  zbBinaryWaterPump.setBinaryInputDescription("Water Pump Status");

  // Set up binary output
  zbBinaryWaterPump.addBinaryOutput();
  zbBinaryWaterPump.setBinaryOutputApplication(0xFFFF); // Other Output BO
  zbBinaryWaterPump.setBinaryOutputDescription("Water Pump Switch");

  /////////////////// Dosing pumps ------------------------
  // Set up binary input
  zbBinaryAirPump.setBinaryInputApplication(0x0078); // Generic Status BI
  zbBinaryAirPump.setBinaryInputDescription("Water Pump Status");

  // Set up binary output
  zbBinaryAirPump.addBinaryOutput();
  zbBinaryAirPump.setBinaryOutputApplication(0xFFFF); // Other Output BO
  zbBinaryAirPump.setBinaryOutputDescription("Water Pump Switch");



  //////////////////// Zigbee endpoint config
  
  // Do I need to set these parameters for all endpoints? Or just the global device somehow?
  _ep.setManufacturerAndModel(MANUFACTURER_NAME, MODEL_NAME);
  _ep.setPowerSource(ZB_POWER_SOURCE_MAINS, 75);
  // Add the endpoints to Zb core
  Zigbee.addEndpoint(&_ep);
  Zigbee.addEndpoint(&_ep);
  Zigbee.addEndpoint(&_ep);
  Zigbee.addEndpoint(&_ep);
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
