//------------------------ Includes ------------------------
#include <Arduino.h>
#include "Zigbee.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_AHTX0.h>

//------------------------ GPIO Definition ------------------------
// GPIO definitions 
#define FAN_A_GPIO       14  // PWM output for Fan A
#define FAN_B_GPIO       27  // PWM output for Fan B
#define LED_GPIO         32  // PWM output for LED lights
#define WATER_PUMP_GPIO  33  // Digital output for Water Pump relay
#define AIR_PUMP_GPIO    25  // Digital output for Air Pump relay
#define DOSE_PUMP_A_GPIO 26  // Digital output for Dose Pump A
#define DOSE_PUMP_B_GPIO 22  // Digital output for Dose Pump B
#define DOSE_PUMP_C_GPIO 23  // Digital output for Dose Pump C
#define WATER_HEATER_GPIO 21 // Digital output for Water Heater relay

#define DS18B20_GPIO     4   // 1-Wire pin for DS18B20 temperature
#define TDS_GPIO         36  // ADC input for TDS sensor (ADC1 channel)
#define pH_GPIO          39  // ADC input for pH sensor (ADC1 channel)

#define BOOT_PIN         9   // Button (active LOW) for Zigbee factory reset


//------------------------ Device parameters ------------------------
#define ANALOG_DEVICE_ENDPOINT_NUMBER      1

#define MODEL_NAME          "HYDROPONICS"
#define MANUFACTURER_NAME   "JIMBO"

//------------------------ Endpoints ------------------------
// Zigbee endpoints for analog sensors
ZigbeeAnalog zbAnalogFanA      = ZigbeeAnalog(1);
ZigbeeAnalog zbAnalogFanB      = ZigbeeAnalog(2);
ZigbeeAnalog zbAnalogLights    = ZigbeeAnalog(3);
ZigbeeAnalog zbAnalogTempAHT10 = ZigbeeAnalog(4);
ZigbeeAnalog zbAnalogHumAHT10  = ZigbeeAnalog(5);
ZigbeeAnalog zbAnalogTempDS18  = ZigbeeAnalog(6);
ZigbeeAnalog zbAnalogPh        = ZigbeeAnalog(7);
ZigbeeAnalog zbAnalogTds       = ZigbeeAnalog(8);

// Zigbee endpoints for binary devices
ZigbeeBinary zbBinaryAirPump     = ZigbeeBinary(9);
ZigbeeBinary zbBinaryWaterPump   = ZigbeeBinary(10);
ZigbeeBinary zbBinaryWaterHeater = ZigbeeBinary(11);
ZigbeeBinary zbBinaryDosePumpA   = ZigbeeBinary(12);
ZigbeeBinary zbBinaryDosePumpB   = ZigbeeBinary(13);
ZigbeeBinary zbBinaryDosePumpC   = ZigbeeBinary(14);

//------------------------ Output Callback ------------------------
// Callback for analog output changes (0–100% -> PWM duty)
void setFanASpeed(float speed) {
  int duty = constrain(int(speed * 2.55), 0, 255);
  analogWrite(FAN_A_GPIO, duty);
}
void setFanBSpeed(float speed) {
  int duty = constrain(int(speed * 2.55), 0, 255);
  analogWrite(FAN_B_GPIO, duty);
}
void setLightsBrightness(float brightness) {
  int duty = constrain(int(brightness * 2.55), 0, 255);
  analogWrite(LED_GPIO, duty);
}

// Callback for binary output changes (turn relay on/off)
void airPumpSwitch(bool state) {
  digitalWrite(AIR_PUMP_GPIO, state ? HIGH : LOW);
  zbBinaryAirPump.setBinaryInput(state);
  zbBinaryAirPump.reportBinaryInput();
}
void waterPumpSwitch(bool state) {
  digitalWrite(WATER_PUMP_GPIO, state ? HIGH : LOW);
  zbBinaryWaterPump.setBinaryInput(state);
  zbBinaryWaterPump.reportBinaryInput();
}
void waterHeaterSwitch(bool state) {
  digitalWrite(WATER_HEATER_GPIO, state ? HIGH : LOW);
  zbBinaryWaterHeater.setBinaryInput(state);
  zbBinaryWaterHeater.reportBinaryInput();
}
void dosePumpASwitch(bool state) {
  digitalWrite(DOSE_PUMP_A_GPIO, state ? HIGH : LOW);
  zbBinaryDosePumpA.setBinaryInput(state);
  zbBinaryDosePumpA.reportBinaryInput();
}
void dosePumpBSwitch(bool state) {
  digitalWrite(DOSE_PUMP_B_GPIO, state ? HIGH : LOW);
  zbBinaryDosePumpB.setBinaryInput(state);
  zbBinaryDosePumpB.reportBinaryInput();
}
void dosePumpCSwitch(bool state) {
  digitalWrite(DOSE_PUMP_C_GPIO, state ? HIGH : LOW);
  zbBinaryDosePumpC.setBinaryInput(state);
  zbBinaryDosePumpC.reportBinaryInput();
}

///// Configure sensors
Adafruit_AHTX0 aht;
OneWire oneWire(DS18B20_GPIO);
DallasTemperature sensors(&oneWire);

//------------------------ Setup ------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting Zigbee hydroponics controller...");

  // Initialize button for factory reset
  pinMode(BOOT_PIN, INPUT_PULLUP);

  // Configure ADC (12-bit, full-scale 3.3V)
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Initialize sensors
  sensors.begin();  // DS18B20
  if (!aht.begin()) {
    Serial.println("Failed to find AHT10 sensor!");
  }

  // Configure PWM outputs
  pinMode(FAN_A_GPIO, OUTPUT);
  pinMode(FAN_B_GPIO, OUTPUT);
  pinMode(LED_GPIO, OUTPUT);
  analogWriteResolution(FAN_A_GPIO, 8);
  analogWriteFrequency(FAN_A_GPIO, 5000);
  analogWriteResolution(FAN_B_GPIO, 8);
  analogWriteFrequency(FAN_B_GPIO, 5000);
  analogWriteResolution(LED_GPIO, 8);
  analogWriteFrequency(LED_GPIO, 5000);
  analogWrite(FAN_A_GPIO, 0);
  analogWrite(FAN_B_GPIO, 0);
  analogWrite(LED_GPIO, 0);

  // Configure relay outputs
  pinMode(AIR_PUMP_GPIO, OUTPUT);
  pinMode(WATER_PUMP_GPIO, OUTPUT);
  pinMode(WATER_HEATER_GPIO, OUTPUT);
  pinMode(DOSE_PUMP_A_GPIO, OUTPUT);
  pinMode(DOSE_PUMP_B_GPIO, OUTPUT);
  pinMode(DOSE_PUMP_C_GPIO, OUTPUT);
  digitalWrite(AIR_PUMP_GPIO, LOW);
  digitalWrite(WATER_PUMP_GPIO, LOW);
  digitalWrite(WATER_HEATER_GPIO, LOW);
  digitalWrite(DOSE_PUMP_A_GPIO, LOW);
  digitalWrite(DOSE_PUMP_B_GPIO, LOW);
  digitalWrite(DOSE_PUMP_C_GPIO, LOW);


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

  /////////////////// DS18b20 ------------------------
  // Set up analog input
  zbAnalogTemp_DS18b20.addAnalogInput();
  zbAnalogTemp_DS18b20.setAnalogInputApplication(ESP_ZB_ZCL_AI_TEMPERATURE_OTHER);
  zbAnalogTemp_DS18b20.setAnalogInputDescription("Water Temperature");
  zbAnalogTemp_DS18b20.setAnalogInputResolution(0.01);

  /////////////////// AHT10 ------------------------
  // Set up analog input
  zbAnalogTemp_aht10.addAnalogInput();
  zbAnalogTemp_aht10.setAnalogInputApplication(ESP_ZB_ZCL_AI_TEMPERATURE_OTHER);
  zbAnalogTemp_aht10.setAnalogInputDescription("Air Temperature");
  zbAnalogTemp_aht10.setAnalogInputResolution(0.01);

  // Set up analog input
  zbAnalogHumidity_aht10.addAnalogInput();
  zbAnalogHumidity_aht10.setAnalogInputApplication(ESP_ZB_ZCL_AI_HUMIDITY_OTHER);
  zbAnalogHumidity_aht10.setAnalogInputDescription("Humidity Percentage");
  zbAnalogHumidity_aht10.setAnalogInputResolution(0.01);

  /////////////////// pH ------------------------------
  zbAnalogPh.addAnalogInput();
  zbAnalogPh.setAnalogInputApplication(ESP_ZB_ZCL_AI_COUNT_UNITLESS_OTHER);
  zbAnalogPh.setAnalogInputDescription("Water pH");
  zbAnalogPh.setAnalogInputResolution(0.01);

  /////////////////// TDS ------------------------------
  zbAnalogTds.addAnalogInput();
  zbAnalogTds.setAnalogInputApplication(ESP_ZB_ZCL_AI_COUNT_UNITLESS_OTHER);
  zbAnalogTds.setAnalogInputDescription("Water TDS");
  zbAnalogTds.setAnalogInputResolution(0.01);

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

  /////////////////// Water heater ------------------------
  // Set up binary input
  zbBinaryWaterHeater.addBinaryInput();
  zbBinaryWaterHeater.setBinaryInputApplication(0x0078); // Generic Status BI
  zbBinaryWaterHeater.setBinaryInputDescription("Water Heater Status");

  // Set up binary output
  zbBinaryWaterHeater.addBinaryOutput();
  zbBinaryWaterHeater.setBinaryOutputApplication(0xFFFF); // Other Output BO
  zbBinaryWaterHeater.setBinaryOutputDescription("Water Heater Switch");

  /////////////////// Dosing pumps ------------------------
  // Set up binary input
  zbBinaryDosePump_a.setBinaryInputApplication(0x0078); // Generic Status BI
  zbBinaryDosePump_a.setBinaryInputDescription("DosePump_a Status");

  // Set up binary output
  zbBinaryDosePump_a.addBinaryOutput();
  zbBinaryDosePump_a.setBinaryOutputApplication(0xFFFF); // Other Output BO
  zbBinaryDosePump_a.setBinaryOutputDescription("DosePump_a Switch");

  // Set up binary input
  zbBinaryDosePump_b.setBinaryInputApplication(0x0078); // Generic Status BI
  zbBinaryDosePump_b.setBinaryInputDescription("DosePump_b Status");

  // Set up binary output
  zbBinaryDosePump_b.addBinaryOutput();
  zbBinaryDosePump_b.setBinaryOutputApplication(0xFFFF); // Other Output BO
  zbBinaryDosePump_b.setBinaryOutputDescription("DosePump_b Switch"); 

  // Set up binary input
  zbBinaryDosePump_c.setBinaryInputApplication(0x0078); // Generic Status BI
  zbBinaryDosePump_c.setBinaryInputDescription("DosePump_c Status");

  // Set up binary output
  zbBinaryDosePump_c.addBinaryOutput();
  zbBinaryDosePump_c.setBinaryOutputApplication(0xFFFF); // Other Output BO
  zbBinaryDosePump_c.setBinaryOutputDescription("DosePump_c Switch"); 

  //////////////////// Set up AHT10 here

  //////////////////// Zigbee endpoint config
  // --- Configure Zigbee endpoints --- 
  // Fan A (analog input + output)
  zbAnalogFanA.addAnalogInput();
  zbAnalogFanA.setAnalogInputApplication(ESP_ZB_ZCL_AI_RPM_OTHER);
  zbAnalogFanA.setAnalogInputDescription("Fan A Speed");
  zbAnalogFanA.setAnalogInputResolution(1);
  zbAnalogFanA.addAnalogOutput();
  zbAnalogFanA.setAnalogOutputApplication(ESP_ZB_ZCL_AI_RPM_OTHER);
  zbAnalogFanA.setAnalogOutputDescription("Fan A Control");
  zbAnalogFanA.setAnalogOutputResolution(1);
  zbAnalogFanA.onAnalogOutputChange(setFanASpeed);

  // Fan B (analog input + output)
  zbAnalogFanB.addAnalogInput();
  zbAnalogFanB.setAnalogInputApplication(ESP_ZB_ZCL_AI_RPM_OTHER);
  zbAnalogFanB.setAnalogInputDescription("Fan B Speed");
  zbAnalogFanB.setAnalogInputResolution(1);
  zbAnalogFanB.addAnalogOutput();
  zbAnalogFanB.setAnalogOutputApplication(ESP_ZB_ZCL_AI_RPM_OTHER);
  zbAnalogFanB.setAnalogOutputDescription("Fan B Control");
  zbAnalogFanB.setAnalogOutputResolution(1);
  zbAnalogFanB.onAnalogOutputChange(setFanBSpeed);

  // LED Lights (analog input + output)
  zbAnalogLights.addAnalogInput();
  zbAnalogLights.setAnalogInputApplication(ESP_ZB_ZCL_AI_PERCENTAGE_OTHER);
  zbAnalogLights.setAnalogInputDescription("Light Brightness");
  zbAnalogLights.setAnalogInputResolution(1);
  zbAnalogLights.addAnalogOutput();
  zbAnalogLights.setAnalogOutputApplication(ESP_ZB_ZCL_AI_PERCENTAGE_OTHER);
  zbAnalogLights.setAnalogOutputDescription("Light Control");
  zbAnalogLights.setAnalogOutputResolution(1);
  zbAnalogLights.onAnalogOutputChange(setLightsBrightness);

  // AHT10: air temperature (analog input)
  zbAnalogTempAHT10.addAnalogInput();
  zbAnalogTempAHT10.setAnalogInputApplication(ESP_ZB_ZCL_AI_TEMPERATURE_OTHER);
  zbAnalogTempAHT10.setAnalogInputDescription("Air Temperature");
  zbAnalogTempAHT10.setAnalogInputResolution(0.01);

  // AHT10: air humidity (analog input)
  zbAnalogHumAHT10.addAnalogInput();
  zbAnalogHumAHT10.setAnalogInputApplication(ESP_ZB_ZCL_AI_HUMIDITY_OTHER);
  zbAnalogHumAHT10.setAnalogInputDescription("Air Humidity");
  zbAnalogHumAHT10.setAnalogInputResolution(0.01);

  // DS18B20: water temperature (analog input)
  zbAnalogTempDS18.addAnalogInput();
  zbAnalogTempDS18.setAnalogInputApplication(ESP_ZB_ZCL_AI_TEMPERATURE_OTHER);
  zbAnalogTempDS18.setAnalogInputDescription("Water Temperature");
  zbAnalogTempDS18.setAnalogInputResolution(0.01);

  // pH sensor (analog input)
  zbAnalogPh.addAnalogInput();
  zbAnalogPh.setAnalogInputApplication(ESP_ZB_ZCL_AI_COUNT_UNITLESS_OTHER);
  zbAnalogPh.setAnalogInputDescription("Water pH");
  zbAnalogPh.setAnalogInputResolution(0.01);

  // TDS sensor (analog input)
  zbAnalogTds.addAnalogInput();
  zbAnalogTds.setAnalogInputApplication(ESP_ZB_ZCL_AI_COUNT_UNITLESS_OTHER);
  zbAnalogTds.setAnalogInputDescription("Water TDS");
  zbAnalogTds.setAnalogInputResolution(0.01);

  // Air Pump (binary input + output)
  zbBinaryAirPump.addBinaryInput();
  zbBinaryAirPump.setBinaryInputApplication(0x0078); // Generic status
  zbBinaryAirPump.setBinaryInputDescription("Air Pump Status");
  zbBinaryAirPump.addBinaryOutput();
  zbBinaryAirPump.setBinaryOutputApplication(0xFFFF);
  zbBinaryAirPump.setBinaryOutputDescription("Air Pump On/Off");
  zbBinaryAirPump.onBinaryOutputChange(airPumpSwitch);

  // Water Pump (binary input + output)
  zbBinaryWaterPump.addBinaryInput();
  zbBinaryWaterPump.setBinaryInputApplication(0x0078);
  zbBinaryWaterPump.setBinaryInputDescription("Water Pump Status");
  zbBinaryWaterPump.addBinaryOutput();
  zbBinaryWaterPump.setBinaryOutputApplication(0xFFFF);
  zbBinaryWaterPump.setBinaryOutputDescription("Water Pump On/Off");
  zbBinaryWaterPump.onBinaryOutputChange(waterPumpSwitch);

  // Water Heater (binary input + output)
  zbBinaryWaterHeater.addBinaryInput();
  zbBinaryWaterHeater.setBinaryInputApplication(0x0078);
  zbBinaryWaterHeater.setBinaryInputDescription("Water Heater Status");
  zbBinaryWaterHeater.addBinaryOutput();
  zbBinaryWaterHeater.setBinaryOutputApplication(0xFFFF);
  zbBinaryWaterHeater.setBinaryOutputDescription("Water Heater On/Off");
  zbBinaryWaterHeater.onBinaryOutputChange(waterHeaterSwitch);

  // Dose Pump A (binary input + output)
  zbBinaryDosePumpA.addBinaryInput();
  zbBinaryDosePumpA.setBinaryInputApplication(0x0078);
  zbBinaryDosePumpA.setBinaryInputDescription("Dose Pump A Status");
  zbBinaryDosePumpA.addBinaryOutput();
  zbBinaryDosePumpA.setBinaryOutputApplication(0xFFFF);
  zbBinaryDosePumpA.setBinaryOutputDescription("Dose Pump A On/Off");
  zbBinaryDosePumpA.onBinaryOutputChange(dosePumpASwitch);

  // Dose Pump B (binary input + output)
  zbBinaryDosePumpB.addBinaryInput();
  zbBinaryDosePumpB.setBinaryInputApplication(0x0078);
  zbBinaryDosePumpB.setBinaryInputDescription("Dose Pump B Status");
  zbBinaryDosePumpB.addBinaryOutput();
  zbBinaryDosePumpB.setBinaryOutputApplication(0xFFFF);
  zbBinaryDosePumpB.setBinaryOutputDescription("Dose Pump B On/Off");
  zbBinaryDosePumpB.onBinaryOutputChange(dosePumpBSwitch);

  // Dose Pump C (binary input + output)
  zbBinaryDosePumpC.addBinaryInput();
  zbBinaryDosePumpC.setBinaryInputApplication(0x0078);
  zbBinaryDosePumpC.setBinaryInputDescription("Dose Pump C Status");
  zbBinaryDosePumpC.addBinaryOutput();
  zbBinaryDosePumpC.setBinaryOutputApplication(0xFFFF);
  zbBinaryDosePumpC.setBinaryOutputDescription("Dose Pump C On/Off");
  zbBinaryDosePumpC.onBinaryOutputChange(dosePumpCSwitch);

  // Add all endpoints to Zigbee core
  Zigbee.addEndpoint(&zbAnalogFanA);
  Zigbee.addEndpoint(&zbAnalogFanB);
  Zigbee.addEndpoint(&zbAnalogLights);
  Zigbee.addEndpoint(&zbAnalogTempAHT10);
  Zigbee.addEndpoint(&zbAnalogHumAHT10);
  Zigbee.addEndpoint(&zbAnalogTempDS18);
  Zigbee.addEndpoint(&zbAnalogPh);
  Zigbee.addEndpoint(&zbAnalogTds);
  Zigbee.addEndpoint(&zbBinaryAirPump);
  Zigbee.addEndpoint(&zbBinaryWaterPump);
  Zigbee.addEndpoint(&zbBinaryWaterHeater);
  Zigbee.addEndpoint(&zbBinaryDosePumpA);
  Zigbee.addEndpoint(&zbBinaryDosePumpB);
  Zigbee.addEndpoint(&zbBinaryDosePumpC);

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
  Serial.println();
}

//------------------------ Main Loop ------------------------
void loop() {
  static uint32_t timeCounter = 0;

  // Every 5 seconds, read sensors and report
  if (!(timeCounter++ % 50)) {  // 100ms * 50 = 5000ms
    // Read DS18B20 (water temp)
    sensors.requestTemperatures();
    float waterTemp = sensors.getTempCByIndex(0);

    // Read AHT10 (air temp and humidity)
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);
    float airTemp = temp.temperature;
    float airHum = humidity.relative_humidity;

    // Read pH and TDS via ADC
    int rawPh = analogRead(pH_GPIO);
    float pHVal = (rawPh * 14.0) / 4095.0;  // Convert ADC to pH (0–14)
    int rawTds = analogRead(TDS_GPIO);
    float tdsVal = (rawTds * 1000.0) / 4095.0; // Convert ADC to ppm

    // Update Zigbee analog inputs and report
    zbAnalogTempDS18.setAnalogInput(waterTemp);
    zbAnalogTempDS18.reportAnalogInput();
    zbAnalogTempAHT10.setAnalogInput(airTemp);
    zbAnalogTempAHT10.reportAnalogInput();
    zbAnalogHumAHT10.setAnalogInput(airHum);
    zbAnalogHumAHT10.reportAnalogInput();
    zbAnalogPh.setAnalogInput(pHVal);
    zbAnalogPh.reportAnalogInput();
    zbAnalogTds.setAnalogInput(tdsVal);
    zbAnalogTds.reportAnalogInput();
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

  delay(100);
}
