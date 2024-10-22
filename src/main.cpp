#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_INA219.h>
#include "Adafruit_LTR390.h"
#include "Adafruit_VEML7700.h"
#include <ArduinoJson.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define INA219_I2C_ADDRESS 0x41
#define SLEEP_PIN 13

Adafruit_LTR390 ltr = Adafruit_LTR390();
Adafruit_VEML7700 veml = Adafruit_VEML7700();
Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;
Adafruit_INA219 ina219(INA219_I2C_ADDRESS);

// Json Setup
JsonDocument telemetryData;

// Anti-theft Detection
const int ledPin = 2;
const float movementThreshold = 0.5;
float prevX = 0, prevY = 0, prevZ = 0;
int count1;

// Function to blink the LED
void blinkLed()
{
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
  }
}

double round2(double value)
{
  return (int)(value * 100 + 0.5) / 100.0;
}

// Example of checking stack usage in a task
void checkStackUsage()
{
  UBaseType_t stackRemaining = uxTaskGetStackHighWaterMark(NULL);
  Serial.print("Remaining stack: ");
  Serial.println(stackRemaining);
}

void timeoutChecker(void* parameters) {
  for (;;) {
    Serial.print("Timeout Checker:");
    Serial.println(count1++);
    if(count1 == 180){
      Serial.println("Shutting Down Now!!");
      digitalWrite(SLEEP_PIN, HIGH);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// Task to detect theft by monitoring the MPU6050 sensor
void theftDetectionTask(void *parameters)
{
  while (1)
  {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calculate the difference between current and previous acceleration values
    float diffX = abs(a.acceleration.x - prevX);
    float diffY = abs(a.acceleration.y - prevY);
    float diffZ = abs(a.acceleration.z - prevZ);

    // If any difference exceeds the threshold, consider it as movement
    if (diffX > movementThreshold || diffY > movementThreshold || diffZ > movementThreshold)
    {
      Serial.println("Movement detected! Triggering anti-theft alert...");
      blinkLed(); // Blink LED as an alert
    }

    // Update the previous values
    prevX = a.acceleration.x;
    prevY = a.acceleration.y;
    prevZ = a.acceleration.z;

    vTaskDelay(500 / portTICK_PERIOD_MS); // Run the task every 500ms
  }
}

// Task to read and print all I2C sensor values
void readAllI2CSensors(void *pvParameters)
{
  while (1)
  {
    // Read BME280 (temperature, pressure, humidity)
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    telemetryData["t"] = round2(bme.readTemperature());

    Serial.println(" °C");

    Serial.print("Pressure = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");
    telemetryData["p"] = round2((bme.readPressure() / 100.0F));

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");
    telemetryData["h"] = round2(bme.readHumidity());

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");
    telemetryData["a"] = round2(bme.readAltitude(SEALEVELPRESSURE_HPA));


    // Read INA219 (voltage, current, power)
    
    Serial.print("Bus Voltage:   ");
    Serial.print(ina219.getBusVoltage_V());
    Serial.println(" V");
    telemetryData["v"] = round2(ina219.getBusVoltage_V());

    Serial.print("Current:       ");
    Serial.print(ina219.getCurrent_mA());
    Serial.println(" mA");
    telemetryData["c"] = round2(ina219.getCurrent_mA());


    // Read LTR390 (UV sensor)
    if (ltr.newDataAvailable())
    {
      Serial.print("UV data: ");
      Serial.println(ltr.readUVS());
      telemetryData["u"] = round2(ltr.readUVS());

    }

    // Read VEML7700 (light sensor) and display all relevant data under the "lux" section
    Serial.println("Lux Sensor Values:");
    Serial.print("  Raw ALS:    ");
    Serial.println(veml.readALS());
    
    Serial.print("  Raw White:  ");
    Serial.println(veml.readWhite());
    Serial.print("  Lux:        ");
    Serial.println(veml.readLux()); // Include raw ALS, raw white, and lux value
    telemetryData["l"] = round2(veml.readLux());


    checkStackUsage();
    // Delay task to control the frequency of sensor readings (single delay for all sensors)
    vTaskDelay(5000 / portTICK_PERIOD_MS); // Adjust delay based on your requirements
  }
}

void setup()
{
  pinMode(ledPin, OUTPUT);   // Set LED pin as output
  digitalWrite(ledPin, LOW); // Start with LED off

  // Initialize Serial for debugging
  Serial.begin(9600);
  while (!Serial)
    delay(10); // Wait for Serial to initialize

  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  if (!mpu.begin(0x69))
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
      delay(10);
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialize BME280
  if (!bme.begin(0x76))
  {
    Serial.println("Could not find a valid BME280 sensor!");
    while (1)
      delay(10);
  }

  // Initialize INA219
  if (!ina219.begin())
  {
    Serial.println("Failed to find INA219 chip!");
    while (1)
      delay(10);
  }

  // Initialize LTR390
  if (!ltr.begin())
  {
    Serial.println("Couldn't find LTR sensor!");
    while (1)
      delay(10);
  }
  ltr.setMode(LTR390_MODE_UVS);
  ltr.setGain(LTR390_GAIN_3);
  ltr.setResolution(LTR390_RESOLUTION_16BIT);

  // Initialize VEML7700
  if (!veml.begin())
  {
    Serial.println("Sensor not found!");
    while (1)
      ;
  }
  veml.setGain(VEML7700_GAIN_1_8);
  veml.setIntegrationTime(VEML7700_IT_100MS);



  // Create FreeRTOS task for all I2C sensor readings
  xTaskCreate(
      readAllI2CSensors,  // Task function
      "I2C Sensors Task", // Task name
      3000,               // Stack size (in words)
      NULL,               // Task parameters
      1,                  // Task priority
      NULL                // Task handle
  );

  // Create task for anti-theft detection
  xTaskCreate(
      theftDetectionTask, // Task function
      "Theft Detection",  // Task name
      2000,               // Stack size (in words)
      NULL,               // Task parameters
      1,                  // Task priority
      NULL                // Task handle
  );

   // Create task for anti-theft detection
  xTaskCreate(
      timeoutChecker, // Task function
      "TimeOut Checker",  // Task name
      2000,               // Stack size (in words)
      NULL,               // Task parameters
      1,                  // Task priority
      NULL                // Task handle
  );
}

void loop()
{
  // The main loop is not used, as tasks handle the execution
  String jsonString;
  serializeJson(telemetryData, jsonString);
  Serial.println(jsonString);
  delay(5000);
}
