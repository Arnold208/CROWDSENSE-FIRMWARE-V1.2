#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_INA219.h>
#include "Adafruit_LTR390.h"
#include "Adafruit_VEML7700.h"
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// Define paths
#define SENSOR_DATA_DIR "/Sensor_Data"
#define LOGS_DIR "/Sensor_Data/logs"
#define TELEMETRY_DIR "/Sensor_Data/telemetry"
#define CONFIG_DIR "/Sensor_Data/config"
#define LOGS_FILE "/Sensor_Data/logs/logs.json"
#define TELEMETRY_FILE "/Sensor_Data/telemetry/telemetry.json"
#define CONFIG_FILE "/Sensor_Data/config/config.json"

#define SEALEVELPRESSURE_HPA (1013.25)
#define INA219_I2C_ADDRESS 0x41
#define SLEEP_PIN 13

Adafruit_LTR390 ltr = Adafruit_LTR390();
Adafruit_VEML7700 veml = Adafruit_VEML7700();
Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;
Adafruit_INA219 ina219(INA219_I2C_ADDRESS);
SoftwareSerial pmsSerial(35, 36);

// Json Setup
JsonDocument telemetryData;

// Anti-theft Detection
const int ledPin = 2;
const float movementThreshold = 0.5;
float prevX = 0, prevY = 0, prevZ = 0;
int count1;
bool isSdCardAvailable = false;

// Buffer to store data when SD card is not available
String dataBuffer = "";

// Function to initialize the SD card and check directories/files
void initializeSdCard()
{
  if (!SD.begin(5))
  {
    Serial.println("SD Card Mount Failed");
    isSdCardAvailable = false;
    return;
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE)
  {
    Serial.println("No SD card attached");
    isSdCardAvailable = false;
    return;
  }

  Serial.println("SD Card initialized.");
  isSdCardAvailable = true;

  // Check and create directories
  if (!SD.exists(SENSOR_DATA_DIR))
  {
    SD.mkdir(SENSOR_DATA_DIR);
    Serial.println("Created Sensor_Data directory");
  }

  if (!SD.exists(LOGS_DIR))
  {
    SD.mkdir(LOGS_DIR);
    Serial.println("Created logs directory");
  }

  if (!SD.exists(TELEMETRY_DIR))
  {
    SD.mkdir(TELEMETRY_DIR);
    Serial.println("Created telemetry directory");
  }

  if (!SD.exists(CONFIG_DIR))
  {
    SD.mkdir(CONFIG_DIR);
    Serial.println("Created config directory");
  }

  // Check and create files
  if (!SD.exists(LOGS_FILE))
  {
    File file = SD.open(LOGS_FILE, FILE_WRITE);
    if (file)
    {
      file.println("[]"); // Initialize as an empty JSON array
      file.close();
      Serial.println("Created logs.json file");
    }
  }

  if (!SD.exists(TELEMETRY_FILE))
  {
    File file = SD.open(TELEMETRY_FILE, FILE_WRITE);
    if (file)
    {
      file.println("[]"); // Initialize as an empty JSON array
      file.close();
      Serial.println("Created telemetry.json file");
    }
  }

  if (!SD.exists(CONFIG_FILE))
  {
    File file = SD.open(CONFIG_FILE, FILE_WRITE);
    if (file)
    {
      file.println("{}"); // Initialize as an empty JSON object
      file.close();
      Serial.println("Created config.json file");
    }
  }
}

// Function to append JSON data to a file, ensuring each entry is on a new line
void appendJsonToFile(fs::FS &fs, const char *path, const String &jsonString)
{
  if (!isSdCardAvailable)
  {
    Serial.println("SD Card not available. Storing data in buffer.");
    // Buffer data for future write when SD card becomes available
    dataBuffer += jsonString + "\n";
    return;
  }

  // Check if the JSON string is empty or invalid (like `{}` or `null`)
  if (jsonString == "{}" || jsonString == "null" || jsonString.length() == 0)
  {
    Serial.println("Invalid JSON data. Skipping write operation.");
    return;
  }

  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    Serial.println("Failed to open file for appending");
    return;
  }

  // Write the JSON string followed by a newline to ensure it appears on a new line
  if (file.println(jsonString))
  {
    Serial.println("JSON data appended");
  }
  else
  {
    Serial.println("Append failed");
  }
  file.close();
}

// Task to check SD card availability continuously
void checkSdCardAvailabilityTask(void *parameters)
{
  while (1)
  {
    if (!isSdCardAvailable)
    {
      initializeSdCard();
    }
    else if (dataBuffer.length() > 0)
    {
      // If SD card becomes available, flush buffer to files
      Serial.println("Flushing buffered data to SD card...");
      appendJsonToFile(SD, TELEMETRY_FILE, dataBuffer);
      dataBuffer = ""; // Clear buffer after successful write
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS); // Check every 5 seconds
  }
}

// Function to write a file (used to create JSON files if they don't exist)
void writeFile(fs::FS &fs, const char *path, const char *message)
{
  if (!isSdCardAvailable)
    return; // Skip if SD card is not available

  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message))
  {
    Serial.println("File written");
  }
  else
  {
    Serial.println("Write failed");
  }
  file.close();
}

// Function to read a file
void readFile(fs::FS &fs, const char *path)
{
  if (!isSdCardAvailable)
    return; // Skip if SD card is not available

  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file)
  {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available())
  {
    Serial.write(file.read());
  }
  file.close();
}

// Function to check and create JSON files if they don't exist
void checkAndCreateFiles()
{
  if (!SD.exists(LOGS_FILE))
  {
    Serial.println("Logs JSON file does not exist. Creating...");
    writeFile(SD, LOGS_FILE, "{}");
  }

  if (!SD.exists(TELEMETRY_FILE))
  {
    Serial.println("Telemetry JSON file does not exist. Creating...");
    writeFile(SD, TELEMETRY_FILE, "{}");
  }

  if (!SD.exists(CONFIG_FILE))
  {
    Serial.println("Config JSON file does not exist. Creating...");
    writeFile(SD, CONFIG_FILE, "{}");
  }
}

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

struct pms5003data
{
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

struct pms5003data data;

boolean readPMSdata(Stream *s)
{
  if (!s->available())
  {
    return false;
  }

  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42)
  {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32)
  {
    return false;
  }

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // get checksum ready
  for (uint8_t i = 0; i < 30; i++)
  {
    sum += buffer[i];
  }

  /* debugging
  for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
  }
  Serial.println();
  */

  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++)
  {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);

  if (sum != data.checksum)
  {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
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

void pmSensor(void *parameters)
{
  for (;;)
  {
    if (readPMSdata(&pmsSerial))
    {
      // reading data was successful!
      Serial.println();
      Serial.println("---------------------------------------");
      Serial.println("Concentration Units (standard)");
      Serial.print("PM 1.0: ");
      Serial.print(data.pm10_standard);
      Serial.print("\t\tPM 2.5: ");
      Serial.print(data.pm25_standard);
      Serial.print("\t\tPM 10: ");
      Serial.println(data.pm100_standard);
      Serial.println("---------------------------------------");
      Serial.println("Concentration Units (environmental)");
      Serial.print("PM 1.0: ");
      Serial.print(data.pm10_env);
      Serial.print("\t\tPM 2.5: ");
      Serial.print(data.pm25_env);
      Serial.print("\t\tPM 10: ");
      Serial.println(data.pm100_env);
      Serial.println("---------------------------------------");
      Serial.print("Particles > 0.3um / 0.1L air:");
      Serial.println(data.particles_03um);
      Serial.print("Particles > 0.5um / 0.1L air:");
      Serial.println(data.particles_05um);
      Serial.print("Particles > 1.0um / 0.1L air:");
      Serial.println(data.particles_10um);
      Serial.print("Particles > 2.5um / 0.1L air:");
      Serial.println(data.particles_25um);
      Serial.print("Particles > 5.0um / 0.1L air:");
      Serial.println(data.particles_50um);
      Serial.print("Particles > 10.0 um / 0.1L air:");
      Serial.println(data.particles_100um);
      Serial.println("---------------------------------------");

      telemetryData["p1"] = round2(data.pm10_env);
      telemetryData["p2"] = round2(data.pm25_env);
      telemetryData["p10"] = round2(data.pm100_env);
    }
    vTaskDelay(2500 / portTICK_PERIOD_MS);
  }
}

void timeoutChecker(void *parameters)
{
  for (;;)
  {
    Serial.print("Timeout Checker:");
    Serial.println(count1++);
    if (count1 == 10)
    {
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

  pmsSerial.begin(9600);

  initializeSdCard();

  // Check and create directories and files if necessary
  // checkAndCreateDirectories();
  // checkAndCreateFiles();

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
  pinMode(SLEEP_PIN, OUTPUT);
  // Create task for anti-theft detection
  xTaskCreate(
      timeoutChecker,    // Task function
      "TimeOut Checker", // Task name
      2000,              // Stack size (in words)
      NULL,              // Task parameters
      1,                 // Task priority
      NULL               // Task handle
  );

  // xTaskCreate(
  //       pmSensor,    // Task function
  //       "PM SENSOR", // Task name
  //       3000,              // Stack size (in words)
  //       NULL,              // Task parameters
  //       2,                 // Task priority
  //       NULL               // Task handle
  //   );

  // Create task to continuously check for SD card availability
  xTaskCreate(
      checkSdCardAvailabilityTask,  // Task function
      "SD Card Availability Check", // Task name
      3000,                         // Stack size (in words)
      NULL,                         // Task parameters
      3,                            // Task priority
      NULL                          // Task handle
  );
}

void loop()
{
  // The main loop is not used, as tasks handle the execution

  if (readPMSdata(&pmsSerial))
  {
    // reading data was successful!
    Serial.println();
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (standard)");
    Serial.print("PM 1.0: ");
    Serial.print(data.pm10_standard);
    Serial.print("\t\tPM 2.5: ");
    Serial.print(data.pm25_standard);
    Serial.print("\t\tPM 10: ");
    Serial.println(data.pm100_standard);
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (environmental)");
    Serial.print("PM 1.0: ");
    Serial.print(data.pm10_env);
    Serial.print("\t\tPM 2.5: ");
    Serial.print(data.pm25_env);
    Serial.print("\t\tPM 10: ");
    Serial.println(data.pm100_env);
    Serial.println("---------------------------------------");
    Serial.print("Particles > 0.3um / 0.1L air:");
    Serial.println(data.particles_03um);
    Serial.print("Particles > 0.5um / 0.1L air:");
    Serial.println(data.particles_05um);
    Serial.print("Particles > 1.0um / 0.1L air:");
    Serial.println(data.particles_10um);
    Serial.print("Particles > 2.5um / 0.1L air:");
    Serial.println(data.particles_25um);
    Serial.print("Particles > 5.0um / 0.1L air:");
    Serial.println(data.particles_50um);
    Serial.print("Particles > 10.0 um / 0.1L air:");
    Serial.println(data.particles_100um);
    Serial.println("---------------------------------------");

    telemetryData["p1"] = round2(data.pm10_env);
    telemetryData["p2"] = round2(data.pm25_env);
    telemetryData["p10"] = round2(data.pm100_env);
  }
  String jsonString;
  JsonDocument jsonData;
  // serializeJson(telemetryData, jsonData);
  serializeJson(telemetryData, jsonString);
  Serial.println(jsonString);

  // Append JSON data to telemetry file
  appendJsonToFile(SD, TELEMETRY_FILE, jsonString);

  delay(5000);
}
