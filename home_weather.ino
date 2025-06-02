// Room Climate Monitor & Alerter - Phase 4: Implementing Alert Logic

// For DHT Sensor
#include <DHT.h>

// For BLE Communication
#include <SoftwareSerial.h>

// --- DHT Sensor Configuration ---
#define DHTPIN 2        // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // Your sensor is DHT11 (blue)
DHT dht(DHTPIN, DHTTYPE);

// --- BLE Module (MH-10) Configuration ---
const int bleRxPin = 10; // Arduino RX (Connect to MH-10 TXD)
const int bleTxPin = 11; // Arduino TX (Connect to MH-10 RXD)
SoftwareSerial bleSerial(bleRxPin, bleTxPin); // RX, TX
const long BLE_BAUD_RATE = 9600; // <<<<< SET THIS TO THE BAUD RATE THAT WORKED (9600 or 115200)

// --- Alert Thresholds ---
const float TEMP_HIGH_THRESHOLD = 26.0; // degrees C
const float TEMP_LOW_THRESHOLD  = 20.0; // degrees C
const float HUM_HIGH_THRESHOLD  = 65.0; // percent
const float HUM_LOW_THRESHOLD   = 30.0; // percent

// --- Battery Monitoring ---
const int BATTERY_PIN = A0;  // Analog pin for battery voltage reading
const float VOLTAGE_REFERENCE = 5.0;  // Arduino reference voltage
const float VOLTAGE_DIVIDER_RATIO = 2.0;  // Adjust based on your voltage divider (if using one)

// --- Buzzer Configuration ---
const int BUZZER_PIN = 8;  // Digital pin connected to the buzzer
const int BUZZER_FREQUENCY = 4000;  // Frequency in Hz for the buzzer tone
const int BUZZER_DURATION = 200;  // Duration of each beep in milliseconds
const long BUZZER_INTERVAL = 15000;  // Interval between buzzer alerts (5 seconds)

// --- Timing ---
unsigned long previousMillisData = 0;    // will store last time data was sent
unsigned long previousMillisAlert = 0;   // will store last time alert status was checked/sent
unsigned long previousMillisBuzzer = 0;  // will store last time buzzer sounded
const long dataInterval = 30000;          // interval at which to send data (milliseconds)
const long alertCheckInterval = 15000;    // interval at which to check for alerts (milliseconds) - can be more frequent

String lastAlertMessage = ""; // To avoid sending the same alert repeatedly
bool temperatureAlert = false;  // Track if temperature is out of range

void setup() {
  // Initialize Serial Monitor for debugging
  Serial.begin(9600);
  Serial.println("Room Climate Monitor - Phase 4: Alerts");

  // Initialize DHT sensor
  dht.begin();
  Serial.println("DHT sensor initialized.");
  delay(2000); // Allow sensor to stabilize

  // Initialize BLE SoftwareSerial
  bleSerial.begin(BLE_BAUD_RATE);
  Serial.print("BLE Serial initialized at ");
  Serial.print(BLE_BAUD_RATE);
  Serial.println(" baud.");
  
  // Initialize battery monitoring
  pinMode(BATTERY_PIN, INPUT);
  
  // Initialize buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);  // Ensure buzzer is off initially
  
  Serial.println("Monitoring climate and checking for alerts...");
}

float readBatteryVoltage() {
  // Read analog value
  int analogValue = analogRead(BATTERY_PIN);
  
  // Convert to voltage
  float voltage = (analogValue * VOLTAGE_REFERENCE) / 1023.0;
  
  // Apply voltage divider ratio if using one
  voltage = voltage * VOLTAGE_DIVIDER_RATIO;
  
  return voltage;
}

float getBatteryPercentage(float voltage) {
  // For a 9V battery:
  // Full: ~9V
  // Empty: ~6V (cutoff voltage)
  // Adjust these values based on your battery type
  const float BATTERY_MAX = 9.0;
  const float BATTERY_MIN = 6.0;
  
  if (voltage >= BATTERY_MAX) return 100.0;
  if (voltage <= BATTERY_MIN) return 0.0;
  
  return ((voltage - BATTERY_MIN) / (BATTERY_MAX - BATTERY_MIN)) * 100.0;
}

void loop() {
  unsigned long currentMillis = millis();

  // Read sensor data
  float humidity = dht.readHumidity();
  float temperatureC = dht.readTemperature();

  // Check if any reads failed
  if (isnan(humidity) || isnan(temperatureC)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    // Optionally send an error message over BLE
    // bleSerial.println("Error: DHT Read Fail");
    delay(2000); // Wait before trying again
    return;
  }

  // Check temperature status for buzzer control
  temperatureAlert = (temperatureC > TEMP_HIGH_THRESHOLD || temperatureC < TEMP_LOW_THRESHOLD);

  // --- Check for Alerts ---
  if (currentMillis - previousMillisAlert >= alertCheckInterval) {
    previousMillisAlert = currentMillis;
    String currentAlert = ""; // Holds the current alert message, if any

    if (temperatureC > TEMP_HIGH_THRESHOLD) {
      currentAlert = "ALERT: Temp HIGH! T=" + String(temperatureC, 1) + "C";
    } else if (temperatureC < TEMP_LOW_THRESHOLD) {
      currentAlert = "ALERT: Temp LOW! T=" + String(temperatureC, 1) + "C";
    } else if (humidity > HUM_HIGH_THRESHOLD) {
      currentAlert = "ALERT: Humidity HIGH! H=" + String(humidity, 0) + "%";
    } else if (humidity < HUM_LOW_THRESHOLD) {
      currentAlert = "ALERT: Humidity LOW! H=" + String(humidity, 0) + "%";
    } else {
      currentAlert = "STATUS: OK"; // No alert, conditions are normal
    }

    // Send alert message if it's new or different from the last one
    if (currentAlert != lastAlertMessage) {
      bleSerial.println(currentAlert);
      Serial.print("Sent via BLE (Alert/Status): ");
      Serial.println(currentAlert);
      lastAlertMessage = currentAlert;
    }
  }

  // --- Buzzer Control for Temperature Alerts ---
  if (temperatureAlert) {
    // Sound buzzer periodically when temperature is out of range
    if (currentMillis - previousMillisBuzzer >= BUZZER_INTERVAL) {
      previousMillisBuzzer = currentMillis;
      
      // Sound the buzzer
      tone(BUZZER_PIN, BUZZER_FREQUENCY, BUZZER_DURATION);
      delay(1000);
      tone(BUZZER_PIN, BUZZER_FREQUENCY, BUZZER_DURATION);
      delay(1000);
      tone(BUZZER_PIN, BUZZER_FREQUENCY, BUZZER_DURATION);
      Serial.println("BUZZER: Temperature alert!");
    }
  }

  // --- Send Regular Data Periodically ---
  if (currentMillis - previousMillisData >= dataInterval) {
    previousMillisData = currentMillis;

    // Read battery voltage
    float batteryVoltage = readBatteryVoltage();
    float batteryPercent = getBatteryPercentage(batteryVoltage);

    String dataString = "DATA:T=";
    dataString += String(temperatureC, 2);
    dataString += ",H:";
    dataString += String(humidity, 2);
    dataString += ",V:";
    dataString += String(batteryVoltage, 2);
    dataString += ",B:";
    dataString += String(batteryPercent, 0);

    bleSerial.println(dataString);
    Serial.print("Sent via BLE (Data): ");
    Serial.println(dataString);
  }
}
