// Room Climate Monitor - Simplified Version
// Sends temperature, humidity, and atmospheric pressure data every 30 seconds
//
// Pin Connections:
// DHT11 Sensor:
//   - Data Pin -> D4
//   - VCC -> 5V
//   - GND -> GND
//
// BMP180 Pressure Sensor (I2C):
//   - SDA -> A4 (Arduino Uno/Nano canonical I2C data pin)
//   - SCL -> A5 (Arduino Uno/Nano canonical I2C clock pin)
//   - VIN -> 5V
//   - GND -> GND
//
// MH-10 BLE Module:
//   - TXD -> D2 (Arduino RX via SoftwareSerial)
//   - RXD -> D3 (Arduino TX via SoftwareSerial)
//   - VCC -> 5V
//   - GND -> GND

// For DHT Sensor
#include <DHT.h>

// For BMP180 Pressure Sensor
#include <Wire.h>
#include <Adafruit_BMP085.h>

// For BLE Communication
#include <SoftwareSerial.h>

// --- BLE Module (MH-10) Configuration ---
const int bleRxPin = 2; // Arduino RX (Connect to MH-10 TXD)
const int bleTxPin = 3; // Arduino TX (Connect to MH-10 RXD)
SoftwareSerial bleSerial(bleRxPin, bleTxPin); // RX, TX
const long BLE_BAUD_RATE = 9600; // Baud rate for BLE communication

// --- DHT Sensor Configuration ---
#define DHTPIN 4        // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT11 sensor (blue)
DHT dht(DHTPIN, DHTTYPE);

// --- Atmospheric Pressure Configuration ---
Adafruit_BMP085 bmp;
bool bmpConnected = false;
const float ALTITUDE = 1045.0; // Set your altitude in meters (e.g., 200.0 for 200m above sea level) Current Calagary 1045m

// --- Timing ---
unsigned long previousMillisData = 0;    // will store last time data was sent
const long dataInterval = 30000;         // interval at which to send data (30 seconds)

void setup() {
  // Initialize Serial Monitor for debugging
  Serial.begin(9600);
  Serial.println("Room Climate Monitor - Simplified Timer Version");

  // Initialize DHT sensor
  dht.begin();
  Serial.println("DHT sensor initialized.");
  delay(2000); // Allow sensor to stabilize

  // Initialize BLE SoftwareSerial
  bleSerial.begin(BLE_BAUD_RATE);
  Serial.print("BLE Serial initialized at ");
  Serial.print(BLE_BAUD_RATE);
  Serial.println(" baud.");

  // Initialize BMP180 sensor
  if (bmp.begin()) {
    bmpConnected = true;
    Serial.println("BMP180 sensor initialized.");
  } else {
    Serial.println("Could not find BMP180 sensor!");
    Serial.println("Check wiring: SDA->A4, SCL->A5, VIN->5V, GND->GND");
  }
  
  Serial.println("Monitoring climate data...");
}


void loop() {
  unsigned long currentMillis = millis();

  // Send data every 30 seconds
  if (currentMillis - previousMillisData >= dataInterval) {
    previousMillisData = currentMillis;
    
    // Read sensor data
    float humidity = dht.readHumidity();
    float temperatureC = dht.readTemperature();
    
    // Check if any reads failed
    if (isnan(humidity) || isnan(temperatureC)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      delay(2000); // Wait before trying again
      return;
    }

    // Read pressure data if BMP180 is connected
    float pressure = 0;
    float seaLevelPressure = 0;
    if (bmpConnected) {
      pressure = bmp.readPressure() / 100.0; // Convert Pa to hPa (absolute pressure)
      seaLevelPressure = bmp.readSealevelPressure(ALTITUDE) / 100.0; // Sea level adjusted pressure
    }

    // Format data string
    String dataString = "DATA:T=";
    dataString += String(temperatureC, 2);
    dataString += ",H=";
    dataString += String(humidity, 2);
    
    // Add pressure if available
    if (bmpConnected) {
      dataString += ",P=";
      dataString += String(seaLevelPressure, 2);  // Use sea level pressure to match weather apps
      dataString += ",PA=";
      dataString += String(pressure, 2);  // Also send absolute pressure
    }

    // Send data via BLE
    bleSerial.println(dataString);
    
    // Debug output to Serial Monitor
    Serial.print("Sent via BLE (Data): ");
    Serial.println(dataString);
  }
}