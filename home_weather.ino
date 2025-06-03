// Room Climate Monitor - Simplified Version
// Sends temperature, humidity, and battery data every 30 seconds

// For DHT Sensor
#include <DHT.h>

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

    // Format data string
    String dataString = "DATA:T=";
    dataString += String(temperatureC, 2);
    dataString += ",H=";
    dataString += String(humidity, 2);

    // Send data via BLE
    bleSerial.println(dataString);
    
    // Debug output to Serial Monitor
    Serial.print("Sent via BLE (Data): ");
    Serial.println(dataString);
  }
}