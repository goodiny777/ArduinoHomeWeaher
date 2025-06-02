Project Summary: Room Climate Monitor & Alerter

Version: 1.3 (Added buzzer alerts for temperature)
Date: June 1, 2025

1. Project Goal

To create an Arduino-based device that monitors ambient room temperature and humidity, transmitting this data wirelessly via Bluetooth Low Energy (BLE) to
a smartphone. The system also sends alerts if predefined climate thresholds (temperature or humidity) are breached.

2. Core Components Used

- Microcontroller: Arduino Uno
- Temperature & Humidity Sensor: DHT11 (blue module)
- BLE Module: MH-10 (or compatible HM-10 clone, currently broadcasting as "MHSoft" or similar)
- Power Source: USB or external power supply for Arduino
- Jumper Wires & Breadboard

3. Arduino Pin Connections

(For detailed breadboard layout, refer to the room_climate_wiring document/immersive)

DHT11 Sensor:

- VCC (or '+') Pin: Arduino 5V
- Data (or 'S') Pin: Arduino Digital Pin D2
- GND (or '-') Pin: Arduino GND

MH-10 BLE Module:

- VCC Pin: Arduino 3.3V
- GND Pin: Arduino GND
- TXD Pin (BLE Transmit): Arduino Digital Pin D10 (SoftwareSerial RX)
- RXD Pin (BLE Receive): Arduino Digital Pin D11 (SoftwareSerial TX)

### Buzzer (Implemented):
- **Buzzer Positive:** Arduino Digital Pin D8
- **Buzzer Negative:** Arduino GND

### LED (Optional - Not yet implemented):
- **LED Anode (via resistor):** Arduino Digital Pin D7

### Battery Monitoring:
- **Battery Voltage Input:** Arduino Analog Pin A0 (via voltage divider if battery > 5V)

4. Current Arduino Sketch

The latest working code is in the artifact: arduino_phase4_alert_logic.

This code successfully reads DHT11 sensor data, checks against thresholds, sends both regular data and alert/status messages over BLE, and activates a buzzer when temperature goes outside the defined range.

Key Configuration in Sketch:

- DHTPIN 2
- DHTTYPE DHT11
- bleRxPin 10, bleTxPin 11
- BLE_BAUD_RATE (ensure this is set to the baud rate that worked for your MH-10, e.g., 9600 or 115200)
- Alert thresholds: TEMP_HIGH_THRESHOLD, TEMP_LOW_THRESHOLD, HUM_HIGH_THRESHOLD, HUM_LOW_THRESHOLD
- Battery monitoring: BATTERY_PIN (A0), VOLTAGE_DIVIDER_RATIO (adjust based on your circuit)
- Buzzer: BUZZER_PIN (D8), BUZZER_FREQUENCY (2000Hz), BUZZER_INTERVAL (5 seconds)

5. BLE Communication Protocol for Custom Mobile App

This section details how your future custom mobile app should interact with the MH-10 BLE module.

Device Name:

- The MH-10 module is likely broadcasting a default name like "MHSoft", "HMSoft", "MLT-BT05", or "BT05"
- This name can be changed using AT commands if desired, but for now, your app should scan for one of these common names or allow the user to select from a
  list of discovered BLE devices

Service UUID:

- MH-10/HM-10 modules typically use a standard (though sometimes considered proprietary or vendor-specific) service for UART-like communication
- The most common Service UUID is: 0000FFE0-0000-1000-8000-00805F9B34FB

Characteristic UUID (for Read/Notify):

Within the FFE0 service, the characteristic used for sending data from the Arduino/MH-10 to your phone (and for your phone to receive it) is typically:
- 0000FFE1-0000-1000-8000-00805F9B34FB
- This characteristic will have the "Notify" property enabled. Your app will need to subscribe to notifications on this characteristic

Data Format (Strings sent from Arduino):

The Arduino sketch sends data as simple newline-terminated strings (println). Your mobile app will receive these strings.

Regular Data Messages:

- Format: DATA:T=XX.XX,H:YY.YY,V:ZZ.ZZ,B:PP  *(Updated: Added battery voltage and percentage)*
- Example: DATA:T=25.70,H:48.30,V:8.50,B:83
- XX.XX is the temperature in Celsius (two decimal places)
- YY.YY is the humidity in percent (two decimal places)
- ZZ.ZZ is the battery voltage in volts (two decimal places)
- PP is the battery percentage (0-100, no decimal)

Alert and Status Messages:

- Format (High Temp Alert): ALERT: Temp HIGH! T=X.XC (e.g., ALERT: Temp HIGH! T=30.5C)
- Format (Low Temp Alert): ALERT: Temp LOW! T=X.XC (e.g., ALERT: Temp LOW! T=17.5C)
- Format (High Humidity Alert): ALERT: Humidity HIGH! H=X% (e.g., ALERT: Humidity HIGH! H=70%)
- Format (Low Humidity Alert): ALERT: Humidity LOW! H=X% (e.g., ALERT: Humidity LOW! H=25%)
- Format (Normal Status): STATUS: OK

Your app will need to parse these strings to extract the type of message (DATA, ALERT, STATUS) and the corresponding values.

Mobile App Interaction Flow:

1. Scan for BLE Peripherals: The app should initiate a BLE scan
2. Identify and Connect:
   - Filter scan results by name (e.g., "MHSoft") or allow the user to select the device
   - Establish a connection to the selected BLE peripheral
3. Discover Services and Characteristics:
   - Once connected, the app must discover the services offered by the peripheral
   - Look for the Service UUID 0000FFE0-....
   - Within that service, look for the Characteristic UUID 0000FFE1-....
4. Subscribe to Notifications:
   - Enable notifications for the FFE1 characteristic. This tells the MH-10 and your phone's BLE stack that your app wants to receive any data sent on this
   characteristic
5. Receive and Process Data:
   - Implement a callback or listener that gets triggered whenever new data (a string) is received from the FFE1 characteristic
   - In the callback:
    - Convert the received byte array to a string (usually UTF-8)
    - Parse the string based on the formats defined above (e.g., check if it starts with "DATA:", "ALERT:", or "STATUS:")
    - Update the app's UI with the temperature, humidity, or alert status
6. Handle Disconnections: Implement logic to handle unexpected disconnections and allow reconnection

6. Next Steps in Project Development

Phase 5: (Optional) Adding Local Indicators:

Implement code to use the Buzzer (on D8) and/or an LED (on D7) to provide local audible/visual alerts when thresholds are breached. This involves adding
pinMode() in setup() and digitalWrite() or tone() calls in the alert logic within loop().

Phase 6: Power Management & Enclosure:

- Consider standalone power options (e.g., 9V battery with a proper voltage regulator for the Arduino, or a USB power bank)
- If using battery power, explore Arduino sleep modes to conserve energy (more advanced)
- Design or find a suitable enclosure for the project

Phase 7: Testing & Refinement:

- Long-term testing for stability and accuracy
- Refine alert thresholds and data sending intervals

Custom Mobile App Development:

Use the BLE communication protocol details above to build your app (e.g., using MIT App Inventor, React Native, Swift for iOS, Kotlin/Java for Android).

This summary should provide a solid foundation for you to resume work on the "Room Climate Monitor & Alerter" and develop your custom mobile application.

