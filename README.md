# Sensor-Data-to-Cloud
Data collected from Sensor and sent to Cloud

Here, we have used BME280 Sensor by Bosch for collecting Temperature, Pressure and Humidity data. The data is transmitted to cloud using NodeMCU ESP8266 12E.

1. Add BME280_wr.cpp and BME280_wr.h to the libraries.
2. Burn the NodeMCU.ino code to the ESP8266. 
3. Change the WiFi username and password according to your own network.
4. Create and account in io.adafruit.com and copy paste the username and AIO key.
5. Connect the sensor as in "Circuit Diagram" attached.
6. Power Up the NodeMCU and you'll be getting the data in Adafruit.
7. Create an account in IFTTT and google for transfering the data into Google Sheets.
8. In the IFTTT, use the Adafruit based applet for storing the data in sheets.
