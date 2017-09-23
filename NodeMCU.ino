/*
 * To operate the BME280 sensor, we need 4 basic APIs begin, setSensorConfig, setPowerMode, readSensorData. 
 * 
 * The sensor supports different power modes and sampling settings which can be configured based on the user need 
 * by using the following values given in the table,
 *-------------------------------------------------------------------------------------------------------------------
 *        power modes       |   Oversampling settings   |    Standby duration settings  |     Filter settings       |
 *--------------------------|---------------------------|-------------------------------|-------------------------- |
 *      BME280_SLEEP_MODE   | BME280_OVERSAMP_SKIPPED   |  BME280_STANDBY_TIME_1_MS     |  BME280_FILTER_COEFF_OFF  |
 *      BME280_FORCED_MODE  | BME280_OVERSAMP_1X        |  BME280_STANDBY_TIME_63_MS    |  BME280_FILTER_COEFF_2    |
 *      BME280_NORMAL_MODE  | BME280_OVERSAMP_2X        |  BME280_STANDBY_TIME_125_MS   |  BME280_FILTER_COEFF_4    |
 *                          | BME280_OVERSAMP_4X        |  BME280_STANDBY_TIME_250_MS   |  BME280_FILTER_COEFF_8    |
 *                          | BME280_OVERSAMP_8X        |  BME280_STANDBY_TIME_500_MS   |  BME280_FILTER_COEFF_16   |
 *                          | BME280_OVERSAMP_16X       |  BME280_STANDBY_TIME_1000_MS  |                           |
 *                          |                           |  BME280_STANDBY_TIME_10_MS    |                           |
 *                          |                           |  BME280_STANDBY_TIME_20_MS    |                           |
 *-------------------------------------------------------------------------------------------------------------------
 *
 */
#include <BME280_wr.h>

#include <ESP8266WiFi.h>
#include <Adafruit_MQTT_Client.h>

#define wifi "8055"
#define password "ikram32156"

#define server "io.adafruit.com"

#define port 1883
#define username "ikram"
#define key "ac375c34de3e413199b73a8a9af46ee3"

WiFiClient esp;

Adafruit_MQTT_Client mqtt(&esp,server,port,username,key);

Adafruit_MQTT_Publish feed0 = Adafruit_MQTT_Publish(&mqtt,username"/feeds/BME_Temp");

Adafruit_MQTT_Publish feed1 = Adafruit_MQTT_Publish(&mqtt,username"/feeds/BME_Pres");

Adafruit_MQTT_Publish feed2 = Adafruit_MQTT_Publish(&mqtt,username"/feeds/BME_Hum");

BME280 bme280;  //Constructor for I2c with device address (BME280 PIN4)SDO->GND Then Dev_Addr = 0x76

void setup()
{  
  Serial.begin(115200); 
  I2C.begin(2,0); 

  int8_t rslt = bme280.begin();
  if (rslt) 
  {
    Serial.println("BME280 Sensor not connected properly ,Please Check your wiring!");
    while (1);
  }
  Serial.println("Init done");

  Serial.println("Adafruit MQTT demo");
  Serial.print("Connecting to ");
  Serial.println(wifi);

  WiFi.begin(wifi,password);

  while(WiFi.status()!=WL_CONNECTED)
  {
     delay(500);
     Serial.print(".");
  }

  Serial.println("WiFi connected");
  Serial.println("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Connecting to MQTT");

  while(mqtt.connect())
  {
    Serial.print(".");
  }
  
  bme280.humidityOverSampling = BME280_OVERSAMP_1X;
  bme280.pressureOverSampling = BME280_OVERSAMP_1X;
  bme280.temperatureOverSampling = BME280_OVERSAMP_1X;
  bme280.standbyDuration = BME280_STANDBY_TIME_1_MS;
  bme280.filterCoefficient = BME280_FILTER_COEFF_OFF;
  
  bme280.setSensorConfig(); // Configuring sensor's sampling, filter and standby duration settings
  bme280.setPowerMode(BME280_NORMAL_MODE); // Sets sensor to NORMAL mode.
  delay(10); //Wait for the first measurement to complete
}

void loop()
{
  if(mqtt.connected())
  {
    bme280.readSensorData();
  
    Serial.print("\nSending Temp. val");
    Serial.print((bme280.temperature)*2);
    Serial.println("....");
    if(feed0.publish((bme280.temperature)*2))
    {
      Serial.println("Success");
    }

    else
    {
      Serial.println("Fail!");
    }

    delay(800);
    
    Serial.print("\nSending Pressure val");
    Serial.print((bme280.pressure));
    Serial.println("....");
    if(feed1.publish(bme280.pressure))
    {
      Serial.println("Success");
    }

    else
    {
      Serial.println("Fail!");
    }

    delay(800);

    Serial.print("\nSending Pressure val");
    Serial.print((bme280.humidity));
    Serial.println("....");
    if(feed2.publish(bme280.humidity))
    {
      Serial.println("Success");
    }

    else
    {
      Serial.println("Fail!");
    }

    delay(800);
  }
}
