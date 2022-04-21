#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#include "DHT.h" 


#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "incubator"
#define AIO_KEY         "aio_NFap80FWRfuPzUt5IehGuuxWSVXU" 

#define dht11 15
#define sound_sensor
#define fan_relay

#define DHTTYPE DHT11
DHT dht(dht11, DHTTYPE);

WiFiClient client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Publish Temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Temperature");
Adafruit_MQTT_Publish Humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Humidity");
Adafruit_MQTT_Publish Battery = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Battery");
Adafruit_MQTT_Publish Data_stream = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Data Strean");
Adafruit_MQTT_Publish on_switch = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/on_switch");

Adafruit_MQTT_Subscribe Temperature_set = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Temp_set");
Adafruit_MQTT_Subscribe Humidity_set = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Hum_set");

void MQTT_connect();

int test = 1;
int battery = 51;

void setup_wifi() {

    WiFi.mode(WIFI_STA); 
    Serial.begin(115200);
    
    WiFiManager wm;
    //wm.resetSettings();

    bool res;
    res = wm.autoConnect("incubator","Incubator@12345"); // password protected ap

    if(!res) {
        Serial.println("Failed to connect");
        // ESP.restart();
    } 
    else {
        //if you get here you have connected to the WiFi    
        Serial.println("connected...yeey :)");
    }

}

void setup(){
      Serial.begin(115200);
  while(!Serial) delay(1);

  Serial.println("All state initialized");

  setup_wifi();

  mqtt.subscribe(&Temperature_set);
  mqtt.subscribe(&Humidity_set);
}

void loop(){

    MQTT_connect();

    // dht sensor reading
    float h = dht.readHumidity();
    float t = dht.readTemperature(); 


    if (! Temperature.publish(t) || ! Humidity.publish(h)) {
      Serial.println(F("Failed"));
    } 
    delay(1000);

}

void MQTT_connect() {
  int8_t ret;

  if (mqtt.connected()) {
    Serial.println("mqtt connected");
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000); 
    retries--;
    if (retries == 0) {
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
  
}
