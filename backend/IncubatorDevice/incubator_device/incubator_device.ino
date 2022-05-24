#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "DHT.h" 


#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME  "incubator"
#define AIO_KEY       "aio_MVpj39VdsahsJwKWZj72tou0WLAq"

#define dht11 14
#define sound_sensor A0
#define fan_relay 0

int battery = 5;

#define DHTTYPE DHT11

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


uint16_t temp_val;
uint16_t hum_val;

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


void setup_wifi() {

    WiFi.mode(WIFI_STA); 
    Serial.begin(115200);
    
    WiFiManager wm;
    //wm.resetSettings();

    bool res;
    display.setCursor(0, 16);
    // Display static text
    display.clearDisplay();
    display.println("Connect to Incubator wifi to set internet");
    display.display();
    res = wm.autoConnect("incubator","Incubator@12345"); // password protected ap
 
    

    if(!res) {
        Serial.println("Failed to connect");
        // ESP.restart();
        display.setCursor(0, 16);
        // Display static text
        display.clearDisplay();
        display.println("Failed to connect to Wifi");
        display.display(); 
    } 
    else {
        //if you get here you have connected to the WiFi    
        Serial.println("connected...yeey :)");
        display.setCursor(0, 16);
        // Display static text
        display.clearDisplay();
        display.println("connected to Wifi");
        display.display();
    }

}

void setup(){
  Serial.begin(115200);
  dht.begin();
  while(!Serial) delay(1);

  Serial.println("All state initialized");

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
//  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  // display.setCursor(0, 16);
  // // Display static text
  // display.println("smartIncubator");
  // display.display(); 
  // display.clearDisplay();

  setup_wifi();

  mqtt.subscribe(&Temperature_set);
  mqtt.subscribe(&Humidity_set);
  pinMode(fan_relay, OUTPUT);
  pinMode(battery, INPUT);
  pinMode(sound_sensor, INPUT);

}

void loop(){

  MQTT_connect();

  // dht sensor reading
  float h = dht.readHumidity();
  float t = dht.readTemperature(); 
  Temperature.publish(t);
  Humidity.publish(h);

  display.setCursor(0, 16);
    // Display static text
  display.clearDisplay();
  display.print("Temp: ");
  display.println(t);
  display.setCursor(0, 26);
  display.print("HUM: ");
  display.println(h);
  display.display();
  Serial.println(h);
  Serial.println(t);

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
      if (subscription == &Temperature_set) {
//      Serial.print(F("Temperature_set: "));
//      Serial.println((char *)Temperature_set.lastread);
      temp_val = atoi((char *)Temperature_set.lastread);  // convert to a number
    }
    if (subscription == &Humidity_set) {
      Serial.print(F("Humidity_set: "));
      Serial.println((char *)Humidity_set.lastread);
      hum_val = atoi((char *)Humidity_set.lastread);  // convert to a number
    }
  }
  float sound_val = analogRead(sound_sensor);
  Data_stream.publish(sound_val);
  if(t >= temp_val && h >= hum_val){
    digitalWrite(fan_relay, HIGH);
  }
  else{
    digitalWrite(fan_relay, LOW);
  }
  
  delay(2000);

}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    display.setCursor(0, 16);
    // Display static text
    display.clearDisplay();
    display.println("connected to mqtt server");
    display.display();
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}
