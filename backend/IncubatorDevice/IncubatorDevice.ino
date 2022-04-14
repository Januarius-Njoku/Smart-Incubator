/* Please see documentation and circuit design
 *  
 * Tx on esp == Value +
 * Rx on esp == value -
 * D6 on esp == menue +
 * D5 on esp == value +
 * 
 * 
 */

#include <ESP8266WiFi.h>
#include <PubSubClient.h> 
#include <WiFiClientSecure.h>
#include <WiFiManager.h>
#include <ESP8266mDNS.h>
#include <WiFiClient.h>
#include <math.h>
#include <EEPROM.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <Servo.h>

//connect the servos to these pins
#define servo1_pin 
#define servo2_pin

//connect the DHT11 sensor to this pin and define the types of sensor used
#define dht_pin 3
#define dht_type DHT11

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels



#define EEPROM_SIZE 100

#ifdef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP8266 ONLY!)
#error Select ESP8266 board.
#endif

Servo servo1;
Servo servo2;
DHT dht(dht_pin, dht_type);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//Server Connections
const char* mqtt_server = "0657fea3e3c14f5cb989d7bffd886dfd.s1.eu.hivemq.cloud";
const int mqtt_port =8883;
const char* mqtt_username = "smartIncubator";
const char* mqtt_password = "SmartIncubator@1234";

//Message buffer
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (1024)
char msg[MSG_BUFFER_SIZE];
int value =0;

/***
 Device battery Status declarations
***/
float tempreature     = 0;
float humidity        = 0;
float inputVoltage    = 0;
float maxPower        = 0;
float heaterOntime    = 0;


//int relay d3;
//int dht d0;
//int sound A0;
//int servo1 d7;
//int servo d8;



bool accesspoint = true;
//Time elapsed
int milliseconds=0;

//Configuration Variable
const char *fingerprint PROGMEM = "16 5B 4B A3 A7 8A 59 C6 14 44 CA FF 3E AF F4 7C 70 7D B1 9F";

//Input & Button Logic
const int numOfInputs = 4;
const int inputPins[numOfInputs] = {1, 14, 12, 3};
int inputState[numOfInputs];
int lastInputState[numOfInputs] = {LOW,LOW,LOW,LOW};
bool inputFlags[numOfInputs] = {LOW,LOW,LOW,LOW};
long lastDebounceTime[numOfInputs] = {0,0,0,0};
long debounceDelay = 5;

//OLED Menu Logic
const int numOfScreens = 5;// number of menues
int currentScreen = 0;
String screens[numOfScreens][1] = {{"Tempreature"}, {"humidity"}, 
  {"inputVoltage"},{"maxPower"},{"heaterOntime"}};
int parameters[numOfScreens];

int ledPin =2;
int dataPin =0;

int pos=0;

WiFiClientSecure espClient;
PubSubClient client(espClient);

bool ispublished;


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


void callback(char* topic, byte* payload, unsigned int length) {
      Serial.print("Message arrived [");
      Serial.print(topic);
      Serial.print("] ");
      
      for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
      }
      Serial.println();

      if ((char)payload[0] == '0') {
        Serial.println("OFF");
        digitalWrite(ledPin, HIGH); 
        digitalWrite(dataPin,HIGH); 
      }
      else if ((char)payload[0] == '1') {
        Serial.println("ON");
        digitalWrite(ledPin, LOW);  
         digitalWrite(dataPin,LOW); 
      }
}


void reconnect() {
  
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "wattflow";
    clientId += String(2345);
    if(client.connect(clientId.c_str(),mqtt_username,mqtt_password,"holder",0,false,"holder",true)) {
      
        Serial.println("wattflow connected");
    
        tempreature=   sin(random(1,10))*40;
        humidity    = 50 + sin(random(1,10))*50;
        humidity    = 50 + sin(random(1,10))*50;
        maxPower    = 300 + sin(random())*100;
        heaterOntime   = ceil(sin(random(1,1000)));
        
        snprintf(msg, MSG_BUFFER_SIZE, "{\"tempreature\":{\"value\":%f,\"unit\":\"C\"},\"humidity\":{\"value\":%f,\"unit\":\"H\"},\"inputVoltage\":{\"value\":%f,\"unit\":\"V\"},\"maxPower\":{\"value\":%f,\"unit\":\"W\"},\"heaterOntime\":{\"value\":%f,\"unit\":\" \"}}", tempreature,humidity,inputVoltage,maxPower,heaterOntime);
        ispublished =   client.publish("data/monitor/incubatorStatus", msg);
        
    
        client.subscribe("2345/control/deviceState");
        client.subscribe("2345/control/deviceSettings");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  dht.begin();

  
  servo1.attach(servo1_pin);
  servo2.attach(servo2_pin);

  //set default point or starting point for the servos
  servo1.write(0);
  servo2.write(0);

  for(int i = 0; i < numOfInputs; i++) {
    pinMode(inputPins[i], INPUT);
    digitalWrite(inputPins[i], HIGH); // pull-up 20k
  }

  //setup oled 
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 16);
  // Display static text
  display.println("smartIncubator");
  display.display();
  delay(5000);
  display.clearDisplay();

  //setup broker
  while(!Serial) delay(1);

  Serial.println("All state initialized");

  setup_wifi();
  espClient.setFingerprint(fingerprint);

  //set to not secure if the finger prints are incorrect
  espClient.setInsecure()
  
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  bool sizeBuff= client.setBufferSize(512);
  Serial.println(sizeBuff);
  pinMode(ledPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  client.subscribe("2345/control/deviceState");
  client.subscribe("2345/control/deviceSettings");
}

void loop() {

  float t = dht.readTemperature();
  float h = dht.readHumidity();

  if(isnan(t) || isnan(h)){
    Serial.print("failed to read value from DHT11");
    return;
  }

  float hi = dht.computeHeatIndex(t);
  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  unsigned long now = millis();
  milliseconds = now;
  if (now - lastMsg > 5000) {
        lastMsg = now;
        if(value<30)value++;
        else value=5;
        tempreature=   sin(random(1,10))*40;
        humidity    = 50 + sin(random(1,10))*50;
        humidity    = 50 + sin(random(1,10))*50;
        maxPower    = 300 + sin(random())*100;
        heaterOntime   = ceil(sin(random(1,1000)));
        snprintf(msg, MSG_BUFFER_SIZE, "{\"tempreature\":{\"value\":%f,\"unit\":\"C\"},\"humidity\":{\"value\":%f,\"unit\":\"H\"},\"inputVoltage\":{\"value\":%f,\"unit\":\"V\"},\"maxPower\":{\"value\":%f,\"unit\":\"W\"},\"heaterOntime\":{\"value\":%f,\"unit\":\" \"}}", tempreature,humidity,inputVoltage,maxPower,heaterOntime);
        ispublished =   client.publish("data/monitor/incubatorStatus", msg);
  }

  setInputFlags();
  resolveInputFlags();

}

void setInputFlags() {
  for(int i = 0; i < numOfInputs; i++) {
    int reading = digitalRead(inputPins[i]);
    if (reading != lastInputState[i]) {
      lastDebounceTime[i] = millis();
    }
    if ((millis() - lastDebounceTime[i]) > debounceDelay) {
      if (reading != inputState[i]) {
        inputState[i] = reading;
        if (inputState[i] == HIGH) {
          inputFlags[i] = HIGH;
        }
      }
    }
    lastInputState[i] = reading;
  }
}

void resolveInputFlags() {
  for(int i = 0; i < numOfInputs; i++) {
    if(inputFlags[i] == HIGH) {
      inputAction(i);
      inputFlags[i] = LOW;
      printScreen();
    }
  }
}

void inputAction(int input) {
  if(input == 0) {
    if (currentScreen == 0) {
      currentScreen = numOfScreens-1;
    }else{
      currentScreen--;
    }
  }else if(input == 1) {
    if (currentScreen == numOfScreens-1) {
      currentScreen = 0;
    }else{
      currentScreen++;
    }
  }else if(input == 2) {
    parameterChange(0);
  }else if(input == 3) {
    parameterChange(1);
  }
}

void parameterChange(int key) {
  if(key == 0) {
    parameters[currentScreen]++;
  }else if(key == 1) {
    parameters[currentScreen]--;
  }
}

void printScreen() {
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 2);
  display.println(screens[currentScreen][0]);
  display.setCursor(0,10);
  display.println(parameters[currentScreen]);
  display.display();
  display.clearDisplay();
}
