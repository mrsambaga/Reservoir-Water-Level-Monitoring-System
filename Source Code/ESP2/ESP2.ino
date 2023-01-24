#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>

//define wifi & mqtt server
const char* ssid = "ASUS_X008";
const char* password = "12345679";
const char* mqtt_server = "192.168.43.188";

WiFiClient espClient;
PubSubClient client(espClient);

//Define LoRa pins
#define ss 5
#define rst 14
#define dio0 2

//Initialize LoRa variable
int rssi;
String Temperature;
String Altitude;
String Pressure;
String Voltage;

//Initialize battery variable
float Percentage = 0;

//ESP Status Variable (0 for error, 1 for good)
int ESP1Status = 1;

//RTC for sending ESP status & wifi reconnect
long lastDataReceived = 0; //Last time ESP received LoRa packet (in s)
long timeNow = 0; //Time now (in s)
long period = 360; //If ESP does not receive LoRa packet for a period of time, send ESP 1 status = error (in s)
long checkWifi = 0; //Check wifi for every certain interval

//Start wifi
void wifiStart(){
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

//Reconnect wifi automatically
void wifiReconnect() {
  if ((WiFi.status() != WL_CONNECTED) && (timeNow > checkWifi)) {
    Serial.println("Reconnecting to WiFi..");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    checkWifi = millis() + 24000;
  }
}

//Start LoRa
void loraStart(){
  //Setup LoRa pins
  LoRa.setPins(ss, rst, dio0);
  
  while (!LoRa.begin(866E6)) {
    Serial.println(".");
    delay(500);
  }
   // Change sync word (0xF3) to match the receiver
  LoRa.setSyncWord(0xF3);
}

//Convert battery voltage to percentage
void batteryPercentageConversion() {
  if (Voltage.toFloat() >= 4.1) {
    Percentage = 100;
  }
  else
  if (Voltage.toFloat() >= 4.0 && Voltage.toFloat() < 4.1) {
    Percentage = 92;
  }
  else
  if (Voltage.toFloat() >= 3.9 && Voltage.toFloat() < 4.0) {
    Percentage = 78;
  }
  else
  if (Voltage.toFloat() >= 3.8 && Voltage.toFloat() < 3.9) {
    Percentage = 61;
  }
  else
  if (Voltage.toFloat() >= 3.7 && Voltage.toFloat() < 3.8) {
    Percentage = 43;
  }
  else
  if (Voltage.toFloat() >= 3.6 && Voltage.toFloat() < 3.7) {
    Percentage = 14;
  }
  else
  if (Voltage.toFloat() >= 3.5 && Voltage.toFloat() < 3.6) {
    Percentage = 5;
  }
  else
  if (Voltage.toFloat() >= 3.0 && Voltage.toFloat() < 3.5) {
    Percentage = 1;
  }
  else {
    Percentage = 0;
  }
}

//Set callback for MQTT
void callback(String topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

}

//Reconnect MQTT connection
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      // Subscribe
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//Send data to MQTT server
void publishData(){
  client.publish("ESP32/DATA_TEMPERATUR", String(Temperature).c_str());
  client.publish("ESP32/DATA_TEKANAN", String(Pressure).c_str());
  client.publish("ESP32/DATA_KETINGGIAN", String(Altitude).c_str());
  client.publish("ESP32/DATA_BATERAI", String(Percentage).c_str());
  client.publish("ESP32/DATA_STATUS1", String(ESP1Status).c_str());
}

void setup() {
  //Initialize ESP32
  Serial.begin(115200);
  while (!Serial);
  delay(2000);
  Serial.println("ESP Started Successfully !");

  //Connect to wifi
  WiFi.mode(WIFI_STA);
  wifiStart();
  Serial.println("WIFI Initializing OK !");

  //Initialize LoRa
  loraStart();
  delay(500);
  Serial.println("LoRa Initializing OK !");
  
  //Set MQTT Server
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
}

void loop() {
  //Update current time
  timeNow = millis()/1000;

  //Reconnect wifi every certain interval
  wifiReconnect();
  
  // Parse LoRa Packet
  int packetSize = LoRa.parsePacket();

  // Execute all the code if ESP receive LoRa packet
  if (packetSize) {
    while (LoRa.available()) {

      String LoRaData = LoRa.readString();
      lastDataReceived = timeNow;
      
      // LoRaData format: Temperature&Pressure#Altitude/Voltage
      // String example: 27.43&1002#95.34/4.2
      Serial.print(LoRaData); 
    
      // Get Temperature, Pressure, Altitude, and Battery Voltage
      int pos1 = LoRaData.indexOf('&');
      int pos2 = LoRaData.indexOf('#');
      int pos3 = LoRaData.indexOf('/');
    
      Temperature = LoRaData.substring(0, pos1); //*C
      Pressure = LoRaData.substring(pos1 +1, pos2); //hPa
      Altitude = LoRaData.substring(pos2+1, pos3); //meter
      Voltage = LoRaData.substring(pos3+1, LoRaData.length()); //volt

      //Set ESP 1 status to active
      ESP1Status = 1;

      //Determine battery percentage estmation
      batteryPercentageConversion();

      //Reconnect client MQTT 
      if (!client.connected()) {
        reconnect();
      }
      if(!client.loop()) {
        client.connect("ESP32Client");
      }

      //Publish sensor data to MQTT topic
      publishData();

      //Print sensor data
      Serial.println("");
      Serial.print("Temperature : ");
      Serial.print(Temperature);
      Serial.println(" *C");
      Serial.print("Pressure : ");
      Serial.print(Pressure);
      Serial.println(" hPa");
      Serial.print("Altitude : ");
      Serial.print(Altitude);
      Serial.println(" m");
      Serial.print("Battery : ");
      Serial.print(Percentage);
      Serial.println(" %");
      Serial.print("ESP 1 Status : ");
      Serial.println(ESP1Status);
      
    }
 
    // Get RSSI (Received Signal Strength Indicator)
    rssi = LoRa.packetRssi();
    Serial.print(" with RSSI ");    
    Serial.println(rssi);
  }
  else //If ESP does not receive LoRa packet for a period of time, send ESP 1 status = error 
  if (timeNow - lastDataReceived > period ) {
    ESP1Status = 0;

    client.publish("ESP32/DATA_STATUS1", String(ESP1Status).c_str());
  }

}
