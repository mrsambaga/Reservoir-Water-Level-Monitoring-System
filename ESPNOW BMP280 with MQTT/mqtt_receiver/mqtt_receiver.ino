#include <esp_now.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;

const char* ssid = "ASUS_X008";
const char* password = "12345678";

// MQTT
const char* mqtt_server = "192.168.43.188";  // IP of the MQTT broker
const char* pressure_topic = "pressure";
const char* temperature_topic = "temperature";
const char* altitude_topic = "altitude";
const char* mqtt_username = "sambaga"; // MQTT username
const char* mqtt_password = "sambaga"; // MQTT password
const char* clientID = "client_sambaga"; // MQTT client ID

WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.print("WiFi connected - ESP IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Wi-Fi Channel: ");
  Serial.println(WiFi.channel());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientID, mqtt_username, mqtt_password)) {
      Serial.println("connected to MQTT broker");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

typedef struct struct_message {
  float temperature;
  float pressure;
  float alt;
} struct_message;

struct_message myData;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Pressure : ");
  Serial.print(myData.pressure);
  Serial.println(" %");
  Serial.print("Temperature : ");
  Serial.print(myData.temperature);
  Serial.println(" *C");
  Serial.print("Altitude : ");
  Serial.print(myData.alt);
  Serial.println(" m");

  String ps="Pres: "+String((float)myData.pressure)+" % ";
  String ts="Temp: "+String((float)myData.temperature)+" C ";
  String hs="Alt: "+String((float)myData.alt)+" m ";

  if (client.publish(temperature_topic, String(myData.temperature).c_str())) {
    Serial.println("Temperature sent!");
  }
  else {
    Serial.println("Temperature failed to send. Reconnecting to MQTT Broker and trying again");
    client.connect(clientID, mqtt_username, mqtt_password);
    delay(10); 
    client.publish(temperature_topic, String(myData.temperature).c_str());
  }

  if (client.publish(pressure_topic, String(myData.pressure).c_str())) {
    Serial.println("Pressure sent!");
  }
  else {
    Serial.println("Pressure failed to send. Reconnecting to MQTT Broker and trying again");
    client.connect(clientID, mqtt_username, mqtt_password);
    delay(10);
    client.publish(pressure_topic, String(myData.pressure).c_str());
  }
  if (client.publish(altitude_topic, String(myData.alt).c_str())) {
    Serial.println("Altitude sent!");
  }
  else {
    Serial.println("Altitude failed to send. Reconnecting to MQTT Broker and trying again");
    client.connect(clientID, mqtt_username, mqtt_password);
    delay(10);
    client.publish(altitude_topic, String(myData.alt).c_str());
  }
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_AP_STA);
  setup_wifi();
  client.setServer(mqtt_server, 1883);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

}

void loop() {
  // put your main code here, to run repeatedly:
   if (!client.connected()) {
    reconnect();
  }
  if(!client.loop())
    client.connect("ESP32Client");
}
