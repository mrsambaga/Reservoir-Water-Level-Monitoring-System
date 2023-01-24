#include <Wire.h>
#include <LoRa.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_INA219.h>


//define the pins used by the transceiver module
#define ss 5
#define rst 14
#define dio0 2
int counter = 0;

//define sleep duration
#define uS_TO_S_FACTOR 1000000 
#define Sleep_Duration 300 

//define sea level pressure
#define SEALEVELPRESSURE_HPA (1013.25)

//initialize sensors
Adafruit_BMP3XX bmp;
Adafruit_INA219 ina219;

//initialize BMP variable
float Temperature = 0;
float Altitude = 0;
float Pressure = 0;

//initialize INA219 variable
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;

//initialize variable for LoRa message
String LoRaMessage = "";

//Start Lora
void loraStart(){
  LoRa.setPins(ss, rst, dio0);
  
  while (!LoRa.begin(866E6)) {
    Serial.println(".");
    delay(500);
  }

  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");
}

//Start BMP388
void bmpStart(){
  bool bmpStatus = bmp.begin_I2C();  
  if (!bmpStatus) {
    Serial.println("Could not find a valid BMP388 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

//Start INA219
void inaStart(){
  bool inaStatus = ina219.begin();  
  if (!inaStatus){
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
}

//Transmit LoRa message
void loraTransmit(){
  Serial.print("Sending packet: ");
  Serial.println(counter);

  //Send LoRa packet to receiver
  LoRaMessage = String(Temperature) + "&" + String(Pressure) + "#" + String(Altitude) + "/" + String(busvoltage);
  LoRa.beginPacket();
  LoRa.print(LoRaMessage);
  LoRa.endPacket();

  counter++;
}

//INA219 sensor reading
void inaReadings(){
  shuntvoltage = ina219.getShuntVoltage_mV();
  loadvoltage = ina219.getBusVoltage_V(); //load voltage & bus voltage are reversed
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  busvoltage = loadvoltage + (shuntvoltage / 1000);
  delay(500);
  shuntvoltage = ina219.getShuntVoltage_mV();
  loadvoltage = ina219.getBusVoltage_V(); //load voltage & bus voltage are reversed
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  busvoltage = loadvoltage + (shuntvoltage / 1000);
}

//BMP388 sensor reading
void bmpReadings(){
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Temperature = bmp.temperature;
  Pressure = bmp.pressure / 100.0;
  Altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  delay(500);
  Temperature = bmp.temperature;
  Pressure = bmp.pressure / 100.0;
  Altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  delay(500);
  Temperature = bmp.temperature;
  Pressure = bmp.pressure / 100.0;
  Altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
}

void setup() {
  // put your setup code here, to run once:
  delay(1000);
  Serial.begin(115200);
  delay(1000);
  while (!Serial);

  esp_sleep_enable_timer_wakeup(Sleep_Duration * uS_TO_S_FACTOR); //Initialize sleep duration

  loraStart();
  inaStart();
  inaReadings();
  bmpStart();
  bmpReadings();
  loraTransmit();
  Serial.flush(); //wait until every string successfully send by LoRa
  esp_deep_sleep_start(); //start deep sleep mode
}

void loop() {
  //When using deep sleep, no need to use loop

}
