//#include "SdFat.h"

#include <ADXL345.h>
#include <Wire.h>

#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>

#include <SPI.h>
#include <Enrf24.h>
#include <nRF24L01.h>
#include <string.h>

#define P2_0 PA8
#define P2_1 PA4
#define P2_2 PA0

Enrf24 radio(P2_0, P2_1, P2_2);  // P2.0=CE, P2.1=CSN, P2.2=IRQ
const uint8_t txaddr[] = { 0xAA, 0xCC, 0xBE, 0xEF, 0x01 };

ADXL345 accelerometer;

float alt;
float x_acc;
float y_acc;
float z_acc;

#define SEALEVELPRESSURE_HPA (1015)
Adafruit_BME280 bme;

//const uint8_t chipSelect = PB12;
//#define FILE_BASE_NAME "Data"
//SdFat sd;
//SdFile file;
//uint32_t logTime;

#define gpsPort Serial1

int endCounter = 0;
bool gpsState = 0;

void setup() {
  Serial.begin(9600);
  //-------------------------------------------------------------------------------
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  radio.begin();
  Serial.println("Radio started.");
  radio.setTXaddress((void*)txaddr);
  //-------------------------------------------------------------------------------
  if (!accelerometer.begin()) {
    Serial.println("ADXL345 didn't started.");
  }
  //-------------------------------------------------------------------------------
  if (!bme.begin()) {
    Serial.println("BME280 didn't started.");
  }
  //-------------------------------------------------------------------------------

  char alt[] = "Alt";
  char xacc[] = "XAcc";
  char yacc[] = "YAcc";
  char zacc[] = "ZAcc";
  //sd.begin(chipSelect, SD_SCK_MHZ(50))
  //file.open(fileName, O_WRONLY | O_CREAT | O_EXCL)

  // Start on a multiple of the sample interval.
  //logTime = micros()/(1000UL*SAMPLE_INTERVAL_MS) + 1;
  //logTime *= 1000UL*SAMPLE_INTERVAL_MS;




  //-------------------------------------------------------------------------------
  gpsPort.begin(9600);
}
void loop() {

  while (gpsPort.available() > 0) {
    char dataraw = gpsPort.read();
    int dataraww = gpsPort.read();
    
    int endline = 36;
    if(dataraw == endline){
      Serial.write("d");
      endCounter++;
    }
    Serial.write(dataraw);
    Serial.write(dataraww);
  }

    Serial.println(endCounter);

  if (gpsState) {


    Vector norm = accelerometer.readNormalize();
    alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
    x_acc = norm.XAxis;
    y_acc = norm.YAxis;
    z_acc = norm.ZAxis;

    String SensorData = String(alt) + "," + String(x_acc) + "," + String(y_acc) + "," + String(z_acc);
    char SensorDataRF[32];
    SensorData.toCharArray(SensorDataRF, 31);

    Serial.println("Altitude, X-Acc, Y-Acc, Z-Acc");
    Serial.println(SensorDataRF);
    if (radio.print(SensorDataRF)) {
      Serial.println("Radio transfered data.");
    }

    radio.flush();

  }
  delay(500);
}
