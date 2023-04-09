#include <ArduinoBLE.h>
#include <Wire.h>
#include "TempHum3.h"

/** 
 * Local defines
 **/
#define HDC2010_ADDRESS_PIN 10
#define ARDUINO_LED_PIN     13
#define SERIAL_BAUDRATE     115200
#define MEASUREMENT_TIME    10      //Seconds

/** 
 * Local variables 
 **/
static float Temperature = 0.0, Humidity = 0.0;
static uint16_t SensorManufacturerID = 0, SensorDeviceID = 0;
static uint16_t TimeCount = 0;

/** 
 * Object declaration 
 **/
TempHum3 HDC2010(HDC2010_ADDRESS_PIN);
//BLEService battery_service("180F");

void setup() {

    //Configure required pins
    pinMode(ARDUINO_LED_PIN,OUTPUT);

    //Init Requerided Modules
    Wire.begin();
    Serial.begin(SERIAL_BAUDRATE);

    //Get HDC2010 Information
    HDC2010.GetDeviceInformation();
    SensorManufacturerID = HDC2010.GetManufacturerID();
    SensorDeviceID = HDC2010.GetDeviceID();
}

void loop() {

  digitalWrite(ARDUINO_LED_PIN,LOW);
  delay(1000);
  digitalWrite(ARDUINO_LED_PIN,HIGH);

  TimeCount++;

  if(TimeCount > MEASUREMENT_TIME)
  {
    HDC2010.Measure(&Temperature, &Humidity);

    Serial.print("Temperature: ");
    Serial.print(Temperature);
    Serial.println(" C");

    Serial.print("Humidity: ");
    Serial.print(Humidity);
    Serial.println(" %RH");

    TimeCount = 0;
  }
}