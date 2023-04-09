/* TempHum3 Header */
#include "TempHum3.h"

/* ----- Private Functions ----- */
void TempHum3::openReg(uint8_t reg)
{

  Wire.beginTransmission(Address);  // Start I2C Transmission
  Wire.write(reg);                        // Point to desired register
  Wire.endTransmission();                 // Finish I2C Transmission

}

uint8_t TempHum3::readReg(uint8_t reg)
{
  
  /* Local Variables */
  uint8_t registerData;

  /* Communication and data obtantion process */
  openReg(reg);                         // Point to required register
  Wire.requestFrom(Address, 1);   // Request 1 byte from device
  Wire.endTransmission();               // End Transmission

  if( 1 <= Wire.available() )
  {
    registerData = Wire.read();
  }

  return registerData;

}

void TempHum3::writeReg(uint8_t reg, uint8_t _data)
{

  Wire.beginTransmission(Address);  // Init I2C Communication
  Wire.write(reg);                  // Point to register
  Wire.write(_data);                // Write data
  Wire.endTransmission();           // End I2C Transmission

}

void TempHum3::GetDeviceInformation(void)
{
  /* Variable Local */
  uint8_t ManufacturerID[2];
  uint8_t DeviceID[2];

  /* ----- Read LOW ID Register ----- */
  openReg(HDC2010_MNFRID_LOW);
  Wire.requestFrom(Address, 1);
  Wire.endTransmission();

  if( 1 <= Wire.available())
  {
    ManufacturerID[0] = Wire.read();
  }

  /* ----- Read HIGH ID Register ----- */
  openReg(HDC2010_MNFRID_HIGH);
  Wire.requestFrom(Address, 1);
  Wire.endTransmission();

  if( 1 <= Wire.available() )
  {
    ManufacturerID[1] = Wire.read();
  }

  /* ----- Read LOW ID Register ----- */  
  openReg(HDC2010_DEVID_LOW);           // Point to Device ID Low Register
  Wire.requestFrom(Address, 1);   // Request 1 byte from open register
  Wire.endTransmission();               // End Transmission

  if( 1 <= Wire.available() )
  {
    DeviceID[0] = Wire.read();
  }

  /* ----- Read HIGH ID Register ----- */ 
  openReg(HDC2010_DEVID_HIGH);          // Point to Device ID High Register
  Wire.requestFrom(Address, 1);   // Request 1 byte from open register
  Wire.endTransmission();

  if( 1 <= Wire.available() )
  {
    DeviceID[1] = Wire.read();
  }

  //Store data
  this->ManufacturerID = ( ManufacturerID[1] << 8 ) + ManufacturerID[0];
  this->DeviceID = ( DeviceID[1] << 8 ) + DeviceID[0];
}

void TempHum3::startMeasure(void)
{
  writeReg(HDC2010_MEASURE_CONF, 0x01); // Start Measurement
}

void TempHum3::calculateTemperature(void)
{
   /* Variables Locales */
  uint8_t   TemperatureValue[2] = { 0x00, 0x00 };
  uint16_t  tempValue = 0x0000;

  const float temp = 165.0/65536.0; //Temporal Value

  /* Read High Device Temperature */
  openReg(HDC2010_TEMPREAD_HIGH);
  Wire.requestFrom(this->Address, 1);
  Wire.endTransmission();

  if( 1 <= Wire.available() )
  {
    TemperatureValue[0] = Wire.read();
  }

  delay(10);  // 10 Seconds Delay
  
  /* Read Low Device Temperature */
  openReg(HDC2010_TEMPREAD_LOW);
  Wire.requestFrom(this->Address, 1);
  Wire.endTransmission();

  if( 1 <= Wire.available() )
  {
    TemperatureValue[1] = Wire.read();
  }

  delay(10);

  /* Calculate Temperature */
  tempValue = (TemperatureValue[0] << 8) + TemperatureValue[1];

  this->Temperature = (temp*tempValue) - 40.0;
}

void TempHum3::calculateHumidity(void)
{
  /* Local Variables */
  uint8_t   HumidityeValue[2] = { 0x00, 0x00 };
  uint16_t  humValue = 0x0000;

  //Multiplication factor
  const float temp = 100.0/65536.0; 

  /* Read High Device Temperature */
  openReg(HDC2010_HUMREAD_HIGH);
  Wire.requestFrom(this->Address, 1);
  Wire.endTransmission();

  if( 1 <= Wire.available() )
  {
    HumidityeValue[0] = Wire.read();
  }

  delay(10);  // 10 Seconds Delay
  
  /* Read Low Device Temperature */
  openReg(HDC2010_HUMREAD_LOW);
  Wire.requestFrom(this->Address, 1);
  Wire.endTransmission();

  if( 1 <= Wire.available() )
  {
    HumidityeValue[1] = Wire.read();
  }

  delay(10);

  /* Calculate Temperature */
  humValue = (HumidityeValue[0] << 8) + HumidityeValue[1];

  this->Humidity = temp*humValue;
}

/* ----- Public Functions ----- */

TempHum3::TempHum3(uint8_t Pin)
{
  //Set ADDR Pin as Output
  this->Pin = Pin;
  pinMode(this->Pin, OUTPUT);

  //Default Address is 0x40
  this->Address = 0x40;
  digitalWrite(this->Pin, LOW);

}

void TempHum3::Init(TempHum3_cfg *devConfig)
{

  /* Local Variables */
  uint8_t temp = 0x00;

  /* Set ADDR Pin as Output */
  pinMode(devConfig->PIN, OUTPUT);

  /* Configure ADDRES Mode */
  switch(devConfig->ADDRESS_MODE)
  {
    case HDC2010_ADDR_0x40:
      digitalWrite(devConfig->PIN, LOW);
      this->Address = 0x40;
      break;

    case HDC2010_ADDR_0x41:
      digitalWrite(devConfig->PIN, HIGH);
      this->Address = 0x41;
      break;

    default:
      break;
  }

  /* Configure Sensor Mode */
  switch(devConfig->MODE)
  {
    case HDC2010_TEMP_AND_HUMID:
      temp |= 0x00;
      break;

    case HDC2010_TEMP_ONLY:
      temp |= 0x02;
      break;
  }

  writeReg(HDC2010_MEASURE_CONF, temp);   // Write to Measure Configuration Register
  temp = 0x00;                            // Clear temp value

  /* Configure Temperature Resolution */
  switch(devConfig->TEMP_RESOLUTION)
  {
    case HDC2010_FOURTEEN_BITS:
      temp |= 0x00;
      break;

    case HDC2010_ELEVEN_BITS:
      temp |= 0x40;
      break;

    case HDC2010_NINE_BITS:
      temp |= 0x80;
      break;
  }

  writeReg(HDC2010_MEASURE_CONF, temp);   // Write to Measure Configuration Register
  temp = 0x00;                            // Clear temp value

  /* Configure Humidity Resolution */
  switch(devConfig->HUMID_RESOLUTION)
  {
    case HDC2010_FOURTEEN_BITS:
      temp |= 0x00;
      break;

    case HDC2010_ELEVEN_BITS:
      temp |= 0x10;
      break;

    case HDC2010_NINE_BITS:
      temp |= 0x20;
      break;
  }

  writeReg(HDC2010_MEASURE_CONF, temp);   // Write to Measure Configuration Register
  temp = 0x00;                            // Clear temp value

}

uint16_t TempHum3::GetDeviceID(void)
{
  return this->DeviceID;
}

uint16_t TempHum3::GetManufacturerID(void)
{
  return this->ManufacturerID;
}

void TempHum3::Measure(float *Temperature, float *Humidity)
{
  //Request measure
  startMeasure();

  //Calculate Temperature and Humidity
  calculateTemperature();
  calculateHumidity();

  //Store data
  (*Temperature) = this->Temperature;
  (*Humidity) = this->Humidity;

}

float TempHum3::GetTemperature(void)
{
  return this->Temperature;
}

float TempHum3::GetHumidity(void)
{
  return this->Humidity;
}