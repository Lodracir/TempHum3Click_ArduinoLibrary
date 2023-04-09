#ifndef TempHum3_h
#define TempHum3_h

/* Headers */
#include <Arduino.h>
#include "Wire.h"

/* Defines */
#define HDC2010_DEVID_LOW		  0xFE
#define HDC2010_DEVID_HIGH		  0xFF
#define HDC2010_MNFRID_LOW		  0xFC
#define HDC2010_MNFRID_HIGH		  0xFD
#define HDC2010_TEMPREAD_LOW	  0x00
#define HDC2010_TEMPREAD_HIGH	  0x01
#define HDC2010_HUMREAD_LOW		  0x02
#define HDC2010_HUMREAD_HIGH	  0x03
#define HDC2010_MEASURE_CONF	  0x0F
#define HDC2010_RSTRDYINT_CONF	  0x0E

/* enum structure options */
typedef enum
{

  HDC2010_ADDR_0x40,
  HDC2010_ADDR_0x41

}addressMode_t;

typedef enum
{

  HDC2010_FOURTEEN_BITS,
  HDC2010_ELEVEN_BITS,
  HDC2010_NINE_BITS

}resolution_t;  // Resolution bits

typedef enum
{

  HDC2010_TEMP_AND_HUMID,
  HDC2010_TEMP_ONLY

}measurementMode_t; // Measurement mode


/* TempHum3 Sensor Configure Structure */
typedef struct
{
  
  uint8_t             PIN;              // Select ADDR Pin
  addressMode_t       ADDRESS_MODE;     // Select address mode
  measurementMode_t   MODE;             // Select measurement mode
  resolution_t        TEMP_RESOLUTION;  // Select Temperature resolution
  resolution_t        HUMID_RESOLUTION; // Select Humidity resolution

}TempHum3_cfg;

/* Sensor Classs */
class TempHum3
{

  private:
    /* Private Variables */
    uint8_t   Address;
    uint8_t   Pin;
    uint16_t  ManufacturerID;
    uint16_t  DeviceID;
    float     Temperature;
    float     Humidity;
     
    /* Private Functions */
    void    openReg(uint8_t reg);                 // Point to desired sensor register
    uint8_t readReg(uint8_t reg);                 // Read data from previously selected register
    void    writeReg(uint8_t reg, uint8_t data);  // Write data to selected register     
    void    startMeasure(void);   
    void    calculateTemperature(void);
    void    calculateHumidity(void);

  public:
    /* Default Costructor */
    TempHum3(uint8_t Pin);

    /* Init Function */
    void      Init(TempHum3_cfg *devConfig);

    /* Return Manufacturer and Device ID values */
    void      GetDeviceInformation(void);        // Get the Manufacturer and Device ID
    uint16_t  GetManufacturerID(void);
    uint16_t  GetDeviceID(void);

    /* Read Temp and Hum data */
    void      Measure(float *Temperature, float *Humidity);
    float     GetTemperature(void);
    float     GetHumidity(void);

};

#endif