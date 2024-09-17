
#include "Params.h"
#include "EEPROM.h"
#include "DS18B20.h"

#define EEPROM_SIZE                 sizeof(float) * EEPROM_PARAM_COUNT + DS18B20_COUNT * (8 * sizeof(uint8_t))

namespace Params
{
  static const char *TAG_PARAM = "PARAMS";
  SemaphoreHandle_t xSemaphore = NULL;
  float params[255];

  bool EEPROM_INIT;

  void Read_DS18B20_Address(int pos)
  {
    // if (!EEPROM_INIT)
    // {
    //   return;
    // }

    // int addr = EEPROM_PARAM_ADDR_END + pos * (8 * sizeof(uint8_t));
    // for (int i = 0; i < 8; i++)    
    // {
    //   addr = addr + i * sizeof(uint8_t);
    //   therm_DS18B20[pos][i] = EEPROM.readByte(addr);
    // }
  }
  void Read_ALL_DS18B20_Address()
  {
    for (int i = 0; i < DS18B20_COUNT; i++)    
    {
      Read_DS18B20_Address(i);
    }
  }


  bool Set_Param(int id, float value)
  {
    bool ret = false;
    if( xSemaphore != NULL )
    {
      if (xSemaphoreTake(xSemaphore, (TickType_t)300) == pdTRUE)
      {
          params[id] = value;
          xSemaphoreGive(xSemaphore);
          return true;
      }
    }
    return ret;
  }

  bool Get_Param(int id, float &value)
  {
    bool ret = false;
    if( xSemaphore != NULL )
    {
      if (xSemaphoreTake(xSemaphore, (TickType_t)300) == pdTRUE)
      {
          value = params[id];
          xSemaphoreGive(xSemaphore);
          return true;
      }
    }
    return ret;
  }

  void EEPROM_Write_Params()
  {
    if (!EEPROM_INIT)
    {
      return;
    }

    int address = 0;

    // EEPROM.writeFloat(address, params.compr_speed);
    // address += sizeof(float);
    // EEPROM.writeFloat(address, params.fan_speed);
    // address += sizeof(float);


    // EEPROM.commit();
  }

  void EEPROM_Read_Params(bool reset)
  {
    if (!EEPROM_INIT)
    {
      return;
    }

    if (!reset)
    {
      int address = 0;
      for (int i = 0; i < EEPROM_PARAM_COUNT; i++)
      {
        params[i] = EEPROM.readFloat(address);
        address += sizeof(float);
      }
    }
    else
    {
      params[PARAM_COMPR_SPEED] = 30.0;
      params[PARAM_COMPR_ON] = 0;




      EEPROM_Write_Params();
    }

  }

  void Params_Init()
  {
    xSemaphore = xSemaphoreCreateMutex();

    if (!EEPROM.begin(EEPROM_SIZE)) 
    {
      ESP_LOGE(TAG_PARAM, "### Failed to initialize EEPROM");
      EEPROM_INIT = false;
    }
    else 
    {
      EEPROM_INIT = true;
    }
    EEPROM_Read_Params(true);
    

  }

} 




















