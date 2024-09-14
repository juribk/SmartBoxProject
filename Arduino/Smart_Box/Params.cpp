#include "Params.h"

#include "EEPROM.h"


namespace Params
{
  SemaphoreHandle_t xSemaphore = NULL;
  float params[255];


  bool EEPROM_INIT;

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

  void EEPROM_Write_Params()
  {
    // if (!EEPROM_INIT)
    // {
    //   return;
    // }

    // int address = 0;

    // EEPROM.writeFloat(address, params.compr_speed);
    // address += sizeof(float);
    // EEPROM.writeFloat(address, params.fan_speed);
    // address += sizeof(float);


    // EEPROM.commit();
  }

  void EEPROM_Read_Params(bool reset)
  {
    // if (!EEPROM_INIT)
    // {
    //   return;
    // }

    // int address = 0;
    // if (!reset)
    // {
    //   params.compr_speed = EEPROM.readFloat(address);
    //   address += sizeof(float);
    //   params.fan_speed = EEPROM.readFloat(address);
    //   address += sizeof(float);

    // }
    // else
    // {
    //   params.compr_speed = 60.0;
    //   params.fan_speed = 60.0;

    //   EEPROM_Write_Params();



    //}

    //DWIN_Set_Param(DEVICE_ADDR_DWIN_COMPR_SPEED_VAL, (int)(params.compr_speed * 10));
    //DWIN_Set_Param(DEVICE_ADDR_DWIN_FAN_SPEED_VAL, (int)(params.fan_speed * 10));
  }

  void Params_Init()
  {
    xSemaphore = xSemaphoreCreateMutex();


    // if (!EEPROM.begin(EEPROM_SIZE)) 
    // {
    //   Serial.println("### Failed to initialize EEPROM");
    //   EEPROM_INIT = false;
    // }
    // else 
    // {
    //   EEPROM_INIT = true;
    // }
    // EEPROM_Read_Params(false);

  }

} 




















