#include <Wire.h>
#include "ADS1115.h"
#include "Params.h"
#include "DWIN.h"

#include <MCP23009.h> 

namespace relay
{
  static const char *TAG = "MCP23009_events";

  MCP23009::MCP23009(uint8_t addr)
  {
    this->addr = addr;
    Init(this->addr);
  }
  // --------------------------------------------------------------------------------------
  bool MCP23009::Init(uint8_t addr)
  {
    this->addr = addr;
    bool ret = true;
    if (!Write_Reg(MCP23009_REG_GPPU, 0xFF)) 
    {
      ret = false;
    }
    if (!Write_Reg(MCP23009_REG_IODIR, 0x00))
    {
      ret = false;
    }
    if (!Write_Reg(MCP23009_REG_GPIO, 0x00))
    {
      ret = false;
    }

    if (!ret)
    {
      ESP_LOGE(TAG, "### Error: MCP23009 initialization error = %d", ret);
    }

    return ret;
  }
  // --------------------------------------------------------------------------------------
  bool MCP23009::Get_GPIO(uint8_t gpio, bool* state)
  {
    uint8_t state_GPIO;
    if (Read_Reg(MCP23009_REG_GPIO, &state_GPIO))
    {
      *state = bitRead(state_GPIO, gpio);
      return true;
    }
    return false;
  }
  // --------------------------------------------------------------------------------------
  bool MCP23009::Set_GPIO(uint8_t gpio, bool state)
  {
    uint8_t state_GPIO;
    if (Read_Reg(MCP23009_REG_GPIO, &state_GPIO))
    {
      state_GPIO = state == RELAY_ON ? bitSet(state_GPIO, gpio) : bitClear(state_GPIO, gpio);
      if (Write_Reg(MCP23009_REG_GPIO, state_GPIO))
      {
        return true;
      }
    }
    return false;
  }
  // --------------------------------------------------------------------------------------
  bool MCP23009::Read_Reg(uint8_t reg, uint8_t *value)
  {
    Wire.beginTransmission(addr);
    Wire.write(0x06); 
    uint8_t ret = Wire.endTransmission();
    if (ret != 0)
    {
      ESP_LOGI(TAG, "### Error Wire.endTransmission() = %d", ret);
      return false;
    }
    else
    {
      Wire.requestFrom(addr, 1);

      int tout = 0;
      while (Wire.available() == 0)
      {
        tout += 1;
        if (tout > 100)
        {
          ESP_LOGI(TAG, "### Error: TimeOut Wire.available()");
          return false;
        }
      }
      *value = Wire.read();
      return true;
    }
  }
  // --------------------------------------------------------------------------------------
  bool MCP23009::Write_Reg(uint8_t reg, uint8_t value)
  {
    Wire.beginTransmission(addr);
    Wire.write(reg); 
    Wire.write(value); 
    uint8_t ret = Wire.endTransmission();
    if (ret != 0)
    {
      ESP_LOGI(TAG, "### Error: Wire.endTransmission() = %d", ret);
    }

    return (ret == 0) ? true : false;
  }
  // --------------------------------------------------------------------------------------


  MCP23009* relay;
  void Relay_Init(uint8_t addr)
  {
    relay = new MCP23009(addr);




  }









}