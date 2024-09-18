
#include "DS18B20.h"
#include "Params.h"
#include "EEPROM.h"
#include "DWIN.h"



namespace ds18b20
{
  static const char *TAG = "DS18B20_events";
  OneWire oneWire(DS18B20_PIN);
  DallasTemperature sensors(&oneWire);

  bool DS18B20_INIT = false;

  Sensor::Sensor(int param_index, int dwin_addr_set, int dwin_addr_val)
  {
    this->param_index = param_index;
    this->dwin_addr_set = dwin_addr_set;
    this->dwin_addr_val = dwin_addr_val;
    this->eeprom_addr = EEPROM_PARAM_ADDR_END + param_index * (8 * sizeof(uint8_t));
    Read_Device_Address();


  }
  void Sensor::Read_Device_Address()
  {
    if (!Params::EEPROM_INIT) return;

    int addr = eeprom_addr;
    for (int i = 0; i < 8; i++)    
    {
      device_address[i] = Params::ReadByte(addr);
      addr += sizeof(uint8_t);
    }
    ESP_LOGE(TAG, "Address: %02X %02X %02X %02X %02X %02X %02X %02X", 
      device_address[0], device_address[1], device_address[2], device_address[3], 
      device_address[4], device_address[5], device_address[6], device_address[7]);

  }
  void Sensor::Write_Device_Address()
  {
    if (!Params::EEPROM_INIT) return;

    int addr = eeprom_addr;
    for (int i = 0; i < 8; i++)    
    {
      Params::WriteByte(addr, device_address[i]);
      addr += sizeof(uint8_t);
    }
    Params::Commit();
  }
  void Sensor::Search_Device_Address()
  {
    oneWire.reset_search();
    if (!oneWire.search(device_address)) 
    {
      ESP_LOGE(TAG, "### Unable to find address for device_address");
    }
    else
    {
      ESP_LOGE(TAG, "Search Address: %02X %02X %02X %02X %02X %02X %02X %02X", 
        device_address[0], device_address[1], device_address[2], device_address[3], 
        device_address[4], device_address[5], device_address[6], device_address[7]);
        Write_Device_Address();
    }

  }

  // ------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------
  Sensor *sensor[DS18B20_COUNT];

  Sensor* Get_Sensor_Dwin_Addr_Set(int dwin_addr_set)
  {
    return sensor[dwin_addr_set - DWIN_TEMP_HOT_IN_SET];
  }
  Sensor* Get_Sensor_Dwin_Addr_Val(int dwin_addr_val)
  {
    return sensor[dwin_addr_val - DWIN_TEMP_VAL];
  }

  void Update()
  {
    sensors.requestTemperatures();    
    for (int i = 0; i < DS18B20_COUNT; i++)
    {
      float temp = sensors.getTempC((uint8_t*) sensor[i]->device_address);
      sensor[i]->value = temp;
      DWIN_Send(sensor[i]->dwin_addr_val, (int)(temp * 10.0));
    }
  }

  void Init()
  {
    sensors.begin();

    //sensor[0] = new Sensor(0, DWIN_TEMP_HOT_IN_SET, DWIN_TEMP_HOT_IN);
    for (int i = 0; i < DS18B20_COUNT; i++)
    {
      sensor[i] = new Sensor(i, DWIN_TEMP_SET + i, DWIN_TEMP_VAL + i);
    }


  }
}
