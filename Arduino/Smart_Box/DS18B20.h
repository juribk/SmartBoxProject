#ifndef DS18B20_H
#define DS18B20_H

#include <OneWire.h>
#include <DallasTemperature.h>

#define DS18B20_COUNT               10
#define DS18B20_PIN                 13

namespace ds18b20
{

  // //extern DeviceAddress therm_DS18B20[DS18B20_COUNT];

  class Sensor 
  {
    public:
      int dwin_addr_set;
      int dwin_addr_val;
      DeviceAddress device_address;
    private:
      int param_index;
      int eeprom_addr;

    public:
      Sensor(int param_index, int dwin_addr_set, int dwin_addr_val);

      void Read_Device_Address();
      void Write_Device_Address();
      void Search_Device_Address();


  };

  extern Sensor *sensor[DS18B20_COUNT];

  Sensor* Get_Sensor_Dwin_Addr_Set(int dwin_addr_set);
  Sensor* Get_Sensor_Dwin_Addr_Val(int dwin_addr_val);
  void Init();
  void Update();
}


#endif // DS18B20_H
