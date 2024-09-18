#ifndef PARAMS_H
#define PARAMS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include <OneWire.h>
#include <DallasTemperature.h>


namespace Params
{
#define EEPROM_PARAM_COUNT          128
#define EEPROM_PARAM_ADDR_END       sizeof(float) * EEPROM_PARAM_COUNT
#define EEPROM_DS18B20_ADDR_END     EEPROM_PARAM_ADDR_END + param_index * (DS18B20_COUNT * sizeof(uint8_t))



#define TYPE_VOID                   0
#define TYPE_SET                    1
#define TYPE_GET                    2

#define DWIN_PARAM_ON               4
#define DWIN_PARAM_OFF              3

extern SemaphoreHandle_t xSemaphore;
extern float params[255];
extern bool EEPROM_INIT;


// --- ID params --------------------------------------
#define PARAM_COMPR_SPEED           0x01
#define DWIN_COMPR_SPEED            0x2010

#define PARAM_COMPR_ON              0x02
#define DWIN_COMPR_ON               0x2015

#define PARAM_FAN_SPEED             0x03
#define DWIN_FAN_SPEED              0x201A

#define PARAM_FAN_ON                0x04
#define DWIN_FAN_ON                 0x201F

#define PARAM_EXCHANGER_ON          0x05
#define DWIN_EXCHANGER_ON           0x2024

// ----------------------------------------------------
#define PARAM_COMPR_CURRENT         0x06
#define PARAM_COMPR_VOLTAGE         0x07
#define DWIN_COMPR_POWER            0x2100 + 10 // 210A
#define PARAM_FAN_CURRENT           0x08
#define PARAM_FAN_VOLTAGE           0x09
#define DWIN_FAN_POWER              0x2100 + 15 // 210F

#define PARAM_COMPR_FREQ_TEMPER     0x0A
#define DWIN_COMPR_FREQ_TEMPER      0x2100 + 20 // 2114
#define PARAM_FAN_FREQ_TEMPER       0x0B
#define DWIN_FAN_FREQ_TEMPER        0x2100 + 25 // 2119

// --- DS18B20 -----------------------------------------
#define DWIN_TEMP_SET               0x2132
#define DWIN_TEMP_HOT_IN_SET        DWIN_TEMP_SET + 00 // 2132
#define DWIN_TEMP_HOT_OUT_SET       DWIN_TEMP_SET + 01 // 2133
#define DWIN_TEMP_COLD_IN_SET       DWIN_TEMP_SET + 02 // 2134
#define DWIN_TEMP_COLD_OUT_SET      DWIN_TEMP_SET + 03 // 2135
#define DWIN_TEMP_COMPRESSOR_SET    DWIN_TEMP_SET + 04 // 2136
#define DWIN_TEMP_EXCHANGER_SET     DWIN_TEMP_SET + 05 // 2137

#define DWIN_TEMP_VAL               0x2146
#define DWIN_TEMP_HOT_IN            DWIN_TEMP_VAL + 00 // 2132 + 20
#define DWIN_TEMP_HOT_OUT           DWIN_TEMP_VAL + 01 // 2147
#define DWIN_TEMP_COLD_IN           DWIN_TEMP_VAL + 02 // 2148
#define DWIN_TEMP_COLD_OUT          DWIN_TEMP_VAL + 03 // 2149
#define DWIN_TEMP_COMPRESSOR        DWIN_TEMP_VAL + 04 // 214A
#define DWIN_TEMP_EXCHANGER         DWIN_TEMP_VAL + 05 // 214B

// --- ADS1115 ------------------------------------------
#define DWIN_ADC_VAL                0x215F
#define DWIN_ADC00_VOLT_220         DWIN_ADC_VAL + 00   // 0x215F                 
#define DWIN_ADC01_CURRENT_220      DWIN_ADC_VAL + 01   // 0x2160                
#define DWIN_ADC02                  DWIN_ADC_VAL + 02   // 0x2161    
#define DWIN_ADC03                  DWIN_ADC_VAL + 03   // 0x2162                
#define DWIN_ADC04                  DWIN_ADC_VAL + 04   // 0x2163                
#define DWIN_ADC05                  DWIN_ADC_VAL + 05   // 0x2164                
#define DWIN_ADC06                  DWIN_ADC_VAL + 06   // 0x2165                
#define DWIN_ADC07                  DWIN_ADC_VAL + 07   // 0x2166                
#define DWIN_ADC08                  DWIN_ADC_VAL + 08   // 0x2167                
#define DWIN_ADC09                  DWIN_ADC_VAL + 09   // 0x2168                
#define DWIN_ADC10                  DWIN_ADC_VAL + 10   // 0x2169                
#define DWIN_ADC11                  DWIN_ADC_VAL + 11   // 0x216A
#define DWIN_ADC12                  DWIN_ADC_VAL + 12   // 0x216B                
#define DWIN_ADC13                  DWIN_ADC_VAL + 13   // 0x216C                
#define DWIN_ADC14                  DWIN_ADC_VAL + 14   // 0x216D                
#define DWIN_ADC15                  DWIN_ADC_VAL + 15   // 0x216E                








// #define DEVICE_ADDR_DWIN_COMPR_ON_SET       0x2030
// #define DEVICE_ADDR_DWIN_COMPR_ON_VAL       0x2031
// #define DEVICE_ADDR_DWIN_FAN_ON_SET         0x2032
// #define DEVICE_ADDR_DWIN_FAN_ON_VAL         0x2033
// #define DEVICE_ADDR_DWIN_EXCHANGER_ON_SET   0x2034
// #define DEVICE_ADDR_DWIN_EXCHANGER_ON_VAL   0x2035

#define GET_BIT(DATA, NUM_BIT) ((DATA >> NUM_BIT) & 0x01)




  void Params_Init();
  bool Set_Param(int id, float value);
  bool Get_Param(int id, float &value);

  void EEPROM_Write_Params();
  bool Commit();
  bool WriteByte(int addr, uint8_t value);
  uint8_t ReadByte(int addr);




}

#endif // PARAMS_H
