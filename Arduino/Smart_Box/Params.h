#ifndef PARAMS_H
#define PARAMS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"


namespace Params
{
#define EEPROM_SIZE                 256

#define TYPE_VOID                   0
#define TYPE_SET                    1
#define TYPE_GET                    2

#define DWIN_PARAM_ON               4
#define DWIN_PARAM_OFF              3

extern SemaphoreHandle_t xSemaphore;
extern float params[255];
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

// -----------------------------------------------------
// #define DEVICE_ADDR_DWIN_COMPR_ON_SET       0x2030
// #define DEVICE_ADDR_DWIN_COMPR_ON_VAL       0x2031
// #define DEVICE_ADDR_DWIN_FAN_ON_SET         0x2032
// #define DEVICE_ADDR_DWIN_FAN_ON_VAL         0x2033
// #define DEVICE_ADDR_DWIN_EXCHANGER_ON_SET   0x2034
// #define DEVICE_ADDR_DWIN_EXCHANGER_ON_VAL   0x2035


  // struct Params_t
  // {


  //   float compr_speed;
  //   float fan_speed;

  //   int compr_on;
  //   int fan_on;
  //   int exhanger_on;


  // };

  // extern Params_t params;


  void Params_Init();
  bool Set_Param(int id, float value);
  bool Get_Param(int id, float &value);

  void EEPROM_Write_Params();




}

#endif // PARAMS_H
