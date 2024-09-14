#ifndef PARAMS_H
#define PARAMS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

namespace Params
{
#define EEPROM_SIZE                 64
#define DWIN_PARAM_ON               4
#define DWIN_PARAM_OFF              3

extern SemaphoreHandle_t xSemaphore;
extern float params[255];
// --- ID params --------------------------------------
#define PARAM_COMPR_SPEED           0x01
#define DWIN_COMPR_SPEED_VAL        0x2010

#define PARAM_COMPR_ON              0x02
#define DWIN_COMPR_ON_VAL           0x2012

#define PARAM_FAN_SPEED             0x03
#define DWIN_FAN_SPEED_VAL          0x2014

#define PARAM_FAN_ON                0x04
#define DWIN_FAN_ON_VAL             0x2016

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

  void EEPROM_Write_Params();




}

#endif // PARAMS_H
