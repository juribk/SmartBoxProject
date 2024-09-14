#ifndef RS485_H
#define RS485_H

#include "Arduino.h" 
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_intr_alloc.h"
//#include "DWIN.ino"

#define BUF_SIZE_RS485            128
#define PATTERN_CHR_NUM_RS485     3         


// struct RS485_Buf_t
// {
//   uint8_t buf[BUF_SIZE_RS485];
//   int size;
// };

// --- Протокол обмена ------------------------------
#define DEVICE_ADDR_FREQ1         0x08
#define DEVICE_ADDR_FREQ2         0x09

#define DEVICE_ADDR_DWIN_COMPR_SPEED_SET    0x1036
#define DEVICE_ADDR_DWIN_COMPR_SPEED_VAL    0x1037
#define DEVICE_ADDR_DWIN_FAN_SPEED_SET      0x1038
#define DEVICE_ADDR_DWIN_FAN_SPEED_VAL      0x1039

#define DEVICE_ADDR_DWIN_COMPR_ON_SET       0x2030
#define DEVICE_ADDR_DWIN_COMPR_ON_VAL       0x2031
#define DEVICE_ADDR_DWIN_FAN_ON_SET         0x2032
#define DEVICE_ADDR_DWIN_FAN_ON_VAL         0x2033
#define DEVICE_ADDR_DWIN_EXCHANGER_ON_SET   0x2034
#define DEVICE_ADDR_DWIN_EXCHANGER_ON_VAL   0x2035

#define DEVICE_ADDR_DWIN_TEST2              0x0002

#define TYPE_SET                            1
#define TYPE_GET                            2
struct RS485_Message_t
{
  int type;
  int device;
  int addr;
  int value;
};
// ---------------------------------------------------

extern QueueHandle_t xQueue_RS485_Rx;
extern QueueHandle_t xQueue_RS485_Tx;
void Rs485_Write(const char* buf, int size, int baud_rate);
void Rs485_Init(int uart_num, int tx_io_num, int rx_io_num, int de_io_num, int baud_rate);
uint16_t CRC_From_Buff(uint8_t *buf, uint16_t length);


#endif // C_RS485
