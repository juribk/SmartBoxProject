#ifndef FREQ_H
#define FREQ_H

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

#define BUF_SIZE_RS485            128
#define PATTERN_CHR_NUM_RS485     3         

// --- Протокол обмена ------------------------------
#define FRQ_TYPE_SET              0x06
#define FRQ_TYPE_GET              0x03

#define FREQ_ADDR_COMPR           0x08
#define FREQ_ADDR_FAN             0x09

#define FREQ_CMD_START            0x9CA7
#define FREQ_CMD_SPEED            0x9CA6
#define FREQ_CMD_CURRENT          0x9CF7
#define FREQ_CMD_VOLTAGE          0x9CF8
#define FREQ_CMD_TEMPER           0x9CF9

namespace Freq
{
  struct FREQ_Message_t
  {
    int device;
    int type;
    int addr;
    int value;
  };
  // ---------------------------------------------------

  //extern QueueHandle_t xQueue_FREQ_Rx;
  extern QueueHandle_t xQueue_FREQ_Tx;


  void FREQ_Command(int cmd, int addr, int value);

  void FREQ_Init(int uart_num, int tx_io_num, int rx_io_num, int de_io_num, int baud_rate);
  float Divide(float val, float divider);
  uint16_t CRC_From_Buff(uint8_t *buf, uint16_t length);
}




#endif // C_FREQ
