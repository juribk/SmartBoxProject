#ifndef DWIN_H
#define DWIN_H

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

#define BUF_SIZE_DWIN             128
#define DEVICE_ADDR_DWIN          0x5A

extern QueueHandle_t xQueue_DWIN_Rx;
extern QueueHandle_t xQueue_DWIN_Tx;

void DWIN_Init(int uart_num, int tx_io_num, int rx_io_num, int de_io_num, int baud_rate);

#endif // DWIN_H