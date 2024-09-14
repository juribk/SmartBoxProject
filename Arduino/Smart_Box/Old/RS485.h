#ifndef RS485_H
#define RS485_H

#include "Arduino.h" 
//#include "DWIN.ino"

#define UART_NUM_RS485            UART_NUM_2
#define BUF_SIZE_RS485            128
#define PATTERN_CHR_NUM_RS485     3         

struct RS485_Buf_t
{
  uint8_t *buf;
  int size;
};

class RS485
{
  public:
    QueueHandle_t uart_queue_rs485;
    QueueHandle_t xQueue_RS485_Tx;
    RS485(int tx_io_num, int rx_io_num, int de_io_num, int baud_rate);

  private:
    static void Rs485_Tx_Event_Task(void *pvParameters);
    static void Rs485_Rx_Event_Task(void *pvParameters);

};


#endif // C_RS485
