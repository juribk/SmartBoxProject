#ifndef DWIN
#define DWIN
#include "Smart_Box.h"

// #define QUEUE_SIZE      8

// struct dwin_message
// {
//   int vp;
//   int value;
// };

// class DWin
// {

//   public:
//     DWin()
//     {
//       xQueue_Rx = xQueueCreate(QUEUE_SIZE, sizeof(dwin_message));
//       xQueue_Tx = xQueueCreate(QUEUE_SIZE, sizeof(dwin_message));
//       xTaskCreate(DWin_Tx_Task, "DWin_Tx_Task", 2048, &xQueue_Tx, tskIDLE_PRIORITY + 1, NULL);

//       Serial.printf("### DWin()\n");
//     }

//     static void DWin_Tx_Task(void* pvParameters)
//     {
//       QueueHandle_t queue_Tx = *((QueueHandle_t*) pvParameters);

//       while(true)  
//       {
//         dwin_message msg;
//         bool receved = xQueueReceive(queue_Tx, &msg, (TickType_t)portMAX_DELAY);
//         if (receved)
//         {
//           uint8_t* buf = (uint8_t*) malloc(8);
//           buf[0] = 0x5A; buf[1] = 0xA5; buf[2] = 0x05; buf[3] = 0x82; 
//           buf[4] = msg.vp >> 8; buf[5] = msg.vp & 0x0F;
//           buf[6] = msg.value >> 8; buf[7] = msg.value & 0x0F;

//           RS485_Buf_t rs485_buf;
//           rs485_buf.buf = buf;
//           rs485_buf.size = 8;
// //          bool sent = xQueueSendToFront(rs485->xQueue_RS485_Tx, &rs485_buf, 0);

//           //uart_write_bytes(UART_NUM_RS485, (const char*)buf, 8);
//         }

//       }
//       vTaskDelete( NULL );
//     }

//     bool Add_Queue_Rx(uint8_t* buf)
//     {
//       dwin_message msg;
//       msg.vp = (buf[4] << 8) + buf[5];
//       msg.value = (buf[6] << 8) + buf[7];

//       //Serial.printf("### Add_Queue_Rx vp=%04X, value=%04X\n", msg.vp, msg.value);
//       bool sent = xQueueSendToFront(xQueue_Rx, &msg, 0);

//       return sent;
//     }
//     bool Get_Queue_Rx(int *vp, int *value, TickType_t xTicksToWait)
//     {
//       dwin_message msg;
//       bool receved = xQueueReceive(xQueue_Rx, &msg, xTicksToWait);
//       if (receved)
//       {
//         *vp = msg.vp;
//         *value = msg.value;
//       }

//       return receved;
//     }
//     bool Add_Queue_Tx(int vp, int value)
//     {
//       dwin_message msg;
//       msg.vp = vp;
//       msg.value = value;
//       bool sent = xQueueSendToFront(xQueue_Tx, &msg, 0);
//       return sent;
//     }


//   private:
//     QueueHandle_t xQueue_Rx;
//     QueueHandle_t xQueue_Tx;



// };
// DWin* dwin;















#endif