#include "DWIN.h"
#include "Params.h"


static const char *TAG_DWIN = "DWIN_events";

QueueHandle_t xQueue_DWIN_Rx;
QueueHandle_t xQueue_DWIN_Tx;
QueueHandle_t xQueue_DWIN;

int m_dwin_uart_num;

struct DWIN_Message_t
{
  int type;
  int addr;
  int value;
};

void DWIN_Rx_Task(void* pvParameters)
{
  QueueHandle_t queue = *((QueueHandle_t*) pvParameters);

  DWIN_Message_t msg_rx;
  // RS485_Message_t msg_tx;
  // bool sent;

   while(1)
   {
      bool receved = xQueueReceive(queue, &msg_rx, (TickType_t)portMAX_DELAY);
      if (receved)
      {

//       Serial.printf("### Get_Queue_Rx DWIN  Device=%04X, Addr=%04X, Value=%04X\n", msg_rx.device, msg_rx.addr, msg_rx.value);        
//       tx_ok = false;
//       msg_tx.device = msg_rx.device;

//       switch (msg_rx.addr)
//       {
//         case DEVICE_ADDR_DWIN_COMPR_SPEED_SET:
//           msg_tx.type = TYPE_SET;
//           msg_tx.device = DEVICE_ADDR_FREQ1;
//           msg_tx.value = msg_rx.value * 10;
//           msg_tx.addr = 0x9CA6;
//           sent = xQueueSendToFront(xQueue_RS485_Tx, &msg_tx, 0);


//           // msg_tx.addr = DEVICE_ADDR_DWIN_COMPR_SPEED_VAL;
//           // msg_tx.value = msg_rx.value;
//           // params.compr_speed = (float)msg_rx.value / 10;
//           // EEPROM_Write_Params();
//           // tx_ok = true;

//           break;
//         case DEVICE_ADDR_DWIN_FAN_SPEED_SET:
//           msg_tx.addr = DEVICE_ADDR_DWIN_FAN_SPEED_VAL;
//           msg_tx.value = msg_rx.value;
//           params.fan_speed = (float)msg_rx.value / 10;
//           EEPROM_Write_Params();
//           tx_ok = true;

//           break;
//         case DEVICE_ADDR_DWIN_COMPR_ON_SET:
//           msg_tx.type = TYPE_SET;
//           msg_tx.device = DEVICE_ADDR_FREQ1;
//           msg_tx.addr = 0x9CA7;
//           if (msg_rx.value == DWIN_PARAM_ON)
//           {
//             msg_tx.value = 0x0001;
//           }
//           else
//           {
//             msg_tx.value = 0x0000;
//           }
//           sent = xQueueSendToFront(xQueue_RS485_Tx, &msg_tx, 0);
//           break;
//         case DEVICE_ADDR_DWIN_FAN_ON_SET:
//           msg_tx.addr = DEVICE_ADDR_DWIN_FAN_ON_VAL;
//           msg_tx.value = msg_rx.value;
//           state.fan_on = msg_rx.value;
//           tx_ok = true;

//           break;
//         case DEVICE_ADDR_DWIN_EXCHANGER_ON_SET:
//           msg_tx.addr = DEVICE_ADDR_DWIN_EXCHANGER_ON_VAL;
//           msg_tx.value = msg_rx.value;
//           state.exhanger_on = msg_rx.value;
//           tx_ok = true;

//           break;

//         default:

//           break;

//       }

//       if (tx_ok)
//       {
//         sent = xQueueSendToFront(xQueue_RS485_Tx, &msg_tx, 0);
//       }

//     }
//     // Частотники
//     else
//     {
//       Serial.printf("### Get_Queue_Rx FREQ  Device=%04X, Addr=%04X, Value=%04X\n", msg_rx.device, msg_rx.addr, msg_rx.value);        

//       switch (msg_rx.addr)
//       {
//         case 0x9CA7: // Запуск останов - ответ
//           state.compr_on = (msg_rx.value == 0x0001) ? DWIN_PARAM_ON : DWIN_PARAM_OFF;

//           msg_tx.device = DEVICE_ADDR_DWIN;
//           msg_tx.addr = DEVICE_ADDR_DWIN_COMPR_ON_VAL;
//           msg_tx.value = state.compr_on;
//           sent = xQueueSendToFront(xQueue_RS485_Tx, &msg_tx, 0);

//           break;

//         case 0x9CA6: // Скорость - ответ
//           params.compr_speed = (float)msg_rx.value / 100;

//           msg_tx.device = DEVICE_ADDR_DWIN;
//           msg_tx.addr = DEVICE_ADDR_DWIN_COMPR_SPEED_VAL;
//           msg_tx.value = params.compr_speed * 10;
//           EEPROM_Write_Params();
//           sent = xQueueSendToFront(xQueue_RS485_Tx, &msg_tx, 0);




//         default:

//           break;

//       }



    }

  }

  vTaskDelete(NULL);
}


void DWIN_Rx_Event_Task(void *pvParameters)
{
  QueueHandle_t queue = *((QueueHandle_t*) pvParameters);

  uart_event_t event;
  size_t buffered_size;
  uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE_DWIN);
  DWIN_Message_t msg;
  bool sent;

  while (true) 
  {
    //Waiting for UART event.
    if (xQueueReceive(queue, (void*)&event, (TickType_t)portMAX_DELAY)) 
    {
      switch (event.type) 
      {
        case UART_DATA:
          uart_read_bytes(m_dwin_uart_num, dtmp, event.size, portMAX_DELAY);

          //msg.device = dtmp[0];
          ESP_LOGI(TAG_DWIN, "[UART SIZE]=%d, [DEVICE]=%04X, [ADDR]=%04X", event.size, dtmp[0], (dtmp[4] << 8) + dtmp[5]);
          //uart_write_bytes(m_dwin_uart_num, dtmp, event.size);
          //if (msg.device == DEVICE_ADDR_DWIN)          
          {
            // 0  1  2  3  4  5  6  7  8
            //5A A5 06 83 10 36 01 00 A0
            //--          -- --    -- --
            if (dtmp[3] == 0x83 && dtmp[6] == 0x01)
            {
              msg.addr = (dtmp[4] << 8) + dtmp[5];
              msg.value = (dtmp[7] << 8) + dtmp[8];

              ESP_LOGI(TAG_DWIN, "[DWIN_Rx_Event_Task] addr=%04X, value=%04X", msg.addr, msg.value);
              sent = xQueueSendToFront(xQueue_DWIN_Rx, &msg, 0);
            }
          }
          break;
        default:
          //ESP_LOGI(TAG_DWIN, "[DWIN_Rx_Event_Task] default: event type=%d", event.type);
          break;
      }
    }
  }

  free(dtmp);
  dtmp = NULL;
  vTaskDelete(NULL);  
}

void DWIN_Init(int uart_num, int tx_io_num, int rx_io_num, int de_io_num, int baud_rate)
{
  uart_config_t uart_config = {
    .baud_rate = baud_rate,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 122,
    .source_clk = UART_SCLK_APB,
  };
  m_dwin_uart_num = uart_num;

  esp_log_level_set(TAG_DWIN, ESP_LOG_INFO);
  ESP_LOGW(TAG_DWIN, "### ESP_LOGI Start %d", 0);


  uart_driver_install(m_dwin_uart_num, BUF_SIZE_DWIN * 2, BUF_SIZE_DWIN * 2, 64, &xQueue_DWIN, 0);
  uart_param_config(m_dwin_uart_num, &uart_config);  
  uart_set_pin(m_dwin_uart_num, tx_io_num, rx_io_num, de_io_num, UART_PIN_NO_CHANGE);
  uart_set_mode(m_dwin_uart_num, UART_MODE_UART);  

  // uart_enable_pattern_det_baud_intr(m_dwin_uart_num, '+', PATTERN_CHR_NUM_RS485, 9, 0, 0);
  // uart_pattern_queue_reset(m_dwin_uart_num, 64);

  xTaskCreate(DWIN_Rx_Event_Task, "DWIN_Rx_Event_Task", 4098, &xQueue_DWIN, tskIDLE_PRIORITY + 2, NULL);

  xQueue_DWIN_Tx = xQueueCreate(8, sizeof(DWIN_Message_t));
  xQueue_DWIN_Rx = xQueueCreate(8, sizeof(DWIN_Message_t));
  xTaskCreate(DWIN_Rx_Task, "DWIN_Rx_Task", 4098, &xQueue_DWIN_Rx, tskIDLE_PRIORITY + 2, NULL);
  //xTaskCreate(Rs485_Tx_Event_Task, "DWIN_Tx_Event_Task", 4098, &xQueue_DWIN_Tx, tskIDLE_PRIORITY + 2, NULL);

  
}

