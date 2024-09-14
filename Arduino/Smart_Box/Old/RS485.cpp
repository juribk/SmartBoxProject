//#include "Smart_Box.h"
#include "RS485.h"

static const char *TAG_RS485 = "rs485_events";

RS485::RS485(int tx_io_num, int rx_io_num, int de_io_num, int baud_rate)
{
  // uart_config_t uart_config = {
  //   .baud_rate = baud_rate,
  //   .data_bits = UART_DATA_8_BITS,
  //   .parity = UART_PARITY_DISABLE,
  //   .stop_bits = UART_STOP_BITS_1,
  //   .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  //   .rx_flow_ctrl_thresh = 122,
  //   .source_clk = UART_SCLK_APB,
  // };

  // esp_log_level_set(TAG_RS485, ESP_LOG_INFO);
  // ESP_LOGW(TAG_RS485, "### ESP_LOGI Start %d", 0);


  // uart_driver_install(UART_NUM_RS485, BUF_SIZE_RS485 * 2, BUF_SIZE_RS485 * 2, 64, &uart_queue_rs485, 0);
  // uart_param_config(UART_NUM_RS485, &uart_config);  
  // uart_set_pin(UART_NUM_RS485, tx_io_num, rx_io_num, de_io_num, UART_PIN_NO_CHANGE);
  // uart_set_mode(UART_NUM_RS485, UART_MODE_RS485_HALF_DUPLEX);  

  // uart_enable_pattern_det_baud_intr(UART_NUM_RS485, '+', PATTERN_CHR_NUM_RS485, 9, 0, 0);
  // uart_pattern_queue_reset(UART_NUM_RS485, 64);

  // xTaskCreate(Rs485_Rx_Event_Task, "Rs485_Rx_Event_Task", 2048, &uart_queue_rs485, tskIDLE_PRIORITY + 2, NULL);

  // xQueue_RS485_Tx = xQueueCreate(8, sizeof(RS485_Buf_t));
  // xTaskCreate(Rs485_Tx_Event_Task, "Rs485_Tx_Event_Task", 2048, &xQueue_RS485_Tx, tskIDLE_PRIORITY + 2, NULL);
}

static void RS485::Rs485_Tx_Event_Task(void *pvParameters)
{
  // QueueHandle_t queue_rs485 = *((QueueHandle_t*) pvParameters);
  // while (true) 
  // {
  //   RS485_Buf_t buf;
  //   if (xQueueReceive(queue_rs485, &buf, (TickType_t)portMAX_DELAY)) 
  //   {
  //     uart_write_bytes(UART_NUM_RS485, (const char*)buf.buf, buf.size);
  //   }
  // }
}
    
static void RS485::Rs485_Rx_Event_Task(void *pvParameters)
{
  // QueueHandle_t queue_rs485 = *((QueueHandle_t*) pvParameters);

  // uart_event_t event;
  // size_t buffered_size;
  // uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE_RS485);
  // while (true) 
  // {
  //   //Waiting for UART event.
  //   if (xQueueReceive(queue_rs485, (void*)&event, (TickType_t)portMAX_DELAY)) 
  //   {
  //     switch (event.type) 
  //     {
  //       case UART_DATA:
  //         // ESP_LOGI(TAG_RS485, "[UART DATA]: %d", event.size);
  //         uart_read_bytes(UART_NUM_RS485, dtmp, event.size, portMAX_DELAY);
  //         //dwin->Add_Queue_Rx(dtmp);              


  //         // ESP_LOGI(TAG_RS485, "[DATA EVT]:");
  //         //uart_write_bytes(m_UART_NUM_RS485, (const char*) dtmp, event.size);

  //         break;

  //       default:
  //         ESP_LOGI(TAG_RS485, "uart event type: %d", event.type);
  //         break;
  //     }
  //   }
  // }

  // free(dtmp);
  // dtmp = NULL;
  // vTaskDelete(NULL);    
}

















