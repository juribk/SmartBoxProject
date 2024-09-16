#include "DWIN.h"
#include "Params.h"
#include "FREQ.h"


static const char *TAG_DWIN = "DWIN_events";

QueueHandle_t xQueue_DWIN_Rx;
QueueHandle_t xQueue_DWIN_Tx;
QueueHandle_t xQueue_DWIN;

int m_dwin_uart_num;

struct DWIN_Message_t
{
  int type;
  int addr;   // Адрес DWIN
  int value;  // Значение DWIN
};

void DWIN_Tx_Task(void* pvParameters)
{
  QueueHandle_t queue = *((QueueHandle_t*) pvParameters);
  uint8_t* buf = (uint8_t*) malloc(BUF_SIZE_DWIN);
  DWIN_Message_t msg_tx;
  bool ret;

  while(1)
  {
    bool receved = xQueueReceive(queue, &msg_tx, (TickType_t)portMAX_DELAY);
    if (receved)
    {
      ESP_LOGI(TAG_DWIN, "Addr=%04X, Value=%04X", msg_tx.addr, msg_tx.value);

      buf[0] = 0x5A; buf[1] = 0xA5; buf[2] = 0x05; buf[3] = 0x82; 
      buf[4] = msg_tx.addr >> 8; buf[5] = msg_tx.addr & 0xFF;
      buf[6] = msg_tx.value >> 8; buf[7] = msg_tx.value & 0xFF;
      uart_write_bytes(m_dwin_uart_num, buf, 8);

    }
  }


  vTaskDelete(NULL);
}

int DWIN_Get_Addr_Param(int dwin_addr)
{
  switch (dwin_addr)
  {
    case DWIN_COMPR_SPEED:
      return PARAM_COMPR_SPEED;
    case DWIN_COMPR_ON:
      return PARAM_COMPR_ON;
    case DWIN_FAN_SPEED:
      return PARAM_FAN_SPEED;
    case DWIN_FAN_ON:
      return PARAM_FAN_ON;
    case DWIN_EXCHANGER_ON:
      return PARAM_EXCHANGER_ON;

    default:
      return 0;
  }

}


void DWIN_Rx_Task(void* pvParameters)
{
  QueueHandle_t queue = *((QueueHandle_t*) pvParameters);

  DWIN_Message_t msg_rx;
  DWIN_Message_t msg_tx;
  float value, value_inc;
  bool ret;

  while(1)
  {
    bool receved = xQueueReceive(queue, &msg_rx, (TickType_t)portMAX_DELAY);
    if (receved)
    {
      ESP_LOGI(TAG_DWIN, "Addr=%04X, Value=%04X", msg_rx.addr, msg_rx.value);
      msg_tx.type = TYPE_SET;
      msg_tx.addr = 0;

      switch (msg_rx.addr)
      {
        // ------------------------------------------------
        // Отправим команду в частотник. После ответа частотника отправим ответ в DWIN и сохраним в параметры
        case DWIN_COMPR_SPEED + 1:
          Freq::FREQ_Command(FREQ_CMD_SPEED, FREQ_ADDR_COMPR, msg_rx.value);
          //Freq::FREQ_Set_Speed(FREQ_ADDR_COMPR, msg_rx.value);
          break;
        case DWIN_COMPR_ON + 1:
          Freq::FREQ_Command(FREQ_CMD_START, FREQ_ADDR_COMPR, msg_rx.value);

          //Freq::FREQ_Start(FREQ_ADDR_COMPR, msg_rx.value); 
          break;
        // ------------------------------------------------
        case DWIN_FAN_SPEED + 1:
          break;
        case DWIN_FAN_ON + 1:
          Freq::FREQ_Command(FREQ_CMD_CURRENT, FREQ_ADDR_COMPR, msg_rx.value);        
          Freq::FREQ_Command(FREQ_CMD_VOLTAGE, FREQ_ADDR_COMPR, msg_rx.value);        
          Freq::FREQ_Command(FREQ_CMD_TEMPER, FREQ_ADDR_COMPR, msg_rx.value);        
          break;
        // ------------------------------------------------
        case DWIN_EXCHANGER_ON + 1:
          break;
        // ------------------------------------------------
        // ------------------------------------------------





        default:
          break;
      }





      if (msg_tx.addr)
      {
        xQueueSendToFront(xQueue_DWIN_Tx, &msg_tx, 0);
      }

    }
  }
  vTaskDelete(NULL);
}

void DWIN_Send(int addr, int value)
{
  DWIN_Message_t msg_tx;
  msg_tx.type = TYPE_SET;
  msg_tx.addr = addr;
  msg_tx.value = value;
  xQueueSendToFront(xQueue_DWIN_Tx, &msg_tx, 0);
}

void DWIN_Rx_Event_Task(void *pvParameters)
{
  QueueHandle_t queue = *((QueueHandle_t*) pvParameters);

  uart_event_t event;
  size_t buffered_size;
  uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE_DWIN);
  DWIN_Message_t msg;
  
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
          // 0  1  2  3  4  5  6  7  8
          //5A A5 06 83 10 36 01 00 A0
          //--          -- --    -- --
          if (dtmp[3] == 0x83 && dtmp[6] == 0x01)
          {
            msg.addr = (dtmp[4] << 8) + dtmp[5];
            msg.value = (dtmp[7] << 8) + dtmp[8];

            ESP_LOGI(TAG_DWIN, "[DWIN_Rx_Event_Task] addr=%04X, value=%04X", msg.addr, msg.value);
            xQueueSendToFront(xQueue_DWIN_Rx, &msg, 0);
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

void DWIN_Init_Value()
{
  DWIN_Send(DWIN_COMPR_SPEED, (int)(Params::params[PARAM_COMPR_SPEED] * 10));
  DWIN_Send(DWIN_COMPR_ON, (int)(Params::params[PARAM_COMPR_ON]));
  DWIN_Send(DWIN_COMPR_ON + 1, (int)(Params::params[PARAM_COMPR_ON]));
  
  DWIN_Send(DWIN_FAN_SPEED, (int)(Params::params[PARAM_FAN_SPEED] * 10));
  DWIN_Send(DWIN_FAN_ON, (int)(Params::params[PARAM_FAN_ON]));
  DWIN_Send(DWIN_FAN_ON + 1, (int)(Params::params[PARAM_COMPR_ON]));

  DWIN_Send(DWIN_EXCHANGER_ON, (int)(Params::params[PARAM_EXCHANGER_ON]));
  DWIN_Send(DWIN_EXCHANGER_ON + 1, (int)(Params::params[PARAM_EXCHANGER_ON]));

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
  xTaskCreate(DWIN_Tx_Task, "DWIN_Tx_Task", 4098, &xQueue_DWIN_Tx, tskIDLE_PRIORITY + 2, NULL);

  DWIN_Init_Value();
}

