#include "RS485.h"

#include "Params.h"



int m_uart_num;
QueueHandle_t xQueue_RS485;

QueueHandle_t xQueue_RS485_Rx;
QueueHandle_t xQueue_RS485_Tx;

static const char *TAG_RS485 = "rs485_events";

void Rs485_Tx_Event_Task(void *pvParameters)
{
  QueueHandle_t queue_rs485 = *((QueueHandle_t*) pvParameters);
  uint8_t* buf = (uint8_t*) malloc(BUF_SIZE_RS485);
  int size;
  int speed;

  while (true) 
  {
    RS485_Message_t msg;
    size = 0;
    if (xQueueReceive(queue_rs485, &msg, (TickType_t)portMAX_DELAY)) 
    {
      ESP_LOGI(TAG_RS485, "[Rs485_Tx_Event_Task] device=%04X, addr=%04X, value=%04X", msg.device, msg.addr, msg.value);
      // if (msg.device == DEVICE_ADDR_DWIN)
      // {

      //   buf[0] = 0x5A; buf[1] = 0xA5; buf[2] = 0x05; buf[3] = 0x82; 
      //   buf[4] = msg.addr >> 8; buf[5] = msg.addr & 0xFF;
      //   buf[6] = msg.value >> 8; buf[7] = msg.value & 0xFF;
      //   size = 8;
      //   speed = 115200;

      // }
      // Частотники
      //else
      {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        if (msg.type == TYPE_SET)
        {
          buf[0] = msg.device; buf[1] = 0x06; 
          buf[2] = msg.addr >> 8; buf[3] = msg.addr & 0xFF;
          buf[4] = msg.value >> 8; buf[5] = msg.value & 0xFF;

          uint16_t crc = CRC_From_Buff(buf, 6);
          buf[6] = crc & 0xFF; buf[7] = crc >> 8;

          size = 8;

        }
        else if (msg.type == TYPE_GET)
        {
          buf[0] = msg.device; buf[1] = 0x03; 
          buf[2] = msg.addr >> 8; buf[3] = msg.addr & 0xFF;
          buf[4] = msg.value >> 8; buf[5] = msg.value & 0xFF;

          uint16_t crc = CRC_From_Buff(buf, 6);
          buf[6] = crc & 0xFF; buf[7] = crc >> 8;

          size = 8;

          //size = 8;
        }
        
        speed = 57600;
      }

      if (size > 0)
      {
        Rs485_Write((const char*)buf, size, speed);
        //uart_write_bytes(m_uart_num, (const char*)buf, size);
      }
    }
  }
  free(buf);
  buf = NULL;
}

void Rs485_Rx_Event_Task(void *pvParameters)
{
  QueueHandle_t queue_rs485 = *((QueueHandle_t*) pvParameters);

  uart_event_t event;
  size_t buffered_size;
  uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE_RS485);
  RS485_Message_t msg;
  bool sent;
  bool rx_ok;

  while (true) 
  {
    //Waiting for UART event.
    if (xQueueReceive(queue_rs485, (void*)&event, (TickType_t)portMAX_DELAY)) 
    {
      switch (event.type) 
      {
        case UART_DATA:
          uart_read_bytes(m_uart_num, dtmp, event.size, portMAX_DELAY);

          msg.device = dtmp[0];
          ESP_LOGI(TAG_RS485, "[RS485 SIZE]=%d, [DEVICE]=%04X", event.size, msg.device);

          rx_ok = false;
          // if (msg.device == DEVICE_ADDR_DWIN)          
          // {
          //   // 0  1  2  3  4  5  6  7  8
          //   //5A A5 06 83 10 36 01 00 A0
          //   //--          -- --    -- --
          //   if (dtmp[3] == 0x83 && dtmp[6] == 0x01)
          //   {
          //     msg.addr = (dtmp[4] << 8) + dtmp[5];
          //     msg.value = (dtmp[7] << 8) + dtmp[8];
          //     rx_ok = true;
          //   }
          // }
          // // Частотники
          // else 
          {
            if (dtmp[1] == 0x03)
            {
              msg.type = TYPE_SET;
              msg.addr = 0;
              msg.value = (dtmp[3] << 8) + dtmp[4];
              rx_ok = true;
            }
            else if (dtmp[1] == 0x06)
            {
              msg.type = TYPE_GET;
              msg.addr = (dtmp[2] << 8) + dtmp[3];
              msg.value = (dtmp[4] << 8) + dtmp[5];
              rx_ok = true;
            }

            uart_set_baudrate(m_uart_num, 57600);
          }

          if (rx_ok)
          {
            ESP_LOGI(TAG_RS485, "[Rs485_Rx_Event_Task] device=%04X, addr=%04X, value=%04X", msg.device, msg.addr, msg.value);
            sent = xQueueSendToFront(xQueue_RS485_Rx, &msg, 0);
          }
          break;
        default:
          //ESP_LOGI(TAG_RS485, "[Rs485_Rx_Event_Task] default: event type=%d", event.type);
          break;
      }
    }
  }

  free(dtmp);
  dtmp = NULL;
  vTaskDelete(NULL);  
}

void Rs485_Write(const char* buf, int size, int baud_rate)
{
  uart_set_baudrate(m_uart_num, baud_rate);

  uart_write_bytes(m_uart_num, buf, size);

}

void Rs485_Init(int uart_num, int tx_io_num, int rx_io_num, int de_io_num, int baud_rate)
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
  m_uart_num = uart_num;

  esp_log_level_set(TAG_RS485, ESP_LOG_INFO);
  ESP_LOGW(TAG_RS485, "### ESP_LOGI Start %d", 0);

  ESP_LOGW(TAG_RS485, "!!!!!!### ESP_LOGI SOC_UART_NUM=%d", SOC_UART_NUM);
  

  uart_driver_install(m_uart_num, BUF_SIZE_RS485 * 2, BUF_SIZE_RS485 * 2, 64, &xQueue_RS485, 0);
  uart_param_config(m_uart_num, &uart_config);  
  uart_set_pin(m_uart_num, tx_io_num, rx_io_num, de_io_num, UART_PIN_NO_CHANGE);
  uart_set_mode(m_uart_num, UART_MODE_RS485_HALF_DUPLEX);  

  // uart_enable_pattern_det_baud_intr(m_uart_num, '+', PATTERN_CHR_NUM_RS485, 9, 0, 0);
  // uart_pattern_queue_reset(m_uart_num, 64);

  xTaskCreate(Rs485_Rx_Event_Task, "Rs485_Rx_Event_Task", 4098, &xQueue_RS485, tskIDLE_PRIORITY + 2, NULL);

  xQueue_RS485_Tx = xQueueCreate(8, sizeof(RS485_Message_t));
  xQueue_RS485_Rx = xQueueCreate(8, sizeof(RS485_Message_t));
  xTaskCreate(Rs485_Tx_Event_Task, "Rs485_Tx_Event_Task", 4098, &xQueue_RS485_Tx, tskIDLE_PRIORITY + 2, NULL);
}


/**
 * @Function
 *      void update_crc16(Uint16 *crc, uint16_t data)
 * @param
 *      crc - указатель CRC
 *      data - байт данных
 * @Summary
 *      Расчитывает CRC.
 *      Перед первым вызовом необходимо инициализировать crc = 0xFFFF;
 */
void update_crc16(uint16_t *crc, uint16_t data)
{
    uint8_t bit;
    *crc ^= data;
    for (bit = 0; bit < 8; bit++) {
        *crc = (*crc & 0x0001) ? ((*crc >> 1) ^ 0xA001) : (*crc >> 1);
    }
}
/**
 * @Function
 *      Uint16 CRC_From_Buff(Uint16 *buf, Uint16 length)
 * @param
 *      Uint16 *buf - буфер
 *      Uint16 length - длина буфера
 * @return
 *      Возвращат CRC
 * @Summary
 *      Расчет CRC буфера
 */
uint16_t CRC_From_Buff(uint8_t *buf, uint16_t length)
{
    uint16_t crc = 0xFFFF;                     // Инициализируем chksum
    uint16_t pos = 0;

    while (pos < length)
    {
        update_crc16(&crc, buf[pos++] & 0xFF);         // Расчет chksum
    }

    return crc;
}



