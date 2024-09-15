#include "FREQ.h"
#include "Params.h"

namespace Freq
{
  static const char *TAG_FREQ = "FREQ_events";
  int m_uart_num;
  QueueHandle_t xQueue_RS485;
  //QueueHandle_t xQueue_FREQ_Rx;
  QueueHandle_t xQueue_FREQ_Tx;

  /**
  * @Function
  *      void FREQ_Start(int frq_num, int start)
  * @param
  *      frq_addr - Адрес частотника
  *      start - 1-стартб 0-Стоп
  * @Summary
  *      Старт/Стоп частотника
  */
  void FREQ_Start(int frq_addr, int start)
  {
    FREQ_Message_t msg_tx;
    msg_tx.type = FRQ_TYPE_SET;
    msg_tx.device = frq_addr;
    msg_tx.addr = 0x9CA7;
    msg_tx.value = start;
    xQueueSendToFront(xQueue_FREQ_Tx, &msg_tx, 0);
  }
  /**
  * @Function
  *      void FREQ_Set_Speed(int frq_addr, float speed)
  * @param
  *      frq_addr - Адрес частотника
  *      speed - Скорость
  * @Summary
  *      Установка скорости вращения 
  */
  void FREQ_Set_Speed(int frq_addr, float speed)
  {
    FREQ_Message_t msg_tx;
    msg_tx.type = FRQ_TYPE_SET;
    msg_tx.device = frq_addr;
    msg_tx.value = speed * 10;
    msg_tx.addr = 0x9CA6;
    xQueueSendToFront(xQueue_FREQ_Tx, &msg_tx, 0);
  }

  void FREQ_Tx_Event_Task(void *pvParameters)
  {
    QueueHandle_t queue = *((QueueHandle_t*) pvParameters);
    uint8_t* buf = (uint8_t*) malloc(BUF_SIZE_RS485);
    int size;
    
    while (true) 
    {
      FREQ_Message_t msg;
      size = 0;
      if (xQueueReceive(queue, &msg, (TickType_t)portMAX_DELAY)) 
      {
        ESP_LOGI(TAG_FREQ, "device=%04X, addr=%04X, value=%04X", msg.device, msg.addr, msg.value);

        buf[0] = msg.device; buf[1] = 0x06; 
        buf[2] = msg.addr >> 8; buf[3] = msg.addr & 0xFF;
        buf[4] = msg.value >> 8; buf[5] = msg.value & 0xFF;

        uint16_t crc = CRC_From_Buff(buf, 6);
        buf[6] = crc & 0xFF; buf[7] = crc >> 8;
        size = 8;

        uart_write_bytes(m_uart_num, (const char*)buf, size);

        // vTaskDelay(10 / portTICK_PERIOD_MS);
        // if (msg.type == TYPE_SET)
        // {
        //   buf[0] = msg.device; buf[1] = 0x06; 
        //   buf[2] = msg.addr >> 8; buf[3] = msg.addr & 0xFF;
        //   buf[4] = msg.value >> 8; buf[5] = msg.value & 0xFF;

        //   uint16_t crc = CRC_From_Buff(buf, 6);
        //   buf[6] = crc & 0xFF; buf[7] = crc >> 8;

        //   size = 8;

        // }
        // else if (msg.type == TYPE_GET)
        // {
        //   buf[0] = msg.device; buf[1] = 0x03; 
        //   buf[2] = msg.addr >> 8; buf[3] = msg.addr & 0xFF;
        //   buf[4] = msg.value >> 8; buf[5] = msg.value & 0xFF;

        //   uint16_t crc = CRC_From_Buff(buf, 6);
        //   buf[6] = crc & 0xFF; buf[7] = crc >> 8;

        //   size = 8;

        // }
          
        // if (size > 0)
        // {
        //   Rs485_Write((const char*)buf, size);
        //   //uart_write_bytes(m_uart_num, (const char*)buf, size);
        // }
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
    FREQ_Message_t msg;

    while (true) 
    {
      //Waiting for UART event.
      if (xQueueReceive(queue_rs485, (void*)&event, (TickType_t)portMAX_DELAY)) 
      {
        switch (event.type) 
        {
          case UART_DATA:
            uart_read_bytes(m_uart_num, dtmp, event.size, portMAX_DELAY);

            ESP_LOGI(TAG_FREQ, "[RX SIZE]=%d, %02X %02X %02X %02X %02X", dtmp[0], dtmp[1], dtmp[2], dtmp[3], dtmp[4]);

            // if (dtmp[1] == 0x03)
            // {
            //   msg.type = TYPE_SET;
            //   msg.addr = 0;
            //   msg.value = (dtmp[3] << 8) + dtmp[4];
            //   rx_ok = true;
            // }
            // else if (dtmp[1] == 0x06)
            // {
            //   msg.type = TYPE_GET;
            //   msg.addr = (dtmp[2] << 8) + dtmp[3];
            //   msg.value = (dtmp[4] << 8) + dtmp[5];
            //   rx_ok = true;
            // }

            // if (rx_ok)
            // {
            //   ESP_LOGI(TAG_RS485, "[Rs485_Rx_Event_Task] device=%04X, addr=%04X, value=%04X", msg.device, msg.addr, msg.value);
            //   sent = xQueueSendToFront(xQueue_RS485_Rx, &msg, 0);
            // }
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


  void FREQ_Init(int uart_num, int tx_io_num, int rx_io_num, int de_io_num, int baud_rate)
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

    esp_log_level_set(TAG_FREQ, ESP_LOG_INFO);

    uart_driver_install(m_uart_num, BUF_SIZE_RS485 * 2, BUF_SIZE_RS485 * 2, 64, &xQueue_RS485, 0);
    uart_param_config(m_uart_num, &uart_config);  
    uart_set_pin(m_uart_num, tx_io_num, rx_io_num, de_io_num, UART_PIN_NO_CHANGE);
    uart_set_mode(m_uart_num, UART_MODE_RS485_HALF_DUPLEX);  

    xTaskCreate(Rs485_Rx_Event_Task, "Rs485_Rx_Event_Task", 4098, &xQueue_RS485, tskIDLE_PRIORITY + 2, NULL);

    xQueue_FREQ_Tx = xQueueCreate(8, sizeof(FREQ_Message_t));
    //xQueue_FREQ_Rx = xQueueCreate(8, sizeof(RS485_Message_t));
    xTaskCreate(FREQ_Tx_Event_Task, "FREQ_Tx_Event_Task", 4098, &xQueue_FREQ_Tx, tskIDLE_PRIORITY + 2, NULL);
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

}

