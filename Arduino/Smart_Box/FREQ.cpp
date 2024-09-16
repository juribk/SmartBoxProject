#include "FREQ.h"
#include "DWIN.h"
#include "Params.h"

namespace Freq
{

  static const char *TAG_FREQ = "FREQ_events";
  int m_uart_num;
  QueueHandle_t xQueue_RS485;
  QueueHandle_t xQueue_FREQ_Rx;
  QueueHandle_t xQueue_FREQ_Tx;

  // --- Функции команд для частотников
  /**
  * @Function
  *      void FREQ_Command(int cmd, int addr, int value)
  * @param
  *      cmd   - Команда
  *      addr  - Адрес частотника
  *      value - Значение
  * @Summary
  *      Старт/Стоп частотника
  */
  void FREQ_Command(int cmd, int addr, int value)
  {
    FREQ_Message_t msg_tx;
    msg_tx.device = addr;
    msg_tx.addr = cmd;
    switch (cmd) 
    {
      case FREQ_CMD_START:
        msg_tx.type = FRQ_TYPE_SET;
        msg_tx.value = value;
        break;
      case FREQ_CMD_SPEED:
        msg_tx.type = FRQ_TYPE_SET;
        msg_tx.value = value * 10;
        break;
      case FREQ_CMD_CURRENT:
        msg_tx.type = FRQ_TYPE_GET;
        msg_tx.value = 1;
        break;
      case FREQ_CMD_VOLTAGE:
        msg_tx.type = FRQ_TYPE_GET;
        msg_tx.value = 1;
        break;
      case FREQ_CMD_TEMPER:
        msg_tx.type = FRQ_TYPE_GET;
        msg_tx.value = 1;
        break;

      default:
        return;
    }

    xQueueSendToFront(xQueue_FREQ_Tx, &msg_tx, 0);

  }

  void FREQ_Tx_Event_Task(void *pvParameters)
  {
    QueueHandle_t queue = *((QueueHandle_t*) pvParameters);
    uint8_t* buf = (uint8_t*) malloc(BUF_SIZE_RS485);
    int size;
    int addr_param;

    while (true) 
    {
      FREQ_Message_t msg_tx;
      FREQ_Message_t msg_rx;
      size = 0;
      if (xQueueReceive(queue, &msg_tx, (TickType_t)portMAX_DELAY)) 
      {
        ESP_LOGI(TAG_FREQ, "device=%04X, addr=%04X, value=%04X", msg_tx.device, msg_tx.addr, msg_tx.value);

        buf[0] = msg_tx.device; buf[1] = msg_tx.type; 
        buf[2] = msg_tx.addr >> 8; buf[3] = msg_tx.addr & 0xFF;
        buf[4] = msg_tx.value >> 8; buf[5] = msg_tx.value & 0xFF;

        uint16_t crc = CRC_From_Buff(buf, 6);
        buf[6] = crc & 0xFF; buf[7] = crc >> 8;
        size = 8;

        uart_write_bytes(m_uart_num, (const char*)buf, size);

        // ----------------------------------------------------------------------------
        // --- Ждем ответа от устройства.
        if (xQueueReceive(xQueue_FREQ_Rx, &msg_rx, (TickType_t)300)) 
        {
          // Все ответы частотника это запрос исполнения или получения параметров.
          // Обработку ответа производим через DWIN
          switch (msg_tx.addr) 
          {
            case FREQ_CMD_START:
              addr_param = (msg_tx.device == FREQ_ADDR_COMPR) ? DWIN_COMPR_ON : DWIN_FAN_ON;
              Params::Set_Param(DWIN_Get_Addr_Param(addr_param), (float)msg_rx.value);
              DWIN_Send(addr_param, msg_rx.value);
              break;
            case FREQ_CMD_SPEED:
              addr_param = (msg_tx.device == FREQ_ADDR_COMPR) ? DWIN_COMPR_SPEED : DWIN_FAN_SPEED;
              Params::Set_Param(DWIN_Get_Addr_Param(addr_param), (float)msg_rx.value / 100);
              DWIN_Send(addr_param, msg_rx.value / 10);
              break;
            case FREQ_CMD_CURRENT:
              ESP_LOGI(TAG_FREQ, "*** FREQ_CMD_CURRENT=%d", msg_rx.value);
            
              break;
            case FREQ_CMD_VOLTAGE:
              ESP_LOGI(TAG_FREQ, "*** FREQ_CMD_VOLTAGE=%d", msg_rx.value);

              break;
            case FREQ_CMD_TEMPER:
              ESP_LOGI(TAG_FREQ, "*** FREQ_CMD_TEMPER=%d", msg_rx.value);

              break;

            default:
              break;
          }



        }
        else
        {
          ESP_LOGE(TAG_FREQ, "Time Out: device=%04X, addr=%04X, value=%04X", msg_tx.device, msg_tx.addr, msg_tx.value);
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

            ESP_LOGI(TAG_FREQ, "[RX SIZE]=%d, %02X %02X %02X %02X %02X %02X", dtmp[0], dtmp[1], dtmp[2], dtmp[3], dtmp[4], dtmp[5]);
            //DWIN_Send(DWIN_COMPR_ON, (dtmp[4] << 8) + dtmp[5]);
            if (dtmp[1] == FRQ_TYPE_GET)
            {
              msg.type = dtmp[1];
              msg.addr = 0;
              msg.value = (dtmp[3] << 8) + dtmp[4];
            }
            else if (dtmp[1] == FRQ_TYPE_SET)
            {
              msg.type = dtmp[1];
              msg.addr = (dtmp[2] << 8) + dtmp[3];
              msg.value = (dtmp[4] << 8) + dtmp[5];
            }
            xQueueSendToFront(xQueue_FREQ_Rx, &msg, 0);

            break;
          default:
            //ESP_LOGI(TAG_RS485, "default: event type=%d", event.type);
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
    xQueue_FREQ_Rx = xQueueCreate(8, sizeof(FREQ_Message_t));
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

