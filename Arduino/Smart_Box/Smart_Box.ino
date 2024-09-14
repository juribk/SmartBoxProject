
#include <OneWire.h>
#include <DallasTemperature.h>

#include <ADS1115_WE.h> 
#include <Wire.h>

#include "Params.h"
#include "RS485.h"
#include "DWIN.h"



const int SensorDataPin = 13;     
OneWire oneWire(SensorDataPin);
DallasTemperature sensors(&oneWire);


#define I2C_ADDRESS 0x4A
ADS1115_WE adc;
bool init_i2c = false;


#define UART_NUM_DWIN             UART_NUM_1
#define UART_NUM_RS485            UART_NUM_2
#define LED_BUILTIN               2

#define ADC_ADDR0                 12
#define ADC_ADDR1                 2
#define ADC_ADDR2                 27
#define ADC_ADDR3                 26

struct State_t
{
  int compr_on;
  int fan_on;
  int exhanger_on;
};
State_t state;




void Rs485_Rx_Task(void* pvParameters)
{
  // RS485_Message_t msg_rx;
  // RS485_Message_t msg_tx;
  // bool sent;

  // while(1)
  // {
  //   bool receved = xQueueReceive(xQueue_RS485_Rx, &msg_rx, (TickType_t)portMAX_DELAY);
  //   bool tx_ok;
  //   if (receved)
  //   {

  //     if (msg_rx.device == DEVICE_ADDR_DWIN)
  //     {
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



  //     }
  //   }

  // }

  vTaskDelete(NULL);
}


void setup() 
{
  Params::Params_Init();
 

  Serial.begin(115200);  

  // --- I2C --------------------------
  Wire.begin();
  adc = ADS1115_WE(I2C_ADDRESS);
  init_i2c = adc.init();
  if (!init_i2c)
  {
    Serial.println("ADS1115 not connected!");
  }
  else
  {
    adc.setVoltageRange_mV(ADS1115_RANGE_6144); 
    adc.setCompareChannels(ADS1115_COMP_0_GND);
  }
  pinMode(ADC_ADDR0, OUTPUT);
  pinMode(ADC_ADDR1, OUTPUT);
  pinMode(ADC_ADDR2, OUTPUT);
  pinMode(ADC_ADDR3, OUTPUT);
  digitalWrite(ADC_ADDR0, HIGH);
  digitalWrite(ADC_ADDR1, HIGH);
  digitalWrite(ADC_ADDR2, HIGH);
  digitalWrite(ADC_ADDR3, HIGH);


  sensors.begin();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(39, OUTPUT);
  pinMode(36, OUTPUT);

  //Rs485_Init(UART_NUM_RS485, 17, 16, 4, 57600);
  DWIN_Init(UART_NUM_DWIN, 19, 18, UART_PIN_NO_CHANGE, 115200);

  //Params_Init();

  //xTaskCreate(Rs485_Rx_Task, "Rs485_Rx_Task", 4098, NULL, tskIDLE_PRIORITY + 1, NULL);

}

void loop() 
{
  while(true)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    // digitalWrite(39, HIGH);
    // digitalWrite(36, HIGH);

    // int vp;
    // int value;
    // //bool ret = dwin->Get_Queue_Rx(&vp, &value, portMAX_DELAY);
    // // if (ret)
    // // {
    // //   Serial.printf("### Get_Queue_Rx vp=%04X, value=%04X\n", vp, value);

    // // }

    delay(500);
    //   digitalWrite(LED_BUILTIN, LOW);
    //   digitalWrite(39, LOW);
    //   digitalWrite(36, LOW);
    // delay(500);

    //  sensors.requestTemperatures(); 
    //  float temperature_Celsius = sensors.getTempCByIndex(0);
    //  //float temperature_Fahrenheit = sensors.getTempFByIndex(0);
    //  Serial.print("Temperature: ");
    //  Serial.print(temperature_Celsius);
    //  Serial.println(" ºC");
    //  Serial.println("");

    // if (init_i2c)
    // {
    //   adc.startSingleMeasurement();
    //   while(adc.isBusy()){}
    //   //int16_t raw = adc.getRawResult();
    //   float adc_v = adc.getResult_V();
    //   //Serial.println(raw, BIN);
    //   Serial.print("ADC= ");
    //   Serial.println(adc_v);    
    // }

  }

}
