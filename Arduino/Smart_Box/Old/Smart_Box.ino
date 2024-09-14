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

#include "Smart_Box.h"
//#include "RS485.h"
//#include "DWIN.ino"



void setup() 
{

  Serial.begin(115200);  

  pinMode(LED_BUILTIN, OUTPUT);

  // put your setup code here, to run once:
  //uart_rs485_Init(17, 16, 4, 115200);

  //dwin = new DWin();
  rs485 = new RS485(17, 16, 4, 115200);
}

void loop() {
  while(true)
  {
    digitalWrite(LED_BUILTIN, HIGH);

    int vp;
    int value;
    //bool ret = dwin->Get_Queue_Rx(&vp, &value, portMAX_DELAY);
    if (ret)
    {
      Serial.printf("### Get_Queue_Rx vp=%04X, value=%04X\n", vp, value);

    }

    // delay(500);
    // digitalWrite(LED_BUILTIN, LOW);
    // delay(500);


  }

}
