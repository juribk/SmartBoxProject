
#include <ADS1115_WE.h> 
#include <Wire.h>

#include "Params.h"
#include "DWIN.h"
#include "FREQ.h"
#include "DS18B20.h"
#include "ADS1115.h"
#include <MCP23009.h> 
#include <PWM.h> 


// #define I2C_ADDRESS 0x4A
// ADS1115_WE adc;
// bool init_i2c = false;

#define UART_NUM_DWIN             UART_NUM_1
#define UART_NUM_RS485            UART_NUM_2
#define LED_BUILTIN               2

// --- Timers --------------------------------------------
hw_timer_t *timer_read_sensors = NULL;
volatile SemaphoreHandle_t semaphore_timer_read_sensors;
void ARDUINO_ISR_ATTR onTimer_Read_Sensors() 
{
  // // Increment the counter and set the time of ISR
  // portENTER_CRITICAL_ISR(&timerMux);
  // isrCounter = isrCounter + 1;
  // lastIsrAt = millis();
  // portEXIT_CRITICAL_ISR(&timerMux);

  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(semaphore_timer_read_sensors, NULL);
}

// --- Setup ---------------------------------------------
void setup() 
{
  Serial.begin(115200);  
  
  // --- I2C -----------------------------
  Wire.begin();

  // --- Params --------------------------
  Params::Params_Init();

  // --- DS18B20 -------------------------
  ds18b20::Init();

  // --- ADS1115 -------------------------
  ads1115::Init(); 

  // --- MCP23009 ------------------------
  relay::Relay_Init(MCP23009_I2C_ADDR);

  // --- PWM -----------------------------
  pwm::Init();


  // --- DWIN & FREQ init ----------------
  Freq::FREQ_Init(UART_NUM_RS485, 17, 16, 4, 57600);
  DWIN_Init(UART_NUM_DWIN, 19, 18, UART_PIN_NO_CHANGE, 115200);

  // --- Timers --------------------------
  semaphore_timer_read_sensors = xSemaphoreCreateBinary();
  timer_read_sensors = timerBegin(1000000);
  timerAttachInterrupt(timer_read_sensors, &onTimer_Read_Sensors);
  timerAlarm(timer_read_sensors, 1000000, true, 0);



  // adc = ADS1115_WE(I2C_ADDRESS);
  // init_i2c = adc.init();
  // if (!init_i2c)
  // {
  //   Serial.println("ADS1115 not connected!");
  // }
  // else
  // {
  //   adc.setVoltageRange_mV(ADS1115_RANGE_6144); 
  //   adc.setCompareChannels(ADS1115_COMP_0_GND);
  // }

  // pinMode(ADC_ADDR0, OUTPUT);
  // pinMode(ADC_ADDR1, OUTPUT);
  // pinMode(ADC_ADDR2, OUTPUT);
  // pinMode(ADC_ADDR3, OUTPUT);
  // digitalWrite(ADC_ADDR0, HIGH);
  // digitalWrite(ADC_ADDR1, HIGH);
  // digitalWrite(ADC_ADDR2, HIGH);
  // digitalWrite(ADC_ADDR3, HIGH);


  pinMode(LED_BUILTIN, OUTPUT);




}

void loop() 
{
  while(true)
  {
    if (xSemaphoreTake(semaphore_timer_read_sensors, 0) == pdTRUE) 
    {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

      Freq::FREQ_Command(FREQ_CMD_TEMPER, FREQ_ADDR_COMPR, 1);        
      Freq::FREQ_Command(FREQ_CMD_CURRENT, FREQ_ADDR_COMPR, 1);        
      Freq::FREQ_Command(FREQ_CMD_VOLTAGE, FREQ_ADDR_COMPR, 1);        

      ds18b20::Update();
      ads1115::Update();


    }


    delay(1000);




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
