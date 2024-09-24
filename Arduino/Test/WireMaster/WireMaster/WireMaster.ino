#include "Wire.h"
#include "Arduino.h" 

// #define _PWM_LOGLEVEL_       4
// #include "ESP32_FastPWM.h"


#define I2C_DEV_ADDR 0x55

#define MCP23009_I2C_ADDR   0x27

uint8_t PWM_Pins1   = 14;
uint8_t PWM_Pins2   = 15;
// ESP32_FAST_PWM* PWM_Instance1;
// ESP32_FAST_PWM* PWM_Instance2;
int pwmFrequency = 60;
int dutyCycle = 50.0f;
bool PWM_INIT1 = false;
bool PWM_INIT2 = false;

uint32_t i = 0;

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  int pwmChannel1 = 0;
  PWM_INIT1 = ledcAttach(PWM_Pins1, pwmFrequency, 12);
  PWM_INIT2 = ledcAttach(PWM_Pins2, pwmFrequency, 12);

  // PWM_Instance1 = new ESP32_FAST_PWM(PWM_Pins1, frequency, dutyCycle);
  // if (PWM_Instance1)
  // {
  //   if (!PWM_Instance1->setPWM())
  //   {
  //     Serial.println(F("Stop here"));

  //   }
  //   else
  //   {
  //     PWM_INIT = true;
  //   }
  // }

 Wire.begin();
}

void loop() 
{
  delay(100);

  if (PWM_INIT1 & PWM_INIT2)
  {
    if (i == 0)
    {
      dutyCycle = 1000;
      i += 1;
    }
    else if (i == 1)
    {
      dutyCycle = 1100;
      i += 1;
    }
    else if (i == 2)
    {
      dutyCycle = 1200;
      i = 0;
    }

    Serial.printf("### dutyCycle = %d\n", dutyCycle);
    ledcWrite(PWM_Pins1, dutyCycle);
    ledcWrite(PWM_Pins2, dutyCycle);
  }

/*
  //Scan addrs
  for(int address = 1; address < 127; address++ ) 
  {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) 
    {
      Serial.printf("I2C device found %d\n", address);
    }
    else
    {
      //Serial.printf("I2C device not %d\n", address);
    }
  }
*/

  /*
  if (i == 1)
  {
    delay(2000);

    Wire.beginTransmission(MCP23009_I2C_ADDR);
    Wire.write(0x06); 
    Wire.write(0xFF); 
    uint8_t error = Wire.endTransmission();
    Serial.printf("## Error: %d\n", error);

  }
  if (i == 2)
  {
    Wire.beginTransmission(MCP23009_I2C_ADDR);
    Wire.write(0x00); 
    Wire.write(0x00); 
    uint8_t error = Wire.endTransmission();
    Serial.printf("## Error: %d\n", error);

  }
  if (i == 3)
  {

    Wire.beginTransmission(MCP23009_I2C_ADDR);
    Wire.write(0x09); 
    Wire.write(0xAA); 
    uint8_t error = Wire.endTransmission();
    Serial.printf("## Error: %d\n", error);

  }
  i += 1;


  // Test read registers
  Wire.beginTransmission(MCP23009_I2C_ADDR);
  Wire.write(0x06); 
  uint8_t error = Wire.endTransmission();
  Serial.printf("## Error: %d\n", error);
  Wire.requestFrom(MCP23009_I2C_ADDR, 1);
  while (Wire.available() == 0);
  int read = Wire.read();
  Serial.printf("## Read: 0x06=%02X\n", read);
*/
  // Serial.printf("Read: %d\n", read);


  // Wire.beginTransmission(MCP23009_I2C_ADDR);
  // Wire.write(0x00);  
  // int read = Wire.read();
  // uint8_t error = Wire.endTransmission(true);

  // Serial.printf("Read: %u\n", read);

  // //


  // //Write message to the slave
  // Wire.beginTransmission(I2C_DEV_ADDR);
  // Wire.printf("Hello World! %lu", i++);
  // uint8_t error = Wire.endTransmission(true);
  // Serial.printf("endTransmission: %u\n", error);

  // //Read 16 bytes from the slave
  // uint8_t bytesReceived = Wire.requestFrom(I2C_DEV_ADDR, 16);
  // Serial.printf("requestFrom: %u\n", bytesReceived);
  // if ((bool)bytesReceived) {  //If received more than zero bytes
  //   uint8_t temp[bytesReceived];
  //   Wire.readBytes(temp, bytesReceived);
  //   log_print_buf(temp, bytesReceived);
  // }
}
