#ifndef MCP23009_H
#define MCP23009_H

namespace relay
{


#define RELAY_EXCHANGER_ON      01
// ...
#define RELAY_HEAT1_1           05
#define RELAY_HEAT1_2           06


#define MCP23009_I2C_ADDR       0x27

#define MCP23009_REG_IODIR      0x00    // I/O DIRECTION REGISTER
#define MCP23009_REG_IPOL       0x01    // INPUT POLARITY REGISTER
#define MCP23009_REG_GPINTEN    0x02    // INTERRUPT-ON-CHANGE CONTROL REGISTER
#define MCP23009_REG_DEFVAL     0x03    // DEFAULT COMPARE REGISTER FOR INTERRUPT-ON-CHANGE
#define MCP23009_REG_INTCON     0x04    // INTERRUPT CONTROL REGISTER
#define MCP23009_REG_IOCON      0x05    // CONFIGURATION REGISTER
#define MCP23009_REG_GPPU       0x06    // PULL-UP RESISTOR CONFIGURATION REGISTER
#define MCP23009_REG_INTF       0x07    // INTERRUPT FLAG REGISTER
#define MCP23009_REG_INTCAP     0x08    // INTERRUPT CAPTURE REGISTER
#define MCP23009_REG_GPIO       0x09    // PORT REGISTER
#define MCP23009_REG_OLAT       0x0A    // OUTPUT LATCH REGISTER (OLAT)

#define RELAY_ON                1
#define RELAY_OFF               0

  class MCP23009
  {
    private:
      uint8_t addr; 
    public:
      MCP23009(uint8_t addr);
      bool Init(uint8_t addr);
      bool Read_Reg(uint8_t reg, uint8_t *value);
      bool Write_Reg(uint8_t reg, uint8_t value);
      bool Set_GPIO(uint8_t gpio, bool set);
      bool Get_GPIO(uint8_t gpio, bool* state);


  };

  extern MCP23009* relay;
  void Relay_Init(uint8_t addr);
}



#endif // MCP23009_H