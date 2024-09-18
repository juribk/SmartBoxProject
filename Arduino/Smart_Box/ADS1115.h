#ifndef ADS1115_H
#define ADS1115_H

#define ADS1115_COUNT               16

#define I2C_ADDRESS                 0x4A

#define ADC_ADDR3                   12
#define ADC_ADDR2                   2
#define ADC_ADDR1                   27
#define ADC_ADDR0                   26

#define ADC_SAMPLING_COUNT          50

namespace ads1115
{
  enum Out_Type 
  {
    TYPE_VOLT_3P3, // 0
    TYPE_TEMC, 
    TYPE_VOLT_220,
    TYPE_CURRENT_220,
  };

  class ADS1115
  {
    public:
      int dwin_addr_val;
      float value;
      Out_Type out_type;
      float max_value;
    private:
      int param_index;


    public:
      ADS1115(int param_index, int dwin_addr_val);
      float TempC();
      float Volt_220();
      float Current_220();
      float Get_Adc_Dwin_Addr_Val();



  };

  extern ADS1115 *ads1115[ADS1115_COUNT];

  void Init();
  void Update();


}
#endif // ADS1115_H