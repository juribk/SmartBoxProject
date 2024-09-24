#include <ADS1115_WE.h> 
#include <Wire.h>

#include "ADS1115.h"
#include "Params.h"
#include "DWIN.h"





namespace ads1115
{
  // --- ACS712------------------------------------------------------------
  
  // ---------------------------------------------------------------

  static const char *TAG = "ADS1115_events";
  ADS1115 *ads1115[ADS1115_COUNT];

  ADS1115_WE adc;
  bool ADS1115_INIT = false;
  double Vs = 3.3;


  ADS1115::ADS1115(int param_index, int dwin_addr_val)
  {
    // Терморезистор
    R1 = 20000.0;      // voltage divider resistor value
    Beta = 3920.0;     // Beta value 3470 3950
    To = 298.15;       // Temperature in Kelvin for 25 degree Celsius
    Ro = 20000.0;      // Resistance of Thermistor at 25 degree Celsius
    Vs = 3.3;

    this->param_index = param_index;
    this->dwin_addr_val = dwin_addr_val;

  }

  float ADS1115::TempC()
  {
    double Vt = Vs - (value - 0.020); // Коррекция. Надо проверить это
    double It = Vt / Ro;
    double Rt = value / It;

    float T = 1 / (1 / To + log(Rt / Ro) / Beta);   // Temperature in Kelvin
    float tempC = T - 273.15;                       // Celsius

    return tempC;
  }

  double v220_Max = 2.890; //2.505;
  double v220_Ref = 218.0;
  float ADS1115::Volt_220()
  {
    /// TODO: Возможно надо отнять среднее напряжение и откалибровать заново .
    /// v220_Ref - ACS712_REF / 2
    float v220 = (v220_Ref / v220_Max) * max_value;    
    return v220;
  }
  #define ACS712_RESILUTION               0.100 // mVperA
  #define ACS712_REF                      5.0 // V
  float ADS1115::Current_220()
  {
    float current = (value - ACS712_REF / 2) / ACS712_RESILUTION;
    return current;
  }

  // ------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------
  float ADS1115::Get_Adc_Dwin_Addr_Val()
  {
    if (out_type == TYPE_TEMC)
    {
      return TempC() * 10;
    }
    else if (out_type == TYPE_VOLT_220)
    {
      return Volt_220() * 10;
    }
    else if (out_type == TYPE_CURRENT_220)
    {
      return Current_220() * 10;
    }
    else // if (out_type == TYPE_VOLT_3P3)
    {
      return value;
    }
  }


  void Set_Addr(uint8_t addr)
  {
    digitalWrite(ADC_ADDR0, GET_BIT(addr, 0));
    digitalWrite(ADC_ADDR1, GET_BIT(addr, 1));
    digitalWrite(ADC_ADDR2, GET_BIT(addr, 2));
    digitalWrite(ADC_ADDR3, GET_BIT(addr, 3));
    delay(1);
  }

  void Update()
  {
    if (!ADS1115_INIT) return;

    for (int i = 0; i < ADS1115_COUNT; i++)
    {
      Set_Addr(i);

      if (ads1115[i]->out_type == Out_Type::TYPE_VOLT_220 || ads1115[i]->out_type == Out_Type::TYPE_CURRENT_220)
      {
        float simple_sum = 0;
        float max_value = 0;
        float adcV = 0;
        for (int s = 0; s < ADC_SAMPLING_COUNT; s++)
        {
          adc.startSingleMeasurement();
          while(adc.isBusy()) {}
          adcV = adc.getResult_V();
          simple_sum += adcV;
          if (adcV > max_value)
          {
            max_value = adcV;
          }

          // Calibrate
          // if (ads1115[i]->out_type == Out_Type::TYPE_VOLT_220)
          // {
          //   Serial.println(adcV);          
          // }
        }
        ads1115[i]->value = simple_sum / ADC_SAMPLING_COUNT;
        ads1115[i]->max_value = max_value;
      }
      else
      {
        adc.startSingleMeasurement();
        while(adc.isBusy()) {}
        ads1115[i]->value = adc.getResult_V();
      }


      DWIN_Send(ads1115[i]->dwin_addr_val, (int)ads1115[i]->Get_Adc_Dwin_Addr_Val());


      //ESP_LOGI(TAG, "### ADS1115[%d]=%f, TEMP=%f, CURRENT=%f, MAX_VALUE=%f - %f", i, ads1115[i]->value, ads1115[i]->TempC(), ads1115[i]->Current_220(), ads1115[i]->max_value, ads1115[i]->Volt_220());
    }

    // --- REF for 3.3V
    Vs = ads1115[2]->value;

    // ESP_LOGI(TAG, "### ADS1115[15] TEMP=%f C", ads1115[15]->TempC());
    // ESP_LOGI(TAG, "### ADS1115[1] TEMP=%f A", ads1115[1]->Current_220());

  }


  void Init()
  {
    // ---------------------------------------------------------------


    // ---------------------------------------------------------------
    adc = ADS1115_WE(I2C_ADDRESS);
    ADS1115_INIT = adc.init();
    if (!ADS1115_INIT)
    {
      ESP_LOGE(TAG, "### ADS1115 not connected!");
    }
    else
    {
      adc.setVoltageRange_mV(ADS1115_RANGE_6144); 
      adc.setCompareChannels(ADS1115_COMP_0_GND);

      pinMode(ADC_ADDR0, OUTPUT);
      pinMode(ADC_ADDR1, OUTPUT);
      pinMode(ADC_ADDR2, OUTPUT);
      pinMode(ADC_ADDR3, OUTPUT);
      Set_Addr(0);
    }
    
    for (int i = 0; i < ADS1115_COUNT; i++)
    {
      ads1115[i] = new ADS1115(i, DWIN_ADC_VAL + i);
      ads1115[i]->out_type = Out_Type::TYPE_TEMC;
    }

    ads1115[DWIN_ADC00_VOLT_220 - DWIN_ADC_VAL]->out_type = Out_Type::TYPE_VOLT_220;
    ads1115[DWIN_ADC01_CURRENT_220 - DWIN_ADC_VAL]->out_type = Out_Type::TYPE_CURRENT_220;

  }


}
