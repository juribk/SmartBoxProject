#include "Params.h"
#include "DWIN.h"

#include <PWM.h> 


namespace pwm
{
  static const char *TAG = "PWM_events";

  PWM::PWM(uint8_t pin, uint32_t frequency, uint32_t resolution)
  {
    Init(pin, frequency, resolution);
  }

  bool PWM::Init(uint8_t pin, uint32_t frequency, uint32_t resolution)
  {
    this->pin = pin;
    this->frequency = frequency;
    this->resolution = resolution;
    duty_cycle_max = pow(2, resolution);

    //ESP_LOGI(TAG, "PWM initialization: pin=%d, frequency=%d, resolution=%d", pin, frequency, resolution);

    initialized = ledcAttach(pin, frequency, resolution);
    if (!initialized )
    {
      ESP_LOGE(TAG, "PWM initialization error! pin=%d, frequency=%d, resolution=%d", pin, frequency, resolution);
    }
    return initialized;
  }

  float PWM::Get_Duty_Cycle_Percent()
  {
    return duty_cycle_percent;
  }

  float PWM::Get_Duty_Cycle()
  {
    return duty_cycle;
  }

  bool PWM::Set_Duty_Cycle_Percent(float percent)
  {
    duty_cycle_percent = percent;
    uint32_t dutyCycle = duty_cycle_percent * ((float)duty_cycle_max / 100.0f);
    return Set_Duty_Cycle(dutyCycle);
  }

  bool PWM::Set_Duty_Cycle(int dutyCycle)
  {
    //ESP_LOGI(TAG, "Set_Duty_Cycle: pin=%d, dutyCycle=%d", pin, dutyCycle);
    return ledcWrite(pin, dutyCycle);
  }


  // -------------------------------------------------
  // -------------------------------------------------
  // -------------------------------------------------
  PWM* pwm[2];
  bool Init()
  {

    pwm[1] = new PWM(PWM_PIN_2, PWM_FREQUENCY, PWM_RESOLUTION);
    pwm[0] = new PWM(PWM_PIN_1, PWM_FREQUENCY, PWM_RESOLUTION);

    /// TODO: Взять из параметров
    pwm[0]->Set_Duty_Cycle(1000);
    pwm[1]->Set_Duty_Cycle(1000);

    return pwm[0]->initialized & pwm[1]->initialized;

    //ledcAttach(PWM_PIN_1, PWM_FREQUENCY, PWM_RESOLUTION);
    //ledcAttach(PWM_PIN_2, PWM_FREQUENCY, PWM_RESOLUTION);
    //ledcWrite(PWM_PIN_1, 1000);
    //ledcWrite(PWM_PIN_2, 1000);
  }



}













