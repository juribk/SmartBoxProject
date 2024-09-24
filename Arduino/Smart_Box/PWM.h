#ifndef PWM_H
#define PWM_H

namespace pwm
{
#define PWM_PIN_1           14
#define PWM_PIN_2           15
#define PWM_RESOLUTION      12
#define PWM_FREQUENCY       60


  class PWM
  {
    private:
      uint8_t pin;
      uint32_t frequency;
      uint32_t duty_cycle;
      float duty_cycle_percent;
      uint32_t resolution;
      uint32_t duty_cycle_max;

    public:
      bool initialized;

    public:
      PWM(uint8_t pin, uint32_t frequency, uint32_t resolution);
      bool Init(uint8_t pin, uint32_t frequency, uint32_t resolution);
      bool Set_Duty_Cycle_Percent(float percent);
      bool Set_Duty_Cycle(int dutyCycle);
      float Get_Duty_Cycle_Percent();
      float Get_Duty_Cycle();

  };


  // -------------------------------------------------
  // -------------------------------------------------
  // -------------------------------------------------
  extern PWM* pwm[2];
  bool Init();






}



#endif // PWM_H