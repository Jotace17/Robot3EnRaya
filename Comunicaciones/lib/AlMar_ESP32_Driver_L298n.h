#ifndef AlMar_ESP32_Driver_L298n
#define AlMar_ESP32_Driver_L298n

//! \file AlMar_ESP32_Driver_L298n.h
//! \brief Set PWM for driver L298n


#include <stdint.h>

#define USING_MCPWM


namespace AlMar 
{
#ifdef ESP32
namespace Esp32 {
class Driver_L298n {
private:
    int pinEN,pinM1,pinM2;
    int pwmFreq;
    bool active=false;
#ifdef USING_MCPWM
  int mcpwm_Unit,mcpwm_Chann;
    static int mcpwm_lastUnitNo,mcpwm_lastChannNo;
#endif 
public:
    //! Constructor
    //! \param : pin_en --> enable pin
    //! \param: pin_m1 --> PWM pin M1
    //! \param: pin_m2 --> PWM pin M2
    //! \param: pwm_freq --> PWM frequency in Hz
    Driver_L298n(int pin_en,int pin_m1,int pin_m2,int pwm_freq);

    //! Start
    //! \return: bool indicating success
    bool begin();

    //! Start
    //! \param : duty --> value between -1.0 and 1.0 for the duty cycle
    //! \return: bool indicating success
	 bool SetDuty(float duty);
private:
} ;

} // namespace Esp32

#endif // ESP32

} // namespace AlMar

#endif // AlMar_ESP32_Driver_L298n