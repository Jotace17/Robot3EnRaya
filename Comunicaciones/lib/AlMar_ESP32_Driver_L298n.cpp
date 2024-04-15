#include "AlMar_ESP32_Driver_L298n.h"
#include "Arduino.h"

#ifdef ESP32

#ifdef USING_MCPWM
#include <driver/mcpwm.h>
#define ALMAR_MCPWM_CHANNEL_INC    1  
#define ALMAR_MCPWM_CHANNEL_COUNT   2
#define ALMAR_MCPWM_GROUP_RESOLUTION  10000000  // https://docs.espressif.com/projects/esp-idf/en/v4.4.6/esp32/api-reference/peripherals/mcpwm.html , mcpwm_init() function
#define ALMAR_MCPWM_TIMER_RESOLUTION  1000000

int AlMar::Esp32::Driver_L298n::mcpwm_lastUnitNo=-1,AlMar::Esp32::Driver_L298n::mcpwm_lastChannNo=ALMAR_MCPWM_CHANNEL_COUNT;
#endif

AlMar::Esp32::Driver_L298n::Driver_L298n(int pin_en,int pin_m1,int pin_m2,int pwm_freq) {
    pinEN=pin_en;
    pinM1=pin_m1;
    pinM2=pin_m2;
    pwmFreq=pwm_freq;
    active=false;

    pinMode(pinM1,OUTPUT);
    digitalWrite(pinM1,0);
    pinMode(pinM2,OUTPUT);
    digitalWrite(pinM2,0);

#ifndef USING_MCPWM
    pinMode(pinEN,OUTPUT);
    analogWrite(pinEN,0);
#else // USING_MCPWM  
    esp_err_t err;
    mcpwm_lastChannNo+=ALMAR_MCPWM_CHANNEL_INC;  
    if (mcpwm_lastChannNo>=ALMAR_MCPWM_CHANNEL_COUNT) {
        mcpwm_lastUnitNo++;
        mcpwm_lastChannNo=0;
        if (mcpwm_lastUnitNo>=MCPWM_UNIT_MAX) {
           ESP_ERROR_CHECK(ESP_ERR_INVALID_ARG);
           return; 
        }
        mcpwm_config_t conf = {
            .frequency = pwm_freq,  // 2000 for pmodhb3 https://digilent.com/reference/_media/reference/pmod/pmodhb3/pmodhb3_rm.pdf
            .cmpr_a=0,
            .cmpr_b=0,
            .duty_mode=MCPWM_DUTY_MODE_0,
            .counter_mode=MCPWM_UP_COUNTER
        };
        mcpwm_Unit=(MCPWM_UNIT_0+mcpwm_lastUnitNo);

         Serial.printf("Initializing unit %d , freq=%d\n",(int) mcpwm_Unit,pwm_freq);
        esp_err_t err=mcpwm_init((mcpwm_unit_t) mcpwm_Unit,MCPWM_TIMER_0,&conf);
        ESP_ERROR_CHECK(err);
    }
    else {
      mcpwm_Unit=(MCPWM_UNIT_0+mcpwm_lastUnitNo);
    }

    mcpwm_Chann=(MCPWM0A+mcpwm_lastChannNo);
    Serial.printf("Initializing GPIO for unit %d, chann %d, pin=%d\n",(int) mcpwm_Unit,(int) mcpwm_Chann,pinEN);

    err=mcpwm_gpio_init((mcpwm_unit_t) mcpwm_Unit, (mcpwm_io_signals_t) mcpwm_Chann,pinEN);
    ESP_ERROR_CHECK(err);

#endif
}

bool AlMar::Esp32::Driver_L298n::begin() {
  active=true;
#ifdef USING_MCPWM
//    dutyCur=0;
    SetDuty(0);
    esp_err_t err=mcpwm_start((mcpwm_unit_t) mcpwm_Unit, MCPWM_TIMER_0);
    ESP_ERROR_CHECK(err);

#endif
  return active;
}

bool AlMar::Esp32::Driver_L298n::SetDuty(float duty) {
  if (! active)
    return false;
  if (duty>0) {
    digitalWrite(pinM2,0);
    digitalWrite(pinM1,1);
  } else if (duty<0) {
    digitalWrite(pinM2,1);
    digitalWrite(pinM1,0);
  } else  { // duty==0
    digitalWrite(pinM2,0);
    digitalWrite(pinM1,0);
  }

  duty=fabs(duty);
#ifndef USING_MCPWM
  if (duty==0) {
      analogWrite(pinEN,0);
  }
  else  {
    analogWrite(pinEN,duty*255);
  }

#else  // USING_MCPWM  
  esp_err_t err=mcpwm_set_duty((mcpwm_unit_t) mcpwm_Unit, MCPWM_TIMER_0, (mcpwm_generator_t) (MCPWM_GEN_A+mcpwm_Chann), duty*100);
  ESP_ERROR_CHECK(err);
  err=mcpwm_set_duty_type((mcpwm_unit_t) mcpwm_Unit, MCPWM_TIMER_0, (mcpwm_generator_t) (MCPWM_GEN_A+mcpwm_Chann), MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
  ESP_ERROR_CHECK(err);
#endif

  return true;
}


#endif // ESP32