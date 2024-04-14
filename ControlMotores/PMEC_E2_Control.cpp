/* This file contains the code which calculates the PID control of one motor */
/* as of the current state this code is not tested!! Therefore unclear if functional
some of the functions might only be placeholder functions, especially the PWM part is not yet final*/

// includes
#include "PMEC_E2_Control.h"
#include <Arduino.h>
#include <driver/mcpwm.h>
#include <soc/mcpwm_periph.h>

// definition of global variables -
// - tbc if could be done with pointers instead of global variables
float _dOld = 0; // diferential value in previous step
float _iOld = 0; // integral value in previous step
int _sat = 0;    // state of saturation of control

float _oldRef = 0;
float _oldAngle = 0;

// definitions for PWM output
int _frec0 = 20000; //20 kHz


// example of a main program, in the end will probably not run here...
void main()
{
    // variable declarations
    float currentAngle;
    float newRef;
    float out;

    while (1) // endless loop, should be done in the end by the scheduler
    {
        // read current angle and the new reference value
        currentAngle = GetAngle();
        newRef = GetReference();

        while (abs(newRef - currentAngle) > 2)
        {
            out = ControlPid(newRef, _oldRef, currentAngle, _oldAngle);
            _oldAngle = currentAngle;
            currentAngle = GetAngle();

            // does this only affect one go on the H bridge, is the second part needed?? Try out, what happens if duty0/1 is negative?
            duty0 = (abs(out)/255)*100;
            duty1 = 100 - duty0;
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, duty0);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, duty1);
        }
        _oldRef = newRef;
    }
}

// pwm setup as in example from campus virtual- unclear if complete, duty cycles must be in porcentage and give 100% together
void conf_pwm0()            
{

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT); // output 0A to pin 
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT); // output 1A to defined pin

    mcpwm_config_t pwm_conf0;                               // Data structure for configuration
    pwm_conf0.frequency = frec0;
    pwm_conf0.cmpr_a = 0;                                   // duty cycle 0%. PWMxA, Value not important since not running
    pwm_conf0.cmpr_b = 0;                                   // duty cycle 0% PWMxB. Value not important since not running
    pwm_conf0.counter_mode = MCPWM_UP_COUNTER;              // UP-down counter
    pwm_conf0.duty_mode = MCPWM_DUTY_MODE_0;                // Active high

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_conf0);    // Configuration of PWM0
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_conf0);  
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_conf0); 
    mcpwm_deadtime_disable(MCPWM_UNIT_0, MCPWM_TIMER_0);
    mcpwm_deadtime_disable(MCPWM_UNIT_0, MCPWM_TIMER_1);
    mcpwm_deadtime_disable(MCPWM_UNIT_0, MCPWM_TIMER_2);

    mcpwm_set_timer_sync_output(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SWSYNC_SOURCE_SYNCIN);

    mcpwm_sync_config_t sync_conf;
    sync_conf.sync_sig = MCPWM_SELECT_TIMER0_SYNC;
    sync_conf.timer_val = 0;
    sync_conf.count_direction = MCPWM_TIMER_DIRECTION_UP;

    mcpwm_sync_configure(MCPWM_UNIT_0, MCPWM_TIMER_0, &sync_conf);
    mcpwm_sync_configure(MCPWM_UNIT_0, MCPWM_TIMER_1, &sync_conf);
    mcpwm_sync_configure(MCPWM_UNIT_0, MCPWM_TIMER_2, &sync_conf);
    mcpwm_timer_trigger_soft_sync(MCPWM_UNIT_0, MCPWM_TIMER_0);

/* is this part really needed in the configuration part??
    float duty0=40; //Ciclo al 40%
    float duty1=60; //Ciclo al 40%
    delay(400);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, duty0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, duty1);*/
}
// Serial.begin(9600); has to go in global setup, do not forget!!

float GetAngle()
{
    // read encoder, probably already existent
    return readAngle;
}

float GetReference()
{
    String refPosition;
    if (Serial.available() > 0)
    {
        Serial.printf("Put in desired position in degree: \n");
        // Read the input position
        refPosition = Serial.readString();
    }
    return float refPos = refPosition.toFloat();
}

float ControlPid(float ref, float refOld, float angle, float angleOld)
{
    // defintion of constants of control
    // those values will be updated with the values from PID tuning with the Simulink Model which provided Juan
    float kP = 5.0;  // proportional gain
    float kI = 10.0; // integral gain
    float kD = -0.5; // differential gain

    // definition of values for filtered derivative (PID from simulink)
    // currently set to 1 as PID tuning has not yet been done, so those are inexistent
    float filterCoeffN = 1.0; // Filter coefficient
    float spWeightB = 1.0;    // Setpoint weight
    float spWeightC = 1.0;    // Setpoint weight

    // definition of timestep (depends on recurring task time in which control will run) & saturation limits
    // for now set to 10ms
    float timeStep = 0.01;
    int satPos = 245; // values took over from Juan tbc if correct
    int satNeg = -245;

    // definition of factors for easier readability of the formulas, only needed if filter will be used
    float fac1 = 1 - filterCoeffN * timeStep;
    float fac2 = kD * filterCoeffN;

    // calculate integral part
    float dI = kI * timeStep * (refOld - angleOld);
    // Antiwind-up
    if (_sat * dI > 0)
    {
        float intPart = _iOld;
    }
    else
    {
        float intPart = _iOld + dI;
    }

    // calculate proportional part
    float propPart = kP * (spWeightB * ref - angle);

    // calculate differential part
    float difPart = fac1 * _dOld + fac2 * spWeightC * (ref - refOld) - fac2 * (angle - angleOld);

    // calculate control output
    float controlOut = propPart + intPart + difPart;

    // limitation of output signal
    if (controlOut > satPos)
    {
        controlOut = satPos;
        _sat = 1;
    }
    else if (controlOut < satNeg)
    {
        controlOut = satNeg;
        _sat = -1;
    }
    else
    {
        _iOld = intPart; // update integral for next step
    }
    _dOld = difPart; // update difPart for next step

    return controlOut;
}