// definition of constants

//PWM output pins - to be defined by the project
#define GPIO_PWM0A_OUT 18           
#define GPIO_PWM1A_OUT 19

// prototype functions
float ControlPid(float ref, float refOld, float angle, float angleOld);

float GetReference();

float GetAngle();

void main();