#include <Arduino.h>
#include "../lib/AlMar_ESP32_Driver_L298n.h"
#include "../lib/AlMar_ESP32_Driver_L298n.cpp"
#include "../lib/ALMar_ESP32_EncoderATM203_Spi2.cpp"

// #include "../lib/PMEC_E2_Control.h"

// definition of pins
#define PIN_M1_EN 7
#define PIN_M1_IN1 6
#define PIN_M1_IN2 14

// definition of pins encoder
#define PIN_CS_M1 10 // (chip select for encoder of motor 1) 10 for SPI3
#define PIN_CS_M2 40 // (chip select for encoder of motor 2)
#define PIN_CS_M3 41 // (chip select for encoder of motor 3)
// common pins for all encoders
#define PIN_SCLK 12 // 36 // 12 for SPI3
#define PIN_MISO 13 // 37 // 13 for SPI3
#define PIN_MOSI 11 // 35 // 11 for SPI3

#define N_MOTORS 3 // 35 // 11 for SPI3

// Define pins
int cs_pins[] = {PIN_CS_M1, PIN_CS_M2, PIN_CS_M3};

// defintion of PWM frecuency
#define PWM_FREQ_HZ 20000

// definition of global variables control
AlMar::Esp32::Driver_L298n *_m1;
float _dOld = 0;
float _iOld = 0;
int _sat = 0;

float _oldRef = 0;
float _oldAngle = 0;
float _newRef;

static int _first_time = 1;

// definition of global variables encoder
AlMar::Esp32::EncoderATM203_Spi2 *_enc;

// put function declarations here:
float GetAngle();
float GetReference();
float ControlPid(float, float, float, float);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  _enc = new AlMar::Esp32::EncoderATM203_Spi2(cs_pins, N_MOTORS, PIN_MOSI, PIN_MISO, PIN_SCLK);

  _m1 = new AlMar::Esp32::Driver_L298n(PIN_M1_EN, PIN_M1_IN1, PIN_M1_IN2, 200);
  _m1->begin();
}

void loop()
{
  // definition of local variables
  float ducy0;              // variable to store duty cycle
  float refPos;             // variable for reference angle of control
  float allowedError = 1.0; // variable to define allowed error of control

  if (_first_time) // only execute in first execution of loop
  {
    _newRef = 20;  // first reference at startup currently set to 20° to be defined with calibration 
    _first_time = 0;
  }

  float currentAngle = GetAngle();    // read angle from encoder

  if (abs(_newRef - currentAngle) > allowedError)   // control loop if angle difference is > allowed error
  {
    /* only for debugging purposes */
    float refdif = _newRef - currentAngle;
    Serial.printf("_newRef-currentAngle = %.2f\t", refdif);
    
    /* calculation of duty cycle by control */
    float ducy_m1 = ControlPid(_newRef, _oldRef, currentAngle, _oldAngle);
    Serial.printf("Control Out: %f.1  \t\t", ducy_m1);

    _oldAngle = currentAngle;         // update value of old angle 

    /* only for debugging purposes */
    Serial.printf("Current Angle deg: %f\t\t### \t\t", currentAngle);
    Serial.printf("ducy: %.2f \t", ducy_m1);
    Serial.printf("_newRef: %f \n", _newRef);

    /* limitation of duty cycle */
    /* to be cleaned up and put into function or into control */
    if (abs(ducy_m1) < 0.001) // motor 877-7174 is not moving with dutycycle lower than 0.11
    {
      _m1->SetDuty(0.0);
    }
    else if ((ducy_m1) < 0.11 && ducy_m1 > 0.0)
    {
      _m1->SetDuty(0.11);
    }
    else if ((ducy_m1) > -0.11 && ducy_m1 < -0.001)
    {
      _m1->SetDuty(-0.11);
    }
    else if ((ducy_m1) > 0.4)
    {
      _m1->SetDuty(0.4);
    }
    else if ((ducy_m1) < -0.4)
    {
      _m1->SetDuty(-0.4);
    }
    else
    {
      _m1->SetDuty(ducy_m1);
    }
  }
  else
  {
    /* stop motor and read actual angle */
    _m1->SetDuty(0); 
    currentAngle = GetAngle();
    Serial.printf("else: angle read %.2f:\n", currentAngle);

    /* reading of new reference position */
    /* currently does only work in loop, because function returns 0 if there is no input in serial content is the same as in GetReference() */
    String refPosition;
    Serial.printf("Put in desired position in degree: \n");
    while (Serial.available() > 0)
    {
      /* Read the input position */
      refPosition = Serial.readString();
      refPosition.trim();
      Serial.printf("ref set to: %f \n", refPosition.toFloat());
      _newRef = refPosition.toFloat();      // pass string to float value for control
    }
  }
  float _oldRef = _newRef;          // update value of previous reference - tbc if it is the right place here?? 

  /* wait 100ms */
  /* only for standalone control code necessary to be done by scheduler in integrated program */
  delay(100);
}

/* function definitions */
float GetAngle()
{
  // read encoder
  int pos = _enc->Read(0);
  float posF = (float)pos;

  if (pos != 0x80000000)
  {
    float readAngle = posF * 360 / 4096;
    return readAngle;
  }
  else
  {
    // do nothing
  }
}

float GetReference()
{
  float refPos;
  String refPosition;
  Serial.printf("Put in desired position in degree: \n");

  while (Serial.available() > 0)
  {
    // Read the input position

    refPosition = Serial.readString();
    refPosition.trim();
    // Serial.printf("ref position: %s", refPosition);
    Serial.printf("ref set to: %f \n", refPosition.toFloat());

    refPos = refPosition.toFloat();
  }
  return refPos;
}

float ControlPid(float ref, float refOld, float angle, float angleOld)
{
  // defintion of constants of control
  float kP = 2.9;  // proportional gain - motor forearm
  float kI = 0.22; // integral gain - motor forearm
  float kD = 0.1;  // differential gain -motor forearm

  // definition of timestep (depends on recurring task time in which control will run) & saturation limits
  // for now set to 10ms
  float timeStep = 0.1;

  // definition of factors for easier readability of the formulas, only needed if filter will be used
  float intPart;

  // calculate integral part
  float dI = kI * timeStep * ((refOld - angleOld) / 360);

  // calculate proportional part
  float propPart = kP * ((ref - angle) / 360);

  // calculate differential part
  float difPart = _dOld + kD * ((ref - refOld) / 360) - kD * ((angle - angleOld) / 360);

  // calculate control output
  float controlOut = (propPart + difPart + intPart);

  return controlOut;
}