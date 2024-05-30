#include <Arduino.h>
#include "../lib/AlMar_ESP32_Driver_L298n.h"
#include "../lib/AlMar_ESP32_Driver_L298n.cpp"
#include "../lib/ALMar_ESP32_EncoderATM203_Spi2.cpp"

// #include "../lib/PMEC_E2_Control.h"

// definition of pins
#define PIN_M1_EN 7
#define PIN_M1_IN1 15
#define PIN_M1_IN2 16

#define PIN_M2_EN 5
#define PIN_M2_IN1 6
#define PIN_M2_IN2 14

#define PIN_M3_EN 17
#define PIN_M3_IN1 18
#define PIN_M3_IN2 8

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
AlMar::Esp32::Driver_L298n *_m2;
AlMar::Esp32::Driver_L298n *_m3;
float _dOld = 0;
float _iOld = 0;
int _sat = 0;

float _oldRef = 0;
float _oldAngle = 0;
float _newRef;
float  _newRef_m2;

static int _first_time = 1;

// definition of global variables encoder
AlMar::Esp32::EncoderATM203_Spi2 *_enc;

// put function declarations here:
float GetAngle();
float GetReference();
float ControlPiM2(float, float, float, float);  // Pi control for arm/shoulder 

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  _enc = new AlMar::Esp32::EncoderATM203_Spi2(cs_pins, N_MOTORS, PIN_MOSI, PIN_MISO, PIN_SCLK);

  _m1 = new AlMar::Esp32::Driver_L298n(PIN_M2_EN, PIN_M2_IN1, PIN_M2_IN2, 200);
  _m1->begin();
  // M2 - shoulder 
  _m2 = new AlMar::Esp32::Driver_L298n(PIN_M2_EN, PIN_M2_IN1, PIN_M2_IN2, 200);
  _m2->begin();

  // M3 - ellbow
  _m3 = new AlMar::Esp32::Driver_L298n(PIN_M3_EN, PIN_M3_IN1, PIN_M3_IN2, 200);
  _m3->begin();
}

void loop()
{
  // definition of local variables
  float ducy_m2;              // variable to store duty cycle for motor 2 (shoulder)
  float ducy_m3;              // variable to store duty cycle for motor 3 (ellbow)
  float refPos;             // variable for reference angle of control
  float allowedError = 1.0; // variable to define allowed error of control

  if (_first_time) // only execute in first execution of loop
  {
    _newRef = 90;  // first reference for motor 1 - limits ca. 30° to 100°
    _newRef_m2 = 150; // first reference for motor 2 - limits ca. 130° - 180°
    //_newRef = 135 ;    // first reference  for ellbow motor
    _first_time = 0;
  }

  float currentAngle = GetAngle();    // read angle from encoder

  if (abs(_newRef - currentAngle) > allowedError)   // control loop if angle difference is > allowed error
  {
    /* only for debugging purposes */
    float refdif = _newRef - currentAngle;
    Serial.printf("_newRef-currentAngle = %.2f\t", refdif);
    
    /* calculation of duty cycle by control */
    float ducy_m2 = ControlPiM2(_newRef, _oldRef, currentAngle, _oldAngle);
    Serial.printf("Control Out: %f.1  \t\t", ducy_m2);

    _oldAngle = currentAngle;         // update value of old angle 

    /* only for debugging purposes */
    Serial.printf("Current Angle deg: %f\t\t### \t\t", currentAngle);
    Serial.printf("ducy: %.2f \t", ducy_m2);
    Serial.printf("_newRef: %f \n", _newRef);

    /* limitation of duty cycle */
    /* to be cleaned up and put into function or into control */
    if (abs(ducy_m2) < 0.001) // motor 877-7174 is not moving with dutycycle lower than 0.11
    {
      _m2->SetDuty(0.0);
    }
    else if ((ducy_m2) < 0.15 && ducy_m2 > 0.0)
    {
      _m2->SetDuty(0.15);
    }
    else if ((ducy_m2) > -0.11 && ducy_m2 < -0.001)
    {
      _m2->SetDuty(-0.11);
    }
    else if ((ducy_m2) > 0.4)
    {
      _m2->SetDuty(0.4);
    }
    else if ((ducy_m2) < -0.4)
    {
      _m2->SetDuty(-0.4);
    }
    else
    {
      _m2->SetDuty(ducy_m2);
    }
  }
  else
  {
    /* stop motor and read actual angle */
    _m2->SetDuty(0); 
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
  delay(10);
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
    Serial.printf("ref set to: %f \t", refPosition.toFloat());

    refPos = refPosition.toFloat();
  }
  return refPos;
}

float ControlPiM2(float ref, float refOld, float angle, float angleOld)
{
  // defintion of constants of control
  float kP = 1.3;
  ;
  if ((ref-angle) > 0) 
  {
    kP = 5 + (0.04*(90-angle));  // adaptive proportional gain - motor forearm 
  }
  else 
  {
    // do nothing
  }
  Serial.printf("kP: %f \n", kP);

  float kI = 2.2;   // integral gain - motor forearm
  //float kD = 0;     // differential gain -motor forearm

  // definition of timestep (depends on recurring task time in which control will run) & saturation limits
  // for now set to 10ms
  float timeStep = 0.01;

  // definition of factors for easier readability of the formulas, only needed if filter will be used
  float intPart;

  // calculate integral part
  float dI = kI * timeStep * ((refOld - angleOld) / 360);

  // calculate proportional part
  float propPart = kP * ((ref - angle) / 360);

  // calculate differential part
  //float difPart = _dOld + kD * ((ref - refOld) / 360) - kD * ((angle - angleOld) / 360);

  // calculate control output
  //float controlOut = (propPart + difPart + intPart);
  float controlOut = (propPart + intPart);

  return controlOut;
}