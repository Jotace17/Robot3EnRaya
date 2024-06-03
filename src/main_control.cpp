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
#define PIN_CS_M1 41 // (chip select for encoder of motor 1) 10 for SPI3
#define PIN_CS_M2 40 // (chip select for encoder of motor 2)
#define PIN_CS_M3 10 // (chip select for encoder of motor 3)
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

float _oldRef_m1 = 0;
float _oldRef_m2 = 0;
float _oldRef_m3 = 0;
float _oldAngle_m1 = 0;
float _oldAngle_m2 = 0;
float _oldAngle_m3 = 0;
float _newRef;
float _newRef_m1;
float _newRef_m2;
float _newRef_m3;


bool m2 = 0;
bool m3 = 1;

static int _first_time = 1;

// definition of global variables encoder
AlMar::Esp32::EncoderATM203_Spi2 *_enc;

// put function declarations here:
void GetAngle(float* angles);
float GetReference();
float ControlPiM1(float, float, float, float);  // adaptive P control for arm/shoulder 
float ControlPiM2(float, float, float, float);  // adaptive P control for arm/shoulder 
float ControlPiM3(float, float, float, float);  // Pi control for forearm/ellbow
float SetDutyDeadZone(int motor, float ducy);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  _enc = new AlMar::Esp32::EncoderATM203_Spi2(cs_pins, N_MOTORS, PIN_MOSI, PIN_MISO, PIN_SCLK);

  _m1 = new AlMar::Esp32::Driver_L298n(PIN_M1_EN, PIN_M1_IN1, PIN_M1_IN2, 200);
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
  float angles[3];            // array for read angle values
  float ducy_m2;              // variable to store duty cycle for motor 2 (shoulder)
  float ducy_m3;              // variable to store duty cycle for motor 3 (ellbow)
  float refPos;               // variable for reference angle of control
  float allowedError = 1.0;   // variable to define allowed error of control

  if (_first_time) // only execute in first execution of loop
  {
    //_newRef = 90;      // first reference for motor 2 - limits ca. 30° to 100°
    _newRef_m1 = 180;  // first reference for motor 2 - limits ca. 30° to 100°
    _newRef_m2 = 90;   // first reference for motor 2 - limits ca. 30° to 100°
    _newRef_m3 = 165;  // first reference for motor 3 - limits ca. 145 - 190°
    _first_time = 0;
  }

  GetAngle(angles);   // read all three angles from the encoders
  
if(m2){
  float currentAngle = angles[1];
  if (abs(_newRef_m2 - currentAngle) > allowedError)   // control loop if angle difference is > allowed error
  {
    /* only for debugging purposes */
    float refdif = _newRef_m2 - currentAngle;
    Serial.printf("_newRef-currentAngle = %.2f\t", refdif);
    
    /* calculation of duty cycle by control */
    float ducy_m2 = ControlPiM2(_newRef_m2, _oldRef_m2, currentAngle, _oldAngle_m2);
    Serial.printf(">Control Out m2: %f  \n", ducy_m2);

    _oldAngle_m2 = currentAngle;         // update value of old angle 

    /* only for debugging purposes */
    Serial.printf(">Current Angle deg m2: %f\n", currentAngle);
    Serial.printf(">ducy: %.2f \t", ducy_m2);
    Serial.printf("_newRef: %f \n", _newRef_m2);

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
    GetAngle(angles);
    currentAngle = angles[1];
    Serial.printf("else: angle read %.2f:\n", currentAngle);

    /* reading of new reference position */
    /* currently does only work in loop, because function returns 0 if there is no input in serial content is the same as in GetReference() */
    String refPosition;
    Serial.printf("D: Put in desired position in degree: \n");
    while (Serial.available() > 0)
    {
      /* Read the input position */
      refPosition = Serial.readString();
      refPosition.trim();
      Serial.printf("ref set to: %f \n", refPosition.toFloat());
      _newRef_m2 = refPosition.toFloat();      // pass string to float value for control
    }
  }
  float _oldRef_m2 = _newRef_m2;          // update value of previous reference - tbc if it is the right place here?? 
}
else if (m3){
  float currentAngle = angles[2];
  Serial.printf(">currentAngle m3: %f \n", currentAngle);
  if (abs(_newRef_m3 - currentAngle) > allowedError)   // control loop if angle difference is > allowed error
  {
    /* calculation of duty cycle by control */
    float ducy_m3 = ControlPiM3(_newRef_m3, _oldRef_m3, currentAngle, _oldAngle_m3);
    _oldAngle_m3 = currentAngle;         // update value of old angle 

    /* only for debugging purposes */
    float refdif = _newRef_m3 - currentAngle;
    Serial.printf(">refDif m3: %f\n", refdif);
    Serial.printf(">ducy_m3: %.2f \n", ducy_m3);
    Serial.printf(">_newRef_m3: %f \n", _newRef_m3);

    float ducyLimM3 =  SetDutyDeadZone(3, ducy_m3);
    _m3->SetDuty(ducyLimM3);
    Serial.printf(">Limitated DuCy M3: %f \n", ducyLimM3);
    /* limitation of duty cycle */
    /* to be cleaned up and put into function or into control */
    /*if (abs(ducy_m3) < 0.001) // motor 877-7174 is not moving with dutycycle lower than 0.11
    {
      _m3->SetDuty(0.0);
    }
    else if ((ducy_m3) < 0.15 && ducy_m3 > 0.0)
    {
      _m3->SetDuty(0.15);
    }
    else if ((ducy_m3) > -0.075 && ducy_m3 < -0.001)
    {
      _m3->SetDuty(-0.075);
    }
    else if ((ducy_m3) > 0.4)
    {
      _m3->SetDuty(0.4);
    }
    else if ((ducy_m3) < -0.4)
    {
      _m3->SetDuty(-0.4);
    }
    else
    {
      _m3->SetDuty(ducy_m3);
    }*/
  }
  else
  {
    /* stop motor and read actual angle */
    _m3->SetDuty(0); 
    GetAngle(angles);
    currentAngle = angles[2];

    /* reading of new reference position */
    /* currently does only work in loop, because function returns 0 if there is no input in serial content is the same as in GetReference() */
    String refPosition;
    Serial.printf("A: Put in desired position in degree: \n");
    while (Serial.available() > 0)
    {
      /* Read the input position */
      refPosition = Serial.readString();
      refPosition.trim();
      _newRef_m3 = refPosition.toFloat();      // pass string to float value for control
      //Serial.printf("ref set to: %f \n", refPosition.toFloat());
      Serial.printf(">ref: %f \n", _newRef_m3);
    }
  }
  float _oldRef_m3 = _newRef_m3;          // update value of previous reference - tbc if it is the right place here?? 
}

else {
  float currentAngle = angles[0];
  if (abs(_newRef_m1 - currentAngle) > allowedError)   // control loop if angle difference is > allowed error
  {
    /* only for debugging purposes */
    float refdif = _newRef_m1 - currentAngle;
    Serial.printf("_newRef_m1-currentAngle = %.2f\t", refdif);
    
    /* calculation of duty cycle by control */
    float ducy_m1 = ControlPiM1(_newRef_m1, _oldRef_m1, currentAngle, _oldAngle_m1);
    Serial.printf(">Control Out m1: %f.1  \n", ducy_m1);

    _oldAngle_m1 = currentAngle;         // update value of old angle 

    /* only for debugging purposes */
    Serial.printf(">Current Angle deg m1: %f\n", currentAngle);
    Serial.printf(">ducy_m1: %.2f \n", ducy_m1);
    Serial.printf("_newRef_m1: %f \n", _newRef_m1);

    /* limitation of duty cycle */
    /* to be cleaned up and put into function or into control */
    /*if (abs(ducy_m1) < 0.001) // motor 877-7174 is not moving with dutycycle lower than 0.11
    {
      _m1->SetDuty(0.0);
    }
    else if ((ducy_m1) < 0.13 && ducy_m1 > 0.0)
    {
      _m1->SetDuty(0.13);
    }
    else if ((ducy_m1) > -0.13 && ducy_m1 < -0.001)
    {
      _m1->SetDuty(-0.13);
    }
    else if ((ducy_m1) > 0.4)
    {
      _m1->SetDuty(0.4);
    }
    else if ((ducy_m1) < -0.4)
    {
      _m1->SetDuty(-0.4);
    }
    else*/
    //{
      _m1->SetDuty(ducy_m1);
    //}
  }
  else
  {
    /* stop motor and read actual angle */
    _m1->SetDuty(0); 
    GetAngle(angles);
    currentAngle = angles[0];
    //Serial.printf("else: angle read %.2f:\n", currentAngle);

    /* reading of new reference position */
    /* currently does only work in loop, because function returns 0 if there is no input in serial content is the same as in GetReference() */
    String refPosition;
    Serial.printf("(%i) ref_m1: %f, error: %f \t %f \n", (abs(_newRef_m1 - currentAngle)) > allowedError, _newRef_m1, (abs(_newRef_m1 - currentAngle)), allowedError);
    
    while (Serial.available() > 0)
    {
      Serial.printf("B: Put in desired position in degree: \n");
      /* Read the input position */
      refPosition = Serial.readString();
      refPosition.trim();
      _newRef_m1 = refPosition.toFloat();      // pass string to float value for control
      //Serial.printf("ref set to: %f \n", refPosition.toFloat());
      Serial.printf(">ref_m1: %f \n", _newRef_m1);
    }
  }
  float _oldRef_m1 = _newRef_m1;          // update value of previous reference - tbc if it is the right place here?? 
}

  /* wait 10ms */
  /* only for standalone control code necessary to be done by scheduler in integrated program */
  delay(10);
}

/* function definitions */
void GetAngle(float* angles)
{
  // read encoder
  int posBase = (float)_enc->Read(0);
  int posShoulder = (float)_enc->Read(1);
  int posEllbow = (float)_enc->Read(2);

  float posAll[] = {(float)posBase,(float)posShoulder,(float)posEllbow};
  
  // Define the size of the posAll array
    const int size = sizeof(posAll) / sizeof(posAll[0]);

  if (posBase != 0x80000000)
  {
    // Perform the multiplication and store the results in readAngle
    for (int i = 0; i < size; i++) {
      angles[i] = posAll[i] * 360.0f / 4096.0f;
    }
    //float readAngle[] = posAll[] * 360 / 4096;
    /*Serial.printf(">Angle m1 (Base): %f \n", *readAngle[0]);
    Serial.printf(">Angle m2 (Shoulder): %f \n", *readAngle[1]);
    Serial.printf(">Angle m3 (Ellbow): %f \n", *readAngle[2]);*/
    //return readAngle;
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

float ControlPiM3(float ref, float refOld, float angle, float angleOld)   // control for ellbow motor
{
  // defintion of proportional gain 
  float kP = 1.8;       // kP for downward movement
  
  if ((ref-angle) > 0)  // upwards
  {
    kP = 3.0;           // kP for upward movement
  }

  // print proportional gain - only for debugging purposes
  Serial.printf("kP: %f \n", kP);

  // calculate proportional part = control output
  float controlOut = kP * ((ref - angle) / 360);

  return controlOut;
}

float ControlPiM1(float ref, float refOld, float angle, float angleOld)   // control for ellbow motor
{
  // defintion of proportional gain 
  float kP = 2;       // kP 
  float kI = 0.05;
  int satPos = 1; // values took over from Juan tbc if correct
  int satNeg = -1;
  // print proportional gain - only for debugging purposes
  Serial.printf(">kP_m1: %f \n", kP);
  Serial.printf(">kI_m1: %f \n", kI);

  float timeStep = 0.01;
  float intPart;

  float dI = kI * timeStep * (refOld - angleOld);
  if (_sat * dI > 0)
    {
        intPart = _iOld;
    }
    else
    {
        intPart = _iOld + dI;
  }
  Serial.printf(">intPar_m1: %f \n", kI);
  // calculate proportional part = control output
  float controlOut = (kP * ((ref - angle) / 360)) + intPart;
  _iOld = intPart;
  return controlOut;
}

float SetDutyDeadZone(int motor, float ducy)
{
  float minLim = 0.001; // minimum duty cycle where it will not be set to 0
  float maxLim = 0.4;   // maximum duty cycle which will be ever used
  float ducyLim = 0;    // limitated duty cycle
  float ducyMinUp;
  float ducyMinDown;

  switch (motor)
  {
  case 1: // base
    // currently empty
    break;
  case 2: // shoulder
    ducyMinUp = 0.15;
    ducyMinDown = -0.11;
    break;
  case 3:
    ducyMinUp = 0.15;
    ducyMinDown = -0.075;
    break;
  }
  if (abs(ducy) < minLim) // motor 877-7174 is not moving with dutycycle lower than 0.11
  {
    ducyLim = 0.0;
  }
  else if ((ducy) < ducyMinUp && ducy > minLim)
  {
    ducyLim = ducyMinUp;
  }
  else if ((ducy) > ducyMinDown && ducy < -minLim)
  {
    ducyLim = ducyMinDown;
  }
  else if ((ducy) > maxLim)
  {
    ducyLim = maxLim;
  }
  else if ((ducy) < -maxLim)
  {
    ducyLim = -maxLim;
  }
  else
  {
    ducyLim = ducy;
  }

  return ducyLim;
}