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

hw_timer_t *timer = NULL;
bool _expired = false;
void IRAM_ATTR timerInterrupt()
{
  _expired = true;
}

float _dOld = 0;
float _iOld = 0;
int _sat = 0;

float _oldRef_m1 = 0;
float _oldRef_m2 = 0;
float _oldRef_m3 = 0;
float _oldAngle_m1 = 0;
float _oldAngle_m2 = 0;
float _oldAngle_m3 = 0;
float _newRef_m1;
float _newRef_m2;
float _newRef_m3;

bool m1 = 0;
bool m2 = 0;
bool m3 = 1;

static int _first_time = 1;
const float _dt = 0.01; // 10ms

// definition of global variables encoder
AlMar::Esp32::EncoderATM203_Spi2 *_enc;

// put function declarations here:
void GetAngle(float *angles);
float GetReference(float *angles);
float ControlPiM1(float, float, float, float); // adaptive P control for arm/shoulder
float ControlPiM2(float, float, float, float); // adaptive P control for arm/shoulder
float ControlPiM3(float, float, float, float); // Pi control for forearm/ellbow
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

  /* timer initialization*/
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &timerInterrupt, true); // Attach the interrupt handling function
  timerAlarmWrite(timer, 1000000 * _dt, true);        // Interrupt every 1 second
  timerAlarmEnable(timer);                            // Enable the alarm
}

void loop()
{
  if (_expired) // only run when interrupt is calling function
  {
    // definition of local variables
    float angles[3];          // array for read angle values
    float ducy_m2;            // variable to store duty cycle for motor 2 (shoulder)
    float ducy_m3;            // variable to store duty cycle for motor 3 (ellbow)
    float refPos;             // variable for reference angle of control
    float allowedError = 1.8; // variable to define allowed error of control

    if (_first_time) // only execute in first execution of loop - set initial reference angles
    {
      _newRef_m1 = 162; // first reference for motor 2 - limits ca. 30° to 100°
      _newRef_m2 = 80;  // first reference for motor 2 - limits ca. 30° to 100°
      _newRef_m3 = 160; // first reference for motor 3 - limits ca. 145 - 190°
      _first_time = 0;
    }

    GetAngle(angles); // read all three angles from the encoders

    /* only for debugging purposes */
    Serial.printf(">A currentAngle m1: %.2f\n", (angles[0]));
    Serial.printf(">A currentAngle m2: %.2f\n", (angles[1]));
    Serial.printf(">A currentAngle m3: %.2f\n", (angles[2]));
    /* */

    if ((abs(_newRef_m2 - angles[1]) > allowedError) && m2)// control loop for shoulder motor if angle difference is > allowed error
    {
      /* calculation of duty cycle by control */
      float ducy_m2 = ControlPiM2(_newRef_m2, _oldRef_m2, angles[1], _oldAngle_m2);
      _oldAngle_m2 = angles[1]; // update value of old angle

      /* limitation of duty cycle */
      float ducyLimM2 = SetDutyDeadZone(2, ducy_m2);
      _m2->SetDuty(ducyLimM2);

      /* only for debugging purposes */
      Serial.printf(">Current Angle m2: %f\n", angles[1]);
      Serial.printf(">ducy m2: %.2f \n", ducy_m2);
      Serial.printf("_newRef m2: %f \n", _newRef_m2);
      Serial.printf(">Limitated DuCy M2: %f \n", ducyLimM2);
      /* */
    }
    else
    {
      /* stop motor and read new reference */
      _m2->SetDuty(0);
      GetReference(angles);
      Serial.printf(">M2 deactivated - ducy m2:%f \n",0.0);
    }
    if ((abs(_newRef_m3 - angles[2]) > allowedError) && m3)// control loop for ellbow motor if angle difference is > allowed error
    {
      /* calculation of duty cycle by control */
      float ducy_m3 = ControlPiM3(_newRef_m3, _oldRef_m3, angles[2], _oldAngle_m3);
      _oldAngle_m3 = angles[2]; // update value of old angle

      /* limitation of duty cycle */
      float ducyLimM3 = SetDutyDeadZone(3, ducy_m3);
      _m3->SetDuty(ducyLimM3);

      /* only for debugging purposes */
      float refdif = _newRef_m3 - angles[2];
      Serial.printf(">currentAngle m3: %f \n", angles[2]);
      Serial.printf(">refDif m3: %f\n", refdif);
      Serial.printf(">ducy_m3: %.2f \n", ducy_m3);
      Serial.printf(">_newRef_m3: %f \n", _newRef_m3);
      Serial.printf(">Limitated DuCy M3: %f \n", ducyLimM3);
      /* */
    }
    else
    {
      /* stop motor and read actual angle -> is this really needed */
      _m3->SetDuty(0);
      GetReference(angles);
      /* only for debugging purposes */
      Serial.printf(">M3 deactivated - ducy m3: %f \n",0.0);
    }

    if (m1)
    {
      /* only for debugging purposes */
      float refdif = _newRef_m1 - angles[0];
      Serial.printf(">refDif m1: %.2f\n", refdif);
      /* */

      if (abs(_newRef_m1 - angles[0]) > allowedError) // control loop if angle difference is > allowed error
      {
        /* only for debugging purposes */
        float refdif = _newRef_m1 - angles[0];
        Serial.printf(">refDif m1: %.2f\n", refdif);

        /* calculation of duty cycle by control */
        float ducy_m1 = ControlPiM1(_newRef_m1, _oldRef_m1, angles[0], _oldAngle_m1);
        _oldAngle_m1 = angles[0]; // update value of old angle

        /* only for debugging purposes */
        Serial.printf(">Current Angle deg m1: %f\n", angles[0]);
        Serial.printf(">Previous Angle deg m1: %f\n", _oldAngle_m1);
        Serial.printf(">ducy_m1: %.2f \n", ducy_m1);
        Serial.printf(">_newRef_m1: %f \n", _newRef_m1);

        float ducyLimM1 = SetDutyDeadZone(1, ducy_m1);
        _m1->SetDuty(ducyLimM1);

        Serial.printf(">Ducylim M1: %f \n", ducyLimM1);
      }
      else
      {
        /* stop motor and read actual angle */
        _m1->SetDuty(0);
        GetAngle(angles);
        GetReference(angles);
        _iOld = 0;
      }
    }

    /* set flag to false until timer/interrupt sets it to true again */
    _expired = false;
  }
}

/* function definitions */
void GetAngle(float *angles)
{
  // read encoder
  int posBase = (float)_enc->Read(0);
  int posShoulder = (float)_enc->Read(1);
  int posEllbow = (float)_enc->Read(2);

  float posAll[] = {(float)posBase, (float)posShoulder, (float)posEllbow};

  // Define the size of the posAll array
  const int size = sizeof(posAll) / sizeof(posAll[0]);

  if ((posBase != 0x80000000) || (posShoulder != 0x80000000) || (posEllbow != 0x80000000))
  {
    // Perform the multiplication and store the results in readAngle
    for (int i = 0; i < size; i++)
    {
      angles[i] = posAll[i] * 360.0f / 4096.0f;
    }
  }
}

float GetReference(float *angles)
{
  /* reading of new reference position */
  String refPosition;
  while (Serial.available() > 0)
  {
    /* Read the input position */
    refPosition = Serial.readString();
    refPosition.trim();
    String motorIndex = refPosition.substring(0, 2);
    float refAngle = refPosition.substring(3).toFloat();
    if ((motorIndex == "m3"))
    {
      _newRef_m3 = refAngle;
      Serial.printf("> new Ref m3: %f \n", _newRef_m3);
    }
    else if (motorIndex == "m2")
    {
      _newRef_m2 = refAngle;
      Serial.printf("> new Ref m2: %f \n", _newRef_m2);
    }
    else if (motorIndex == "m1")
    {
      _newRef_m1 = refAngle;
      Serial.printf("> new Ref m1: %f \n", _newRef_m1);
    }
  }

  return 0;
}

float ControlPiM2(float ref, float refOld, float angle, float angleOld)
{
  // defintion of constants of control
  float kP = 1.3;
  ;
  if ((ref - angle) > 0)
  {
    kP = 5 + (0.04 * (90 - angle)); // adaptive proportional gain - motor forearm
  }
  Serial.printf("kP: %f \n", kP);
  // calculate proportional part
  float propPart = kP * ((ref - angle) / 360);

  float controlOut = propPart;
  return controlOut;
}

float ControlPiM3(float ref, float refOld, float angle, float angleOld) // control for ellbow motor
{
  // defintion of proportional gain
  float kP = 1.8; // kP for downward movement

  if ((ref - angle) > 0) // upwards
  {
    kP = 3.0; // kP for upward movement
  }

  // calculate proportional part = control output
  float controlOut = kP * ((ref - angle) / 360);

  return controlOut;
}

float ControlPiM1(float ref, float refOld, float angle, float angleOld) // control for ellbow motor
{
  // defintion of proportional gain
  float kP = 0.8; // kP
  float kI = 0.0;
  int satPos = 1; 
  int satNeg = -1;

  float intPart;

  float dI = kI * _dt * (refOld - angleOld);
  float satmult = _sat * dI;
  Serial.printf(">satmult: %f \n", satmult);
  Serial.printf(">dI: %f \n", dI);
  if (_sat * dI > 0)
  {
    intPart = _iOld;
  }
  else
  {
    intPart = _iOld + dI;
  }
  // calculate proportional part 
  /* only for debugging / calibration purposes */
  float controlOut_i = (kP * ((ref - angle) / 360)) + intPart;
  Serial.printf(">Control Out I: %f \n", controlOut_i);
  Serial.printf(">Part I I: %f \n", intPart);
  /* */
  float controlOut = (kP * ((ref - angle) / 360)) + intPart;

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
    _oldRef_m1 = _newRef_m1; // update value of previous reference - tbc if it is the right place here??
    
    /* only for debugging purposes */
    Serial.printf(">Control Out m1: %f \n", controlOut);
    Serial.printf(">_oldRef_m1: %f \n", _oldRef_m1);
    Serial.printf(">_sat: %i \n", _sat);
  return controlOut;
}

float SetDutyDeadZone(int motor, float ducy)  /* limitation of duty cycle and implementation of dead zone for each motor */
{
  float minLim = 0.001; // minimum duty cycle where it will not be set to 0
  float maxLim = 0.4;   // maximum duty cycle which will be ever used
  float ducyLim = 0.0;    // limitated duty cycle
  float ducyMinUp;
  float ducyMinDown;

  switch (motor)
  {
  case 1: // base
    ducyMinUp = 0.12;
    ducyMinDown = -0.12;
    break;
  case 2: // shoulder
    ducyMinUp = 0.15;
    ducyMinDown = -0.11;
    break;
  case 3: // ellbow
    ducyMinUp = 0.15;
    ducyMinDown = -0.075;
    break;
  }

  if (abs(ducy) < minLim) 
  {
    ducyLim = 0.0;
  }
  else if (ducy < ducyMinUp && ducy > minLim)
  {
    ducyLim = ducyMinUp;
  }
  else if (ducy > ducyMinDown && ducy < -minLim)
  {
    ducyLim = ducyMinDown;
  }
  else if (ducy > maxLim)
  {
    ducyLim = maxLim;
  }
  else if (ducy < -maxLim)
  {
    ducyLim = -maxLim;
  }
  else
  {
    ducyLim = ducy;
  }

  return ducyLim;
}