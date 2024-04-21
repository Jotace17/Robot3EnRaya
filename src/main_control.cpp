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

#define PWM_FREQ_HZ 200

// definition of global variables control
AlMar::Esp32::Driver_L298n *_m1;
float _dOld = 0;
float _iOld = 0;
int _sat = 0;

float _oldRef = 0;
float _oldAngle = 0;
float newRef;

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
  float ducy0;    // variable to store duty cycle

  float out;
  float refPos;
  if (_first_time) // equal to first_time != 0
  {
    // currentAngle = GetAngle();
    // currentAngle = 0;
    // Serial.printf("initial current angle set to: %.1f", currentAngle);
    newRef = 50;
    _first_time = 0;
  }

  float currentAngle = GetAngle();

  if (abs(newRef - currentAngle) > 3)
  {
    float refdif = newRef - currentAngle;
    Serial.printf("newRef-currentAngle = %.2f\n", refdif);
    out = ControlPid(newRef, _oldRef, currentAngle, _oldAngle);
    Serial.printf("Control Out: %f.1\n", out);
    // if (GetAngle()!=NULL)
    //{
    _oldAngle = currentAngle;
    // currentAngle = GetAngle();
    Serial.printf("deg: %f\t\t### \n", currentAngle);

    //float ducy_m1 = (abs(out) / 255);
    float ducy_m1 = out;
    Serial.printf("ducy: %.2f\n", ducy_m1);
    if (abs(ducy_m1) < 0.003  )  // motor 877-7165 is not moving with dutycycle lower than 0.11
    {
      _m1->SetDuty(0.0);
    }
    else if ((ducy_m1) < 0.11 && ducy_m1 > 0.0)
    {
      _m1->SetDuty(0.11);
    }
    else if ((ducy_m1) > -0.11 && ducy_m1 <-0.003)
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
    _m1->SetDuty(0);            // set ducy cycle 0 to stop motor, as end position had ben reached 
    Serial.printf("else\n");
    currentAngle = GetAngle();
    Serial.printf("else: angle read %.2f:\n", currentAngle);
    // delay(400);
    //newRef = GetReference();
    // newRef = currentAngle + 3;
  }
  float _oldRef = newRef;

  delay(1);
}

// put function definitions here:#
float GetAngle()
{
  // read encoder
  int pos = _enc->Read(0);
  float posF = (float)pos;
  // int pos2 = _enc->Read(1); // lee encoder 0 (M1)

  if (pos != 0x80000000)
  {
    // Serial.printf("MOTOR %i \t\t ### \t\t\t MOTOR %i\n", 0, 1);
    float readAngle = posF * 360 / 4096;
    // Serial.printf("deg: %f, read: 0x%08x \t\t### \n", readAngle, pos);
    return readAngle;
  }
  else
  {
    // Serial.printf("\t ### ERROR LECTURA: %lu\n", pos);
  }
  //delay(20);
}

float GetReference()
{
  float refPos;
  String refPosition;

  if (Serial.available() > 0)
  {
    // Read the input position
    Serial.printf("Put in desired position in degree: \n");
    refPosition = Serial.readString();
    Serial.printf("ref position: %s", refPosition);
  }
  delay(400);
  return refPos = refPosition.toFloat();
}

float ControlPid(float ref, float refOld, float angle, float angleOld)
{
  // defintion of constants of control
  // those values will be updated with the values from PID tuning with the Simulink Model which provided Juan
  float kP = 0.8; // proportional gain
  float kI = 0;   // integral gain
  float kD = 0; // differential gain

  // definition of timestep (depends on recurring task time in which control will run) & saturation limits
  // for now set to 10ms
  float timeStep = 0.001;

  // definition of factors for easier readability of the formulas, only needed if filter will be used
  float intPart;

  // calculate integral part
  /*float dI = kI * timeStep * (refOld - angleOld);
  // Antiwind-up
  if (_sat * dI > 0)
  {
    intPart = _iOld;
  }
  else
  {
    intPart = _iOld + dI;
  }*/

  // calculate proportional part
  float propPart = kP * ((ref - angle)/360);

  // calculate differential part
  float difPart = _dOld + kD * ((ref - refOld)/360) - kD * ((angle - angleOld)/360);

  // calculate control output
  //float controlOut = -(propPart + intPart + difPart);
  float controlOut = (propPart+difPart);
  // limitation of output signal
  /*if (controlOut > satPos)
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
  _dOld = difPart; // update difPart for next step*/
  /*if (abs(controlOut) < 1)
  {
    return 0;
  }
  else
  {
    return controlOut;
  }*/
  return controlOut;
}