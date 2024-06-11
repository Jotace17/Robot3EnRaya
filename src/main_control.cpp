#include <Arduino.h>
#include "../lib/AlMar_ESP32_Driver_L298n.h"
#include "../lib/AlMar_ESP32_Driver_L298n.cpp"
#include "../lib/ALMar_ESP32_EncoderATM203_Spi2.cpp"
#include <iostream>
#include <cmath>
#include <vector>
#include <ESP32Servo.h>

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

//definition of constants Servo
// definition of constants
#define GRIPPER_PIN 42              //definition of pin used to connect servomotor of gripper to ESP

#define GRIPPER_OPEN 175              //definition of servoangle when gripper completely open
#define GRIPPER_CLOSE_POS 50        //definition of servoangle when gripper closed to position to grip game piece
#define GRIPPER_CLOSE_COMP 0      //definition of servoangle when gripper completely closed 

// definition of constants for cinematics functions
#define L_ARM1 300
#define L_ARM2 210

#define X_A 0
#define Y_A 0

using namespace std;

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
float _newRef_s1;

bool m1 = 0;
bool m2 = 0;
bool m3 = 0;
bool s1 = 1;

Servo _servoGripper;

const float _dt = 0.01; // 10ms

// definition of global variables encoder
AlMar::Esp32::EncoderATM203_Spi2 *_enc;

// put function declarations here:
void GetAngle(float *angles);
float GetReference(float *angles);
float ControlPiM1(float, float, float, float); // adaptive P control for arm/shoulder
float ControlPiM2(float, float, float, float); // adaptive P control for arm/shoulder
float ControlPiM3(float, float);               // Pi control for forearm/ellbow
float SetDutyDeadZone(int motor, float ducy);
vector<float> KinetInver(const float posx, const float posy, const float posz);
void ConfServo();
void OpenGripper();                       // function opening the gripper 
void CloseGripper(int gripperPosition);   // function closing the gripper 
int GetGripperState();                    // function to read out state of the gripper

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  ConfServo();

  _enc = new AlMar::Esp32::EncoderATM203_Spi2(cs_pins, N_MOTORS, PIN_MOSI, PIN_MISO, PIN_SCLK);

  _m1 = new AlMar::Esp32::Driver_L298n(PIN_M1_EN, PIN_M1_IN1, PIN_M1_IN2, 200);
  _m1->begin();
  // M2 - shoulder
  _m2 = new AlMar::Esp32::Driver_L298n(PIN_M2_EN, PIN_M2_IN1, PIN_M2_IN2, 200);
  _m2->begin();
  // M3 - ellbow
  _m3 = new AlMar::Esp32::Driver_L298n(PIN_M3_EN, PIN_M3_IN1, PIN_M3_IN2, 200);
  _m3->begin();

  /* set initial reference values */
  _newRef_m1 = 172; // first reference for motor 2 - limits ca. 30° to 100°
  _newRef_m2 = 94;  // first reference for motor 2 - limits ca. 30° to 100°
  _newRef_m3 = 188; // first reference for motor 3 - limits ca. 145 - 190°

  /* timer initialization*/
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &timerInterrupt, true); // Attach the interrupt handling function
  timerAlarmWrite(timer, 1000000 * _dt, true);        // Interrupt every 0.01 second
  timerAlarmEnable(timer);                            // Enable the alarm
}

void loop()
{
  if (_expired) // only run when interrupt is calling function
  {
    unsigned long runTime;
    unsigned long startTime;
    unsigned long endTime;
    // definition of local variables
    float angles[3];          // array for read angle values
    float ducy_m2;            // variable to store duty cycle for motor 2 (shoulder)
    float ducy_m3;            // variable to store duty cycle for motor 3 (ellbow)
    float refPos;             // variable for reference angle of control
    float allowedError = 1.0; // variable to define allowed error of control
    float allowedErrorM1 = 1.0;
    vector<float> q;
    int gripperState = 0;
    // vector<float> Pos = {380,0,145};  // position 5 of board, 130mm for height of gripper
    // vector<float> Pos = {419.25,-39.25,145}; //position 9
    // vector<float> Pos = {340.75,39.25,145}; //1
    // vector<float> Pos = {380,-39.25,145}; //6
    vector<float> Pos = {419.25, 0, 145}; // 8
    // vector<float> Pos = {340.75,0,145}; //2
    startTime = millis();
    GetAngle(angles); // read all three angles from the encoders

    /* only for debugging purposes */
    Serial.printf(">A currentAngle m1: %.2f\n", (angles[0]));
    Serial.printf(">A currentAngle m2: %.2f\n", (angles[1]));
    Serial.printf(">A currentAngle m3: %.2f\n", (angles[2]));
    /* */

    /* call of inverse cinematics to calculate reference angles for caluclated point */
    q = KinetInver(Pos[0], Pos[1], Pos[2]);

    /* read servo state */
    gripperState = GetGripperState();

    /* only for debugging purposes */
    Serial.printf(">angulo m1: %.2f\n", (q[0]));
    Serial.printf(">angulo m2: %.2f\n", (q[1]));
    Serial.printf(">angulo m3: %.2f\n", (q[2]));
    Serial.printf(">ServoState: %i\n", gripperState);
    /* */
    //OpenGripper();
    if ((abs(_newRef_m2 - angles[1]) > allowedError) && m2) // control loop for shoulder motor if angle difference is > allowed error
    {
      /* calculation of duty cycle by control */
      float ducy_m2 = ControlPiM2(_newRef_m2, _oldRef_m2, angles[1], _oldAngle_m2);
      _oldAngle_m2 = angles[1]; // update value of old angle

      /* limitation of duty cycle */
      float ducyLimM2 = SetDutyDeadZone(2, ducy_m2);
      _m2->SetDuty(ducyLimM2);

      /* only for debugging purposes */
      // Serial.printf(">Current Angle m2: %f\n", angles[1]);
      // Serial.printf(">ducy m2: %.2f \n", ducy_m2);
      // Serial.printf("_newRef m2: %f \n", _newRef_m2);
      // Serial.printf(">Limitated DuCy M2: %f \n", ducyLimM2);
      /* */
    }
    else
    {
      /* stop motor and read new reference */
      _m2->SetDuty(0);
      GetReference(angles);
    }
    if ((abs(_newRef_m3 - angles[2]) > allowedError) && m3) // control loop for ellbow motor if angle difference is > allowed error
    {
      /* calculation of duty cycle by control */
      float ducy_m3 = ControlPiM3(_newRef_m3, angles[2]);
      _oldAngle_m3 = angles[2]; // update value of old angle

      /* limitation of duty cycle */
      float ducyLimM3 = SetDutyDeadZone(3, ducy_m3);
      _m3->SetDuty(ducyLimM3);

      /* only for debugging purposes */
      float refdif = _newRef_m3 - angles[2];
      // Serial.printf(">currentAngle m3: %f \n", angles[2]);
      // Serial.printf(">refDif m3: %f\n", refdif);
      // Serial.printf(">ducy_m3: %.2f \n", ducy_m3);
      // Serial.printf(">_newRef_m3: %f \n", _newRef_m3);
      // Serial.printf(">Limitated DuCy M3: %f \n", ducyLimM3);
      /* */
    }
    else
    {
      /* stop motor and read actual angle -> is this really needed */
      _m3->SetDuty(0);
      GetReference(angles);
      /* only for debugging purposes */
      // Serial.printf(">M3 deactivated - ducy m3: %f \n",0.0);
    }

    if (m1)
    {
      /* only for debugging purposes */
      float refdif = _newRef_m1 - angles[0];
      // Serial.printf(">refDif m1: %.2f\n", refdif);
      /* */

      if (abs(_newRef_m1 - angles[0]) > allowedErrorM1) // control loop if angle difference is > allowed error
      {
        /* only for debugging purposes */
        float refdif = _newRef_m1 - angles[0];
        // Serial.printf(">refDif m1: %.2f\n", refdif);

        /* calculation of duty cycle by control */
        float ducy_m1 = ControlPiM1(_newRef_m1, _oldRef_m1, angles[0], _oldAngle_m1);
        _oldAngle_m1 = angles[0]; // update value of old angle

        /* only for debugging purposes */
        // Serial.printf(">Current Angle deg m1: %f\n", angles[0]);
        // Serial.printf(">Previous Angle deg m1: %f\n", _oldAngle_m1);
        // Serial.printf(">ducy_m1: %.2f \n", ducy_m1);
        // Serial.printf(">_newRef_m1: %f \n", _newRef_m1);

        // float ducyLimM1 = SetDutyDeadZone(1, ducy_m1);  // limitation already in control
        _m1->SetDuty(ducy_m1);

        // Serial.printf(">Ducylim M1: %f \n", ducyLimM1);
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
    if (s1)
    {
      int servoPos = _servoGripper.read();
      Serial.printf(">ServoPos Angle: %i \n", servoPos);
      CloseGripper(_newRef_s1);
    }

    /* set flag to false until timer/interrupt sets it to true again */
    _expired = false;
    endTime = millis();
    runTime = endTime - startTime;
    Serial.printf(">Program Runtime: %lu \n", runTime);
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
    }
    else if (motorIndex == "m2")
    {
      _newRef_m2 = refAngle;
    }
    else if (motorIndex == "m1")
    {
      _newRef_m1 = refAngle;
    }
    else if (motorIndex == "s1")
    {
      _newRef_s1 = refAngle;
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

float ControlPiM3(float ref, float angle) // control for ellbow motor
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
  float kP = 0.018; // kP
  float kI = 0.15;
  float sat = 1.0;
  float satPos = 0.20;
  float satNeg = -0.20;

  float intPart;

  float dI = kI * _dt * (ref - angle);
  // float satmult = _sat * dI;
  // Serial.printf(">satmult: %f \n", satmult);
  // Serial.printf("> AntiWindUp: %f \n", (abs((refOld - angleOld) * kI)));
  if (abs((refOld - angleOld) * kI) > sat)
  {
    intPart = _iOld;
  }
  else
  {
    intPart = _iOld + dI;
  }
  // calculate proportional part
  /* only for debugging / calibration purposes */
  // float controlOut_i = (kP * ((ref - angle) / 360)) + intPart;
  // Serial.printf(">Control Out I: %f \n", controlOut_i);
  // Serial.printf(">Part I I: %f \n", intPart);
  /* */
  float controlOut = (kP * ((ref - angle))) + intPart;
  Serial.printf(">Control Out Raw: %f \n", controlOut);

  // limitation of output signal
  if (controlOut > satPos)
  {
    controlOut = satPos;
  }
  else if (controlOut < satNeg)
  {
    controlOut = satNeg;
  }
  /*else
  {*/
  _iOld = intPart; // update integral for next step
  //}
  _oldRef_m1 = _newRef_m1; // update value of previous reference - tbc if it is the right place here??

  /* only for debugging purposes */
  // Serial.printf(">Control Out m1: %f \n", controlOut);
  // Serial.printf(">Current I part: %f\n", intPart);
  // Serial.printf(">_oldRef_m1: %f \n", _oldRef_m1);
  // Serial.printf(">_sat: %i \n", _sat);
  return controlOut;
}

float SetDutyDeadZone(int motor, float ducy) /* limitation of duty cycle and implementation of dead zone for each motor */
{
  float minLim = 0.001; // minimum duty cycle where it will not be set to 0
  float maxLim = 0.4;   // maximum duty cycle which will be ever used
  float ducyLim = 0.0;  // limitated duty cycle
  float ducyMinUp;
  float ducyMinDown;

  switch (motor)
  {
  case 2: // shoulder
    ducyMinUp = 0.15;
    ducyMinDown = -0.11;
    break;
  case 3: // ellbow
    ducyMinUp = 0.15;
    ducyMinDown = -0.075;
    break;
  default:
    // do nothing
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

vector<float> KinetDir(const float q1, const float q2, const float q3)
{
  // definition of local variables
  vector<float> Pos;
  float r;
  float x, y, z;

  r = L_ARM1 * cos(q2) + L_ARM2 * cos(q3 - q2);

  cout << "La distancia r es : " << r << endl;

  x = r * sin(q1);
  y = r * cos(q1);
  z = L_ARM1 * sin(q2) + L_ARM2 * sin(q2 - q3);

  Pos = {x, y, z};

  cout << "El valor de x es : " << x << endl;
  cout << "El valor de y es : " << y << endl;
  cout << "El valor de z es : " << z << endl;

  return Pos;
} // Fin de la funcion Kinet_Dir()

vector<float> KinetInver(const float posx, const float posy, const float posz)
{
  // definition of local variables
  vector<float> angleCin;
  float q0 = 0, q1 = 0, q2 = 0;

  // calculation of inverse cinematics
  float r = sqrt(pow(posx, 2) + pow(posy, 2));
  float lg = sqrt(pow(posz, 2) + pow(r, 2));
  float beta = acos((pow(L_ARM1, 2) + pow(L_ARM2, 2) - pow(r, 2) - pow(posz, 2)) / (2 * L_ARM1 * L_ARM2));

  float tau = acos(r / lg);
  float gamma = acos((pow(L_ARM2, 2) + pow(lg, 2) - pow(L_ARM1, 2)) / (2 * L_ARM2 * lg));
  float alpha = M_PI - gamma - beta;

  // calculation of angles [rad]
  q0 = atan2(posx, posy);
  q1 = tau + alpha;
  q2 = M_PI + beta;

  // conversion to degrees with value 0 in field 5 & introduction of transmision factor for ellbow and shoulder
  q0 = q0 * (180.0 / M_PI) - 90.0;
  q1 = 1.31 * (q1 * (180.0 / M_PI) - 50.89);
  q2 = 0.43 * (q2 * (180.0 / M_PI) - 284.4);

  // adding offset of encoders
  q0 = q0 + 163.0;
  q1 = q1 + 55.8;
  q2 = q2 + 155.6;

  // creation of return vector
  angleCin = {q0, q1, q2};

  return angleCin;
}

void ConfServo()
{
  ESP32PWM::allocateTimer(1);
  _servoGripper.setPeriodHertz(50);
  _servoGripper.attach(GRIPPER_PIN);
}
// function opening the gripper 
void OpenGripper()    
{
  _servoGripper.write(GRIPPER_OPEN);
}

// function closing the gripper 
void CloseGripper(int gripperPosition)
{
  _servoGripper.write(gripperPosition);
}

// function to read out state of the gripper
int GetGripperState()
{      
  //definition of local variables   
  int servoPos = 200;                               // servo angle: can take angles between 0 and 180 degrees, therefore 200 invalid
  int servoState = 0;                               // servo state: 0 - invalid, 1 - open, 2 - closed to position, 3 - closed completely

  servoPos = _servoGripper.read();                  // reading out the angle of the servomotor
  switch (servoPos + 1) {                           // clasification of the state based on the angle
    case GRIPPER_OPEN ... (GRIPPER_OPEN + 5):
      servoState = 1;
      break;
    case (GRIPPER_CLOSE_POS - 5) ... (GRIPPER_CLOSE_POS + 5):
      servoState = 2;
      break;
    case (GRIPPER_CLOSE_COMP - 10) ... GRIPPER_CLOSE_COMP:
      servoState = 3;
      break;
  }
  return servoState;
}