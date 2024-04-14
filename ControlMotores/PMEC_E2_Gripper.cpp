// includes
#include <ESP32Servo.h>
#include "P_MEC_E2_Gripper.h"

// definition of global variables
Servo _servoGripper;

// initialisation of the servomotor 
void InitGripper()
{
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
