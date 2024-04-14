// definition of constants
#define GRIPPER_PIN 12              //definition of pin used to connect servomotor of gripper to ESP

#define GRIPPER_OPEN 0              //definition of servoangle when gripper completely open
#define GRIPPER_CLOSE_POS 80        //definition of servoangle when gripper closed to position to grip game piece
#define GRIPPER_CLOSE_COMP 180      //definition of servoangle when gripper completely closed 

// prototype functions
void InitGripper();                       // initialisation of the servomotor 

void OpenGripper();                       // function opening the gripper 
void CloseGripper(int gripperPosition);   // function closing the gripper 
int GetGripperState();                    // function to read out state of the gripper

