#include "Arduino.h"

#define MOTOR_A 0
#define MOTOR_B 1

#define MOTOR_FORWARD 0
#define MOTOR_BACKWARD 1

class TB66MotorController {

 public:
    TB66MotorController(int pinAIN1, int pinAIN2, int pinPWMA, int pinBIN1, int pinBIN2, int pinPWMB, int STBY);

    void setup();
    void motorDrive(int motorNumber, int motorDirection, int motorSpeed);
    void motorBrake(int motorNumber);
    void motorStop(int motorNumber);
    void motorsStandby();

 private:
  int pinAIN1; //Direction
  int pinAIN2; //Direction
  int pinPWMA; //Speed

  //Motor 2
  int pinBIN1; //Direction
  int pinBIN2; //Direction
  int pinPWMB; //Speed

  //Standby
  int pinSTBY;
    
   
};
