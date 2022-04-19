
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>

#include "TB66MotorController.h"

/*TEST*/
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

#define I2C_ADDRESS 0x3C
SSD1306AsciiWire oled;
// Define proper RST_PIN if required.
#define RST_PIN -1
/*END TEST*/

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_timer_t tick_timer;
rcl_timer_t left_tick_timer;

rcl_publisher_t right_publisher;
std_msgs__msg__Int32 right_tick_msg;

rcl_publisher_t left_publisher;
std_msgs__msg__Int32 left_tick_msg;



//Define the Pins for Motors
//Motor 1
int pinAIN1 = 11; //Direction
int pinAIN2 = 8; //Direction
int pinPWMA = 3; //Speed

//Motor 2
int pinBIN1 = 12; //Direction
int pinBIN2 = 13; //Direction
int pinPWMB = 5; //Speed

//Standby
int pinSTBY = 7;

#define WHEEL_DIST 0.35 //is this distance of wheel to center or distance between wheels?
const double TICKS_PER_METER = 2786; //3100;

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_RIGHT_A 28 //Yellow wire to motor
#define ENC_IN_RIGHT_B 29 //White wire to motor


#define ENC_IN_LEFT_A 30 //yellow
#define ENC_IN_LEFT_B 31 //white 


// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// Keep track of the number of wheel ticks
volatile int left_wheel_tick_count = 0;
volatile int right_wheel_tick_count = 0;

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;

// Y-intercept for the PWM-Linear Velocity relationship for the robot
const int b = 52;

// Proportional constant, which was measured by measuring the
// PWM-Linear Velocity relationship for the robot.
const int K_P = 256;

// Correction multiplier for drift. Chosen through experimentation.
const int DRIFT_MULTIPLIER = 120;


// Set maximum and minimum limits for the PWM values
const int PWM_MIN = 20;
const int PWM_MAX = 200;
const int PWM_INCREMENT = 1;


double targetVelLeftWheel = 0;
double targetVelRightWheel = 0;

// Set linear velocity and PWM variable values for each wheel
double velLeftWheel = 0;
double velRightWheel = 0;
double pwmLeftReq = 0;
double pwmRightReq = 0;

// Record the time that the last velocity command was received
double lastCmdVelReceived = 0;

// Time interval for measurements in milliseconds
long previousMillis = 0;
long currentMillis = 0;

long last_screen_update = 0;

TB66MotorController motorController(pinAIN1, pinAIN2, pinPWMA, pinBIN1, pinBIN2, pinPWMB, pinSTBY);


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void setup()
{

  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000L);

  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);

  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

#if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
#else // RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
#endif // RST_PIN >= 0

  oled.setFont(Adafruit5x7);

  oled.clear();
  oled.println(".....TR2.....");

  set_microros_transports();
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
            &subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "cmd_vel"));

  // create publisher right wheel
  RCCHECK(rclc_publisher_init_best_effort(
            &right_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "right_tick_publisher"));

  // create publisher left wheel
  RCCHECK(rclc_publisher_init_best_effort(
            &left_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "left_tick_publisher"));

  //  right timer,
  const unsigned int timer_timeout = 66;
  RCCHECK(rclc_timer_init_default(
            &tick_timer,
            &support,
            RCL_MS_TO_NS(timer_timeout),
            timer_callback));

  RCCHECK(rclc_timer_init_default(
            &left_tick_timer,
            &support,
            RCL_MS_TO_NS(timer_timeout),
            left_timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &tick_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &left_tick_timer));

  right_tick_msg.data = 0;
  left_tick_msg.data = 0;

  velLeftWheel = 0.;
  velRightWheel = 0.;
}

void error_loop() {
  while (1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(750);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&right_publisher, &right_tick_msg, NULL));
    right_tick_msg.data = right_wheel_tick_count;

  }
  calc_vel_right_wheel();
}

void left_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&left_publisher, &left_tick_msg, NULL));
    left_tick_msg.data = left_wheel_tick_count;

  }
  calc_vel_left_wheel();
}



void subscription_callback(const void *msgin) {

  const geometry_msgs__msg__Twist * cmdVel = (const geometry_msgs__msg__Twist *)msgin;

  lastCmdVelReceived = (millis() / 1000);

  float dx = cmdVel->linear.x;
  float dr = cmdVel->angular.z;

  targetVelRightWheel = (dx + dr * WHEEL_DIST / 2.0);
  targetVelLeftWheel = (dx - dr * WHEEL_DIST / 2.0);


  // Calculate the PWM value given the desired velocity
  //pwmLeftReq = K_P * left;
  //pwmRightReq = K_P * right;

  // Check if we need to turn
  //  if (cmdVel->angular.z != 0.0) {
  //
  //    // Turn left
  //    if (cmdVel->angular.z > 0.0) {
  //      pwmLeftReq = -PWM_TURN;
  //      pwmRightReq = PWM_TURN;
  //    }
  //    // Turn right
  //    else {
  //      pwmLeftReq = PWM_TURN;
  //      pwmRightReq = -PWM_TURN;
  //    }
  //  }
  // Go straight
  //  if (cmdVel->angular.z == 0.0) {
  //
  //    // Remove any differences in wheel velocities
  //    // to make sure the robot goes straight
  //    static double prevDiff = 0;
  //    static double prevPrevDiff = 0;
  //    double currDifference = velLeftWheel - velRightWheel;
  //    double avgDifference = (prevDiff + prevPrevDiff + currDifference) / 3;
  //    prevPrevDiff = prevDiff;
  //    prevDiff = currDifference;
  //
  //    // Correct PWM values of both wheels to make the vehicle go straight
  //    pwmLeftReq -= (int)(avgDifference * DRIFT_MULTIPLIER);
  //    pwmRightReq += (int)(avgDifference * DRIFT_MULTIPLIER);
  //  }

  // Handle low PWM values
  //  if (abs(pwmLeftReq) < PWM_MIN) {
  //    pwmLeftReq = 0;
  //  }
  //  if (abs(pwmRightReq) < PWM_MIN) {
  //    pwmRightReq = 0;
  //  }

}


// Increment the number of ticks
void right_wheel_tick() {

  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);

  if (val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }

  if (Direction_right) {

    if (right_wheel_tick_count == encoder_maximum) {
      right_wheel_tick_count = encoder_minimum;
    }
    else {
      right_wheel_tick_count++;
    }
  }
  else {
    if (right_wheel_tick_count == encoder_minimum) {
      right_wheel_tick_count = encoder_maximum;
    }
    else {
      right_wheel_tick_count--;
    }
  }

}

// Increment the number of ticks
void left_wheel_tick() {

  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);

  if (val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }

  if (Direction_left) {
    if (left_wheel_tick_count == encoder_maximum) {
      left_wheel_tick_count = encoder_minimum;
    }
    else {
      left_wheel_tick_count++;
    }
  }
  else {
    if (left_wheel_tick_count == encoder_minimum) {
      left_wheel_tick_count = encoder_maximum;
    }
    else {
      left_wheel_tick_count--;
    }
  }

}

void calc_vel_left_wheel() {

  // Previous timestamp
  static double prevTime = 0;

  // Variable gets created and initialized the first time a function is called.
  static int prevLeftCount = 0;

  int numOfTicks = left_wheel_tick_count - prevLeftCount;

  double timeDiff = millis() - prevTime;

  if (timeDiff != 0)
  {
    // Calculate wheel velocity in meters per second
    velLeftWheel = numOfTicks / TICKS_PER_METER / (timeDiff / 1000.);

    // Keep track of the previous tick count
    prevLeftCount = left_wheel_tick_count;

    // Update the timestamp
    prevTime = millis();
  }
  if (numOfTicks == 0)
  {
    velLeftWheel = 0.;
  }

}

// Calculate the right wheel linear velocity in m/s every time a
void calc_vel_right_wheel() {

  // Previous timestamp
  static double prevRightTime = 0;

  // Variable gets created and initialized the first time a function is called.
  static int prevRightCount = 0;

  // Manage rollover and rollunder when we get outside the 16-bit integer range
  int numOfTicks = right_wheel_tick_count - prevRightCount;

  double timeDiff = millis() - prevRightTime;
  if (timeDiff != 0)
  {
    // Calculate wheel velocity in meters per second
    velRightWheel = numOfTicks / TICKS_PER_METER / (timeDiff / 1000.);

    prevRightCount = right_wheel_tick_count;

    prevRightTime = millis();
  }

  if (numOfTicks == 0)
  {
    velRightWheel = 0.;
  }
}

bool valueWithinRange(double val, double value, double modifier) {

  double minVal = value - (value * modifier);
  double maxVal = value + (value * modifier);

  return val >= minVal && val <= maxVal;
}

void set_pwm_values() {

  bool leftBackward = false;
  bool rightBackward = false;

  //FIGURE OUT DIRECTION
  if (targetVelLeftWheel < 0) {
    leftBackward = true;
  }

  if (targetVelRightWheel < 0) {
    rightBackward = true;
  }

  bool goingStraight = targetVelRightWheel == targetVelLeftWheel && targetVelRightWheel != 0 && targetVelLeftWheel != 0;

  double modifier = 0.1;
  
  if (goingStraight) {
    double avgVeloc = (velLeftWheel + velRightWheel) / 2.;
    if (valueWithinRange(avgVeloc, targetVelLeftWheel, modifier) == false) {
      if (targetVelLeftWheel < avgVeloc) { //slow down
         pwmLeftReq -= PWM_INCREMENT;
         pwmRightReq -= PWM_INCREMENT;
      } else if (targetVelLeftWheel > avgVeloc) {
         pwmLeftReq += PWM_INCREMENT;
         pwmRightReq += PWM_INCREMENT;
      }
    }
  }
  else {
    
    //LEFT
    if (targetVelLeftWheel == 0) {
      pwmLeftReq = 0;
    } else if (valueWithinRange(velLeftWheel, targetVelLeftWheel, modifier)) {

      //    if (goingStraight && leftBackward == false) { //we only try to correct when going forward
      //      if (velLeftWheel < velRightWheel) {
      //        pwmLeftReq += PWM_INCREMENT;
      //      } else if (velLeftWheel > velRightWheel) {
      //        pwmLeftReq -= PWM_INCREMENT;
      //      }
      //    }

    } else if (targetVelLeftWheel < velLeftWheel) {
      pwmLeftReq -= PWM_INCREMENT;
    }
    else if (targetVelLeftWheel > velLeftWheel) {
      pwmLeftReq += PWM_INCREMENT;
    }

    //RIGHT
    if (targetVelRightWheel == 0) {
      pwmRightReq = 0;
    } else if (valueWithinRange(velRightWheel, targetVelRightWheel, modifier)) {

      //    if (goingStraight && rightBackward == false) {
      //      if (velRightWheel < velLeftWheel) {
      //        pwmRightReq += PWM_INCREMENT;
      //      } else if (velRightWheel > velLeftWheel) {
      //        pwmRightReq -= PWM_INCREMENT;
      //      }
      //    }

    } else if (targetVelRightWheel < velRightWheel) {
      pwmRightReq -= PWM_INCREMENT;
    }
    else if (targetVelRightWheel > velRightWheel) {
      pwmRightReq += PWM_INCREMENT;
    }
  }



  if (pwmRightReq > PWM_MAX) {
    pwmRightReq = PWM_MAX;
  }
  else if (pwmRightReq < -PWM_MAX) {
    pwmRightReq = -PWM_MAX;
  }

  if (pwmLeftReq > PWM_MAX) {
    pwmLeftReq = PWM_MAX;
  }
  else if (pwmLeftReq < -PWM_MAX) {
    pwmLeftReq = -PWM_MAX;
  }

  int pwmLeftOut = 0;
  int pwmRightOut = 0;

  
  //ASSIGN THE VALUES
  pwmLeftOut = abs(pwmLeftReq);
  pwmRightOut = abs(pwmRightReq);

  // Conditional operator to limit PWM output at the maximum
  pwmLeftOut = (pwmLeftOut > PWM_MAX) ? PWM_MAX : pwmLeftOut;
  pwmRightOut = (pwmRightOut > PWM_MAX) ? PWM_MAX : pwmRightOut;

  // PWM output cannot be less than 0
  pwmLeftOut = (pwmLeftOut < 0) ? 0 : pwmLeftOut;
  pwmRightOut = (pwmRightOut < 0) ? 0 : pwmRightOut;

  // Set the PWM value on the pins
  if (!rightBackward) {
    motorController.motorDrive(MOTOR_A, MOTOR_FORWARD, (pwmRightOut));
  } else {
    motorController.motorDrive(MOTOR_A, MOTOR_BACKWARD, (pwmRightOut));
  }


  if (!leftBackward) {
    motorController.motorDrive(MOTOR_B, MOTOR_FORWARD, (pwmLeftOut));
  } else {
    motorController.motorDrive(MOTOR_B, MOTOR_BACKWARD, (pwmLeftOut));
  }

}

void loop()
{
  //long now = millis();
  //  if (now - last_screen_update > 500)
  //  {
  //    oled.clear();
  //
  //    oled.print("L pwm: ");
  //    oled.print(pwmLeftReq);
  //    oled.print(" v: ");
  //    oled.println(velLeftWheel);
  //    oled.print(" t: ");
  //    oled.println(targetVelLeftWheel);
  //
  //    oled.print("R pwm :");
  //    oled.print(pwmRightReq);
  //    oled.print(" v: ");
  //    oled.println(velRightWheel);
  //    oled.print(" t: ");
  //    oled.println(targetVelRightWheel);
  //
  //    last_screen_update = now;
  //  }

  // Stop the car if there are no cmd_vel messages
  //  if((millis()/1000) - lastCmdVelReceived > 1) {
  //    pwmLeftReq = 0;
  //    pwmRightReq = 0;
  //  }

  set_pwm_values();

  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

}
