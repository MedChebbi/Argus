#include "servo_arm.h"

// Constructor
ARMServo :: ARMServo(int min_, int max_, uint8_t timer_, uint8_t pin_){
  set_defaults(min_, max_, timer_, pin_);
}

// Assign private values
void ARMServo :: set_defaults(int min_, int max_, uint8_t timer_, uint8_t pin_){
  min_us = min_;  
  max_us = max_;
  timer = timer_;
  pin = pin_;
}

// Attach pins, attach PWM channels, and set PWM frequency
void ARMServo :: setup_servo(){
  ESP32PWM::allocateTimer(timer);
  my_servo.setPeriodHertz(PWM_FREQUENCY);
  my_servo.attach(pin, min_us, max_us);
}

// Actuate the servo to certain angle
void ARMServo :: actuate(int ang){
  my_servo.write(ang);
}

// Release the ressources allocated for servo
void ARMServo :: release_servo(){
  my_servo.detach();
}
