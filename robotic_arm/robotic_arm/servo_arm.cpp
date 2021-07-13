#include "servo_arm.h"

// Constructor
ARMServo :: ARMServo(uint8_t channel_, uint8_t range_){
  set_defaults(channel_, range_);
}

// Assign private values
void ARMServo :: set_defaults(uint8_t channel_, uint8_t range_){
  channel = channel_;
  range   = range_;
  
  switch(range){
    case _60_DEG_SERVO:
      us_2_deg = U_SEC_PER_DEG_60;
      break;
    case _90_DEG_SERVO:
      us_2_deg = U_SEC_PER_DEG_90;
      break;
    case _120_DEG_SERVO:
      us_2_deg = U_SEC_PER_DEG_120;
      break;
    case _180_DEG_SERVO:
      us_2_deg = U_SEC_PER_DEG_180;
      break;
    default:
      us_2_deg = U_SEC_PER_DEG_180; 
    break;
  }
}

// Attach pin, PWM channel, and set PWM frequency
void ARMServo :: attach_servo(uint8_t pin_){
  attach_servo(pin_, DEFAULT_MIN_US, DEFAULT_MAX_US);
}

void ARMServo :: attach_servo(uint8_t pin_, int min_, int max_){
  pin     = pin_;
  min_us  = min_;  
  max_us  = max_;
  
  ledcSetup(channel, PWM_FREQUENCY, RESOLUTION_8);
  ledcAttachPin(pin, channel);
}

// Actuate the servo to certain angle
void ARMServo :: actuate(int ang){
  ledcWrite(channel, ang * us_2_deg);
}
