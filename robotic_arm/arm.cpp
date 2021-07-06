#include "arm.h"

void setup_servos(){
  // Setup PWM channel
  ledcSetup(SHOULDER_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(ELBOW_CHANNEL,    PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(WRIST_CHANNEL,    PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(GRIPPER_CHANNEL,  PWM_FREQUENCY, PWM_RESOLUTION);
  
  // Attach PWM channel to LED pin
  ledcAttachPin(SERVO_SHOULDER, SHOULDER_CHANNEL);
  ledcAttachPin(SERVO_ELBOW,    ELBOW_CHANNEL);
  ledcAttachPin(SERVO_WRIST,    WRIST_CHANNEL);
  ledcAttachPin(SERVO_GRIPPER,  GRIPPER_CHANNEL);
}

// Arm controlling task; Servo controls in here
void arm(void *params){
  setup_servos();
  
  while(1){
    ;
  }
}
