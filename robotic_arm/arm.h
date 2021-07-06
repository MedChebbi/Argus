#pragma once
  #include <Ramp.h> // https://github.com/siteswapjuggler/RAMP

  // Servo pins
  #define SERVO_SHOULDER          16
  #define SERVO_ELBOW             17
  #define SERVO_WRIST             18
  #define SERVO_GRIPPER           19
  
  // PWM params (reference: https://randomnerdtutorials.com/esp32-pwm-arduino-ide/)
  #define PWM_FREQUENCY           50    // PWM signal frequency
  #define PWM_RESOLUTION          8     // Signal resolution in bits (8 --> [0, 255])
  #define ELBOW_CHANNEL           1     // PWM channels
  #define SHOULDER_CHANNEL        0
  #define WRIST_CHANNEL           2     
  #define GRIPPER_CHANNEL         3
  
  // Func prototypes
  void setup_servos();
  void arm(void *params);
