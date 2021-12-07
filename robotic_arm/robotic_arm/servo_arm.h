#ifndef __SERVO_ARM__
  #define __SERVO_ARM__

  /******************************************************************************** 
   * Decided to do things myself for better control over the PWM channels & Timers 
   * (didn't like how I need to allocate all timers to drive the servos)  
   ********************************************************************************/
  //#include <ESP32Servo.h> // https://github.com/madhephaestus/ESP32Servo
  
  #include "Arduino.h"

  // Servo params (PWM reference: https://randomnerdtutorials.com/esp32-pwm-arduino-ide/)
  #define SERVO_SHOULDER          16    // Servo pins
  #define SERVO_ELBOW             17
  #define SERVO_WRIST             18
  #define SERVO_GRIPPER           19
  
  #define PWM_FREQUENCY           50    // PWM signal frequency (20ms for servo --> 50Hz)
  
  #define CHANNEL_0               0     // PWM channels (max 16 channels on the ESP32)
  #define CHANNEL_1               1
  #define CHANNEL_2               2
  #define CHANNEL_3               3
  #define CHANNEL_4               4
  
  #define RESOLUTION_8            8     // PWM signal resolution (min:1, max:16)
  #define RESOLUTION_16           16
  
  #define _60_DEG_SERVO           60  // Types of servos (max range in degrees)
  #define _90_DEG_SERVO           90
  #define _120_DEG_SERVO          120
  #define _180_DEG_SERVO          180

  #define DEFAULT_MIN_US          1000  // Default min/max range in us
  #define DEFAULT_MAX_US          2000

  // us/deg (60° servo)
  #define U_SEC_PER_DEG_60        (((DEFAULT_MAX_US)-(DEFAULT_MIN_US))/(_60_DEG_SERVO))
  #define U_SEC_PER_DEG_90        (((DEFAULT_MAX_US)-(DEFAULT_MIN_US))/(_90_DEG_SERVO))
  #define U_SEC_PER_DEG_120       (((DEFAULT_MAX_US)-(DEFAULT_MIN_US))/(_120_DEG_SERVO))
  #define U_SEC_PER_DEG_180       (((DEFAULT_MAX_US)-(DEFAULT_MIN_US))/(_180_DEG_SERVO))
  
  // 
  #define MID_GRIP_ANG            40    // The angle at which the gripper is closed
  #define MAX_GRIP_ANG            20    // The max angle the gripper could open/close

  // Custom servo class
  class ARMServo{
    private:
    
      int min_us;   // Min-Max ranges
      int max_us;
      uint8_t channel;
      uint8_t pin;
      float deg_2_useconds;
      uint8_t range;
       
    public:
      
      // Func prototypes
      ARMServo(uint8_t channel_, uint8_t range_);
      void attach_servo(uint8_t pin_, int min_, int max_);
      void attach_servo(uint8_t pin_);
      void actuate(int ang);
  };
  
#endif
