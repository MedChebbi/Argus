#ifndef __SERVO_ARM__
  #define __SERVO_ARM__

  #include <ESP32Servo.h> // https://github.com/madhephaestus/ESP32Servo

  // Servo pins
  #define SERVO_SHOULDER          16
  #define SERVO_ELBOW             17
  #define SERVO_WRIST             18
  #define SERVO_GRIPPER           19

  // PWM params (reference: https://randomnerdtutorials.com/esp32-pwm-arduino-ide/)
  #define PWM_FREQUENCY           50    // PWM signal frequency
  #define TIMER_1                 0     // PWM channels
  #define TIMER_2                 1     
  #define TIMER_3                 2     
  #define TIMER_4                 3

  // 
  #define MAX_GRIP_ANG            60    // The maximum angle the gripper could open

  // Custom servo class
  class ARMServo{
    private:
    
      int min_us = 1000;   // Min-Max ranges
      int max_us = 2000;
      uint8_t timer;
      uint8_t pin;
      
    public:

      Servo my_servo;

      // Func prototypes
      ARMServo(int min_, int max_, uint8_t timer_, uint8_t pin_);
      void set_defaults(int min_, int max_, uint8_t timer_, uint8_t pin_);
      void setup_servo();
      void actuate(int ang);
  };
  
#endif
