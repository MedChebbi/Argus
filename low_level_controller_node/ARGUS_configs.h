/*
 * This is meant to be a config file where all pins declarations 
 * and robot parameters are to be defined.
 * --------------------------------------------------------------
 * This file must contain any parameter that could be modified,
 * and then be included in all files needing such params.
 */
 
#ifndef __ARGUS_CONFIGS_H__
  #define __ARGUS_CONFIGS_H__

  #define AB_ENCODER // Comment this line if you'll work with the normal encoder

  #ifdef AB_ENCODER
    // AB encoder pins (interrupt pins on the Arduino Mega)
    #define RIGHT_ENCODER_A_PIN               2
    #define RIGHT_ENCODER_B_PIN               3
    #define LEFT_ENCODER_A_PIN                19
    #define LEFT_ENCODER_B_PIN                18
    
    #define TICKS_PER_ROTATION                25
    
  #else // Working with normal encoder
    
    // Encoders' pins --> Must be hardware interrupt enabled
    #define RIGHT_ENCODER_PIN                 2 // Interrupt pin 0 
    #define LEFT_ENCODER_PIN                  3 // Interrupt pin 1 
    
    #define TICKS_PER_ROTATION                25

  #endif // AB_ENCODER

  // IMU pins
  #define I2C_SDA_PIN                         20
  #define I2C_SCL_PIN                         21

  // State machine variables and macro for main controlling state
  #define UPDATE_STATE                        0
  #define EXECUTE_STATE                       1
  #define REQUEST_UPDATE_STATE(STATE)         ((STATE) = (UPDATE_STATE)) // Macro to signal that we need an update
  #define REQUEST_EXECUTE_STATE(STATE)        ((STATE) = (EXECUTE_STATE)) // Macro to signal that we need to execute

  // Ros serial config
  #define ROS_SERIAL_COMM_BAUDRATE            115200
  
  // Direction for the counter/controller to count/move appropriately
  #define DIR_FORWARD                         0
  #define DIR_BACKWARD                        1

  // Motors' pins and config parameters --> enable pins must be pwm enabled (~)
  #define RIGHT_MOTOR_ENABLE                  5
  #define RIGHT_MOTOR_PIN_1                   6
  #define RIGHT_MOTOR_PIN_2                   7
  #define LEFT_MOTOR_ENABLE                   8
  #define LEFT_MOTOR_PIN_1                    9
  #define LEFT_MOTOR_PIN_2                    10
  
  #define MAX_MOTOR_RPM                       300U
  #define PWM_RESOLUTION                      255U
  #define DEFUALT_MAX_RPM                     230.0f
  #define DEFUALT_MIN_RPM                     -230.0f
    
  // PID controller's parameters
  #define PID_SAMPLING_TIME                   95 // Sampling time in ms
  #define AGGRESSIVNESS_THRESHOLD             30U // when to apply aggressive correction against when to do conservative one
  #define AGGRESSIVE_KP                       ((double)4.0f) // PID controller params for when the adjustment must be big
  #define AGGRESSIVE_KI                       ((double)1.3f)
  #define AGGRESSIVE_KD                       ((double)0.4f)
  #define CONSERVATIVE_KP                     ((double)2.0f) // PID controller params for when the adjustment must be small
  #define CONSERVATIVE_KI                     ((double)0.9f)
  #define CONSERVATIVE_KD                     ((double)0.1f)

  // Vehicle's config
  #define WHEEL_RADIUS                        0.033f // in meters
  #define WHEEL_CIRCUMFERENCE                 ((PI) * 2 * (WHEEL_RADIUS))
  #define BASELINE_DISTANCE                   0.1f // wheel separation distance in meters

  // Kinematics parameters
  #define RPM_TO_RAD_PER_S                    (2.0f * (PI) / 60.0f)
  #define DIST_PER_RADIAN                     WHEEL_CIRCUMFERENCE
  #define GET_DIST_TRAVELED(COUNT)            (((float)(COUNT)/(float)(TICKS_PER_ROTATION))*WHEEL_CIRCUMFERENCE)

  // ARGUS state update time
  #define ARGUS_UPDTAE_PERIOD                 10000U // 10ms --> 100Hz
  #define ODOM_PUBLISH_PERIOD                 20000U // 20ms --> 50Hz

#endif // __ARGUS_CONFIGS_H__
