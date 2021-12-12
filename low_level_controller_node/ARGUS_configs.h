/*
 * This is meant to be a config file where all pins declarations 
 * and robot parameters are to be defined.
 * --------------------------------------------------------------
 * This file must contain any parameter that could be modified,
 * and then be included in all files needing such params.
 */
 
#ifndef __ARGUS_CONFIGS_H__
  #define __ARGUS_CONFIGS_H__

  /* ------------------------------------------------------------------------ */
  /* ------------------------ CONDITIONAL COMPILATION ----------------------- */
  /* ------------------------------------------------------------------------ */
  
  // Comment this line if you'll work with the normal encoder
  #define AB_ENCODER 
  
  // Advertised data config --> Comment any one that you don't want to advertise
//  #define ADVERTISE_POSE_DATA 
//  #define ADVERTISE_IMU_DATA
//  #define ADVERTISE_RANGE_DATA
  
  /* ------------------------------------------------------------------------ */
  /* --------------------------- PINS ASSIGNMENTS --------------------------- */
  /* ------------------------------------------------------------------------ */

  #ifdef AB_ENCODER
    // AB encoder pins (interrupt pins on the Arduino Mega)
    #define RIGHT_ENCODER_A_PIN               2
    #define RIGHT_ENCODER_B_PIN               3
    #define LEFT_ENCODER_A_PIN                19
    #define LEFT_ENCODER_B_PIN                18
  #else // Working with normal encoder
    // Encoders' pins --> Must be hardware interrupt enabled
    #define RIGHT_ENCODER_PIN                 2 // Interrupt pin 0 
    #define LEFT_ENCODER_PIN                  3 // Interrupt pin 1 
  #endif // AB_ENCODER

  // IMU pins
  #define I2C_SDA_PIN                         20
  #define I2C_SCL_PIN                         21

  // Motor pins (Enable pins must be PWM enabled)
  #define RIGHT_MOTOR_ENABLE                  4
  #define RIGHT_MOTOR_PIN_1                   5
  #define RIGHT_MOTOR_PIN_2                   6
  #define LEFT_MOTOR_PIN_1                    7
  #define LEFT_MOTOR_PIN_2                    8
  #define LEFT_MOTOR_ENABLE                   9
  
  // Throwing mechanism (actuated by a servo)
  #define THROWING_MECH_SERVO_PIN             10
  
  // Ultrasonic sensor pins
  #define ULTRASONIC_TRIG_PIN                 16
  #define ULTRASONIC_ECHO_PIN                 17

  /* ------------------------------------------------------------------------ */
  /* -------------------------- ODOMETRY PARAMETERS ------------------------- */
  /* ------------------------------------------------------------------------ */
  
  // Encoder counts per rotation
  #define TICKS_PER_ROTATION                  1000
  
  // Direction for the counter/controller to count/move appropriately
  #define DIR_FORWARD                         0
  #define DIR_BACKWARD                        1

  // Motors config parameters
  #define MAX_MOTOR_RPM                       300U
  #define PWM_RESOLUTION                      255U
  #define DEFUALT_MAX_RPM                     300.0f
  #define DEFUALT_MIN_RPM                     -300.0f
  
  // PID controller's parameters
  #define PID_SAMPLING_TIME                   95 // Sampling time in ms
  #define AGGRESSIVNESS_THRESHOLD             30U // when to apply aggressive correction against when to do conservative one
  #define AGGRESSIVE_KP                       ((double)1.0f) // PID controller params for when the adjustment must be big
  #define AGGRESSIVE_KI                       ((double)0.0f)
  #define AGGRESSIVE_KD                       ((double)0.0f)
  #define CONSERVATIVE_KP                     ((double)1.0f) // PID controller params for when the adjustment must be small
  #define CONSERVATIVE_KI                     ((double)0.0f)
  #define CONSERVATIVE_KD                     ((double)0.0f)

  // Vehicle's config
  #define WHEEL_RADIUS                        0.040f // in meters
  #define WHEEL_CIRCUMFERENCE                 ((PI) * 2 * (WHEEL_RADIUS))
  #define BASELINE_DISTANCE                   0.28f // wheel separation distance in meters

  // Kinematics parameters
  #define RPM_TO_RAD_PER_S                    (2.0f * (PI) / 60.0f)
  #define DIST_PER_RADIAN                     WHEEL_CIRCUMFERENCE
  #define GET_DIST_TRAVELED(COUNT)            (((float)(COUNT)/(float)(TICKS_PER_ROTATION))*WHEEL_CIRCUMFERENCE)

  /* ------------------------------------------------------------------------ */
  /* -------------------------- ARGUS MAIN CONFIGS -------------------------- */
  /* ------------------------------------------------------------------------ */
  
  // ARGUS state update time
  #define ARGUS_UPDTAE_PERIOD                 10000U // 10ms --> 100Hz
  #define ODOM_PUBLISH_PERIOD                 200000U // 200ms --> 5Hz
  
  // State machine variables and macro for main controlling state
  #define UPDATE_STATE                        0
  #define EXECUTE_STATE                       1
  #define REQUEST_UPDATE_STATE(STATE)         ((STATE) = (UPDATE_STATE)) // Macro to signal that we need an update
  #define REQUEST_EXECUTE_STATE(STATE)        ((STATE) = (EXECUTE_STATE)) // Macro to signal that we need to execute
  
  // Ros serial config
  #define ROS_SERIAL_COMM_BAUDRATE            115200

#endif // __ARGUS_CONFIGS_H__
