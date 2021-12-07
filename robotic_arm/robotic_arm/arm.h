#ifndef __ROBOTIC_ARM__
  #define __ROBOTIC_ARM__

  #include "interpolation.h"
  #include "trigonometry.h"
  #include "servo_arm.h"
  #include "sequence.h"
  #include "log_err.h"
  #include "Arduino.h"

  // Commands queue
  #define CMD_QUEUE_LEN           24        // How many commands to hold
  #define CMD_LEN                 (1+3+3+1) // How long each commands will be (max length) 
                                            // cmd_char, first digit, second digit, '\0'

  // Bitmasks for the flags
  #define UNUSED_BITMASK_1        0x80   
  #define UNUSED_BITMASK_2        0x40   
  #define SLEEP_BITMASK           0x20      // whether in sleep position or not
  #define UNUSED_BITMASK_3        0x10   
  #define IN_SEQUENCE_BITMASK     0x08      // Not done with sequence/move yet
  #define STATE_BITMASK           0x07      // 3 last bits for the state of the machine

  #define COORDINATE_LEN          (1+3) // Each point is 3 digits long + '\0'
  #define DEFAULT_COMMAND         "d000000\0"
  
  // States of the machine
  enum{SEQUENCE_STATE, SINGLE_STATE, HALT_STATE, PARSE_STATE, ACTUATE_STATE, GRIPPER_STATE};
  
  // Func prototypes     
  void arm(void *params);                       // Animate the robotic arm
  bool assign_cmd(char command[CMD_LEN]);       // Assign commands to the robotic arm
  LOG_MSG get_log();                            // Returns the oldest error (queue)
  void setup_servos();
  
  static void actuate_servos(float *ang);
  static uint8_t decode_action(uint8_t flag, char c);  // Decode commands to figure the state
  static void assign_wrist_angle(const char * w_cmd);
  static void grip(char open_, float perc);            // Actuate the gripper
  static bool report(uint8_t err);                     // Adds errors to the queue
  
#endif
  
