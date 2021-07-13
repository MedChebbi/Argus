#ifndef __ROBOTIC_ARM__
  #define __ROBOTIC_ARM__

    #include "interpolation.h"
    #include "trigonometry.h"
    #include "servo_arm.h"
    #include "sequence.h"
    #include "Arduino.h"
  
    // Commands queue
    #define CMD_QUEUE_LEN           24    // How many commands to hold
    #define CMD_LEN                 6     // How long each commands will be (max length)

    // Error log queue
    #define LOG_QUEUE_LEN           6     // How many error messages to hold
    #define LOG_BUFF_LEN            24    // How long each error msg will be (max length)

    // Bitmasks for the flags
    #define UNUSED_BITMASK_1        0x80   
    #define UNUSED_BITMASK_2        0x40   
    #define SLEEP_BITMASK           0x20  // whether in sleep position or not
    #define UNUSED_BITMASK_3        0x10   
    #define IN_SEQUENCE_BITMASK     0x08  // Not done with sequence/move yet
    #define STATE_BITMASK           0x07  // 3 last bits for the state of the machine
    
    // States of the machine
    #define SEQUENCE_STATE          0
    #define SINGLE_STATE            1
    #define HALT_STATE              2
    #define PARSE_STATE             3
    #define ACTUATE_STATE           4
    #define GRIPPER_STATE           5

    // LOGGING CODES
    #define INVALID_LOG             0
    #define UNKNOWN_LOG             1

    struct LOG_MSG{
      bool have_log = false;
      char log_buf[LOG_BUFF_LEN];         // Error reporter buff
    };
    
    // Func prototypes                    
    void setup_servos();                    
    uint8_t decode_action(uint8_t flag, char c);  // Decode commands to figure the state
    bool assign_cmd(char command[CMD_LEN]);       // Assign commands to the robotic arm
    void arm(void *params);                       // Animate the robotic arm
    void grip(char open_, float perc);            // Actuate the gripper
    void actuate_servos(float *ang);
    LOG_MSG get_log();                            // Returns the oldest error (queue)
    bool log_error(uint8_t err);                  // Adds errors to the queue

#endif
  
