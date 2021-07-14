#ifndef __ROBOTIC_ARM__
  #define __ROBOTIC_ARM__

    #include "interpolation.h"
    #include "trigonometry.h"
    #include "servo_arm.h"
    #include "sequence.h"
    #include "log_err.h"
    #include "Arduino.h"
  
    // Commands queue
    #define CMD_QUEUE_LEN           24    // How many commands to hold
    #define CMD_LEN                 6     // How long each commands will be (max length)

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
    
    // Func prototypes     
    void actuate_servos(float *ang);
    void arm(void *params);                       // Animate the robotic arm
    bool assign_cmd(char command[CMD_LEN]);       // Assign commands to the robotic arm
    uint8_t decode_action(uint8_t flag, char c);  // Decode commands to figure the state
    LOG_MSG get_log();                            // Returns the oldest error (queue)
    void grip(char open_, float perc);            // Actuate the gripper
    bool log_error(uint8_t err);                  // Adds errors to the queue
    void setup_servos();                    
    
#endif
  
