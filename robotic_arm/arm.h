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

  // Commands queue
  #define CMD_QUEUE_LEN           24    // How many commands to hold
  #define CMD_LEN                 6     // How long each commands will be (max length)

  // Default sequences creation
  #define SEQUENCE_LEN            12    // Max steps count in each sequence
  #define SEQ_ID_LEN              5     // Length of the sequence ID
  #define NUM_SEQEUNCES           2     // Number of default sequences

  // Bitmasks for the flags
  #define BUSY_BITMASK            0x80  // MSB in the flag is for the busy flag
 
  struct pair{
    uint8_t x;     // Value in millimeter
    uint8_t z;  
  }; // Holds the (x, z) point to go to
  
  struct SEQUENCE{
    char id_ [SEQ_ID_LEN];
    pair sequence[SEQUENCE_LEN];
  }; // Holds the sequence of points to target
  
  // Func prototypes
  void setup_servos();                    // Setup servo pins, PWM channels, and frequency
  bool assign_cmd(char command[CMD_LEN]); // Assign commands to the robotic arm
  void arm(void *params);                 // Animate the robotic arm

  