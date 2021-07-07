/*
 * Designed the robotic arm to receive commands through a queue,
 * There're 4 types of commands:
 *           --> "d****\0" : if the command starts with a 'd' char, the robotic
 *           |               arm is to go through the sequence of points with 
 *           |               id_ == "***\0" (* : char)
 *           --> "n****\0" : if the command starts with a 'n' char, the robotic 
 *           |               arm is to go to the point X = **(1st 2*), Z = ** (2nd 2*)
 *           --> "g****\0" : if the command starts with a 'g' char, the robotic 
 *           |               is to actuate the gripper with Open = * (1st *), and 
 *           |               percentage of opening/closing == ***% (2nd, 3rd, 4th *)
 *           --> "h****\0" : if the command starts with a 'h' char, the robotic 
 *                           is to halt and enter "sleep mode" but before that it shall
 *                           take the default position 0000 (reserved for "closed")
 */

#include "arm.h"

// Define any sequence of points desired
SEQUENCE default_seq[NUM_SEQEUNCES] = {
                // First sequence
                {
                 .id_ = "0000", // First sequence id --> dedicated to sleep position
                 .sequence = { // Points' sequence to target
                     {0, 0}, {3, 3}, {4, 4}, {4, 4}, {4, 4}, {4, 4},
                     {1, 1}, {2, 2}, {8, 8}, {4, 4}, {4, 4}, {9, 9}
                  }
                },

                // Second sequence
                {
                 .id_ = "0001", 
                 .sequence = {
                     {0, 0}, {3, 3}, {4, 4}, {4, 4}, {4, 4}, {4, 4},
                     {1, 1}, {2, 2}, {8, 8}, {4, 4}, {4, 4}, {9, 9}
                  }
                },
}; // End of default_seq initialization

// Globals
static QueueHandle_t cmds_q;          // Commands' queue to be used by the robotic arm

// Arm controlling task; Servo controls in here
void arm(void *params){
  // Queue to hold commands for the robotic arm to execute
  cmds_q = xQueueCreate(CMD_QUEUE_LEN, CMD_LEN);
  
  setup_servos();
  
  // Locals
  char cmd[CMD_LEN];  // The command holder
  int target_x;       // The target point in the (X, Z) plane
  int target_z;
  char str_x[2];      // Used in the parsing of the (X, Z) from commands
  char str_z[2];
  
  // FLAG(8 bits): {busy, gripper, open/close gripper, sleep position, X, X, state [2 bits]} 
  uint8_t flag = 0x00;
  
  while(1){
    if(flag & BUSY_BITMASK){ // Busy doing some other commands --> don't parse anything
      // Figure what are we busy with
      switch(flag & STATE_BITMASK){
        case SEQUENCE_STATE:{
          
          // Determine sequence step & calculate servo pos
          /* CODE HERE */
          
          break;
        }
        case SINGLE_STATE:{
          if(GRIPPER_BITMASK & flag){
            
            // Decode gripper action (haven't decided yet)
            /* CODE HERE */
            
          }
          else{ // We have a target point to head to
            memcpy(str_x, cmd+1, 2); // Parse 2nd and 3rd elements of cmd
            memcpy(str_z, cmd+3, 2); // Parse 4th and 5th elements of cmd
            target_x = strtol(str_x, NULL, 10);
            target_z = strtol(str_z, NULL, 10);
            
            // Calculate servo position
            /* CODE HERE */
          
          }
          break;
        }
        case HALT_STATE:{
          if(!(flag & SLEEP_BITMASK)){ // Not in sleep position
            memcpy(cmd, "d0000", CMD_LEN); // Apply sleep sequence
            //   // Clear last 2 bits   // Set the right state 
            flag = (flag & ~STATE_BITMASK) | SEQUENCE_STATE;
          }
          else flag &= ~BUSY_BITMASK; // Reset busy flag (we're sleeping --> not busy)
          break;
        }
      }
      
      // Apply the deduced actions (calculated servo positions and gripper actuation)
      /* CODE HERE */
       
    }
    else{ // Not busy, could parse commands
      
      // There's a command to execute
      if(xQueueReceive(cmds_q, (void*)&cmd, 0) == pdTRUE){
        switch(cmd[0]){
          case 'd': // One of the default sequences shall be executed
            flag = (flag & ~STATE_BITMASK) | SEQUENCE_STATE;
          break;
          
          case 'n': // Go to specific point
          case 'g': // Actuate gripper
            flag = (flag & ~STATE_BITMASK) | SINGLE_STATE;
            break;
          
          case 'h': // Halt command, used to put the robotic arm into "sleep"
            flag = (flag & ~STATE_BITMASK) | HALT_STATE;
            break;
        }
        flag |= 1 << 7; // Set the busy flag
      }
    }
  }
}

// Add command to the commands' queue
bool assign_cmd(char command[CMD_LEN]){
  return xQueueSend(cmds_q, (void*)&command, 0) == pdTRUE; // We successfully added the cmd
}

// Attach pins, attach PWM channels, and PWM frequency
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