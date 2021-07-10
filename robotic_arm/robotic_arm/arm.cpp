/*
 * Designed the robotic arm to receive commands through a queue,
 * There're 4 types of commands:
 *           --> "d****\0" : if the command starts with a 'd' char, the robotic
 *           |               arm is to go through the sequence of points with 
 *           |               id_ == "****\0" (* : char)
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

// Define any sequence of 12 points desired (refer to sequence.h)
SEQUENCE default_seq[NUM_SEQEUNCES] = {
                // First sequence (dedicated to sleep position)
                {
                 .id_ = "0000", // First sequence id
                 .len = 12, 
                 .sequence = {  // Points' sequence to target (centimeter)
                     {0, 0}, {3, 3}, {4, 4}, {4, 4}, {4, 4}, {4, 4},
                     {1, 1}, {2, 2}, {8, 8}, {4, 4}, {4, 4}, {9, 9}
                  }
                },

                // Second sequence
                {
                 .id_ = "0001", 
                 .len = 12,
                 .sequence = {
                     {0, 0}, {3, 3}, {4, 4}, {4, 4}, {4, 4}, {4, 4},
                     {1, 1}, {2, 2}, {8, 8}, {4, 4}, {4, 4}, {9, 9}
                  }
                },
}; // End of default_seq initialization

// Globals
static QueueHandle_t cmds_q;          // Commands' queue to be used by the robotic arm
static QueueHandle_t log_q;           // Log error messages inside this queue
Interpolation interp_x;               // Interpolation objects
Interpolation interp_z;
float *angles;                        // Arm angles to be calculated
char err_buf[LOG_BUFF_LEN];           // Error reporter buff

// Servo objects    // [min_range, max_range, timer, pin]
ARMServo shoulder_servo(500, 2500, TIMER_1, SERVO_SHOULDER);
ARMServo elbow_servo(500, 2500, TIMER_2, SERVO_ELBOW);
ARMServo wrist_servo(500, 2500, TIMER_3, SERVO_WRIST);
ARMServo gripper_servo(500, 2500, TIMER_4, SERVO_GRIPPER);

/*********************************************************************/

// Arm controlling task; Servo controls in here
void arm(void *params){
  
  cmds_q = xQueueCreate(CMD_QUEUE_LEN, CMD_LEN);
  log_q  = xQueueCreate(LOG_QUEUE_LEN, LOG_BUFF_LEN);

  setup_servos();
  
  // Locals
  char cmd[CMD_LEN];                  // The command holder
  int target_x;                       // The target point in the (X, Z) plane (centimeter)
  int target_z;
  char str_x[2];                      // Used in the parsing of the (X, Z) from commands
  char str_z[2];
  int seq_idx;                        // Keeps track of the current sequence
  uint8_t seq_step = 0;               // Keeps track of the current sequence step
  
  // FLAG(8 bits): {busy, gripper, --, sleep state, new target, in sequence , state [2 bits]} 
  uint8_t flag = 0x00;
  
  while(1){
    if(flag & BUSY_BITMASK){ // Busy doing some other commands --> don't parse anything
      if(flag & NEW_TARGET_BITMASK){ // There's a new target --> decoding time
        
        switch(flag & STATE_BITMASK){
          case SEQUENCE_STATE:{

            // Not in sequence already ?
            if(!(flag & IN_SEQUENCE_BITMASK)){
              // determine which sequence to play
              int idx_t = strtol(cmd+1, NULL, 10);
              seq_idx = idx_t > NUM_SEQEUNCES ? 0 : idx_t;
              flag |= IN_SEQUENCE_BITMASK; 
            }
            
            if(seq_step == default_seq[seq_idx].len){ // End of sequence
              flag &= ~IN_SEQUENCE_BITMASK;
              flag &= ~BUSY_BITMASK;
            }
            else{ // Still in sequence
              target_x = default_seq[seq_idx].sequence[seq_step].x; // Refer to sequence.h
              target_z = default_seq[seq_idx].sequence[seq_step].z;  
              seq_step++;
            }

            flag &= ~NEW_TARGET_BITMASK;  // Reset new target flag
            break;
          }
          case SINGLE_STATE:{
            if(flag & GRIPPER_BITMASK){ // Decode gripper action and apply it
              char percentage[3];
              uint8_t grip_p = 100;
              memcpy(percentage, cmd+2, 3);
              grip_p = strtol(percentage, NULL, 10);
              
              // Cap the grip percentage to 100%
              grip_p = grip_p > 100 ? 1 : grip_p / 100;
              
              // Apply grip action
              grip(cmd[1], grip_p);
            }
            else{ // We have a target point to determine then head to
              memcpy(str_x, cmd+1, 2); // Parse 2nd and 3rd elements of cmd
              memcpy(str_z, cmd+3, 2); // Parse 4th and 5th elements of cmd
              target_x = strtol(str_x, NULL, 10);
              target_z = strtol(str_z, NULL, 10);
              flag &= ~NEW_TARGET_BITMASK; // Reset new target mask
            }
            break;
          }
          case HALT_STATE:{
            if(flag & SLEEP_BITMASK) flag &= ~BUSY_BITMASK; // Reset busy flag (we're sleeping already --> not busy)
            else{ // Not in sleep position
              memcpy(cmd, "d0000", CMD_LEN); // Apply sleep sequence
              //   // Clear last 2 bits   // Set the right state 
              flag = (flag & ~STATE_BITMASK) | SEQUENCE_STATE;
            } 
            break;
          }
        } // END Switch arm states
      } // END If new target
      
      else{ // Interpolating a value OR going to interpolate
        
        // Interpolate (update ramp step)
        float x = interp_x.go(target_x*10, INTERPOLATION_TIME); // target_ * 10 --> turn to millimeter
        float z = interp_z.go(target_z*10, INTERPOLATION_TIME); 

        // Calculate servo angles 
        angles = get_angles(x, z);
      }

      actuate_servos(angles);

      // Reached point ?
      if(interp_x.finished() && interp_z.finished()){
        
        // Am I in a sequence ?
        if(flag & IN_SEQUENCE_BITMASK){
          flag |= NEW_TARGET_BITMASK;  // Set new target flag (target == next sequence step)
        }
        
        // I'm not in a sequence AND I've reached the target --> no longer busy
        else{
          flag &= ~BUSY_BITMASK;
        }
      }
    } // END If busy
    
    else{ // Not busy --> could parse commands
      
      // There's a command to execute
      if(xQueueReceive(cmds_q, (void*)&cmd, 0) == pdTRUE){
        switch(cmd[0]){
          case 'd': // One of the default sequences shall be executed
            flag = (flag & ~STATE_BITMASK) | SEQUENCE_STATE;
          break;
          
          case 'n': // Go to specific point
            flag = (flag & ~STATE_BITMASK) | SINGLE_STATE;
            break;
            
          case 'g': // Actuate gripper
            flag = (flag & ~STATE_BITMASK) | SINGLE_STATE;
            flag |= GRIPPER_BITMASK; // Set gripper flag
            break;
          
          case 'h': // Halt command, used to put the robotic arm into "sleep"
            flag = (flag & ~STATE_BITMASK) | HALT_STATE;
            break;
        }
        flag |= 1 << 7;               // Set the busy flag
        flag |= NEW_TARGET_BITMASK;   // Set the new target falg
      }
    } // END Not busy
  }
}

/*********************************************************************/
// Communication functions
/*********************************************************************/

// Add command to the commands' queue (to be called by the main code)
bool assign_cmd(char command[CMD_LEN]){
  return xQueueSend(cmds_q, (void*)&command, 0) == pdTRUE; // We successfully added the cmd
}

// Return error messages (to be called by the main code)
char* get_error(){
  xQueueReceive(log_q, (void*)&err_buf, 0);
  return err_buf;
}

// To be used by the arm task to log errors
bool log_error(char *err){
  if(strlen(err) > LOG_BUFF_LEN){
    return false;
  }
  return xQueueSend(log_q, (void*)&err, 0) == pdTRUE; // We successfully logged the error
}

/*********************************************************************/
// Servo functions
/*********************************************************************/

// Actuate gripper
void grip(char open_, uint8_t perc){
  switch(open_){
    case '0':{ // Close gripper
      gripper_servo.actuate(perc * MAX_GRIP_ANG); // Gotta check the sign of action
      break;
    }
    
    case '1':{ // Open gripper
      gripper_servo.actuate(perc * -1 * MAX_GRIP_ANG); // Gotta check the sign of action
      break;
    }
    
    default:{  // Invalid input
      /* CODE HERE */
      // Maybe return error code ?
      break;
    }
  }
}

// Setup all servos
void setup_servos(){
  shoulder_servo.setup_servo();
  elbow_servo.setup_servo();
  wrist_servo.setup_servo();
  gripper_servo.setup_servo();
}

// Apply actions to servos
void actuate_servos(float *ang){
  shoulder_servo.actuate(ang[0]);
  elbow_servo.actuate(ang[1]);
  wrist_servo.actuate(ang[2]);
}
