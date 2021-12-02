/*
 * Designed the robotic arm to receive commands through a queue,
 * There're 4 types of commands:
 *           --> "d******\0"   : if the command starts with a 'd' char, the robotic
 *           |                   arm is to go through the sequence of points with 
 *           |                   id_ == "******\0" (* : char)
 *           --> "n******\0" : if the command starts with a 'n' char, the robotic 
 *           |                 arm is to go to the point X = ***(1st 3*), Z = *** (2nd 3*)
 *           --> "g******\0" : if the command starts with a 'g' char, the robotic 
 *           |                 is to actuate the gripper with Open = * (1st *), and 
 *           |                 percentage of opening/closing == ***% (2nd, 3rd, 4th *),
 *           |                 and degree of the gripper = **° (5th, 6th *)
 *           --> "h------\0" : if the command starts with a 'h' char, the robotic 
 *                             is to halt and enter "sleep mode" but before that it shall
 *                             take the default position 0000 (reserved for "closed")
 *                             (- : don't care)
 */

#include "arm.h"

/*
// Define any sequence of 12 points desired (refer to sequence.h)
SEQUENCE default_seq[NUM_SEQEUNCES] = {
                // First sequence (dedicated to sleep position)
                {
                 .id_ = "0000",
                 .len = 4, 
                 .sequence = {  // Points' sequence to target (centimeter)
                     {0, 0}, {3, 3}, {4, 4}, {4, 4}, {0, 0}, {0, 0},
                     {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}
                  }
                },

                // Second sequence
                {
                 .id_ = "0001", 
                 .len = 12,
                 .sequence = {
                     {0, 0}, {3, 3}, {4, 4}, {4, 2}, {4, 7}, {4, 4},
                     {1, 1}, {2, 2}, {8, 8}, {4, 11}, {4, 0}, {9, 12}
                  }
                },
}; // End of default_seq initialization
*/

CMD_SEQUENCE cmd_sequence[NUM_SEQEUNCES] = {
                // First sequence (dedicated to sleep position)
                {
                 .id_  = "000000",
                 .len  = 4, 
                 .cmd_ = {  // Commands' sequence to target 
                     "n000000", "n150150", "g000050", "n120320", "g110000", "g000500",
                     "n000000", "n000000", "g005000", "n120000", "n151500", "g005000",
                  }
                },

                // Second sequence
                {
                 .id_  = "000001",
                 .len  = 5, 
                 .cmd_ = {  // Commands' sequence to target 
                     "n777777", "n150150", "g000050", "n120320", "g110000", "g000500",
                     "n000000", "n000000", "g005000", "n120000", "n151500", "g005000",
                  }
                },
}; // End of cmd_seq initialization

// Globals
static QueueHandle_t cmds_q;          // Commands' queue to be used by the robotic arm
static QueueHandle_t log_q;           // Log error messages inside this queue
float *angles;                        // Arm angles to be calculated

// Interpolation objects
Interpolation interp_x;
Interpolation interp_z;

// Servo objects    // [PWM channel, range°]
ARMServo shoulder_servo(CHANNEL_0, _180_DEG_SERVO);
ARMServo elbow_servo(CHANNEL_1, _180_DEG_SERVO);
ARMServo wrist_servo(CHANNEL_2, _180_DEG_SERVO);
ARMServo gripper_servo(CHANNEL_3, _180_DEG_SERVO);

/*********************************************************************/
// Main task --> Robotic arm animation
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
  int seq_idx;                        // Keeps track of the current sequence
  int prev_seq_idx;
  uint8_t seq_step = 0;               // Keeps track of the current sequence step
  
  // FLAG(8 bits): {--, --, sleep state, --, in sequence, state [3 bits]} 
  uint8_t flag = PARSE_STATE;
  
  while(1){
    switch(flag & STATE_BITMASK){
      
      case PARSE_STATE:{
        
        // There's a command to execute
        if(xQueueReceive(cmds_q, (void*)&cmd, 0) == pdTRUE) flag = decode_action(flag, cmd[0]);
        break;
      } // END PARSE state
      
      case SEQUENCE_STATE:{
        
        // Not in sequence already ?
        if(!(flag & IN_SEQUENCE_BITMASK)){
          // determine which sequence to play
          int idx_t = strtol(cmd+1, NULL, 10);
          seq_idx = idx_t > NUM_SEQEUNCES ? 0 : idx_t;

          // Repeating same sequence? --> no action required
          if(seq_idx == prev_seq_idx){
            flag = (flag & ~STATE_BITMASK) | PARSE_STATE;
            break;
          }
          else{
            prev_seq_idx = seq_idx;
            flag |= IN_SEQUENCE_BITMASK;
          }
          if(seq_idx == 0) flag |= SLEEP_BITMASK;
        }
        
        if(seq_step == cmd_sequence[seq_idx].len){ // End of sequence
          flag &= ~IN_SEQUENCE_BITMASK;
          seq_step = 0;
          flag = (flag & ~STATE_BITMASK) | PARSE_STATE;
          report(SEQUENCE_DONE);
        }
        else{ // Still in sequence
          /*
          target_x = default_seq[seq_idx].sequence[seq_step].x;
          target_z = default_seq[seq_idx].sequence[seq_step].z;
          */
          // Get new command in sequence
          strcpy(cmd, cmd_sequence[seq_idx].cmd_[seq_step++]);
          flag = decode_action(flag, cmd[0]);
        }

        break;
      } // END SEQUENCE state
      
      case HALT_STATE:{

        if(flag & SLEEP_BITMASK) flag = (flag & ~STATE_BITMASK) | PARSE_STATE;
        else{ // Not in sleep position
          memcpy(cmd, "d0000\0", CMD_LEN); // Apply sleep sequence
          //   // Clear last 2 bits   // Set the right state 
          flag = (flag & ~STATE_BITMASK) | SEQUENCE_STATE;
          flag &= ~IN_SEQUENCE_BITMASK;
        } 
        break;
      } // END HALT state
    
      case SINGLE_STATE:{
        char str_x[COORDINATE_LEN]; // Used in the parsing of the (X, Z) from commands
        char str_z[COORDINATE_LEN];
        
        memcpy(str_x, cmd+1, (COORDINATE_LEN - 1) * sizeof(cmd[0])); // Parse 2nd and 3rd elements of cmd
        memcpy(str_z, cmd+COORDINATE_LEN, (COORDINATE_LEN - 1) * sizeof(cmd[0])); // Parse 4th and 5th elements of cmd
        str_x[(COORDINATE_LEN - 1)] = str_z[(COORDINATE_LEN - 1)] = '\0';
        target_x = strtol(str_x, NULL, 10);
        target_z = strtol(str_z, NULL, 10);

        // Interpolate and reach point
        flag = (flag & ~STATE_BITMASK) | ACTUATE_STATE;

        // To indicate that we moved from our previous position
        if(!(flag & IN_SEQUENCE_BITMASK)) prev_seq_idx = -1; 
        flag &= ~SLEEP_BITMASK;
        break;
      } // END SINGLE state
    
      case ACTUATE_STATE:{
        
        // Interpolate (update ramp step)
        float x = interp_x.go(target_x, INTERPOLATION_TIME); 
        float z = interp_z.go(target_z, INTERPOLATION_TIME); 
        
        angles = get_angles(x, z);
        actuate_servos(angles);

        // Reached point ?
        if(interp_x.finished() && interp_z.finished()){
          report(POINT_REACHED);
          if(flag & IN_SEQUENCE_BITMASK) flag = (flag & ~STATE_BITMASK) | SEQUENCE_STATE;
          else flag = (flag & ~STATE_BITMASK) | PARSE_STATE;
        }
        break;
      } // END ACTUATE state

      case GRIPPER_STATE:{

        char percentage[4];
        float grip_p = 100;
        memcpy(percentage, cmd+2, 3);
        percentage[3] = '\0';
        grip_p = strtol(percentage, NULL, 10);
        grip_p = grip_p > 100 ? 1 : grip_p / 100; // Cap the grip percentage to 100%
        
        // (Open/Close, Percentage)
        grip(cmd[1], grip_p); 
        
        if(flag & IN_SEQUENCE_BITMASK) flag = (flag & ~STATE_BITMASK) | SEQUENCE_STATE;
        else flag = (flag & ~STATE_BITMASK) | PARSE_STATE;
        
        break;
      } // END GRIP state
    } // END Switch arm states
  } // END SUPER LOOP
}

/*********************************************************************/
// Decoding functions
/*********************************************************************/

// Decode commands to figure the state
uint8_t decode_action(uint8_t flag, char c){
  switch(c){
    case 'd': // One of the default sequences shall be executed
      return (flag & ~STATE_BITMASK) | SEQUENCE_STATE;
    
    case 'n': // Go to specific point
      return (flag & ~STATE_BITMASK) | SINGLE_STATE;
      
    case 'g': // Actuate gripper
      return (flag & ~STATE_BITMASK) | GRIPPER_STATE;
    
    case 'h': // Halt command, used to put the robotic arm into "sleep"
      return (flag & ~STATE_BITMASK) | HALT_STATE;

    default:{
      report(UNKNOWN_LOG); 
      return flag;
    }
  }
}

/*********************************************************************/
// Communication functions
/*********************************************************************/

// Add command to the commands' queue (to be called by the main code)
bool assign_cmd(char command[CMD_LEN]){
  char temp[CMD_LEN];
  strcpy(temp, command); // This is a hack and shouldn't be done this way
  return xQueueSend(cmds_q, (void*)&temp, 0) == pdTRUE; // We successfully added the cmd
}

// Return error messages (to be called by the main code)
LOG_MSG get_log(){
  LOG_MSG log_msg;
  UBaseType_t n_msg = uxQueueMessagesWaiting(log_q);
  if(n_msg > 0){
    log_msg.have_log = xQueueReceive(log_q, (void*)&log_msg.log_buf, 0) == pdTRUE;
  }
  return log_msg;
}

// To be used by the arm task to log errors
bool report(uint8_t err){
  char err_buf[LOG_BUFF_LEN];
  switch(err){
    case INVALID_LOG: 
      strcpy(err_buf, "FAULTY COMMAND");
      break;
    case UNKNOWN_LOG: 
      strcpy(err_buf, "UNKNOWN COMMAND");
      break;
    case GRIPPING_DONE:
      strcpy(err_buf, "GRIPPING DONE");
      break;
    case SEQUENCE_DONE:
      strcpy(err_buf, "SEQUENCE DONE");
      break;
    case POINT_REACHED:
      strcpy(err_buf, "POINT REACHED");
      break; 
    default: break;
  }
  return xQueueSend(log_q, (void*)&err_buf, 0) == pdTRUE; // We successfully logged the error
}

/*********************************************************************/
// Servo functions
/*********************************************************************/

// Actuate gripper
void grip(char open_, float perc){
  switch(open_){
    case '0':{ // Close gripper
      gripper_servo.actuate(MAX_GRIP_ANG + (perc * MID_GRIP_ANG)); 
      report(GRIPPING_DONE); 
      break;
    }
    
    case '1':{ // Open gripper
      gripper_servo.actuate(MAX_GRIP_ANG - (perc * MID_GRIP_ANG));
      report(GRIPPING_DONE);
      break;
    }
    
    default:{  // Invalid input
      report(INVALID_LOG);
      break;
    }
  }
}

// Setup all servos
void setup_servos(){        
  // (Pin, Min_us, Max_us)
  shoulder_servo.attach_servo(SERVO_SHOULDER, 500, 2500);
  elbow_servo.attach_servo(SERVO_ELBOW);
  wrist_servo.attach_servo(SERVO_WRIST);
  gripper_servo.attach_servo(SERVO_GRIPPER);
}

// Apply actions to servos
void actuate_servos(float *ang){
  shoulder_servo.actuate(ang[0]);
  elbow_servo.actuate(ang[1]);
  wrist_servo.actuate(ang[2]);
}
