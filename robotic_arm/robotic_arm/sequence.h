#ifndef __SEQUENCE__
  #define __SEQUENCE__

    #include "Arduino.h"
    
    // Default sequences creation
    #define MAX_SEQUENCE_LEN        12    // Max steps count in each sequence
    #define SEQ_ID_LEN              5     // Length of the sequence ID
    #define NUM_SEQEUNCES           2     // Number of default sequences
    #define SEQ_CMD_LEN             6 

    /*
    // Default sequences container 
    struct pair{
      uint8_t x;     // Value in centimeter
      uint8_t z;  
    }; // Holds the (x, z) point to go to

    // This holds a sequence of pure points to go to
    struct SEQUENCE{
      char id_ [SEQ_ID_LEN]; // For now, the id is the idx .. might make a hashing func later
      uint8_t len;
      pair sequence[MAX_SEQUENCE_LEN];
    }; // Holds the sequence of points to target
    */

    // This holds a sequence of commands, which would allow for gripping to happen in the middle of the sequence
    struct CMD_SEQUENCE{
      char id_[SEQ_ID_LEN];
      uint8_t len;
      char cmd_[MAX_SEQUENCE_LEN][SEQ_CMD_LEN];
    };
    
#endif
