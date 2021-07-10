#ifndef __SEQUENCE__
  #define __SEQUENCE__

    #include "Arduino.h"
    
    // Default sequences creation
    #define MAX_SEQUENCE_LEN        12    // Max steps count in each sequence
    #define SEQ_ID_LEN              5     // Length of the sequence ID
    #define NUM_SEQEUNCES           2     // Number of default sequences

    // Default sequences container 
    struct pair{
      uint8_t x;     // Value in millimeter
      uint8_t z;  
    }; // Holds the (x, z) point to go to
    
    struct SEQUENCE{
      char id_ [SEQ_ID_LEN];
      uint8_t len;
      pair sequence[MAX_SEQUENCE_LEN];
    }; // Holds the sequence of points to target
    
#endif
