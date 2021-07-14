#ifndef __LOG_ERR__    
  #define __LOG_ERR__
    
    // Error log queue
    #define LOG_QUEUE_LEN           6     // How many error messages to hold
    #define LOG_BUFF_LEN            24    // How long each error msg will be (max length)
    
    // LOGGING CODES
    #define INVALID_LOG             0
    #define UNKNOWN_LOG             1
    #define ACTION_DONE             2
    #define SEQUENCE_DONE           3

    struct LOG_MSG{
      bool have_log = false;
      char log_buf[LOG_BUFF_LEN];         // Error reporter buff
    };

#endif
