#ifndef __LOG_ERR__    
  #define __LOG_ERR__
    
    // Error log queue
    #define LOG_QUEUE_LEN           6     // How many error messages to hold
    #define LOG_BUFF_LEN            24    // How long each error msg will be (max length)
    
    // LOGGING CODES
    enum{INVALID_LOG, UNKNOWN_LOG, GRIPPING_DONE, SEQUENCE_DONE, POINT_REACHED};

    struct LOG_MSG{
      bool have_log = false;
      char log_buf[LOG_BUFF_LEN];         // Error reporter buff
    };

#endif
