#include "arm.h"

// Configs
#define DEBUG                   true  // If ture, serial monitor will display stuff

// max number of items the serial queue could hold
#define QUEUE_LEN               10
#define BUFF_LEN                255   // CLI buffer

// Cores settings; Only works in dual-core ESP32
static const BaseType_t pro_cpu = 0;
static const BaseType_t app_cpu = 1;

// Globals
static QueueHandle_t msg_q; 

// DEBUG Tasks
void print_messages(void *params);
void check_log(void *params);
void ser_cmd_parser(void *params);

/* ---------------------------------------------------------------------------------- */

void setup() {
  
  // 1st main Task --> Arm controller (arm.h[arm.cpp])
  xTaskCreatePinnedToCore(arm,             // Function to run
                          "ARM",           // Name of task
                          2048,            // Stack size (bytes in ESP32, words in FreeRTOS)
                          NULL,            // Params to pass to function
                          1,               // Task priority (0 to configMAX_PRIORITIES -1)
                          NULL,            // Task handle
                          app_cpu);        // Pin to core 1
                          
  /************************************************************************************** 
   * The logs printing task reads from a queue that would be created by the arm task    *
   * --> I should probably make a mutex that would not allow it to read until the queue *
   * is created, but for now, I just put the arm task creation at first and all is good *
   **************************************************************************************/
  #if DEBUG 
    // Configure Serial
    Serial.begin(115200);
    
    // Create serial queue
    msg_q = xQueueCreate(QUEUE_LEN, BUFF_LEN);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    char str[BUFF_LEN] = "--- Robotic arm started ---";
    xQueueSend(msg_q, (void*)&str, 0);
    
    // 1st Task (DEBUG) --> Serial printer
    xTaskCreatePinnedToCore(print_messages,  // Function to run
                            "Print msgs",    // Name of task
                            2048,            // Stack size (bytes in ESP32, words in FreeRTOS)
                            NULL,            // Params to pass to function
                            1,               // Task priority (0 to configMAX_PRIORITIES -1)
                            NULL,            // Task handle
                            tskNO_AFFINITY); // Run on any core available
  
    // 2nd Task (DEBUG) --> Parser of commands coming from serial port 
    xTaskCreatePinnedToCore(ser_cmd_parser,             
                            "CMD",           
                            1024,            // Stack size (bytes in ESP32, words in FreeRTOS)
                            NULL,            // Params to pass to function
                            1,               // Task priority (0 to configMAX_PRIORITIES -1)
                            NULL,            // Task handle
                            pro_cpu);        // Pin to core 0
                            
    // 3rd Task (DEBUG) --> Prints logs from the robotic arm
    xTaskCreatePinnedToCore(check_log,             
                            "LOG",           
                            1024,            
                            NULL,            
                            1,               
                            NULL,            
                            tskNO_AFFINITY); 
  #endif // DEBUG

  // Delete setup and loop tasks
  vTaskDelete(NULL);
}

void loop() {
  // Nothing  
}

/* ---------------------------------------------------------------------------------- */

// Serial printing task; No Serial call anywhere out of here 
void print_messages(void *params){
  char buff[BUFF_LEN];
  while(1){
    //             //The queue //item addrs //timeout in ticks
    if(xQueueReceive(msg_q, (void*)&buff, 0) == pdTRUE){
      Serial.println(buff);
    }
    // Yield to other tasks in order not to trigger the watchdog
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// This task checks for error logs reported by the robotic arm (& prints them)
void check_log(void *params){
  LOG_MSG msg;
  while(1){
    msg = get_log();
    if(msg.have_log){
      xQueueSend(msg_q, (void*)&msg.log_buf, 0);
    }
    // Yield to other tasks
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// This task parses commands from the serial monitor for testing/debugging purposes
void ser_cmd_parser(void *params){
  char s[CMD_LEN];
  char c;
  int idx = 0;
  while(1){
    if(Serial.available() > 0){
      c = Serial.read();
      if(c != '\n'){
        s[idx++] = c;
      }
      else{
        s[idx] = '\0';
        idx = 0;
        assign_cmd(s);
      }
    }
    // Yield to other tasks
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}
