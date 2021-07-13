#include "arm.h"

// Configs
#define DEBUG                   true // If ture, serial monitor will display stuff

// max number of items the serial queue could hold
#define QUEUE_LEN               10
#define BUFF_LEN                255   // CLI buffer

// Cores settings; Only works in dual-core ESP32
static const BaseType_t pro_cpu = 0;
static const BaseType_t app_cpu = 1;

// Globals
static QueueHandle_t msg_q; 

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

//
void ser_cmd_parser(void *params){
    
  // Testing
  char s[6];
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


void setup() {
  // Don't even include in code if we're not debugging
  #if DEBUG 
    // Configure Serial
    Serial.begin(115200);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // Create serial queue
    msg_q = xQueueCreate(QUEUE_LEN, BUFF_LEN);
    
    char str[BUFF_LEN] = "--- Robotic arm started ---";
    xQueueSend(msg_q, (void*)&str, 0);
    
    // 1st Task --> Serial printer
    xTaskCreatePinnedToCore(print_messages,  // Function to run
                            "Print msgs",    // Name of task
                            2048,            // Stack size (bytes in ESP32, words in FreeRTOS)
                            NULL,            // Params to pass to function
                            1,               // Task priority (0 to configMAX_PRIORITIES -1)
                            NULL,            // Task handle
                            tskNO_AFFINITY); // Run on any core available
  
    // 2nd Task --> Parser of commands coming from serial port 
    xTaskCreatePinnedToCore(ser_cmd_parser,             
                            "CMD",           
                            1024,            // Stack size (bytes in ESP32, words in FreeRTOS)
                            NULL,            // Params to pass to function
                            1,               // Task priority (0 to configMAX_PRIORITIES -1)
                            NULL,            // Task handle
                            pro_cpu);        // Pin to core 0
  #endif // DEBUG

  // 1st main Task --> Arm controller (arm.h[arm.cpp])
  xTaskCreatePinnedToCore(arm,             // Function to run
                          "ARM",           // Name of task
                          2048,           // Stack size (bytes in ESP32, words in FreeRTOS)
                          NULL,            // Params to pass to function
                          1,               // Task priority (0 to configMAX_PRIORITIES -1)
                          NULL,            // Task handle
                          app_cpu);        // Pin to core 1

  // Delete setup and loop tasks
  vTaskDelete(NULL);
}

void loop() {
  // Nothing  
}
