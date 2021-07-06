#include <Ramp.h> // https://github.com/siteswapjuggler/RAMP

// Configs
#define DEBUG           true // If ture, serial monitor will display stuff

// max number of items the serial queue could hold
#define QUEUE_LEN       10
#define BUFF_LEN        255 // CLI buffer

// Only works in dual-core ESP32
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
      /*
      // Report core ID
      sprintf(buff, "Task Serial Printer, running on core %i", xPortGetCoreID());
      Serial.println(buff);
      */
    }
    /*
    // Let the task yield to the scheduler so that no watchdog would trigger
    vTaskDelay(1 / portTICK_PERIOD_MS);
    */
  }
}

void setup() {
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
  #endif // DEBUG

  // Delete setup and loop tasks
  vTaskDelete(NULL);
}

void loop() {
}