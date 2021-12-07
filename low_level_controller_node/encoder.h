/*
 * Single file class for the normal encoder (non A-B) functionalities
 * ------------------------------------------------------------------
 * This encoder requires the direction from you.
 */

#ifndef __ENCODER_H__
  #define __ENCODER_H__

  #include "Arduino.h"
  #include "ARGUS_configs.h"

  /* ---------------------------------------------- */
  /* ----------------- CLASS BODY ----------------- */
  /* ---------------------------------------------- */
  
  class encoder{
    private:
      
      int8_t _direction; // Holds the direction we're turning in
      volatile long _counter; // Ticks' counter
      void (*ISR_callback)(void);
      
    public:
      
      void begin(uint8_t pin, void (*ISR_callback)(void), uint8_t trigger_on);
      void handle_interrupt(void);
      void set_direction(int8_t direction);
      long get_count();
      void reset_count();
  };

  /* --------------------------------------------------- */
  /* ----------------- CLASS FUNCTIONS ----------------- */
  /* --------------------------------------------------- */
  
  // Ref: https://forum.arduino.cc/t/attachinterrupts-callback-method-from-a-c-class/461957/6
  void encoder :: begin(uint8_t irq_pin, void (*ISR_callback)(void), uint8_t trigger_on){
    _direction = DIR_FORWARD;
    attachInterrupt(digitalPinToInterrupt(irq_pin), ISR_callback, trigger_on);
  }
  
  inline void encoder :: handle_interrupt(void){
    if(_direction == DIR_FORWARD)_counter++;
    else _counter--;
  }
  
  void encoder :: set_direction(int8_t direction){
    _direction = direction;
  }
  
  long encoder :: get_count(){
    return _counter;
  }
  
  void encoder :: reset_count(){
    _counter = 0;
  }

#endif // __ENCODER_H__
