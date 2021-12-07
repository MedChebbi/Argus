/*
 * Single file class for the AB encoder functionalities
 * ------------------------------------------------------------------
 * This encoder doesn't requires the direction since it's calculated automatically.
 */

#ifndef __ENCODER_AB_H__
  #define __ENCODER_AB_H__

  #include "Arduino.h"
  #include "ARGUS_configs.h"

  #define PIN_A_SET_BMSK              0x01 // 0000 0001
  #define PIN_B_SET_BMSK              0x02 // 0000 0010

  // Macros used to update the ticks counter appropriately
  #define CLOCKWISE(COUNTER)          ((COUNTER)++)
  #define ANTI_CLOCKWISE(COUNTER)     ((COUNTER)--)

  /* ---------------------------------------------- */
  /* ----------------- CLASS BODY ----------------- */
  /* ---------------------------------------------- */
  
  class encoder_ab{
    private:

      volatile uint8_t _pin_state;
      volatile long _counter; // Ticks' counter
      void (*ISR_callback_A)(void);
      void (*ISR_callback_B)(void);
      
    public:

      // Interrupt pin A, Interrupt pin B, ISR A, ISR B, Trigger event {Falling, Rising, Change}
      void begin(uint8_t irq_pin_A, uint8_t irq_pin_B, void (*ISR_callback_A)(void), void (*ISR_callback_B)(void), uint8_t trigger_on);
      void handle_interrupt_A(void);
      void handle_interrupt_B(void);
      long get_count();
      void reset_count();
  };

  /* --------------------------------------------------- */
  /* ----------------- CLASS FUNCTIONS ----------------- */
  /* --------------------------------------------------- */

  // Interrupt pin A, Interrupt pin B, ISR A, ISR B, Trigger event {Falling, Rising, Change}
  // Ref: https://forum.arduino.cc/t/attachinterrupts-callback-method-from-a-c-class/461957/6
  void encoder_ab :: begin(uint8_t irq_pin_A, uint8_t irq_pin_B, void (*ISR_callback_A)(void), void (*ISR_callback_B)(void), uint8_t trigger_on){
    attachInterrupt(digitalPinToInterrupt(irq_pin_A), ISR_callback_A, trigger_on);
    attachInterrupt(digitalPinToInterrupt(irq_pin_B), ISR_callback_B, trigger_on);
    _pin_state = _counter = 0;
  }

  // Function to handle the interrupt from the the A counter
  inline void encoder_ab :: handle_interrupt_A(void){
    if(_pin_state & PIN_A_SET_BMSK) return; // A flag is already set --> return immediately
    if(_pin_state & PIN_B_SET_BMSK) CLOCKWISE(_counter); // B is already set --> turning clockwise
    else ANTI_CLOCKWISE(_counter);

    _pin_state |= PIN_A_SET_BMSK;
  }

  // Function to handle the interrupt from the the B counter
  inline void encoder_ab :: handle_interrupt_B(void){
    if(_pin_state & PIN_B_SET_BMSK) return; // B flag is already set --> return immediately
    if(_pin_state & PIN_A_SET_BMSK) ANTI_CLOCKWISE(_counter); // A is already set --> turning anti-clockwise
    else CLOCKWISE(_counter);

    _pin_state |= PIN_B_SET_BMSK;
  }
  
  long encoder_ab :: get_count(){ return _counter; }
  
  void encoder_ab :: reset_count(){ _counter = 0; }

#endif // __ENCODER_AB_H__
