/*
 * Single file class for the rpm controller class 
 * ---------------------------------------------------
 * 1- Instantiate the controller and begin it
 * 2- Set the direction and the target RPM (once set, they'll stay the same until changed)
 * 3- Run the control function while providing the ticks' count from the encoder
 */

#ifndef __RPM_CONTROLLER_H__
  #define __RPM_CONTROLLER_H__

  #include <PID_v1.h> // https://playground.arduino.cc/Code/PIDLibrary/
  
  #include "ARGUS_configs.h"

  /* ---------------------------------------------- */
  /* ----------------- CLASS BODY ----------------- */
  /* ---------------------------------------------- */

  class RPM_controller{
    private:

      // Motor pins/direction variables
      uint8_t _enable_pin;
      uint8_t _motor_pin_1;
      uint8_t _motor_pin_2;
      
      // RPM control variables
      double _target_rpm;
      double _pid_input_rpm;
      double _pid_output_rpm;
      unsigned long _last_update;
      PID _pid_motor_controller = PID(&_pid_input_rpm, &_pid_output_rpm, &_target_rpm, CONSERVATIVE_KP, CONSERVATIVE_KI, CONSERVATIVE_KD, DIRECT);
      
      // Private functions
      float _rpm_to_pwm();
      void _pid_correct();
      void _drive_motor(float pwm);
   
    public:

      RPM_controller(uint8_t enable_pin, uint8_t motor_pin_1, uint8_t motor_pin_2);

      // Starts the PID controller and initializes variables
      void begin();
      void begin(float min_rpm, float max_rpm);
      
      // Settings of the controller
      void set_target_rpm(float target_rpm);
      double get_rpm();
      
      // This function is meant to be repeatedly called to drive the motors
      void control(long ticks);
  };

  /* --------------------------------------------------- */
  /* ----------------- CLASS FUNCTIONS ----------------- */
  /* --------------------------------------------------- */

  // CONSTRUCTOR
  RPM_controller :: RPM_controller(uint8_t enable_pin, uint8_t motor_pin_1, uint8_t motor_pin_2){
    _enable_pin = enable_pin;
    _motor_pin_1 = motor_pin_1;
    _motor_pin_2 = motor_pin_2;

    pinMode(enable_pin, OUTPUT);
    pinMode(motor_pin_1, OUTPUT);
    pinMode(motor_pin_2, OUTPUT);
  }

  // Starts the controller 
  void RPM_controller :: begin(){
    begin(DEFUALT_MIN_RPM, DEFUALT_MAX_RPM);
  }
  
  void RPM_controller :: begin(float min_rpm, float max_rpm){
    _target_rpm = _pid_input_rpm = _pid_output_rpm = 0;
    _last_update = micros();

    _pid_motor_controller.SetSampleTime(PID_SAMPLING_TIME);
    _pid_motor_controller.SetOutputLimits(min_rpm, max_rpm);
    _pid_motor_controller.SetMode(AUTOMATIC); // Turn on the PID controller
  }

  // Here's the function that will orchestrate all these functions to drive the motor correctly
  void RPM_controller :: control(long ticks){
    // 1- Calculate RPM based on ticks
    // HOW: ticks / ticks_per_rotation --> number of turns * duration of sampling [in seconds] --> RPS / 60 --> RPM
    _pid_input_rpm = (ticks / TICKS_PER_ROTATION) * ((micros() - _last_update) * 1e-6) / 60;

    // 2- Get the Corrected RPM value that we want to reach
    _pid_correct();
    
    // 3- Get the corresponding PWM value 
    // 4- Drive the motor
    _drive_motor(_rpm_to_pwm());

    // Update the update time for next sampling
    _last_update = micros();
  }

  // Get the value of rpm that must be set to reach the target rpm
  void RPM_controller :: _pid_correct(){
    // We're close to the target, use conservative tuning parameters
    if(abs(_target_rpm - _pid_input_rpm) <= AGGRESSIVNESS_THRESHOLD){
      _pid_motor_controller.SetTunings(CONSERVATIVE_KP, CONSERVATIVE_KI, CONSERVATIVE_KD);
    }
    else{ 
      _pid_motor_controller.SetTunings(AGGRESSIVE_KP, AGGRESSIVE_KI, AGGRESSIVE_KD);
    }

    // Get the corrected value
    _pid_motor_controller.Compute();
  }
  
  // Converts RPM to PWM
  float RPM_controller :: _rpm_to_pwm(){
    return (((float) _pid_output_rpm / (float) MAX_MOTOR_RPM) * PWM_RESOLUTION);
  }

  // This funtion controls the motors' speed and rotation direction
  void RPM_controller :: _drive_motor(float pwm){
    uint8_t pwm_drive, _direction;

    // Get the pwm to the appropriate range (0 -> 255)
    if(pwm < 0){
      _direction = DIR_BACKWARD;
      pwm_drive = constrain(pwm,-255, 0);
      pwm_drive = map(pwm_drive, 0, -255, 0, 255);
    }
    else{
      _direction = DIR_FORWARD;
      pwm_drive = constrain(pwm, 0, 255);
    }

    // Set motor's speed
    analogWrite(_enable_pin, pwm_drive);
    
    if(_direction == DIR_FORWARD){ // Forward
      digitalWrite(_motor_pin_1, HIGH);
      digitalWrite(_motor_pin_2, LOW);
    }
    else{
      digitalWrite(_motor_pin_1, LOW);
      digitalWrite(_motor_pin_2, HIGH);
    }
  }

  // Setting of the controller
  void RPM_controller :: set_target_rpm(float target_rpm){ _target_rpm = target_rpm; }

  // Returns the current target RPM 
  double RPM_controller :: get_rpm(){ return _target_rpm; }
  
#endif // __RPM_CONTROLLER_H__
