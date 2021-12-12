#ifndef __ULTRASONIC_DIST_H__
  #define __ULTRASONIC_DIST_H__

  #define SPEED_OF_SOUND_IN_AIR       0.034f // Speed of sound in Air in cm/s
  #define MAX_DISTANCE                200 // In centimeter 

  void ultrasonic_setup(){
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);
    pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  }

  uint16_t calculate_distance_cm(){
    
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG_PIN, HIGH); // With this pulse the ultraSonic wave is created
    delayMicroseconds(10); // These delays determines the wave length
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    
    uint16_t sound_travel_duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
    
    uint16_t distance_cm = sound_travel_duration * SPEED_OF_SOUND_IN_AIR / 2;
    
    // Cap the measured distance to MAX_DISTANCE
    return distance_cm > MAX_DISTANCE ? distance_cm : MAX_DISTANCE;
  }
#endif // __ULTRASONIC_DIST_H__
