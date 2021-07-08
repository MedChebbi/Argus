#ifndef __TRIGONOMETRY__
  #define __TRIGONOMETRY__

  #include "Arduino.h"

  #define NUM_ANGLES       3    // Shoulder, elbow, wrist
  #define SEGMENT_LEN      110  // Millimeter (Both have the same length)
  #define DEG_TO_MS        11   // Equivalence between degrees and milliseconds for servo (deg° ~ 11ms)

  // Functions' prototypes
  float* get_angles(float x, float z); // Returns a pointer to an array of angles
  int rad_to_ms(float ang);            // Converts angle in radians to millisecond equivalent for servo
#endif
