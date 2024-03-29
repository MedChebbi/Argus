#ifndef __TRIGONOMETRY__
  #define __TRIGONOMETRY__

  #include "Arduino.h"

  #define NUM_ANGLES       3    // Shoulder, elbow, wrist
  #define SEGMENT_LEN      110  // Millimeter (Both have the same length)

  // Functions' prototypes
  float* get_angles(float x, float z); // Returns a pointer to an array of angles
  void set_parallel();
  void reset_parallel();
  static float safe_div(int a, int b);
  static int8_t sign(float x);

#endif
