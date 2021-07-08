/* 
 * Inspiration: https://github.com/XRobots/BallContraption/blob/main/IK%20Arm/Arduino/001/001.ino
 */

#ifndef __INTERPOLATION__
  #define __INTERPOLATION__
  
  #include <Ramp.h> // https://github.com/siteswapjuggler/RAMP

  // Reach the target point in how many milliseconds
  #define INTERPOLATION_TIME      1000 

  // 
  class Interpolation{
    public:
      rampInt my_ramp;
      int previous = -1; // This would hold the previous value we interpolated to
      int go(int input, int duration);
  };

#endif
