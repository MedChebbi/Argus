/*
 * This is the software for the low level controller of the ARGUS open source 
 * robot project. 
 * --------------------------------------------------------------------------
 * It was made to run on an "Arduino Mega 2560" board.
 */

#include "imu.h"
#include "robot.h"
#include "ros_stuff.h"
#include "ARGUS_configs.h"
#include "rpm_controller.h"
#include "ultrasonic_dist.h"
#include "kinematics_controller.h"

#ifdef AB_ENCODER // AB encoder to be used

  #include "encoder_AB.h" 
  // Encoder objects for each wheel
  encoder_ab *right_wheel_encoder;
  encoder_ab *left_wheel_encoder;
  
#else // Normal encoder to be used

  #include "encoder.h"
  // Encoder objects for each wheel
  encoder *right_wheel_encoder;
  encoder *left_wheel_encoder;

#endif

// State Machine var used to keep track of which state we're executing
volatile uint8_t SM_state = UPDATE_STATE;

// Create an IMU instance
TwoWire i2c_two_wire = TwoWire();
IMU my_imu;

// Robot object that will hold the state of the robot (pose and twist)
robot *ARGUS;

// RPM controller objects for each wheel
RPM_controller *right_wheel_controller;
RPM_controller *left_wheel_controller;

// RPM data container for both wheels
RPM_RL wheels_rpm;

double target_vel_x;  // These are the commands received from the High level controller
double target_vel_z;

/* ------------------------------------------------------------------ */

void setup(){

  // Setup the ros variables
  ros_setup();
  
  #ifdef AB_ENCODER // AB encoder to be used
    // Hardware node objects
    right_wheel_encoder = new encoder_ab();
    right_wheel_encoder->begin(RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN, 
                               GET_ISR_ENC(right_wheel_encoder, handle_interrupt_A),
                               GET_ISR_ENC(right_wheel_encoder, handle_interrupt_B));
  
    left_wheel_encoder = new encoder_ab();
    left_wheel_encoder->begin(LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN, 
                              GET_ISR_ENC(left_wheel_encoder, handle_interrupt_A),
                              GET_ISR_ENC(left_wheel_encoder, handle_interrupt_B));
  #else // Use of normal (non AB) encoders
    // Hardware node objects
    right_wheel_encoder = new encoder();
    right_wheel_encoder->begin(RIGHT_ENCODER_PIN, GET_ISR_ENC(right_wheel_encoder, handle_interrupt), FALLING);
  
    left_wheel_encoder = new encoder();
    left_wheel_encoder->begin(LEFT_ENCODER_PIN, GET_ISR_ENC(left_wheel_encoder, handle_interrupt), FALLING);
  #endif
  
  right_wheel_controller = new RPM_controller(RIGHT_MOTOR_ENABLE, RIGHT_MOTOR_PIN_1, RIGHT_MOTOR_PIN_2);
  right_wheel_controller->begin();
  
  left_wheel_controller = new RPM_controller(LEFT_MOTOR_ENABLE, LEFT_MOTOR_PIN_1, LEFT_MOTOR_PIN_2);
  left_wheel_controller->begin();

  // Instanciate new robot object
  ARGUS = new robot();
  ARGUS->reset(); // You can either reset it to a given state (x, y, theta) or call reset without any params to set it at (0, 0, 0)

  // Setup the IMU instance
  i2c_init_mpu6050(&my_imu, &i2c_two_wire);
  i2c_setup_mpu6050(&my_imu, MPU6050_FULL);
}

void loop(){
  static unsigned long last_ARGUS_update, last_pose_publish;
  unsigned long time_now = micros();

  // Keep the node alive
  ros_spin();
    
  switch(SM_state){
    case UPDATE_STATE:{
      
      #ifndef AB_ENCODER // Use of non-AB encoder that needs to be told the direction
        #error "The direction choice isn't implemented yet! --> Add it first"
        uint8_t robot_direction = DIR_FORWARD; // DIR_BACKWARD
        // Set the counting direction for the encoder
        right_wheel_encoder->set_direction(robot_direction);
        left_wheel_encoder->set_direction(robot_direction);
      #endif
       
      // Get the desired RPM for each wheel from the received target velocities
      wheels_rpm = get_rpm_from_vel(target_vel_x, target_vel_z);

      // Assign the calculated RPM targets for each wheel
      left_wheel_controller->set_target_rpm(wheels_rpm.left_wheel_rpm);
      right_wheel_controller->set_target_rpm(wheels_rpm.right_wheel_rpm);

      REQUEST_EXECUTE_STATE(SM_state); // Go to execute state
      break;
    }

    case EXECUTE_STATE:{

      // Update the state of ARGUS each 100th of a second
      if((time_now - last_ARGUS_update) >= ARGUS_UPDTAE_PERIOD){
        // Get the tick counts for each wheel
        long right_count = right_wheel_encoder->get_count();
        right_wheel_encoder->reset_count();
        
        long left_count = left_wheel_encoder->get_count();
        left_wheel_encoder->reset_count();
  
        // Actuate the motors appropriately
        right_wheel_controller->control(right_count);
        left_wheel_controller->control(left_count);
        
        // Calculate the distance traveled by each wheel 
        float distance_traveled_by_right_wheel = GET_DIST_TRAVELED(right_count);
        float distance_traveled_by_left_wheel  = GET_DIST_TRAVELED(left_count);
        
        // Update the state of ARGUS
        ARGUS->update(time_now, distance_traveled_by_right_wheel, distance_traveled_by_left_wheel);
        last_ARGUS_update = time_now;
      }

      // Send the Pose2D data on a fixed rate
      if((time_now - last_pose_publish) >= ODOM_PUBLISH_PERIOD){
        
        #ifdef ADVERTISE_POSE_DATA
          // Get Pose2D data and publish it
          float x, y, theta;
          ARGUS->get_pose(x, y, theta);
          publish_pose(x, y, theta);
        #endif

        #ifdef ADVERTISE_IMU_DATA
          // Read IMU data and publish it
          i2c_read_acc_mpu6050(&my_imu);
          i2c_read_gyr_mpu6050(&my_imu);
          publish_imu(my_imu.acc_xyz, my_imu.gyr_xyz);
        #endif

        #ifdef ADVERTISE_RANGE_DATA
          // Publish the distance to the obstacle
          publish_distance(calculate_distance_cm());
        #endif
        
        last_pose_publish = time_now;
      }
      
      break;
    }

    default: break;
  } // END __ SM
} // END __ loop()
