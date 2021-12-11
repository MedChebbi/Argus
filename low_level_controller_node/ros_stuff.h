#ifndef __ROS_STUFF_H__
  #define __ROS_STUFF_H__

  // Variables used in the main file
  extern double target_vel_x;  // These are the commands received from the High level controller
  extern double target_vel_z;
  extern volatile uint8_t SM_state;
  
  #include "ARGUS_configs.h"
  
  #include <ros.h> // SOLVE THIS: https://github.com/ros-drivers/rosserial/pull/525/commits/517793a9a99e61372f3cbd9a1e10a95da2c61327#diff-a808a0be494e829000029760259d95ba4944bc078a94b8f20b84dc2d9443743c
  #include <geometry_msgs/Twist.h>
  #include <geometry_msgs/Pose2D.h>
  #include <sensor_msgs/Imu.h>
  #include <std_msgs/Byte.h>

  /* ------------------------------------------------------------------------ */
  /* ------------------------- FUNCTIONS DEFINITION ------------------------- */
  /* ------------------------------------------------------------------------ */
  
  void ros_setup();
  void ros_spin();

  // Publishing functions
  void publish_pose(float x, float y,float theta);
  void publish_imu(float *acc, float *gyr);
  void publish_distance(byte dist);
  
  // Callback function for the communication with the high level controller
  void velocity_callback(const geometry_msgs::Twist &vel);

  /* ------------------------------------------------------------------------ */
  /* ---------------------------- NODES CREATION ---------------------------- */
  /* ------------------------------------------------------------------------ */
  
  // Hardware node parameters and variables
  ros::NodeHandle nh;

  // Subscribe to the commanding topic
  ros::Subscriber<geometry_msgs::Twist> cmd_vel_subcriber("control/cmd_vel", velocity_callback);

  #ifdef ADVERTISE_POSE_DATA
    geometry_msgs::Pose2D pose_2d;
    ros::Publisher pose2d_publisher("geometry_msgs/Pose2D", &pose_2d);
  #endif

  #ifdef ADVERTISE_IMU_DATA
    sensor_msgs::Imu imu_data;
    ros::Publisher imu_publisher("sensor_msgs/Imu", &imu_data);
  #endif

  #ifdef ADVERTISE_RANGE_DATA
    std_msgs::Byte distance_data;
    ros::Publisher dist_publisher("std_msgs/Float32", &distance_data);
  #endif
  
  /* ------------------------------------------------------------------------ */
  /* ------------------------ FUNCTIONS DECLARATIONS ------------------------ */
  /* ------------------------------------------------------------------------ */

  /* --------------------------- SETUP FUNCTIONS ---------------------------- */
  // Setup all nodes 
  void ros_setup(){
    // ROS connection
    nh.getHardware()->setBaud(ROS_SERIAL_COMM_BAUDRATE); // Establish a serial connection
    nh.initNode();

    nh.subscribe(cmd_vel_subcriber); // Subscribe to the commanding topic

    #ifdef ADVERTISE_POSE_DATA
      nh.advertise(pose2d_publisher); // Advertise the Pose topic to publish pose data to  
    #endif

    #ifdef ADVERTISE_IMU_DATA
      nh.advertise(imu_publisher); 
    #endif

    #ifdef ADVERTISE_RANGE_DATA
      nh.advertise(dist_publisher); 
    #endif
  }

  // Function to keep the nodes alive
  void ros_spin(){ nh.spinOnce(); }

  /* -------------------------- SENDING FUNCTIONS --------------------------- */

  #ifdef ADVERTISE_POSE_DATA
    // Function to publish pose data to geometry_msgs/Pose2D topic
    void publish_pose(float x, float y,float theta){
      pose_2d.x = x, pose_2d.y = y, pose_2d.theta = theta;
      pose2d_publisher.publish(&pose_2d);
    }
  #endif

  #ifdef ADVERTISE_IMU_DATA
    // Function to publish IMU data to sensor_msgs/Imu topic
    void publish_imu(float *acc, float *gyr){
      imu_data.linear_acceleration.x = acc[0];
      imu_data.linear_acceleration.y = acc[1];
      imu_data.linear_acceleration.z = acc[2];
      imu_data.angular_velocity.x = gyr[0];
      imu_data.angular_velocity.y = gyr[1];
      imu_data.angular_velocity.z = gyr[2];
  
      imu_publisher.publish(&imu_data);
    }
  #endif

  #ifdef ADVERTISE_RANGE_DATA
    // Function to publish distance data
    void publish_distance(byte dist){
      distance_data.data = dist;
      dist_publisher.publish(&distance_data);
    }
  #endif
  
  /* ------------------------- RECEIVING FUNCTIONS -------------------------- */
  
  // Callback from the /cmd_vel topic
  void velocity_callback(const geometry_msgs::Twist &vel){
       target_vel_x = vel.linear.x;
       target_vel_z = vel.angular.z;
       REQUEST_UPDATE_STATE(SM_state);
  }
  
#endif // __ROS_STUFF_H__
