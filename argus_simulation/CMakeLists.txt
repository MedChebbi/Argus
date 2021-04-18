cmake_minimum_required(VERSION 3.0.2)
project(argus_simulation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  geometry_msgs
  sensor_msgs
  std_msgs
  tf
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
###################################
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs tf gazebo_ros
  DEPENDS gazebo_ros
)

###########
## Build ##
###########


#############
## Install ##
#############

install(DIRECTORY launch models rviz worlds 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
