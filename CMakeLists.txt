cmake_minimum_required(VERSION 2.8.3)
project(rtk_driver)

include_directories( "/usr/include/eigen3" )

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES rtk_driver
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/rtk_driver/include/rtk_driver
)

# Library definitions
add_library(NMEA0183 src/NMEA0183.cpp)
add_library(MXT906B_USB src/MXT906B_USB.cpp)

# Executables
add_executable(rtk_pub src/rtk_pub_node.cpp)
target_link_libraries(rtk_pub 
   NMEA0183 
   MXT906B_USB
   ${catkin_LIBRARIES} 
   ${EIGEN3_LIBRARIES})

add_executable(rtk_sub src/rtk_sub_node.cpp)
target_link_libraries(rtk_sub ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
