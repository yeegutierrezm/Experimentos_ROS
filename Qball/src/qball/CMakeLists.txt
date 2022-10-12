cmake_minimum_required(VERSION 3.0.2)
project(qball)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation 
  gazebo_msgs 
  roscpp 
  rospy  
  std_srvs 
  geometry_msgs 
  sensor_msgs 
  nav_msgs
  tf 
  tf2_ros
  std_msgs
  gazebo_ros
)
include_directories( include ${catkin_INCLUDE_DIRS})

find_package(gazebo REQUIRED)
include_directories( include ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
## add_message_files(
##   FILES
##   prop_speed.msg
##   edrone_cmd.msg
## )

 generate_messages(
   DEPENDENCIES
   std_msgs  
 )

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES 
	DEPENDS 
	    roscpp 
	    gazebo_ros
      std_msgs 
      geometry_msgs 
      tf
      message_runtime
      dynamic_reconfigure
)

###########
## Build ##
###########

add_library(set_speed_joint_plugin src/set_speed_joint_plugin.cpp)
add_dependencies(set_speed_joint_plugin qball_generate_messages_cpp)
target_link_libraries (set_speed_joint_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS
  set_speed_joint_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
