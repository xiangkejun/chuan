cmake_minimum_required(VERSION 2.8.3)
project(xx_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Res.msg
   Flag.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   Flag_xx.srv

 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )


## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )


catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES xx_msgs
  CATKIN_DEPENDS message_generation roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)






