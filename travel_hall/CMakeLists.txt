cmake_minimum_required(VERSION 2.8.3)
project(travel_hall)

find_package(catkin REQUIRED COMPONENTS
  move_base
  roboclaw_driver
  tortugabot_bringup

  actionlib_lisp
  actionlib_msgs
  move_base_msgs
  roslisp
  cl_tf
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   actionlib_msgs#   move_base_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES travel_hall
#  CATKIN_DEPENDS actionlib_lisp actionlib_msgs move_base_msgs roslisp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
