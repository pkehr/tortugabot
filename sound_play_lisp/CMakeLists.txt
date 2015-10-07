cmake_minimum_required(VERSION 2.8.3)
project(sound_play_lisp)

find_package(catkin REQUIRED COMPONENTS
  roslisp
  sound_play
  actionlib_lisp
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_lisp roslisp sound_play message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)