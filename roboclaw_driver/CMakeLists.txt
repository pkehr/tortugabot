cmake_minimum_required(VERSION 2.8.3)
project(roboclaw_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cereal_port
  nav_msgs
  std_msgs
  geometry_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp cereal_port nav_msgs std_msgs geometry_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(roboclaw_driver
  src/roboclaw_driver.cpp
)

target_link_libraries(roboclaw_driver
  ${catkin_LIBRARIES}
)
