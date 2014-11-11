
Drivers and ROS support for the small TortugaBot

You will also need to run this into your rosbuild workspace (for example where this README is):

  svn co https://isr-uc-ros-pkg.googlecode.com/svn/stacks/serial_communication/trunk cereal_trunk


Don't forget to add this directory to your ROS_PACKAGE_PATH variable, like this, in your ~/.bashrc:

  export ROS_PACKAGE_PATH=${HOME}/ros/rosbuild_ws_tortuga:${ROS_PACKAGE_PATH}

We will also piggyback on the turtlebot software stack, so you should make a catkin_workspace and put the turtlebot stack there.


