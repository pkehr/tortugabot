
Drivers and ROS support for the small TortugaBot

We will also piggyback on the turtlebot software stack, so you should put the turtlebot stack in your workspace.

The easiest way to install everything, including the code in this repository, is to use the rosinstall file `rosinstall.yaml`. So you just call the following in the `src` directory of your catkin workspace:

```bash
$ wstool merge https://raw.githubusercontent.com/code-iai/tortugabot/master/rosinstall.yaml
$ wstool update
$ cd .. & rosdep install --from-paths src --ignore-src
```

If you never used `wstool` in your workspace before, you need to initialize the workspace first:

```bash
$ wstool init
```

You will additionally need the following system dependencies:
```bash
$ sudo apt-get install ros-DISTRO-joy ros-DISTRO-openni2-camera ros-DISTRO-openni2-launch
```
