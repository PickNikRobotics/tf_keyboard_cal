# Manual TF Calibration Tools

Description: Move /tf frames around using your keyboard, an Rviz GUI, or interactive markers - a simple calibration-by-eye tool!

Features:

 - Integrated Rviz panel with multiple calibration approaches

TF Keyboard Calibration developed by Andy McEvoy, [Dave Coleman](http://dav.ee/), and [Sammy Pfeiffer](http://github.com/awesomebytes) at the University of Colorado Boulder, PAL Robotics, MDA US Systems, and PickNik LLC.

Status:

 * [![Build Status](https://travis-ci.org/davetcoleman/tf_manual_cal.svg)](https://travis-ci.org/davetcoleman/tf_manual_cal) Travis - Continuous Integration
 * [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__tf_manual_cal__ubuntu_xenial_amd64__binary)](http://build.ros.org/view/Kbin_uX64/job/Kbin_uX64__tf_manual_cal__ubuntu_xenial_amd64__binary/) ROS Buildfarm - AMD64 Xenial Debian Build
 * [![Build Status](http://build.ros.org/buildStatus/icon?job=Kdev__tf_manual_cal__ubuntu_xenial_amd64)](http://build.ros.org/view/Kdev/job/Kdev__tf_manual_cal__ubuntu_xenial_amd64/) ROS Buildfarm - AMD64 Xenial Devel Build

![](resources/keyboard_screenshot.png)

Screenshot of calibration using keyboard shortcuts

![](resources/interactive_marker_screenshot.png)

Screenshot of calibration using interactive markers and the mouse

[Video example](https://www.youtube.com/watch?v=C9BbFv-C9Zo) of interactive marker tf calibration.

## Install

### Ubuntu Debian

> Note: this package has not been released yet

    sudo apt-get install ros-kinetic-tf-manual-cal

### Build from Source

To build this package, ``git clone`` this repo into a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) and be sure to install necessary dependencies by running the following command in the root of your catkin workspace:

    rosdep install -y --from-paths src --ignore-src --rosdistro kinetic

## Code API

> Note: this package has not been released yet

See [Class Reference](http://docs.ros.org/kinetic/api/tf_manual_cal/html/)

## Usage of TF Manual Cal:

Start the demo:

    roslaunch tf_keyboard_cal rviz_demo.launch

You can now use the keyboard shorcuts below to move the frame around:

    Manual alignment of camera to world CS:
    =======================================
    MOVE: X  Y  Z  R  P  YAW
    ------------------------
    up    q  w  e  r  t  y
    down  a  s  d  f  g  h
    Fast: u
    Med:  i
    Slow: o
    Save: p

Create a launch file and configuration file similar to the demos in the package's ``config/`` and ``launch/`` folders.

## Demo TF Listener

If you want to get the Eigen or ROS message formatted pose from the interactive marker or keyboard calibration, see the template code:

    src/demo_tf_listener.cpp

## Contribute

Please send PRs for new helper functions, fixes, etc!
