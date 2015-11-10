# TF Keyboard Calibration

Move /tf frames around using your keyboard!

<img align="right" src="https://raw.githubusercontent.com/davetcoleman/tf_keyboard_cal/jade-devel/resources/thing.png" />

## Usage:

To test, create a new ``/thing`` coordiante from the following demo:

    roslaunch tf_keyboard_cal tf_keyboard_world_to_thing.launch

Start Rviz and use the TF display to visualize its effect.

    roslaunch tf_keyboard_cal simple_display.launch

You can now use the keyboard shorcuts below to move the frame around. **NOTE:** Be sure to have the little black window focused on to recieve keyboard input.

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
