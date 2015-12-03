cmake_minimum_required(VERSION 2.8.3)
project(tf_keyboard_cal)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  keyboard
  roscpp
  tf2
  tf
  roslib
  cmake_modules
  rosparam_shortcuts
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    manual_tf_aligment
  CATKIN_DEPENDS 
    keyboard roscpp 
    tf2
    tf
    rosparam_shortcuts
  DEPENDS 
    Eigen
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN_INCLUDE_DIRS}
)

# Library
add_library(manual_tf_alignment
  src/manual_tf_alignment.cpp
)
target_link_libraries(manual_tf_alignment
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Executable
add_executable(tf_keyboard
  src/tf_keyboard.cpp
)
target_link_libraries(tf_keyboard
  manual_tf_alignment
  ${catkin_LIBRARIES}
)

