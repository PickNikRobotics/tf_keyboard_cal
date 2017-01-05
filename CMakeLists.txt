cmake_minimum_required(VERSION 2.8.3)
project(tf_keyboard_cal)

# C++ 11
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  keyboard
  roscpp
  tf2
  tf
  roslib
  roslint
  cmake_modules
  rosparam_shortcuts
  rviz_visual_tools
  tf_conversions
  interactive_markers
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_manual_tf_alignment
    ${PROJECT_NAME}_imarker_simple
  CATKIN_DEPENDS
    keyboard roscpp
    tf2
    tf
    rosparam_shortcuts
    interactive_markers
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
add_library(${PROJECT_NAME}_manual_tf_alignment
  src/manual_tf_alignment.cpp
)
target_link_libraries(${PROJECT_NAME}_manual_tf_alignment
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Library
add_library(${PROJECT_NAME}_imarker_simple
  src/imarker_simple.cpp
)
target_link_libraries(${PROJECT_NAME}_imarker_simple
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Executable
add_executable(${PROJECT_NAME}
  src/tf_keyboard.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_manual_tf_alignment
  ${catkin_LIBRARIES}
)

# Executable
add_executable(${PROJECT_NAME}_demo_tf_listener
  src/demo_tf_listener.cpp
)
# Rename C++ executable without namespace
set_target_properties(${PROJECT_NAME}_demo_tf_listener
  PROPERTIES OUTPUT_NAME demo_tf_listener PREFIX "")
# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_demo_tf_listener
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Test for correct C++ source code
roslint_cpp()

#############
## Install ##
#############

# Install libraries
install(TARGETS ${PROJECT_NAME}_manual_tf_alignment ${PROJECT_NAME}_imarker_simple
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install executables
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_demo_tf_listener
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install scripts
install(PROGRAMS
  scripts/tf_interactive_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
