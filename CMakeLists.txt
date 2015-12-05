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
    ${PROJECT_NAME}_manual_tf_alignment
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
add_library(${PROJECT_NAME}_manual_tf_alignment
  src/manual_tf_alignment.cpp
)
target_link_libraries(${PROJECT_NAME}_manual_tf_alignment
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

# Install

# Install libraries
install(TARGETS ${PROJECT_NAME}_manual_tf_alignment
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install executables
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
