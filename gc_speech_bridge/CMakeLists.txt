cmake_minimum_required(VERSION 2.8.3)
project(gc_speech_bridge)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sound_play)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sound_play
)

include_directories(${catkin_INCLUDE_DIRS})

###########
## Build ##
###########

## Declare a cpp executable
add_executable(text2speech src/speech_database.cpp)
target_link_libraries(text2speech ${catkin_LIBRARIES})

## Specify libraries to link a library or executable target against
target_link_libraries(text2speech   ${catkin_LIBRARIES})
target_link_libraries(text2speech curl)
target_link_libraries(text2speech jansson)

#############
## Install ##
#############

#install(PROGRAMS
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(TARGETS text2speech
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
