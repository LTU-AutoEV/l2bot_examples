cmake_minimum_required(VERSION 1.8.3)
project(l2bot_examples)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs

  # For Camera Example
  cv_bridge
  image_transport

  # For dynamic reconfigure examples
  dynamic_reconfigure
)

# For dynamic_reconfigure
generate_dynamic_reconfigure_options(
    cfg/StopOnWhite.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES l2bot
#  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(forward src/forward.cpp)
add_executable(joy_nav src/joy_nav.cpp)
add_executable(joy_setup src/joy_setup.cpp)
add_executable(cam_pub src/cam_pub.cpp)
add_executable(hello_world_pub src/hello_world_pub.cpp)
add_executable(hello_world_sub src/hello_world_sub.cpp)
add_executable(cam_edge_detect src/cam_edge_detect.cpp)
add_executable(stop_on_white src/stop_on_white.cpp)

# deps for dynamic_reconfigure
add_dependencies(stop_on_white ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(forward ${catkin_LIBRARIES})
target_link_libraries(joy_nav ${catkin_LIBRARIES})
target_link_libraries(joy_setup ${catkin_LIBRARIES})
target_link_libraries(cam_pub ${catkin_LIBRARIES})
target_link_libraries(hello_world_pub ${catkin_LIBRARIES})
target_link_libraries(hello_world_sub ${catkin_LIBRARIES})
target_link_libraries(cam_edge_detect ${catkin_LIBRARIES})
target_link_libraries(stop_on_white ${catkin_LIBRARIES})

