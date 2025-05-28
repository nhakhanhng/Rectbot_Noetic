cmake_minimum_required(VERSION 3.0.2)
project(rectbot_bt)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  behaviortree_cpp_v3
  roscpp
  rospy
  nav_msgs
  move_base_msgs
  actionlib
  geometry_msgs
)

# find_package(behaviortree_cpp_v3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES rectbot_bt
 CATKIN_DEPENDS roscpp rospy nav_msgs geometry_msgs behaviortree_cpp_v3 visualization_msgs move_base_msgs actionlib
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # ${behaviortree_cpp_v3_INCLUDE_DIRS}
)

# add_executable(PositionProcess_node src/PositionProcess_node.cpp )
# add_executable(SoundDirectProcess_node src/SoundDirectProcess_node.cpp )
add_executable(BT_node src/BT_node.cpp 
  src/PositionProcess_node.cpp
  src/SoundDirectProcess_node.cpp
) 

# add_dependencies(SoundDirectProcess_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(PositionProcess_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(BT_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
# target_link_libraries(SoundDirectProcess_node
#   ${catkin_LIBRARIES}
#   # BT::behaviortree_cpp_v3
# )

# target_link_libraries(PositionProcess_node
#   ${catkin_LIBRARIES}
#   # BT::behaviortree_cpp_v3
# )

target_link_libraries(BT_node
  ${catkin_LIBRARIES}
  # BT::behaviortree_cpp_v3
)

## Install the executable
install(TARGETS BT_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install the include directory
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch bt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
