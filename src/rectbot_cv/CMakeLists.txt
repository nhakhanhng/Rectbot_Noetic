cmake_minimum_required(VERSION 3.1)

# CUDA configuration
set(CMAKE_CUDA_ARCHITECTURES 60 61 62 70 72 75 86 89 90)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)


project(rectbot_cv LANGUAGES CXX CUDA)

# Find ROS packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  vision_msgs
  message_generation
  image_geometry
  geometry_msgs 
  tf
  tf2
  tf2_ros
  message_generation
)

# Find CUDA
find_package(CUDA REQUIRED)
message(STATUS "CUDA Libs: \n${CUDA_LIBRARIES}\n")
get_filename_component(CUDA_LIB_DIR ${CUDA_LIBRARIES} DIRECTORY)
message(STATUS "CUDA Headers: \n${CUDA_INCLUDE_DIRS}\n")

# Find OpenCV
find_package(OpenCV REQUIRED)


#Eigen
find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 Libs: \n${EIGEN3_INCLUDE_DIRS}\n")

# TensorRT
set(TensorRT_INCLUDE_DIRS /usr/include /usr/include/aarch-linux-gnu)
set(TensorRT_LIBRARIES /usr/lib/aarch64-linux-gnu)
message(STATUS "TensorRT Libs: \n\n${TensorRT_LIBRARIES}\n")
message(STATUS "TensorRT Headers: \n${TensorRT_INCLUDE_DIRS}\n")


file(GLOB BYTE_TRACK_HEADERS components/ByteTrack/include/*.h)
file(GLOB BYTE_TRACK_SOURCES components/ByteTrack/src/*.cpp)

# Define the include directory for ByteTrack
set(BYTE_TRACK_INCLUDE_DIR components/ByteTrack/include)

add_message_files(
  FILES
  KeyPoint.msg
  PoseObject.msg
  PoseObjectArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  vision_msgs
  # rectbot_cv
)


# ROS catkin package configuration
catkin_package(
  INCLUDE_DIRS include ${BYTE_TRACK_INCLUDE_DIR}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport vision_msgs visualization_msgs image_geometry geometry_msgs tf tf2 tf2_ros message_runtime
)

# Include directories from both projects
include_directories(
  include
  ${BYTE_TRACK_INCLUDE_DIR}
  # ${BYTE_TRACK_HEADERS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${TensorRT_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Link directories
link_directories(
  ${CUDA_LIBRARIES}
  ${CUDA_LIB_DIR}
  ${TensorRT_LIBRARIES}
)



# Add the executable with all required source files
add_executable(rectbot_object_detection_node
  src/rectbot_object_detection_node.cpp
  src/yolov8.cpp
)

add_executable(rectbot_object_mapping_node
  src/rectbot_object_mapping_node.cpp
  ${BYTE_TRACK_SOURCES}
)



add_dependencies(rectbot_object_detection_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(rectbot_object_mapping_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(rectbot_cv ${catkin_LIBRARIES})

# Link all required libraries
target_link_libraries(rectbot_object_detection_node 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  nvinfer
  nvinfer_plugin
  cudart
)

target_link_libraries(rectbot_object_mapping_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  # nvinfer
  # nvinfer_plugin
  # cudart
  -lrealsense2
)

# Add definition for OpenCV version check
if (${OpenCV_VERSION} VERSION_GREATER_EQUAL 4.7.0)
    message(STATUS "Build with -DBATCHED_NMS")
    add_definitions(-DBATCHED_NMS)
endif ()

# Keep all the original install targets
#############
## Install ##
#############

# install nodes
install(TARGETS rectbot_object_detection_node rectbot_object_mapping_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# Removed src/yolov8.cpp as it is a source file and not an executable script
install(PROGRAMS
  # src/main.cpp
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY components/ByteTrack/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# install(DIRECTORY config/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
# )

# install(DIRECTORY models/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
# )

install(FILES
  package.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)