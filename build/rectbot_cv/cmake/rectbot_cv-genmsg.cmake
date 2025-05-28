# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rectbot_cv: 3 messages, 0 services")

set(MSG_I_FLAGS "-Irectbot_cv:/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Ivision_msgs:/opt/ros/melodic/share/vision_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rectbot_cv_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObjectArray.msg" NAME_WE)
add_custom_target(_rectbot_cv_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rectbot_cv" "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObjectArray.msg" "rectbot_cv/PoseObject:rectbot_cv/KeyPoint:sensor_msgs/Image:geometry_msgs/Pose2D:vision_msgs/ObjectHypothesisWithPose:std_msgs/Float32:geometry_msgs/Pose:geometry_msgs/PoseWithCovariance:vision_msgs/Detection2D:std_msgs/Header:geometry_msgs/Quaternion:vision_msgs/BoundingBox2D:geometry_msgs/Point"
)

get_filename_component(_filename "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/KeyPoint.msg" NAME_WE)
add_custom_target(_rectbot_cv_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rectbot_cv" "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/KeyPoint.msg" "std_msgs/Float32"
)

get_filename_component(_filename "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObject.msg" NAME_WE)
add_custom_target(_rectbot_cv_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rectbot_cv" "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObject.msg" "sensor_msgs/Image:rectbot_cv/KeyPoint:geometry_msgs/Pose2D:vision_msgs/ObjectHypothesisWithPose:std_msgs/Float32:geometry_msgs/Pose:geometry_msgs/PoseWithCovariance:vision_msgs/Detection2D:std_msgs/Header:geometry_msgs/Quaternion:vision_msgs/BoundingBox2D:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(rectbot_cv
  "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObjectArray.msg"
  "${MSG_I_FLAGS}"
  "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObject.msg;/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/KeyPoint.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/ObjectHypothesisWithPose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/Detection2D.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/BoundingBox2D.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rectbot_cv
)
_generate_msg_cpp(rectbot_cv
  "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/KeyPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rectbot_cv
)
_generate_msg_cpp(rectbot_cv
  "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObject.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/KeyPoint.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/ObjectHypothesisWithPose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/Detection2D.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/BoundingBox2D.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rectbot_cv
)

### Generating Services

### Generating Module File
_generate_module_cpp(rectbot_cv
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rectbot_cv
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rectbot_cv_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rectbot_cv_generate_messages rectbot_cv_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObjectArray.msg" NAME_WE)
add_dependencies(rectbot_cv_generate_messages_cpp _rectbot_cv_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/KeyPoint.msg" NAME_WE)
add_dependencies(rectbot_cv_generate_messages_cpp _rectbot_cv_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObject.msg" NAME_WE)
add_dependencies(rectbot_cv_generate_messages_cpp _rectbot_cv_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rectbot_cv_gencpp)
add_dependencies(rectbot_cv_gencpp rectbot_cv_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rectbot_cv_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(rectbot_cv
  "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObjectArray.msg"
  "${MSG_I_FLAGS}"
  "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObject.msg;/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/KeyPoint.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/ObjectHypothesisWithPose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/Detection2D.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/BoundingBox2D.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rectbot_cv
)
_generate_msg_eus(rectbot_cv
  "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/KeyPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rectbot_cv
)
_generate_msg_eus(rectbot_cv
  "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObject.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/KeyPoint.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/ObjectHypothesisWithPose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/Detection2D.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/BoundingBox2D.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rectbot_cv
)

### Generating Services

### Generating Module File
_generate_module_eus(rectbot_cv
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rectbot_cv
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(rectbot_cv_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(rectbot_cv_generate_messages rectbot_cv_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObjectArray.msg" NAME_WE)
add_dependencies(rectbot_cv_generate_messages_eus _rectbot_cv_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/KeyPoint.msg" NAME_WE)
add_dependencies(rectbot_cv_generate_messages_eus _rectbot_cv_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObject.msg" NAME_WE)
add_dependencies(rectbot_cv_generate_messages_eus _rectbot_cv_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rectbot_cv_geneus)
add_dependencies(rectbot_cv_geneus rectbot_cv_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rectbot_cv_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(rectbot_cv
  "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObjectArray.msg"
  "${MSG_I_FLAGS}"
  "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObject.msg;/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/KeyPoint.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/ObjectHypothesisWithPose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/Detection2D.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/BoundingBox2D.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rectbot_cv
)
_generate_msg_lisp(rectbot_cv
  "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/KeyPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rectbot_cv
)
_generate_msg_lisp(rectbot_cv
  "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObject.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/KeyPoint.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/ObjectHypothesisWithPose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/Detection2D.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/BoundingBox2D.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rectbot_cv
)

### Generating Services

### Generating Module File
_generate_module_lisp(rectbot_cv
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rectbot_cv
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rectbot_cv_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rectbot_cv_generate_messages rectbot_cv_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObjectArray.msg" NAME_WE)
add_dependencies(rectbot_cv_generate_messages_lisp _rectbot_cv_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/KeyPoint.msg" NAME_WE)
add_dependencies(rectbot_cv_generate_messages_lisp _rectbot_cv_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObject.msg" NAME_WE)
add_dependencies(rectbot_cv_generate_messages_lisp _rectbot_cv_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rectbot_cv_genlisp)
add_dependencies(rectbot_cv_genlisp rectbot_cv_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rectbot_cv_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(rectbot_cv
  "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObjectArray.msg"
  "${MSG_I_FLAGS}"
  "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObject.msg;/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/KeyPoint.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/ObjectHypothesisWithPose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/Detection2D.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/BoundingBox2D.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rectbot_cv
)
_generate_msg_nodejs(rectbot_cv
  "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/KeyPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rectbot_cv
)
_generate_msg_nodejs(rectbot_cv
  "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObject.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/KeyPoint.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/ObjectHypothesisWithPose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/Detection2D.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/BoundingBox2D.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rectbot_cv
)

### Generating Services

### Generating Module File
_generate_module_nodejs(rectbot_cv
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rectbot_cv
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(rectbot_cv_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(rectbot_cv_generate_messages rectbot_cv_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObjectArray.msg" NAME_WE)
add_dependencies(rectbot_cv_generate_messages_nodejs _rectbot_cv_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/KeyPoint.msg" NAME_WE)
add_dependencies(rectbot_cv_generate_messages_nodejs _rectbot_cv_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObject.msg" NAME_WE)
add_dependencies(rectbot_cv_generate_messages_nodejs _rectbot_cv_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rectbot_cv_gennodejs)
add_dependencies(rectbot_cv_gennodejs rectbot_cv_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rectbot_cv_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(rectbot_cv
  "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObjectArray.msg"
  "${MSG_I_FLAGS}"
  "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObject.msg;/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/KeyPoint.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/ObjectHypothesisWithPose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/Detection2D.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/BoundingBox2D.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rectbot_cv
)
_generate_msg_py(rectbot_cv
  "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/KeyPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rectbot_cv
)
_generate_msg_py(rectbot_cv
  "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObject.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/KeyPoint.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/ObjectHypothesisWithPose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/Detection2D.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/BoundingBox2D.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rectbot_cv
)

### Generating Services

### Generating Module File
_generate_module_py(rectbot_cv
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rectbot_cv
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rectbot_cv_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rectbot_cv_generate_messages rectbot_cv_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObjectArray.msg" NAME_WE)
add_dependencies(rectbot_cv_generate_messages_py _rectbot_cv_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/KeyPoint.msg" NAME_WE)
add_dependencies(rectbot_cv_generate_messages_py _rectbot_cv_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/arar/Documents/rectbot_ws/src/rectbot_cv/msg/PoseObject.msg" NAME_WE)
add_dependencies(rectbot_cv_generate_messages_py _rectbot_cv_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rectbot_cv_genpy)
add_dependencies(rectbot_cv_genpy rectbot_cv_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rectbot_cv_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rectbot_cv)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rectbot_cv
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(rectbot_cv_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(rectbot_cv_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET vision_msgs_generate_messages_cpp)
  add_dependencies(rectbot_cv_generate_messages_cpp vision_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rectbot_cv)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rectbot_cv
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(rectbot_cv_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(rectbot_cv_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET vision_msgs_generate_messages_eus)
  add_dependencies(rectbot_cv_generate_messages_eus vision_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rectbot_cv)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rectbot_cv
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(rectbot_cv_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(rectbot_cv_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET vision_msgs_generate_messages_lisp)
  add_dependencies(rectbot_cv_generate_messages_lisp vision_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rectbot_cv)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rectbot_cv
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(rectbot_cv_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(rectbot_cv_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET vision_msgs_generate_messages_nodejs)
  add_dependencies(rectbot_cv_generate_messages_nodejs vision_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rectbot_cv)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rectbot_cv\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rectbot_cv
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(rectbot_cv_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(rectbot_cv_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET vision_msgs_generate_messages_py)
  add_dependencies(rectbot_cv_generate_messages_py vision_msgs_generate_messages_py)
endif()
