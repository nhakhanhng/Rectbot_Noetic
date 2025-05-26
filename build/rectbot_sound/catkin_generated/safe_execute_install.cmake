execute_process(COMMAND "/SLAM/rectbot_ws/build/rectbot_sound/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/SLAM/rectbot_ws/build/rectbot_sound/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
