execute_process(COMMAND "/home/ros_user/cw2_catkin_ws/build/comp0037/comp0037_mapper/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ros_user/cw2_catkin_ws/build/comp0037/comp0037_mapper/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
