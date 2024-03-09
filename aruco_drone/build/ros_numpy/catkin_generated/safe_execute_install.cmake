execute_process(COMMAND "/home/jlukas/Desktop/My_Project/ROS1/aruco_drone/build/ros_numpy/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/jlukas/Desktop/My_Project/ROS1/aruco_drone/build/ros_numpy/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
