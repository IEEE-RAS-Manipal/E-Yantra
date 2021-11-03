execute_process(COMMAND "/home/ch13f_1419/E-Yantra/SS_1302_WS/build/mavros/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ch13f_1419/E-Yantra/SS_1302_WS/build/mavros/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
