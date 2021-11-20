cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

generate_arduino_firmware(arduino
  SRCS src/Sketch.cpp src/DepthSensorDriver.cpp src/ThrusterArrayDriver.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD uno
  PORT /dev/ttyACM0
)