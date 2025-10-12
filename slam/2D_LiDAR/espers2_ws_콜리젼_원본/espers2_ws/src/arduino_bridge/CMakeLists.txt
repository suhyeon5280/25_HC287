cmake_minimum_required(VERSION 3.5)
project(arduino_bridge)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    arduino_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
