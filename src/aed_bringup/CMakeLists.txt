cmake_minimum_required(VERSION 3.8)
project(aed_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../maps/
  DESTINATION share/${PROJECT_NAME}/maps
  FILES_MATCHING
  PATTERN "*.yaml"
  PATTERN "*.pgm"
)

ament_package()
