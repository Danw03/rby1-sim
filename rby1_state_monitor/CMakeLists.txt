cmake_minimum_required(VERSION 3.10)
project(rby1_state_monitor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

if(DEFINED ENV{RBY1_SDK_PATH})
  set(RBY1_SDK_PATH $ENV{RBY1_SDK_PATH})
else()
  message(FATAL_ERROR
    "RBY1_SDK_PATH is not set. "
    "Use the same SDK path used by rby1_driver.")
endif()

if(NOT EXISTS "${RBY1_SDK_PATH}/include/rby1-sdk/robot.h")
  message(FATAL_ERROR
    "RB-Y1 SDK header not found under ${RBY1_SDK_PATH}/include")
endif()

find_library(
  RBY1_SDK_LIBRARY
  NAMES rby1-sdk
  PATHS
    "${RBY1_SDK_PATH}/lib"
    "${RBY1_SDK_PATH}/build/src"
  NO_DEFAULT_PATH
)

if(NOT RBY1_SDK_LIBRARY)
  message(FATAL_ERROR
    "librby1-sdk.so was not found under ${RBY1_SDK_PATH}/lib "
    "or ${RBY1_SDK_PATH}/build/src")
endif()

message(STATUS "RB-Y1 SDK root: ${RBY1_SDK_PATH}")
message(STATUS "RB-Y1 SDK library: ${RBY1_SDK_LIBRARY}")

add_executable(
  power_servo_monitor
  src/power_servo_monitor.cpp
)

ament_target_dependencies(
  power_servo_monitor
  rclcpp
  std_msgs
)

target_include_directories(
  power_servo_monitor
  PRIVATE
    "${RBY1_SDK_PATH}/include"
    "${RBY1_SDK_PATH}/build/private"
    /usr/include/eigen3
)

target_link_libraries(
  power_servo_monitor
  "${RBY1_SDK_LIBRARY}"
)

set_target_properties(
  power_servo_monitor
  PROPERTIES
    BUILD_RPATH "${RBY1_SDK_PATH}/lib;${RBY1_SDK_PATH}/build/src"
    INSTALL_RPATH "${RBY1_SDK_PATH}/lib;${RBY1_SDK_PATH}/build/src"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

install(
  TARGETS power_servo_monitor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
