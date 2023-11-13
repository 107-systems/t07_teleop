#######################################################################################
cmake_minimum_required(VERSION 3.8)
#######################################################################################
project(t07_teleop)
set(T07_TELEOP_TARGET ${PROJECT_NAME}_node)
#######################################################################################
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Werror -Wextra -Wpedantic)
endif()
#######################################################################################
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mp-units REQUIRED)
#######################################################################################
include_directories(
  include
)
#######################################################################################
add_executable(${T07_TELEOP_TARGET}
  src/main.cpp
)
#######################################################################################
target_compile_features(${T07_TELEOP_TARGET} PRIVATE cxx_std_20)
ament_target_dependencies(${T07_TELEOP_TARGET} rclcpp std_msgs mp-units)
#######################################################################################
install(TARGETS
  ${T07_TELEOP_TARGET}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
#######################################################################################
ament_package()
#######################################################################################
