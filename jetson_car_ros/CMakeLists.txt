cmake_minimum_required(VERSION 2.8.3)
project(jetson_car)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  MotorsControl.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(motors_driver src/motors_driver.cpp)
target_link_libraries(motors_driver
   ${catkin_LIBRARIES}
)


