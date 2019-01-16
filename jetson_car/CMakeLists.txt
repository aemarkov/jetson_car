cmake_minimum_required(VERSION 2.8.3)
project(jetson_car)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  message_generation
  tf
)

find_package(PCL 1.2 REQUIRED COMPONENTS
        common
        io
        visualization
        filters
        segmentation)

find_package(ompl)

add_message_files(
  FILES
  MotorsControl.msg
  DigitalMap.msg
  CarState.msg
  CarPath.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS}  ./src)

add_executable(motors_driver_node src/motors_driver_node.cpp)
target_link_libraries(motors_driver_node ${catkin_LIBRARIES})
add_dependencies(motors_driver_node  ${catkin_EXPORTED_TARGETS})

add_executable(odometry_logger_node src/odometry_logger_node.cpp)
target_link_libraries(odometry_logger_node ${catkin_LIBRARIES})

add_executable(mapper_node src/mapper/mapper_node.cpp src/mapper/ransac_segmentation/RANSACSegmentation.cpp src/mapper/ransac_segmentation/RANSACSegmentation.h src/mapper/GridCoord.h)
target_link_libraries(mapper_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(mapper_node  ${catkin_EXPORTED_TARGETS})

add_executable(motion_planner_node src/motion_planner/motion_planner_node.cpp)
target_link_libraries(motion_planner_node ${catkin_LIBRARIES} ${OMPL_LIBRARIES})
add_dependencies(motion_planner_node  ${catkin_EXPORTED_TARGETS})