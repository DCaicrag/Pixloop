cmake_minimum_required(VERSION 3.0.2)
project(accel_brake_map_calibrator)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
  tf
  geometry_msgs
  raw_vehicle_cmd_converter
  tf2_ros
  pix_hooke_driver_msgs
  sensor_msgs
  nav_msgs
  message_generation
)

add_service_files(
  FILES
  SaveCalibration.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    accel_brake_map_calibrator
  CATKIN_DEPENDS
    roscpp
    std_msgs
    visualization_msgs
    tf
    geometry_msgs
    raw_vehicle_cmd_converter
    tf2_ros
    pix_hooke_driver_msgs
    sensor_msgs
    nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(accel_brake_map_calibrator src/accel_brake_map_calibrator_node.cpp)
target_link_libraries(accel_brake_map_calibrator
  ${catkin_LIBRARIES}
)
add_dependencies(accel_brake_map_calibrator ${catkin_EXPORTED_TARGETS})

add_executable(accel_brake_map_calibrator_node src/main.cpp)
target_link_libraries(accel_brake_map_calibrator_node
  accel_brake_map_calibrator
  ${catkin_LIBRARIES}
)
add_dependencies(accel_brake_map_calibrator_node ${catkin_EXPORTED_TARGETS})

install(
  TARGETS
    accel_brake_map_calibrator_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS
    accel_brake_map_calibrator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
    rviz
    config
    data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
