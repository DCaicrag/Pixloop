cmake_minimum_required(VERSION 3.0.2)
project(yolo_ros)

## Compilación de mensajes personalizados
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  message_generation
  cv_bridge
)

## Archivos de mensajes
add_message_files(
  FILES
  BoundingBox.msg
  BoundingBoxes.msg
)

## Generar mensajes y servicios
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
