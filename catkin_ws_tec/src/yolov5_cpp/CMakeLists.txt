cmake_minimum_required(VERSION 3.0.2)
project(yolov5_cpp)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  image_transport
  visualization_msgs
  vision_msgs
)

find_package(OpenCV 4.8.1 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES yolov5_cpp
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

# Since pkg-config cannot find OpenCV, manually specify the include directory
# You need to provide the correct path to the OpenCV include directory
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)



## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/yolov5.cpp)

## Add cmake target dependencies of the executable
## This ensures the message headers are built before your executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS} # If you have find_package(OpenCV) in your CMakeLists.txt
  # ... add any other libraries you use
)
