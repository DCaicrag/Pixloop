cmake_minimum_required(VERSION 3.0.2)
project(pointscloud_detector)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  pcl_ros
  image_transport
  visualization_msgs
  vision_msgs
  message_filters
  tf2_ros
  image_geometry
)

# Eigen3 is usually found automatically, but you can find it like this if needed
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4.8.1 REQUIRED)
# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR} # Add this for Eigen3
  ${PCL_INCLUDE_DIRS} # Add this for PCL
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pointscloud_detector
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/PointsDectector.cpp)

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES} # Add this for Eigen3
  ${PCL_LIBRARIES} # Add this for PCL
  ${OpenCV_LIBS}
)

