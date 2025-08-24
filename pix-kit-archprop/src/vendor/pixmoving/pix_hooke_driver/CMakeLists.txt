cmake_minimum_required(VERSION 3.0.2)
project(pix_hooke_driver)

find_package(catkin REQUIRED COMPONENTS
  can_msgs
  roscpp
  std_msgs
  pix_hooke_driver_msgs
  autoware_vehicle_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES pix_hooke_driver
 CATKIN_DEPENDS can_msgs roscpp std_msgs pix_hooke_driver_msgs autoware_vehicle_msgs
# DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/pix_driver.cpp
# )

# src/steering_report_502.cc
# src/{name}.cc
add_executable(${PROJECT_NAME}_report_node 
src/v2a_drivestafb_530.cc
src/v2a_brakestafb_531.cc
src/v2a_steerstafb_532.cc
src/v2a_vehicleworkstafb_534.cc
src/v2a_powerstafb_535.cc
src/v2a_vehiclestafb_536.cc
src/v2a_vehiclefltsta_537.cc
src/v2a_chassiswheelrpmfb_539.cc
src/v2a_chassiswheeltirepressfb_540.cc
src/v2a_chassiswheelanglefb_541.cc


src/report_node.cc
src/Byte.cc
)

# src/steering_command_102.cc
# src/{name}.cc
add_executable(${PROJECT_NAME}_command_node
src/a2v_drivectrl_130.cc
src/a2v_brakectrl_131.cc
src/a2v_steerctrl_132.cc
src/a2v_vehiclectrl_133.cc
src/a2v_wheelctrl_135.cc


src/command_node.cc
src/Byte.cc
)

add_executable(
  control_converter_node
  src/control_converter_node.cpp
)

add_executable(
  report_converter_node
  src/report_converter_node.cpp
)


add_dependencies(${PROJECT_NAME}_report_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_command_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(control_converter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(report_converter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_report_node
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_command_node
  ${catkin_LIBRARIES}
)

target_link_libraries(
  control_converter_node
  ${catkin_LIBRARIES}
)

target_link_libraries(
  report_converter_node
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_report_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_command_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS control_converter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS report_converter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

