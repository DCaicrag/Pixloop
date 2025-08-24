cmake_minimum_required(VERSION 3.0.2)
project(pix_hooke_driver_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  std_msgs
)

file(GLOB MSGS_PATH ./mgs/*.msg)

# GearReport.msg
# {name}.msg
add_message_files(
  FILES
  	v2a_drivestafb_530.msg
	v2a_brakestafb_531.msg
	v2a_steerstafb_532.msg
	v2a_vehicleworkstafb_534.msg
	v2a_powerstafb_535.msg
	v2a_vehiclestafb_536.msg
	v2a_vehiclefltsta_537.msg
	v2a_chassiswheelrpmfb_539.msg
	v2a_chassiswheeltirepressfb_540.msg
	v2a_chassiswheelanglefb_541.msg
	a2v_drivectrl_130.msg
	a2v_brakectrl_131.msg
	a2v_steerctrl_132.msg
	a2v_vehiclectrl_133.msg
	a2v_wheelctrl_135.msg

)

generate_messages(
  DEPENDENCIES
  std_msgs
  pix_hooke_driver_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pix_driver_msg
#  CATKIN_DEPENDS message_generation message_runtime roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
