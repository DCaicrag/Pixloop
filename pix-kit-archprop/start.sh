source ~/pix/pix-kit-archprop/install/setup.bash
roslaunch autoware_launch autoware.launch map_path:=/home/neousys/map/factory_2022 vehicle_model:=hooke sensor_model:=hooke_sensor_kit &
sleep 30
rostopic pub  -1 /autoware/engage std_msgs/Bool "data: true"
rostopic pub -1 /vehicle/engage std_msgs/Bool "data: true"
rostopic pub  -l /planning/scenario_planning/max_velocity std_msgs/Float32 "data: 1" #speed m/s

