Header header 
# 车辆转向控制使能 [] [0|1]
int8 ACU_ChassisSteerEnCtrl 
# 转向模式控制 [] [0|15]
int8 ACU_ChassisSteerModeCtrl 
# 车辆转向控制（前） [deg] [-500|500]
int32 ACU_ChassisSteerAngleTarget 
# 车辆转向控制（后） [deg] [-500|500]
int32 ACU_ChassisSteerAngleRearTarget 
# 车辆方向盘转角速度控制 [deg/s] [0|500]
float32 ACU_ChassisSteerAngleSpeedCtrl 
# 校验sum=byte0 xor byte1 xor...byte6 [] [0|255]
int32 ACU_CheckSum_132 
