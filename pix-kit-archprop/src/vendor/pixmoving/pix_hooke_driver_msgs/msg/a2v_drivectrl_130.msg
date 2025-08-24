Header header 
# 车辆加速控制使能 [] [0|1]
int8 ACU_ChassisDriverEnCtrl 
# 驱动模式控制 [] [0|3]
int8 ACU_ChassisDriverModeCtrl 
# 档位控制 [] [0|15]
int8 ACU_ChassisGearCtrl 
# 车辆速度控制 [m/s] [0|50]
float32 ACU_ChassisSpeedCtrl 
# 车辆油门控制 [%] [0|100]
float32 ACU_ChassisThrottlePdlTarget 
# 循环计数0~15 [] [0|15]
int32 ACU_DriveLifeSig 
# 校验sum=byte0 xor byte1 xor...byte6 [] [0|255]
int32 ACU_CheckSum_130 
