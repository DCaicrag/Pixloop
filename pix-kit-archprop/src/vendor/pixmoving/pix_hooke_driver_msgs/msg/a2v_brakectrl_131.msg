Header header 
# 车辆刹车控制使能 [] [0|1]
int8 ACU_ChassisBrakeEn 
# AEB使能（预留，自动驾驶实现） [] [0|1]
int8 ACU_ChassisAebCtrl 
# 车辆刹车控制 [%] [0|100]
float32 ACU_ChassisBrakePdlTarget 
# 驻车控制 [] [0|3]
int8 ACU_ChassisEpbCtrl 
# 循环计数0~15 [] [0|15]
int32 ACU_BrakeLifeSig 
# 校验sum=byte0 xor byte1 xor...byte6 [] [0|255]
int32 ACU_CheckSum_131 
