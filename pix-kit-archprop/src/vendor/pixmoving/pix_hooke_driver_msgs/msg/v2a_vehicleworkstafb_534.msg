Header header 
# 驾驶模式反馈 [] [0|3]
int8 VCU_DrivingModeFb 
# 车辆上电状态反馈 [] [0|3]
int8 VCU_ChassisPowerStaFb 
# DC工作状态 [] [0|3]
int8 VCU_ChassisPowerDcSta 
# 车辆限速状态 [] [0|1]
int8 VCU_ChassisSpeedLimitedModeFb 
# 功率限制状态 [] [0|1]
int8 VCU_ChassisPowerLimiteSta 
# 节能模式反馈 [] [0|3]
int8 VCU_SysEcoMode 
# 车辆限速值反馈 [m/s] [0|50]
float32 VCU_ChassisSpeedLimitedValFb 
# 低压蓄电池电压 [V] [0|25]
float32 VCU_ChassisLowPowerVoltSta 
# 紧急停车状态反馈 [] [0|15]
int8 VCU_ChassisEStopStaFb 
# 车辆前碰撞传感器反馈 [] [0|1]
int8 VCU_CrashFrontSta 
# 车辆后碰撞传感器反馈 [] [0|1]
int8 VCU_CrashRearSta 
# 车辆左碰撞传感器反馈 [] [0|1]
int8 VCU_CrashLeftSta 
# 车辆右碰撞传感器反馈 [] [0|1]
int8 VCU_CrashRightSta 
# VCU循环计数 [] [0|15]
int32 VCU_Life 
# 校验sum=byte0 xor byte1 xor...byte6 [] [0|255]
int32 VCU_CheckSum 
