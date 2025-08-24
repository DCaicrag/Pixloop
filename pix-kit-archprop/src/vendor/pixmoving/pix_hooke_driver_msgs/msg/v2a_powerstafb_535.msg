Header header 
# 预留 [] [0|15]
int32 VCU_ChassisBmsReserved_1 
# 车辆充电状态 [] [0|3]
int8 VCU_ChassisPowerChargeSta 
# 充电枪连接状态 [] [0|1]
int8 VCU_ChassisPowerChargeSockSta 
# 车辆动力电池电量 [%] [0|100]
int32 VCU_ChassisPowerSocFb 
# 车辆动力电池电压 [V] [0|1000]
float32 VCU_ChassisPowerVoltFb 
# 车辆动力电池电流 [A] [-1000|1000]
float32 VCU_ChassisPowerCurrFb 
# BMS最高单体温度 [℃] [-40|80]
int32 VCU_ChassisBmsMaxTemp 
# 预留 [] [0|255]
int32 VCU_ChassisBmsReserved_2 
