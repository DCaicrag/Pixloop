#include <math.h>
#include <iostream>
#include "ros/ros.h"
#include "can_msgs/Frame.h"
#include "std_msgs/Header.h"

// name = 
// # include "pix pix_driver_msgs/{name}.h"
#include "pix_hooke_driver_msgs/v2a_drivestafb_530.h"
#include "pix_hooke_driver_msgs/v2a_brakestafb_531.h"
#include "pix_hooke_driver_msgs/v2a_steerstafb_532.h"
#include "pix_hooke_driver_msgs/v2a_vehicleworkstafb_534.h"
#include "pix_hooke_driver_msgs/v2a_powerstafb_535.h"
#include "pix_hooke_driver_msgs/v2a_vehiclestafb_536.h"
#include "pix_hooke_driver_msgs/v2a_vehiclefltsta_537.h"
#include "pix_hooke_driver_msgs/v2a_chassiswheelrpmfb_539.h"
#include "pix_hooke_driver_msgs/v2a_chassiswheeltirepressfb_540.h"
#include "pix_hooke_driver_msgs/v2a_chassiswheelanglefb_541.h"


// # include "{name}.hpp"
#include "v2a_drivestafb_530.hpp"
#include "v2a_brakestafb_531.hpp"
#include "v2a_steerstafb_532.hpp"
#include "v2a_vehicleworkstafb_534.hpp"
#include "v2a_powerstafb_535.hpp"
#include "v2a_vehiclestafb_536.hpp"
#include "v2a_vehiclefltsta_537.hpp"
#include "v2a_chassiswheelrpmfb_539.hpp"
#include "v2a_chassiswheeltirepressfb_540.hpp"
#include "v2a_chassiswheelanglefb_541.hpp"


static can_msgs::Frame can_frame_msg;

// static ros::Publisher pub_{can_name};
static ros::Publisher pub_v2a_drivestafb;
static ros::Publisher pub_v2a_brakestafb;
static ros::Publisher pub_v2a_steerstafb;
static ros::Publisher pub_v2a_vehicleworkstafb;
static ros::Publisher pub_v2a_powerstafb;
static ros::Publisher pub_v2a_vehiclestafb;
static ros::Publisher pub_v2a_vehiclefltsta;
static ros::Publisher pub_v2a_chassiswheelrpmfb;
static ros::Publisher pub_v2a_chassiswheeltirepressfb;
static ros::Publisher pub_v2a_chassiswheelanglefb;


// static pix_driver_msgs::{name} {name}_msg;
static pix_hooke_driver_msgs::v2a_drivestafb_530 v2a_drivestafb_530_msg;
static pix_hooke_driver_msgs::v2a_brakestafb_531 v2a_brakestafb_531_msg;
static pix_hooke_driver_msgs::v2a_steerstafb_532 v2a_steerstafb_532_msg;
static pix_hooke_driver_msgs::v2a_vehicleworkstafb_534 v2a_vehicleworkstafb_534_msg;
static pix_hooke_driver_msgs::v2a_powerstafb_535 v2a_powerstafb_535_msg;
static pix_hooke_driver_msgs::v2a_vehiclestafb_536 v2a_vehiclestafb_536_msg;
static pix_hooke_driver_msgs::v2a_vehiclefltsta_537 v2a_vehiclefltsta_537_msg;
static pix_hooke_driver_msgs::v2a_chassiswheelrpmfb_539 v2a_chassiswheelrpmfb_539_msg;
static pix_hooke_driver_msgs::v2a_chassiswheeltirepressfb_540 v2a_chassiswheeltirepressfb_540_msg;
static pix_hooke_driver_msgs::v2a_chassiswheelanglefb_541 v2a_chassiswheelanglefb_541_msg;


// static {name}.replace('_', '').capitalize()  {name}_entity
static V2adrivestafb530  v2a_drivestafb_530_entity;
static V2abrakestafb531  v2a_brakestafb_531_entity;
static V2asteerstafb532  v2a_steerstafb_532_entity;
static V2avehicleworkstafb534  v2a_vehicleworkstafb_534_entity;
static V2apowerstafb535  v2a_powerstafb_535_entity;
static V2avehiclestafb536  v2a_vehiclestafb_536_entity;
static V2avehiclefltsta537  v2a_vehiclefltsta_537_entity;
static V2achassiswheelrpmfb539  v2a_chassiswheelrpmfb_539_entity;
static V2achassiswheeltirepressfb540  v2a_chassiswheeltirepressfb_540_entity;
static V2achassiswheelanglefb541  v2a_chassiswheelanglefb_541_entity;



// callback func
static void can_callback(const can_msgs::Frame &msg)
{
    can_frame_msg = msg;
    std_msgs::Header header;
    header.frame_id = "pix";
    header.stamp = can_frame_msg.header.stamp;
    
    if(can_frame_msg.id==v2a_drivestafb_530_entity.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
        byte_temp[i] = can_frame_msg.data[i];
        }
        v2a_drivestafb_530_entity.update_bytes(byte_temp);
        v2a_drivestafb_530_entity.Parse();
        v2a_drivestafb_530_msg.header = header;
		v2a_drivestafb_530_msg.VCU_ChassisDriverEnSta = v2a_drivestafb_530_entity.vcu_chassisdriverensta;
		v2a_drivestafb_530_msg.VCU_ChassisDiverSlopover = v2a_drivestafb_530_entity.vcu_chassisdiverslopover;
		v2a_drivestafb_530_msg.VCU_ChassisDriverModeSta = v2a_drivestafb_530_entity.vcu_chassisdrivermodesta;
		v2a_drivestafb_530_msg.VCU_ChassisGearFb = v2a_drivestafb_530_entity.vcu_chassisgearfb;
		v2a_drivestafb_530_msg.VCU_ChassisSpeedFb = v2a_drivestafb_530_entity.vcu_chassisspeedfb;
		v2a_drivestafb_530_msg.VCU_ChassisThrottlePaldFb = v2a_drivestafb_530_entity.vcu_chassisthrottlepaldfb;
		v2a_drivestafb_530_msg.VCU_ChassisAccelerationFb = v2a_drivestafb_530_entity.vcu_chassisaccelerationfb;

        pub_v2a_drivestafb.publish(v2a_drivestafb_530_msg);
    }
    
    if(can_frame_msg.id==v2a_brakestafb_531_entity.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
        byte_temp[i] = can_frame_msg.data[i];
        }
        v2a_brakestafb_531_entity.update_bytes(byte_temp);
        v2a_brakestafb_531_entity.Parse();
        v2a_brakestafb_531_msg.header = header;
		v2a_brakestafb_531_msg.VCU_ChassisBrakeEnSta = v2a_brakestafb_531_entity.vcu_chassisbrakeensta;
		v2a_brakestafb_531_msg.VCU_VehicleBrakeLampFb = v2a_brakestafb_531_entity.vcu_vehiclebrakelampfb;
		v2a_brakestafb_531_msg.VCU_ChassisEpbFb = v2a_brakestafb_531_entity.vcu_chassisepbfb;
		v2a_brakestafb_531_msg.VCU_ChassisBrakePadlFb = v2a_brakestafb_531_entity.vcu_chassisbrakepadlfb;
		v2a_brakestafb_531_msg.VCU_AebEnStaFb = v2a_brakestafb_531_entity.vcu_aebenstafb;
		v2a_brakestafb_531_msg.VCU_AebTriggerStaFb = v2a_brakestafb_531_entity.vcu_aebtriggerstafb;

        pub_v2a_brakestafb.publish(v2a_brakestafb_531_msg);
    }
    
    if(can_frame_msg.id==v2a_steerstafb_532_entity.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
        byte_temp[i] = can_frame_msg.data[i];
        }
        v2a_steerstafb_532_entity.update_bytes(byte_temp);
        v2a_steerstafb_532_entity.Parse();
        v2a_steerstafb_532_msg.header = header;
		v2a_steerstafb_532_msg.VCU_ChassisSteerEnSta = v2a_steerstafb_532_entity.vcu_chassissteerensta;
		v2a_steerstafb_532_msg.VCU_ChassisSteerSlopover = v2a_steerstafb_532_entity.vcu_chassissteerslopover;
		v2a_steerstafb_532_msg.VCU_ChassisSteerWorkMode = v2a_steerstafb_532_entity.vcu_chassissteerworkmode;
		v2a_steerstafb_532_msg.VCU_ChassisSteerModeFb = v2a_steerstafb_532_entity.vcu_chassissteermodefb;
		v2a_steerstafb_532_msg.VCU_ChassisSteerAngleFb = v2a_steerstafb_532_entity.vcu_chassissteeranglefb;
		v2a_steerstafb_532_msg.VCU_ChassisSteerAngleRearFb = v2a_steerstafb_532_entity.vcu_chassissteeranglerearfb;
		v2a_steerstafb_532_msg.VCU_ChassisSteerAngleSpeedFb = v2a_steerstafb_532_entity.vcu_chassissteeranglespeedfb;

        pub_v2a_steerstafb.publish(v2a_steerstafb_532_msg);
    }
    
    if(can_frame_msg.id==v2a_vehicleworkstafb_534_entity.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
        byte_temp[i] = can_frame_msg.data[i];
        }
        v2a_vehicleworkstafb_534_entity.update_bytes(byte_temp);
        v2a_vehicleworkstafb_534_entity.Parse();
        v2a_vehicleworkstafb_534_msg.header = header;
		v2a_vehicleworkstafb_534_msg.VCU_DrivingModeFb = v2a_vehicleworkstafb_534_entity.vcu_drivingmodefb;
		v2a_vehicleworkstafb_534_msg.VCU_ChassisPowerStaFb = v2a_vehicleworkstafb_534_entity.vcu_chassispowerstafb;
		v2a_vehicleworkstafb_534_msg.VCU_ChassisPowerDcSta = v2a_vehicleworkstafb_534_entity.vcu_chassispowerdcsta;
		v2a_vehicleworkstafb_534_msg.VCU_ChassisSpeedLimitedModeFb = v2a_vehicleworkstafb_534_entity.vcu_chassisspeedlimitedmodefb;
		v2a_vehicleworkstafb_534_msg.VCU_ChassisPowerLimiteSta = v2a_vehicleworkstafb_534_entity.vcu_chassispowerlimitesta;
		v2a_vehicleworkstafb_534_msg.VCU_SysEcoMode = v2a_vehicleworkstafb_534_entity.vcu_sysecomode;
		v2a_vehicleworkstafb_534_msg.VCU_ChassisSpeedLimitedValFb = v2a_vehicleworkstafb_534_entity.vcu_chassisspeedlimitedvalfb;
		v2a_vehicleworkstafb_534_msg.VCU_ChassisLowPowerVoltSta = v2a_vehicleworkstafb_534_entity.vcu_chassislowpowervoltsta;
		v2a_vehicleworkstafb_534_msg.VCU_ChassisEStopStaFb = v2a_vehicleworkstafb_534_entity.vcu_chassisestopstafb;
		v2a_vehicleworkstafb_534_msg.VCU_CrashFrontSta = v2a_vehicleworkstafb_534_entity.vcu_crashfrontsta;
		v2a_vehicleworkstafb_534_msg.VCU_CrashRearSta = v2a_vehicleworkstafb_534_entity.vcu_crashrearsta;
		v2a_vehicleworkstafb_534_msg.VCU_CrashLeftSta = v2a_vehicleworkstafb_534_entity.vcu_crashleftsta;
		v2a_vehicleworkstafb_534_msg.VCU_CrashRightSta = v2a_vehicleworkstafb_534_entity.vcu_crashrightsta;
		v2a_vehicleworkstafb_534_msg.VCU_Life = v2a_vehicleworkstafb_534_entity.vcu_life;
		v2a_vehicleworkstafb_534_msg.VCU_CheckSum = v2a_vehicleworkstafb_534_entity.vcu_checksum;

        pub_v2a_vehicleworkstafb.publish(v2a_vehicleworkstafb_534_msg);
    }
    
    if(can_frame_msg.id==v2a_powerstafb_535_entity.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
        byte_temp[i] = can_frame_msg.data[i];
        }
        v2a_powerstafb_535_entity.update_bytes(byte_temp);
        v2a_powerstafb_535_entity.Parse();
        v2a_powerstafb_535_msg.header = header;
		v2a_powerstafb_535_msg.VCU_ChassisBmsReserved_1 = v2a_powerstafb_535_entity.vcu_chassisbmsreserved_1;
		v2a_powerstafb_535_msg.VCU_ChassisPowerChargeSta = v2a_powerstafb_535_entity.vcu_chassispowerchargesta;
		v2a_powerstafb_535_msg.VCU_ChassisPowerChargeSockSta = v2a_powerstafb_535_entity.vcu_chassispowerchargesocksta;
		v2a_powerstafb_535_msg.VCU_ChassisPowerSocFb = v2a_powerstafb_535_entity.vcu_chassispowersocfb;
		v2a_powerstafb_535_msg.VCU_ChassisPowerVoltFb = v2a_powerstafb_535_entity.vcu_chassispowervoltfb;
		v2a_powerstafb_535_msg.VCU_ChassisPowerCurrFb = v2a_powerstafb_535_entity.vcu_chassispowercurrfb;
		v2a_powerstafb_535_msg.VCU_ChassisBmsMaxTemp = v2a_powerstafb_535_entity.vcu_chassisbmsmaxtemp;
		v2a_powerstafb_535_msg.VCU_ChassisBmsReserved_2 = v2a_powerstafb_535_entity.vcu_chassisbmsreserved_2;

        pub_v2a_powerstafb.publish(v2a_powerstafb_535_msg);
    }
    
    if(can_frame_msg.id==v2a_vehiclestafb_536_entity.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
        byte_temp[i] = can_frame_msg.data[i];
        }
        v2a_vehiclestafb_536_entity.update_bytes(byte_temp);
        v2a_vehiclestafb_536_entity.Parse();
        v2a_vehiclestafb_536_msg.header = header;
		v2a_vehiclestafb_536_msg.VCU_VehiclePosLampFb = v2a_vehiclestafb_536_entity.vcu_vehicleposlampfb;
		v2a_vehiclestafb_536_msg.VCU_VehicleHeadLampFb = v2a_vehiclestafb_536_entity.vcu_vehicleheadlampfb;
		v2a_vehiclestafb_536_msg.VCU_VehicleLeftLampFb = v2a_vehiclestafb_536_entity.vcu_vehicleleftlampfb;
		v2a_vehiclestafb_536_msg.VCU_VehicleRightLampFb = v2a_vehiclestafb_536_entity.vcu_vehiclerightlampfb;
		v2a_vehiclestafb_536_msg.VCU_VehicleHighBeamFb = v2a_vehiclestafb_536_entity.vcu_vehiclehighbeamfb;
		v2a_vehiclestafb_536_msg.VCU_VehicleFogLampFb = v2a_vehiclestafb_536_entity.vcu_vehiclefoglampfb;
		v2a_vehiclestafb_536_msg.VCU_VehicleHazardWarLampFb = v2a_vehiclestafb_536_entity.vcu_vehiclehazardwarlampfb;
		v2a_vehiclestafb_536_msg.VCU_VehicleBodyLampFb = v2a_vehiclestafb_536_entity.vcu_vehiclebodylampfb;
		v2a_vehiclestafb_536_msg.VCU_VehicleReadLampFb = v2a_vehiclestafb_536_entity.vcu_vehiclereadlampfb;
		v2a_vehiclestafb_536_msg.ACU_VehicleWindowFb = v2a_vehiclestafb_536_entity.acu_vehiclewindowfb;
		v2a_vehiclestafb_536_msg.VCU_VehicleDoorStaFb = v2a_vehiclestafb_536_entity.vcu_vehicledoorstafb;
		v2a_vehiclestafb_536_msg.VCU_VehicleWipersStaFb = v2a_vehiclestafb_536_entity.vcu_vehiclewipersstafb;
		v2a_vehiclestafb_536_msg.VCU_VehicleSafetyBelt1 = v2a_vehiclestafb_536_entity.vcu_vehiclesafetybelt1;
		v2a_vehiclestafb_536_msg.VCU_VehicleSafetyBelt2 = v2a_vehiclestafb_536_entity.vcu_vehiclesafetybelt2;
		v2a_vehiclestafb_536_msg.VCU_VehicleSafetyBelt3 = v2a_vehiclestafb_536_entity.vcu_vehiclesafetybelt3;
		v2a_vehiclestafb_536_msg.VCU_VehicleSafetyBelt4 = v2a_vehiclestafb_536_entity.vcu_vehiclesafetybelt4;

        pub_v2a_vehiclestafb.publish(v2a_vehiclestafb_536_msg);
    }
    
    if(can_frame_msg.id==v2a_vehiclefltsta_537_entity.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
        byte_temp[i] = can_frame_msg.data[i];
        }
        v2a_vehiclefltsta_537_entity.update_bytes(byte_temp);
        v2a_vehiclefltsta_537_entity.Parse();
        v2a_vehiclefltsta_537_msg.header = header;
		v2a_vehiclefltsta_537_msg.VCU_SysMotorOverTempSta = v2a_vehiclefltsta_537_entity.vcu_sysmotorovertempsta;
		v2a_vehiclefltsta_537_msg.VCU_SysBmsOverTempSta = v2a_vehiclefltsta_537_entity.vcu_sysbmsovertempsta;
		v2a_vehiclefltsta_537_msg.VCU_SysBrakeOverTempSta = v2a_vehiclefltsta_537_entity.vcu_sysbrakeovertempsta;
		v2a_vehiclefltsta_537_msg.VCU_SysSteerOverTempSta = v2a_vehiclefltsta_537_entity.vcu_syssteerovertempsta;
		v2a_vehiclefltsta_537_msg.VCU_SysUnderVolt = v2a_vehiclefltsta_537_entity.vcu_sysundervolt;
		v2a_vehiclefltsta_537_msg.VCU_SysFlt = v2a_vehiclefltsta_537_entity.vcu_sysflt;
		v2a_vehiclefltsta_537_msg.VCU_SysBrakeFlt = v2a_vehiclefltsta_537_entity.vcu_sysbrakeflt;
		v2a_vehiclefltsta_537_msg.VCU_SysParkingFlt = v2a_vehiclefltsta_537_entity.vcu_sysparkingflt;
		v2a_vehiclefltsta_537_msg.VCU_SysSteerFrontFlt = v2a_vehiclefltsta_537_entity.vcu_syssteerfrontflt;
		v2a_vehiclefltsta_537_msg.VCU_SysSteerBackFlt = v2a_vehiclefltsta_537_entity.vcu_syssteerbackflt;
		v2a_vehiclefltsta_537_msg.VCU_SysMotorLfFlt = v2a_vehiclefltsta_537_entity.vcu_sysmotorlfflt;
		v2a_vehiclefltsta_537_msg.VCU_SysMotorRfFlt = v2a_vehiclefltsta_537_entity.vcu_sysmotorrfflt;
		v2a_vehiclefltsta_537_msg.VCU_SysMotorLrFlt = v2a_vehiclefltsta_537_entity.vcu_sysmotorlrflt;
		v2a_vehiclefltsta_537_msg.VCU_SysMotorRrFlt = v2a_vehiclefltsta_537_entity.vcu_sysmotorrrflt;
		v2a_vehiclefltsta_537_msg.VCU_SysBmsFlt = v2a_vehiclefltsta_537_entity.vcu_sysbmsflt;
		v2a_vehiclefltsta_537_msg.VCU_SysDcFlt = v2a_vehiclefltsta_537_entity.vcu_sysdcflt;

        pub_v2a_vehiclefltsta.publish(v2a_vehiclefltsta_537_msg);
    }
    
    if(can_frame_msg.id==v2a_chassiswheelrpmfb_539_entity.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
        byte_temp[i] = can_frame_msg.data[i];
        }
        v2a_chassiswheelrpmfb_539_entity.update_bytes(byte_temp);
        v2a_chassiswheelrpmfb_539_entity.Parse();
        v2a_chassiswheelrpmfb_539_msg.header = header;
		v2a_chassiswheelrpmfb_539_msg.VCU_ChassisWheelRpmLf = v2a_chassiswheelrpmfb_539_entity.vcu_chassiswheelrpmlf;
		v2a_chassiswheelrpmfb_539_msg.VCU_ChassisWheelRpmRf = v2a_chassiswheelrpmfb_539_entity.vcu_chassiswheelrpmrf;
		v2a_chassiswheelrpmfb_539_msg.VCU_ChassisWheelRpmLr = v2a_chassiswheelrpmfb_539_entity.vcu_chassiswheelrpmlr;
		v2a_chassiswheelrpmfb_539_msg.VCU_ChassisWheelRpmRr = v2a_chassiswheelrpmfb_539_entity.vcu_chassiswheelrpmrr;

        pub_v2a_chassiswheelrpmfb.publish(v2a_chassiswheelrpmfb_539_msg);
    }
    
    if(can_frame_msg.id==v2a_chassiswheeltirepressfb_540_entity.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
        byte_temp[i] = can_frame_msg.data[i];
        }
        v2a_chassiswheeltirepressfb_540_entity.update_bytes(byte_temp);
        v2a_chassiswheeltirepressfb_540_entity.Parse();
        v2a_chassiswheeltirepressfb_540_msg.header = header;
		v2a_chassiswheeltirepressfb_540_msg.VCU_ChassisWheelTirePressLf = v2a_chassiswheeltirepressfb_540_entity.vcu_chassiswheeltirepresslf;
		v2a_chassiswheeltirepressfb_540_msg.VCU_ChassisWheelTirePressRf = v2a_chassiswheeltirepressfb_540_entity.vcu_chassiswheeltirepressrf;
		v2a_chassiswheeltirepressfb_540_msg.VCU_ChassisWheelTirePressLr = v2a_chassiswheeltirepressfb_540_entity.vcu_chassiswheeltirepresslr;
		v2a_chassiswheeltirepressfb_540_msg.VCU_ChassisWheelTirePressRr = v2a_chassiswheeltirepressfb_540_entity.vcu_chassiswheeltirepressrr;

        pub_v2a_chassiswheeltirepressfb.publish(v2a_chassiswheeltirepressfb_540_msg);
    }
    
    if(can_frame_msg.id==v2a_chassiswheelanglefb_541_entity.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
        byte_temp[i] = can_frame_msg.data[i];
        }
        v2a_chassiswheelanglefb_541_entity.update_bytes(byte_temp);
        v2a_chassiswheelanglefb_541_entity.Parse();
        v2a_chassiswheelanglefb_541_msg.header = header;
		v2a_chassiswheelanglefb_541_msg.VCU_ChassisWheelAngleLf = v2a_chassiswheelanglefb_541_entity.vcu_chassiswheelanglelf;
		v2a_chassiswheelanglefb_541_msg.VCU_ChassisWheelAngleRf = v2a_chassiswheelanglefb_541_entity.vcu_chassiswheelanglerf;
		v2a_chassiswheelanglefb_541_msg.VCU_ChassisWheelAngleLr = v2a_chassiswheelanglefb_541_entity.vcu_chassiswheelanglelr;
		v2a_chassiswheelanglefb_541_msg.VCU_ChassisWheelAngleRr = v2a_chassiswheelanglefb_541_entity.vcu_chassiswheelanglerr;

        pub_v2a_chassiswheelanglefb.publish(v2a_chassiswheelanglefb_541_msg);
    }
    
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pix_hooke_driver_report_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/received_messages", 1, can_callback);

    // pub_{can_name} = nh.advertise<pix_driver_msgs::{name}>("/pix/{can_name}_report", 1, true);
    pub_v2a_drivestafb = nh.advertise<pix_hooke_driver_msgs::v2a_drivestafb_530>("/pix_hooke/v2a_drivestafb", 1, true);
	pub_v2a_brakestafb = nh.advertise<pix_hooke_driver_msgs::v2a_brakestafb_531>("/pix_hooke/v2a_brakestafb", 1, true);
	pub_v2a_steerstafb = nh.advertise<pix_hooke_driver_msgs::v2a_steerstafb_532>("/pix_hooke/v2a_steerstafb", 1, true);
	pub_v2a_vehicleworkstafb = nh.advertise<pix_hooke_driver_msgs::v2a_vehicleworkstafb_534>("/pix_hooke/v2a_vehicleworkstafb", 1, true);
	pub_v2a_powerstafb = nh.advertise<pix_hooke_driver_msgs::v2a_powerstafb_535>("/pix_hooke/v2a_powerstafb", 1, true);
	pub_v2a_vehiclestafb = nh.advertise<pix_hooke_driver_msgs::v2a_vehiclestafb_536>("/pix_hooke/v2a_vehiclestafb", 1, true);
	pub_v2a_vehiclefltsta = nh.advertise<pix_hooke_driver_msgs::v2a_vehiclefltsta_537>("/pix_hooke/v2a_vehiclefltsta", 1, true);
	pub_v2a_chassiswheelrpmfb = nh.advertise<pix_hooke_driver_msgs::v2a_chassiswheelrpmfb_539>("/pix_hooke/v2a_chassiswheelrpmfb", 1, true);
	pub_v2a_chassiswheeltirepressfb = nh.advertise<pix_hooke_driver_msgs::v2a_chassiswheeltirepressfb_540>("/pix_hooke/v2a_chassiswheeltirepressfb", 1, true);
	pub_v2a_chassiswheelanglefb = nh.advertise<pix_hooke_driver_msgs::v2a_chassiswheelanglefb_541>("/pix_hooke/v2a_chassiswheelanglefb", 1, true);
	
    // add another publisher

    ros::spin();
    return 0;

}
