#include "ros/ros.h"
#include "can_msgs/Frame.h"


// include- msgs header file
// Example: #include "pix_driver_msgs/BrakeCommand.h"
// #include pix_driver_msgs/protocols["name"].h
#include "pix_hooke_driver_msgs/a2v_drivectrl_130.h"
#include "pix_hooke_driver_msgs/a2v_brakectrl_131.h"
#include "pix_hooke_driver_msgs/a2v_steerctrl_132.h"
#include "pix_hooke_driver_msgs/a2v_vehiclectrl_133.h"
#include "pix_hooke_driver_msgs/a2v_wheelctrl_135.h"



// include- Parse header file
// Example: #include "brake_command_101.hpp"
// #include protocols["name"].cpp
#include "a2v_drivectrl_130.hpp"
#include "a2v_brakectrl_131.hpp"
#include "a2v_steerctrl_132.hpp"
#include "a2v_vehiclectrl_133.hpp"
#include "a2v_wheelctrl_135.hpp"



// global variable: msg
/*
Example:
static can_msgs::Frame can_brake;
static pix_driver_msgs::BrakeCommand brake_command_msg;

static can_msgs::Frame can_protocols["name"].split("_",1)[0];
static pix_driver_msgs::protocols["name"] protocols["name"]_msg
*/
static pix_hooke_driver_msgs::a2v_drivectrl_130 a2v_drivectrl_130_msg;
static pix_hooke_driver_msgs::a2v_brakectrl_131 a2v_brakectrl_131_msg;
static pix_hooke_driver_msgs::a2v_steerctrl_132 a2v_steerctrl_132_msg;
static pix_hooke_driver_msgs::a2v_vehiclectrl_133 a2v_vehiclectrl_133_msg;
static pix_hooke_driver_msgs::a2v_wheelctrl_135 a2v_wheelctrl_135_msg;

static can_msgs::Frame can_a2v_drivectrl_130;
static can_msgs::Frame can_a2v_brakectrl_131;
static can_msgs::Frame can_a2v_steerctrl_132;
static can_msgs::Frame can_a2v_vehiclectrl_133;
static can_msgs::Frame can_a2v_wheelctrl_135;


// global variable:  send entity
// Example: static Brakecommand101 brake_command;
// static protocols["name"] protocols["name"]_entity;
static A2vdrivectrl130 a2v_drivectrl_130_entity;
static A2vbrakectrl131 a2v_brakectrl_131_entity;
static A2vsteerctrl132 a2v_steerctrl_132_entity;
static A2vvehiclectrl133 a2v_vehiclectrl_133_entity;
static A2vwheelctrl135 a2v_wheelctrl_135_entity;



// global variable: Enable and send time stamp
/*
Example:
static bool brake_enable, gear_enable, park_enable, steer_enable, throttle_enable;
static int brake_prev_t=0, gear_prev_t=0, park_prev_t=0, steer_prev_t=0, throttle_prev_t=0;

protocols["name"].split("_",1)[0]_enable, ...
protocols["name"].split("_",1)[0]_prev_t=0, ...
*/
static bool a2v_drivectrl_130_enable, a2v_brakectrl_131_enable, a2v_steerctrl_132_enable, a2v_vehiclectrl_133_enable, a2v_wheelctrl_135_enable;
static int a2v_drivectrl_130_prev_t=0, a2v_brakectrl_131_prev_t=0, a2v_steerctrl_132_prev_t=0, a2v_vehiclectrl_133_prev_t=0, a2v_wheelctrl_135_prev_t=0;


static int time_diff = 100000000;
static ros::Publisher pub_can;


// define send canID callback function

static void a2v_drivectrl_130_callback(const pix_hooke_driver_msgs::a2v_drivectrl_130 &msg)
{
    
    a2v_drivectrl_130_entity.Reset();
    a2v_drivectrl_130_msg = msg;
    can_a2v_drivectrl_130.header.stamp = ros::Time::now();
    can_a2v_drivectrl_130.dlc = 8;
    a2v_drivectrl_130_entity.UpdateData(
        a2v_drivectrl_130_msg.ACU_ChassisDriverEnCtrl,
	a2v_drivectrl_130_msg.ACU_ChassisDriverModeCtrl,
	a2v_drivectrl_130_msg.ACU_ChassisGearCtrl,
	a2v_drivectrl_130_msg.ACU_ChassisSpeedCtrl,
	a2v_drivectrl_130_msg.ACU_ChassisThrottlePdlTarget,
	a2v_drivectrl_130_msg.ACU_DriveLifeSig,
	a2v_drivectrl_130_msg.ACU_CheckSum_130
    );
    can_a2v_drivectrl_130.id = a2v_drivectrl_130_entity.ID;
    can_a2v_drivectrl_130.is_extended= false;
    uint8_t *A;
    A = a2v_drivectrl_130_entity.get_data();
    for(unsigned int i=0;i<8;i++)
    {
        can_a2v_drivectrl_130.data[i] = *A;
        A += 1;
    }
    int t_nsec = 0;
    t_nsec = can_a2v_drivectrl_130.header.stamp.toNSec();
    a2v_drivectrl_130_prev_t = t_nsec;
}

    
static void a2v_brakectrl_131_callback(const pix_hooke_driver_msgs::a2v_brakectrl_131 &msg)
{
    
    a2v_brakectrl_131_entity.Reset();
    a2v_brakectrl_131_msg = msg;
    can_a2v_brakectrl_131.header.stamp = ros::Time::now();
    can_a2v_brakectrl_131.dlc = 8;
    a2v_brakectrl_131_entity.UpdateData(
        a2v_brakectrl_131_msg.ACU_ChassisBrakeEn,
	a2v_brakectrl_131_msg.ACU_ChassisAebCtrl,
	a2v_brakectrl_131_msg.ACU_ChassisBrakePdlTarget,
	a2v_brakectrl_131_msg.ACU_ChassisEpbCtrl,
	a2v_brakectrl_131_msg.ACU_BrakeLifeSig,
	a2v_brakectrl_131_msg.ACU_CheckSum_131
    );
    can_a2v_brakectrl_131.id = a2v_brakectrl_131_entity.ID;
    can_a2v_brakectrl_131.is_extended= false;
    uint8_t *A;
    A = a2v_brakectrl_131_entity.get_data();
    for(unsigned int i=0;i<8;i++)
    {
        can_a2v_brakectrl_131.data[i] = *A;
        A += 1;
    }
    int t_nsec = 0;
    t_nsec = can_a2v_brakectrl_131.header.stamp.toNSec();
    a2v_brakectrl_131_prev_t = t_nsec;
}

    
static void a2v_steerctrl_132_callback(const pix_hooke_driver_msgs::a2v_steerctrl_132 &msg)
{
    
    a2v_steerctrl_132_entity.Reset();
    a2v_steerctrl_132_msg = msg;
    can_a2v_steerctrl_132.header.stamp = ros::Time::now();
    can_a2v_steerctrl_132.dlc = 8;
    a2v_steerctrl_132_entity.UpdateData(
        a2v_steerctrl_132_msg.ACU_ChassisSteerEnCtrl,
	a2v_steerctrl_132_msg.ACU_ChassisSteerModeCtrl,
	a2v_steerctrl_132_msg.ACU_ChassisSteerAngleTarget,
	a2v_steerctrl_132_msg.ACU_ChassisSteerAngleRearTarget,
	a2v_steerctrl_132_msg.ACU_ChassisSteerAngleSpeedCtrl,
	a2v_steerctrl_132_msg.ACU_CheckSum_132
    );
    can_a2v_steerctrl_132.id = a2v_steerctrl_132_entity.ID;
    can_a2v_steerctrl_132.is_extended= false;
    uint8_t *A;
    A = a2v_steerctrl_132_entity.get_data();
    for(unsigned int i=0;i<8;i++)
    {
        can_a2v_steerctrl_132.data[i] = *A;
        A += 1;
    }
    int t_nsec = 0;
    t_nsec = can_a2v_steerctrl_132.header.stamp.toNSec();
    a2v_steerctrl_132_prev_t = t_nsec;
}

    
static void a2v_vehiclectrl_133_callback(const pix_hooke_driver_msgs::a2v_vehiclectrl_133 &msg)
{
    
    a2v_vehiclectrl_133_entity.Reset();
    a2v_vehiclectrl_133_msg = msg;
    can_a2v_vehiclectrl_133.header.stamp = ros::Time::now();
    can_a2v_vehiclectrl_133.dlc = 8;
    a2v_vehiclectrl_133_entity.UpdateData(
        a2v_vehiclectrl_133_msg.ACU_VehiclePosLampCtrl,
	a2v_vehiclectrl_133_msg.ACU_VehicleHeadLampCtrl,
	a2v_vehiclectrl_133_msg.ACU_VehicleLeftLampCtrl,
	a2v_vehiclectrl_133_msg.ACU_VehicleRightLampCtrl,
	a2v_vehiclectrl_133_msg.ACU_VehicleHighBeamCtrl,
	a2v_vehiclectrl_133_msg.ACU_VehicleFogLampCtrl,
	a2v_vehiclectrl_133_msg.ACU_VehicleBodyLightCtrl,
	a2v_vehiclectrl_133_msg.ACU_VehicleReadLightCtrl,
	a2v_vehiclectrl_133_msg.ACU_VehicleVoice,
	a2v_vehiclectrl_133_msg.ACU_VehicleWipersCtrl,
	a2v_vehiclectrl_133_msg.ACU_VehicleDoorCtrl,
	a2v_vehiclectrl_133_msg.ACU_VehicleWindowCtrl,
	a2v_vehiclectrl_133_msg.ACU_ChassisSpeedLimiteMode,
	a2v_vehiclectrl_133_msg.ACU_ChassisSpeedLimiteVal,
	a2v_vehiclectrl_133_msg.ACU_CheckSumEn
    );
    can_a2v_vehiclectrl_133.id = a2v_vehiclectrl_133_entity.ID;
    can_a2v_vehiclectrl_133.is_extended= false;
    uint8_t *A;
    A = a2v_vehiclectrl_133_entity.get_data();
    for(unsigned int i=0;i<8;i++)
    {
        can_a2v_vehiclectrl_133.data[i] = *A;
        A += 1;
    }
    int t_nsec = 0;
    t_nsec = can_a2v_vehiclectrl_133.header.stamp.toNSec();
    a2v_vehiclectrl_133_prev_t = t_nsec;
}

    
static void a2v_wheelctrl_135_callback(const pix_hooke_driver_msgs::a2v_wheelctrl_135 &msg)
{
    
    a2v_wheelctrl_135_entity.Reset();
    a2v_wheelctrl_135_msg = msg;
    can_a2v_wheelctrl_135.header.stamp = ros::Time::now();
    can_a2v_wheelctrl_135.dlc = 8;
    a2v_wheelctrl_135_entity.UpdateData(
        a2v_wheelctrl_135_msg.ACU_MotorTorqueLfCtrl,
	a2v_wheelctrl_135_msg.ACU_MotorTorqueRfCtrl,
	a2v_wheelctrl_135_msg.ACU_MotorTorqueLrCtrl,
	a2v_wheelctrl_135_msg.ACU_MotorTorqueRrCtrl
    );
    can_a2v_wheelctrl_135.id = a2v_wheelctrl_135_entity.ID;
    can_a2v_wheelctrl_135.is_extended= false;
    uint8_t *A;
    A = a2v_wheelctrl_135_entity.get_data();
    for(unsigned int i=0;i<8;i++)
    {
        can_a2v_wheelctrl_135.data[i] = *A;
        A += 1;
    }
    int t_nsec = 0;
    t_nsec = can_a2v_wheelctrl_135.header.stamp.toNSec();
    a2v_wheelctrl_135_prev_t = t_nsec;
}

    


//  define timer callback function
void timer_callback(const ros::TimerEvent &te)
{
    int now;
    now = ros::Time::now().toNSec();
    /*Example: 
    // brake
    if(now-brake_prev_t>time_diff)
    {
        for(uint i=0;i<8;i++)
        {   
            can_brake.id = brake_command.ID;
            can_brake.data[i] = 0;
        }
        pub_can.publish(can_brake);
    }
    else{
        pub_can.publish(can_brake);   
    }
    */
    
    // a2v_drivectrl_130
    if(now - a2v_drivectrl_130_prev_t>time_diff)
    {
        for(uint i=0; i<8; i++)
        {   
            can_a2v_drivectrl_130.id = a2v_drivectrl_130_entity.ID;
            can_a2v_drivectrl_130.data[i] = 0;
        }
        pub_can.publish(can_a2v_drivectrl_130);
    }
    else{
        pub_can.publish(can_a2v_drivectrl_130);
    }
    
    // a2v_brakectrl_131
    if(now - a2v_brakectrl_131_prev_t>time_diff)
    {
        for(uint i=0; i<8; i++)
        {   
            can_a2v_brakectrl_131.id = a2v_brakectrl_131_entity.ID;
            can_a2v_brakectrl_131.data[i] = 0;
        }
        pub_can.publish(can_a2v_brakectrl_131);
    }
    else{
        pub_can.publish(can_a2v_brakectrl_131);
    }
    
    // a2v_steerctrl_132
    if(now - a2v_steerctrl_132_prev_t>time_diff)
    {
        for(uint i=0; i<8; i++)
        {   
            can_a2v_steerctrl_132.id = a2v_steerctrl_132_entity.ID;
            can_a2v_steerctrl_132.data[i] = 0;
        }
        pub_can.publish(can_a2v_steerctrl_132);
    }
    else{
        pub_can.publish(can_a2v_steerctrl_132);
    }
    
    // a2v_vehiclectrl_133
    if(now - a2v_vehiclectrl_133_prev_t>time_diff)
    {
        for(uint i=0; i<8; i++)
        {   
            can_a2v_vehiclectrl_133.id = a2v_vehiclectrl_133_entity.ID;
            can_a2v_vehiclectrl_133.data[i] = 0;
        }
        pub_can.publish(can_a2v_vehiclectrl_133);
    }
    else{
        pub_can.publish(can_a2v_vehiclectrl_133);
    }
    
    // a2v_wheelctrl_135
    if(now - a2v_wheelctrl_135_prev_t>time_diff)
    {
        for(uint i=0; i<8; i++)
        {   
            can_a2v_wheelctrl_135.id = a2v_wheelctrl_135_entity.ID;
            can_a2v_wheelctrl_135.data[i] = 0;
        }
        pub_can.publish(can_a2v_wheelctrl_135);
    }
    else{
        pub_can.publish(can_a2v_wheelctrl_135);
    }
    
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pix_hooke_driver_command_node");
    ros::NodeHandle nh;

    // creat ros Subscriber
    // Example: ros::Subscriber sub_brake = nh.subscribe("/pix/brake_command", 1, brake_callback);
    ros::Subscriber sub_a2v_drivectrl_130 = nh.subscribe("/pix_hooke/a2v_drivectrl_130", 1, a2v_drivectrl_130_callback);
	ros::Subscriber sub_a2v_brakectrl_131 = nh.subscribe("/pix_hooke/a2v_brakectrl_131", 1, a2v_brakectrl_131_callback);
	ros::Subscriber sub_a2v_steerctrl_132 = nh.subscribe("/pix_hooke/a2v_steerctrl_132", 1, a2v_steerctrl_132_callback);
	ros::Subscriber sub_a2v_vehiclectrl_133 = nh.subscribe("/pix_hooke/a2v_vehiclectrl_133", 1, a2v_vehiclectrl_133_callback);
	ros::Subscriber sub_a2v_wheelctrl_135 = nh.subscribe("/pix_hooke/a2v_wheelctrl_135", 1, a2v_wheelctrl_135_callback);
	
    //  creat ros publisher
    pub_can = nh.advertise<can_msgs::Frame>("/sent_messages", 10, false);

    ros::Timer set_speed = nh.createTimer(ros::Duration(1/50.0), timer_callback);
    ros::spin();

    return 0;

}