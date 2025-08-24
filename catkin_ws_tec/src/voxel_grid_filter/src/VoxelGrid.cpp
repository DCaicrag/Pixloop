#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include "std_msgs/String.h"

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/crop_box.h>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

// C++
#include <iostream>
#include <vector>
#include <algorithm>

// Eigen
#include <Eigen/Dense>

// OpenCV
#include <opencv2/opencv.hpp>

using namespace std;

class VoxelGrid
{
private:
    // voxelgrid resolution
    float voxel_leaf_size_x_ = 0.1;
    float voxel_leaf_size_y_ = 0.1;
    float voxel_leaf_size_z_ = 0.1; 
    using PointCloudMsg = sensor_msgs::PointCloud2;
    using PointCloudMsg2 = sensor_msgs::PointCloud2;

    // ROI boundaries
    double roi_max_x_ = 7.5; //FRONT THE CAR
    double roi_max_y_ = 2.5;  //LEFT THE CAR
    double roi_max_z_ = 0.2; //UP THE VELODYNE

    double roi_min_x_ = -0.1; //BACK THE CAR
    double roi_min_y_ = -2.5; //RIGHT THE CAR
    double roi_min_z_ = -1.5; //DOWN THE VELODYNE

    Eigen::Vector4f ROI_MAX_POINT, ROI_MIN_POINT;

    ros::NodeHandle nh_;
    ros::Publisher pub2_;

    ros::Subscriber sub_;

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);


public:
    VoxelGrid(/* args */);
    ~VoxelGrid();
};

VoxelGrid::VoxelGrid(/* args */)
{


    // cam_sub.subscribe(nh_, "/flir_adk/camera_info", 10);
    // cam_sub.subscribe(nh_, "/zed/zed_node/right/camera_info", 1)
    sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/points_raw", 10, &VoxelGrid::pointCloudCallback, this);
    pub2_ = nh_.advertise<sensor_msgs::PointCloud2>("point_cloud", 1000);


    ROI_MAX_POINT = Eigen::Vector4f(roi_max_x_, roi_max_y_, roi_max_z_, 1);
    ROI_MIN_POINT = Eigen::Vector4f(roi_min_x_, roi_min_y_, roi_min_z_, 1);

    ROS_INFO("voxel grid filter node ready");


}

VoxelGrid::~VoxelGrid()
{
}



void VoxelGrid::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*msg, *input_cloud);

    // Apply ROI filtering
    pcl::CropBox<pcl::PointXYZI> roi_filter;
    roi_filter.setInputCloud(input_cloud);
    roi_filter.setMax(ROI_MAX_POINT);
    roi_filter.setMin(ROI_MIN_POINT);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZI>);
    roi_filter.filter(*cloud_roi);

    VoxelGrid::PointCloudMsg2 downsampled_cloud_msg_rio;
    pcl::toROSMsg(*cloud_roi, downsampled_cloud_msg_rio);
    pub2_.publish(downsampled_cloud_msg_rio);

    // // create voxel grid object
    // pcl::VoxelGrid<pcl::PointXYZI> vg;
    // vg.setInputCloud(cloud_roi);
    // vg.setLeafSize(voxel_leaf_size_x_, voxel_leaf_size_y_, voxel_leaf_size_z_);

    // pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // vg.filter(*filtered_cloud);

 

}


int main(int argc, char** argv) {
    ros::init(argc, argv, "voxelgrid");
    VoxelGrid VoxelGrid;
    ros::spin();

    return 0;
}