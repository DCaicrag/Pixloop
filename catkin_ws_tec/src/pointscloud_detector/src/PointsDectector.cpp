#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

#include <std_msgs/Header.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>


#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/Detection3D.h>
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/ObjectHypothesisWithPose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <image_geometry/pinhole_camera_model.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
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


class PointsDectector
{
private:


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::PointCloud2, vision_msgs::Detection2DArray> SyncPolicy;
    message_filters::Subscriber<sensor_msgs::CameraInfo> cam_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub;
    message_filters::Subscriber<vision_msgs::Detection2DArray> det_sub;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    float _cluster_tolerance = 0.5;
    int _min_cluster_size = 50;
    int _max_cluster_size =  25000;

    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    ros::Publisher _detection3d_pub;
    ros::Publisher _marker_pub;

    ros::Timer timer_;

    image_geometry::PinholeCameraModel _cam_model;

    pcl::PointCloud<pcl::PointXYZ> msg2TransformedCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    std::tuple<vision_msgs::Detection3DArray, sensor_msgs::PointCloud2> projectCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, const vision_msgs::Detection2DArrayConstPtr& detections2d_msg, const std_msgs::Header& heade);
    pcl::PointCloud<pcl::PointXYZ> cloud2TransformedCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, const std_msgs::Header& header);
    pcl::PointCloud<pcl::PointXYZ> euclideanClusterExtraction(const pcl::PointCloud<pcl::PointXYZ>& cloud);
    void createBoundingBox(vision_msgs::Detection3DArray& detections3d_msg, const pcl::PointCloud<pcl::PointXYZ>& cloud,
                            const std::vector<vision_msgs::ObjectHypothesisWithPose>& detections_results);
    // Timer callback function
    void timerCallback(const ros::TimerEvent&);
    void callback_sync(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const vision_msgs::Detection2DArrayConstPtr& detections2d_msg);
    visualization_msgs::MarkerArray createMarkerArray(const vision_msgs::Detection3DArray& detections3d_msg);


public:
    PointsDectector(/* args */);
    ~PointsDectector();
    void initialize();
};

PointsDectector::PointsDectector(/* args */)
{


    // cam_sub.subscribe(nh_, "/flir_adk/camera_info", 10);
    // cam_sub.subscribe(nh_, "/zed/zed_node/right/camera_info", 1);
    cam_sub.subscribe(nh_, "/zed/zed_node/right/camera_info_throttled", 1);

    lidar_sub.subscribe(nh_, "/points_raw", 1);
    det_sub.subscribe(nh_, "/detections", 1);

    sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), cam_sub, lidar_sub, det_sub);


    sync->registerCallback(boost::bind(&PointsDectector::callback_sync, this, _1, _2, _3));
    _detection3d_pub = nh_.advertise<vision_msgs::Detection3DArray>("detection3d_result", 1);
    _marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("detection_marker", 1);
    
    publisher_ = nh_.advertise<std_msgs::String>("chatter", 1000);
    timer_ = nh_.createTimer(ros::Duration(1.0), &PointsDectector::timerCallback, this);
    ROS_INFO("node created df");

}

PointsDectector::~PointsDectector()
{
}

void PointsDectector::initialize()
{
  tf_buffer_.reset(new tf2_ros::Buffer(ros::Duration(2.0), true));
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
}

void PointsDectector::timerCallback(const ros::TimerEvent&) {
    std_msgs::String msg;

    msg.data = "Hello, World!";

    publisher_.publish(msg);

    ROS_INFO("%s", msg.data.c_str());
}


void PointsDectector::callback_sync(const sensor_msgs::CameraInfo::ConstPtr& cam,
                                        const sensor_msgs::PointCloud2ConstPtr& cloud,
                                        const vision_msgs::Detection2DArrayConstPtr& detections)
{
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    visualization_msgs::MarkerArray marker_array_msg;

    cameraMatrix = (cv::Mat1d(3, 3) << cam->K[0], cam->K[1], cam->K[2],
                                       cam->K[3], cam->K[4], cam->K[5],
                                       cam->K[6], cam->K[7], cam->K[8]);

    distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
    _cam_model.fromCameraInfo(cam);
    transformed_cloud = msg2TransformedCloud(cloud);
    auto [detections3d_msg, detection_cloud_msg] = projectCloud(transformed_cloud, detections, cloud->header);
    marker_array_msg = createMarkerArray(detections3d_msg);
    _detection3d_pub.publish(detections3d_msg);
    _marker_pub.publish(marker_array_msg);
    // ROS_INFO("matching");
}

pcl::PointCloud<pcl::PointXYZ> PointsDectector::msg2TransformedCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;

    geometry_msgs::TransformStamped tf;

    std::string target_frame = _cam_model.tfFrame();
    std::string source_frame = cloud_msg->header.frame_id;
    
    // ROS_INFO("Source frame: %s, Target frame: %s", source_frame.c_str(), target_frame.c_str());

    // Remove leading '/' if present
    if (!target_frame.empty() && target_frame[0] == '/') {
        target_frame.erase(0, 1);
    }
    if (!source_frame.empty() && source_frame[0] == '/') {
        source_frame.erase(0, 1);
}
    try
    {
        tf = tf_buffer_->lookupTransform(target_frame, source_frame, cloud_msg->header.stamp);
        pcl::fromROSMsg(*cloud_msg, cloud);
        Eigen::Affine3d transform = tf2::transformToEigen(tf.transform);
        pcl::transformPointCloud(cloud, transformed_cloud, transform);
    }
    catch (tf2::TransformException& e)
    {
        ROS_WARN("%s", e.what());
    }
    return transformed_cloud;
}

pcl::PointCloud<pcl::PointXYZ> PointsDectector::cloud2TransformedCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                                                            const std_msgs::Header& header)
{
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    geometry_msgs::TransformStamped tf;
    
    std::string target_frame = _cam_model.tfFrame();
    std::string source_frame = header.frame_id;

    // Remove leading '/' if present
    if (!target_frame.empty() && target_frame[0] == '/') {
        target_frame.erase(0, 1);
    }
    if (!source_frame.empty() && source_frame[0] == '/') {
        source_frame.erase(0, 1);
    }

    try
    {   if (!target_frame.empty() && !source_frame.empty()){
            tf = tf_buffer_->lookupTransform(target_frame, source_frame, header.stamp);
            Eigen::Affine3d transform = tf2::transformToEigen(tf.transform);
            pcl::transformPointCloud(cloud, transformed_cloud, transform);
        }

    }
    catch (tf2::TransformException& e)
    {
        ROS_WARN("%s", e.what());
    }
    return transformed_cloud;
}

pcl::PointCloud<pcl::PointXYZ>
PointsDectector::euclideanClusterExtraction(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  pcl::PointCloud<pcl::PointXYZ> closest_cluster;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  std::vector<pcl::PointIndices> cluster_indices;
  float min_distance = std::numeric_limits<float>::max();
  tree->setInputCloud(cloud.makeShared());
  ec.setInputCloud(cloud.makeShared());
  ec.setClusterTolerance(_cluster_tolerance);
  ec.setMinClusterSize(_min_cluster_size);
  ec.setMaxClusterSize(_max_cluster_size);
  ec.setSearchMethod(tree);
  ec.extract(cluster_indices);
  for (const auto& cluster_indice : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
    Eigen::Vector4f centroid;
    for (int indice : cluster_indice.indices)
    {
      cloud_cluster.points.push_back(cloud.points[indice]);
    }
    pcl::compute3DCentroid(cloud_cluster, centroid);
    float distance = centroid.norm();
    if (distance < min_distance)
    {
      min_distance = distance;
      closest_cluster = cloud_cluster;
    }
  }
  return closest_cluster;
}

void PointsDectector::createBoundingBox(
    vision_msgs::Detection3DArray& detections3d_msg, const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const std::vector<vision_msgs::ObjectHypothesisWithPose>& detections_results)
{
  vision_msgs::Detection3D detection3d;
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
  pcl::PointXYZ min_pt, max_pt;
  Eigen::Vector4f centroid;
  Eigen::Vector4f bbox_center;
  Eigen::Vector4f transformed_bbox_center;
  Eigen::Affine3f transform;
  pcl::compute3DCentroid(cloud, centroid);
  double theta = -atan2(centroid[1], sqrt(pow(centroid[0], 2) + pow(centroid[2], 2)));
  transform = Eigen::Affine3f::Identity();
  transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
  pcl::transformPointCloud(cloud, transformed_cloud, transform);
  pcl::getMinMax3D(transformed_cloud, min_pt, max_pt);
  transformed_bbox_center =
      Eigen::Vector4f((min_pt.x + max_pt.x) / 2, (min_pt.y + max_pt.y) / 2, (min_pt.z + max_pt.z) / 2, 1);
  bbox_center = transform.inverse() * transformed_bbox_center;
  detection3d.bbox.center.position.x = bbox_center[0];
  detection3d.bbox.center.position.y = bbox_center[1];
  detection3d.bbox.center.position.z = bbox_center[2];
  Eigen::Quaternionf q(transform.inverse().rotation());
  detection3d.bbox.center.orientation.x = q.x();
  detection3d.bbox.center.orientation.y = q.y();
  detection3d.bbox.center.orientation.z = q.z();
  detection3d.bbox.center.orientation.w = q.w();
  detection3d.bbox.size.x = max_pt.x - min_pt.x;
  detection3d.bbox.size.y = max_pt.y - min_pt.y;
  detection3d.bbox.size.z = max_pt.z - min_pt.z;
  detection3d.results = detections_results;
  detections3d_msg.detections.push_back(detection3d);
}

std::tuple<vision_msgs::Detection3DArray, sensor_msgs::PointCloud2>
PointsDectector::projectCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                   const vision_msgs::Detection2DArrayConstPtr& detections2d_msg,
                                   const std_msgs::Header& header)
{
  pcl::PointCloud<pcl::PointXYZ> detection_cloud_raw;
  pcl::PointCloud<pcl::PointXYZ> detection_cloud;
  pcl::PointCloud<pcl::PointXYZ> closest_detection_cloud;
  pcl::PointCloud<pcl::PointXYZ> combine_detection_cloud;
  vision_msgs::Detection3DArray detections3d_msg;
  sensor_msgs::PointCloud2 combine_detection_cloud_msg;
  detections3d_msg.header = header;
  for (const auto& detection : detections2d_msg->detections)
  {
    ROS_INFO("detections: %ld", detection.results[0].id);
    for (const auto& point : cloud.points)
    {
      cv::Point3d pt_cv(point.x, point.y, point.z);
      cv::Point2d uv = _cam_model.project3dToPixel(pt_cv);
      if (point.z > 0 && uv.x > 0 && uv.x >= detection.bbox.center.x - detection.bbox.size_x / 2 &&
          uv.x <= detection.bbox.center.x + detection.bbox.size_x / 2 &&
          uv.y >= detection.bbox.center.y - detection.bbox.size_y / 2 &&
          uv.y <= detection.bbox.center.y + detection.bbox.size_y / 2)
      {
        detection_cloud_raw.points.push_back(point);
        

      }
    }
    detection_cloud = cloud2TransformedCloud(detection_cloud_raw, header);
    if (!detection_cloud.points.empty())
    {
      


      closest_detection_cloud = euclideanClusterExtraction(detection_cloud);
      createBoundingBox(detections3d_msg, closest_detection_cloud, detection.results);
      combine_detection_cloud.insert(combine_detection_cloud.end(), closest_detection_cloud.begin(),
                                     closest_detection_cloud.end());
      detection_cloud_raw.points.clear();

      ROS_INFO("detections: %lu", detections3d_msg.detections.size());
    }
  }
  pcl::toROSMsg(combine_detection_cloud, combine_detection_cloud_msg);
  combine_detection_cloud_msg.header = header;
  return std::forward_as_tuple(detections3d_msg, combine_detection_cloud_msg);
}

visualization_msgs::MarkerArray PointsDectector::createMarkerArray(const vision_msgs::Detection3DArray& detections3d_msg)
{
    visualization_msgs::MarkerArray marker_array;

    for(size_t i = 0; i < detections3d_msg.detections.size(); ++i)
    {
        if (std::isfinite(detections3d_msg.detections[i].bbox.size.x) &&
        std::isfinite(detections3d_msg.detections[i].bbox.size.y) &&
        std::isfinite(detections3d_msg.detections[i].bbox.size.z))
        {
            const auto& detection = detections3d_msg.detections[i];
            const auto& bbox = detection.bbox;

            visualization_msgs::Marker marker;
            marker.header.frame_id = "velodyne";
            marker.header.stamp = detections3d_msg.header.stamp;
            marker.ns = "bounding_boxes";
            marker.id = i;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position = bbox.center.position;
            marker.pose.orientation = bbox.center.orientation;

            marker.scale.x = bbox.size.x;
            marker.scale.y = bbox.size.y;
            marker.scale.z = bbox.size.z;

          
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.5;

            marker_array.markers.push_back(marker);
        }  
    }

    return marker_array;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "PointsDectector");
    PointsDectector PointsDectector;
    PointsDectector.initialize();
    ros::spin();

    return 0;
}