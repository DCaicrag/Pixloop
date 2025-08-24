#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/Detection3D.h>
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/ObjectHypothesisWithPose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Point.h>
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
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
// C++
#include <iostream>
#include <vector>
#include <algorithm>

// Eigen
#include <Eigen/Dense>

// OpenCV
#include <opencv2/opencv.hpp>


#include "obstacle_detector.hpp"

using namespace std;

struct BBox
{
    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float z_min;
    float z_max;
    double r = 1.0;
    double g = 1.0;
    double b = 0.0;
};


struct Point {
    float x, y;
};


struct Zone {
    std::vector<Point> corners; // Four corners of the zone

    Zone(const Point& p1, const Point& p2, const Point& p3, const Point& p4) {
        corners.push_back(p1);
        corners.push_back(p2);
        corners.push_back(p3);
        corners.push_back(p4);
    }

    bool contains(const pcl::PointXYZ& point) const {
        int count = 0;
        size_t n = corners.size();

        for (size_t i = 0; i < n; i++) {
            size_t j = (i + 1) % n;
            if (isIntersecting(point, corners[i], corners[j])) {
                count++;
            }
        }

        return count % 2 == 1; // Inside if odd number of intersections
    }

private:
    bool isIntersecting(const pcl::PointXYZ& p_orig, const Point& p1, const Point& p2) const {
        Point p = {p_orig.x, p_orig.y};

        if (p.y == p1.y || p.y == p2.y) {
            p.y += 0.0001; 
        }

        if (p.y < std::min(p1.y, p2.y) || p.y > std::max(p1.y, p2.y)) {
            return false;
        }

        if (p.x > std::max(p1.x, p2.x)) {
            return true;
        }

        if (p.x < std::min(p1.x, p2.x)) {
            return false;
        }

        double slope = (p2.y - p1.y) / (p2.x - p1.x);
        double y_intercept = p1.y - slope * p1.x;
        double intersect_x = (p.y - y_intercept) / slope;

        return p.x < intersect_x;
    }
};

class ObjectDetection
{
private:
// variables
    float GROUND_THRESHOLD = 0.03;
    float CLUSTER_THRESH = 0.18;
    int CLUSTER_MAX_SIZE = 5000;
    int CLUSTER_MIN_SIZE = 10;
    size_t obstacle_id_;

    bool USE_PCA_BOX = false;
    float DISPLACEMENT_THRESH = 1.0;
    float IOU_THRESH = 1.1;
    bool USE_TRACKING = true;
    std::vector<Box> curr_boxes_; 
    std::vector<Box> prev_boxes_;


    Zone front_zone;
    Zone front_zone_warning;

    std_msgs::Int32 message_int_warning_value;



    std::shared_ptr<lidar_obstacle_detector::ObstacleDetector<pcl::PointXYZ>> obstacle_detector;
    ros::Subscriber sub_;
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    ros::Publisher wall_warning;
    ros::Publisher zone_publisher_;
    ros::Publisher publisher_warning_;



    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void publisherboxes(std::vector<BBox>&& bboxes, const std_msgs::Header& header);
    void box3dcreation(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters, const std_msgs::Header& header);
    void check_zones_all_points_version2(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters);
    void warning_display(const int warning_code);
    void publish_zone(const Zone& zone1, const Zone& zone2);
    visualization_msgs::Marker create_zone_marker(const Zone& zone, int id, const std::string& color);


public:
    ObjectDetection(/* args */);
    ~ObjectDetection();
};

ObjectDetection::ObjectDetection(/* args */): front_zone(Point{1.19,-1.12}, Point{2.69,-1.12}, Point{2.69, 0.85}, Point{1.19, 0.85}), front_zone_warning(Point{2.72,-1.12}, Point{4.0, -1.12}, Point{4.0,0.85}, Point{2.72,0.85})
{
    sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/point_cloud", 10, &ObjectDetection::pointCloudCallback, this);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
    wall_warning = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    zone_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_zone_array", 10);
    publisher_warning_ =  nh_.advertise<std_msgs::Int32>("warning_status", 10);


}

ObjectDetection::~ObjectDetection()
{
}

void ObjectDetection::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {



    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *input_cloud);
    if (input_cloud->empty()) {

        std::cout << "Received empty point cloud" << std::endl;

        return;
    }

    try {

        auto cloud_clusters = obstacle_detector->clustering(input_cloud, CLUSTER_THRESH, CLUSTER_MIN_SIZE, CLUSTER_MAX_SIZE);
        if (!cloud_clusters.empty()) {
            box3dcreation(std::move(cloud_clusters), msg->header);
            check_zones_all_points_version2(std::move(cloud_clusters));
            publish_zone(front_zone, front_zone_warning);


            // std::cout << "Number of clustersasdasd: " << cloud_clusters.size() << std::endl;
        }

    } catch (const std::exception& e) {
        ROS_ERROR("Error processing point cloud: %s", e.what());
    }

}

void ObjectDetection::box3dcreation(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters, const std_msgs::Header& header)
{

    std::vector<BBox> bboxes;
    
    int num_reasonable_clusters = 0;
    
    for (auto& cluster : cloud_clusters)
    {

        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D<pcl::PointXYZ>(*cluster, min_pt, max_pt);

        BBox bbox;
        bbox.x_min = min_pt[0];
        bbox.y_min = min_pt[1];
        bbox.z_min = min_pt[2];
        bbox.x_max = max_pt[0];
        bbox.y_max = max_pt[1];
        bbox.z_max = max_pt[2];

        bboxes.push_back(bbox);
        num_reasonable_clusters++;

    }

    publisherboxes(std::move(bboxes), header);

}

void ObjectDetection::publisherboxes(std::vector<BBox>&& bboxes, const std_msgs::Header& header)
 {
    visualization_msgs::MarkerArray marker_array;
    int id = 0;
    const std_msgs::Header& inp_header = header;

    // Rest of your code remains the same with necessary adjustments
    // Replace `visualization_msgs::Marker` with `visualization_msgs::Marker`
    // Replace `geometry_msgs::Point` with `geometry_msgs::Point`
    // ...
    for (const auto& bbox : bboxes)
    {
        // Create the marker for the top square
        visualization_msgs::Marker top_square_marker;
        top_square_marker.header = inp_header;
        top_square_marker.ns = "bounding_boxes";
        top_square_marker.id = id++;
        top_square_marker.type = visualization_msgs::Marker::LINE_STRIP;
        top_square_marker.action = visualization_msgs::Marker::ADD;
        top_square_marker.pose.orientation.w = 1.0;
        top_square_marker.scale.x = 0.06;
        top_square_marker.color.r = bbox.r;
        top_square_marker.color.g = bbox.g;
        top_square_marker.color.b = bbox.b;
        top_square_marker.color.a = 1.0;

        // Add the points to the top square marker
        geometry_msgs::Point p1, p2, p3, p4;
        p1.x = bbox.x_max; p1.y = bbox.y_max; p1.z = bbox.z_max;
        p2.x = bbox.x_min; p2.y = bbox.y_max; p2.z = bbox.z_max;
        p3.x = bbox.x_min; p3.y = bbox.y_min; p3.z = bbox.z_max;
        p4.x = bbox.x_max; p4.y = bbox.y_min; p4.z = bbox.z_max;
        top_square_marker.points.push_back(p1);
        top_square_marker.points.push_back(p2);
        top_square_marker.points.push_back(p3);
        top_square_marker.points.push_back(p4);
        top_square_marker.points.push_back(p1);

        // Add the top square marker to the array
        marker_array.markers.push_back(top_square_marker);

        // Create the marker for the bottom square
        visualization_msgs::Marker bottom_square_marker;
        bottom_square_marker.header = inp_header;
        bottom_square_marker.ns = "bounding_boxes";
        bottom_square_marker.id = id++;
        bottom_square_marker.type = visualization_msgs::Marker::LINE_STRIP;
        bottom_square_marker.action = visualization_msgs::Marker::ADD;
        bottom_square_marker.pose.orientation.w = 1.0;
        bottom_square_marker.scale.x = 0.04;
        bottom_square_marker.color.r = bbox.r;
        bottom_square_marker.color.g = bbox.g;
        bottom_square_marker.color.b = bbox.b;
        bottom_square_marker.color.a = 1.0;

        // Add the points to the bottom square marker
        geometry_msgs::Point p5, p6, p7, p8;
        p5.x = bbox.x_max; p5.y = bbox.y_max; p5.z = bbox.z_min;
        p6.x = bbox.x_min; p6.y = bbox.y_max; p6.z = bbox.z_min;
        p7.x = bbox.x_min; p7.y = bbox.y_min; p7.z = bbox.z_min;
        p8.x = bbox.x_max; p8.y = bbox.y_min; p8.z = bbox.z_min;

        bottom_square_marker.points.push_back(p5);
        bottom_square_marker.points.push_back(p6);
        bottom_square_marker.points.push_back(p7);
        bottom_square_marker.points.push_back(p8);
        bottom_square_marker.points.push_back(p5); // connect the last point to the first point to close the square

        // Add the bottom square marker to the marker array
        marker_array.markers.push_back(bottom_square_marker);


        // Create the marker for the lines connecting the top and bottom squares
        visualization_msgs::Marker connecting_lines_marker;
        connecting_lines_marker.header = inp_header;
        connecting_lines_marker.ns = "bounding_boxes";
        connecting_lines_marker.id = id++;
        connecting_lines_marker.type = visualization_msgs::Marker::LINE_LIST;
        connecting_lines_marker.action = visualization_msgs::Marker::ADD;
        connecting_lines_marker.pose.orientation.w = 1.0;
        connecting_lines_marker.scale.x = 0.04;
        connecting_lines_marker.color.r = 1.0;
        connecting_lines_marker.color.g = 0.0;
        connecting_lines_marker.color.b = 0.0;
        connecting_lines_marker.color.a = 1.0;

        // Add the points to the connecting lines marker
        connecting_lines_marker.points.push_back(p1);
        connecting_lines_marker.points.push_back(p5);

        connecting_lines_marker.points.push_back(p2);
        connecting_lines_marker.points.push_back(p6);

        connecting_lines_marker.points.push_back(p3);
        connecting_lines_marker.points.push_back(p7);

        connecting_lines_marker.points.push_back(p4);
        connecting_lines_marker.points.push_back(p8);

        // Add the connecting lines marker to the marker array
        marker_array.markers.push_back(connecting_lines_marker);


        // Create a marker for the corners
        visualization_msgs::Marker corner_marker;
        corner_marker.header = inp_header;
        corner_marker.ns = "bounding_boxes";
        corner_marker.id = id++;
        corner_marker.type = visualization_msgs::Marker::SPHERE;
        corner_marker.action = visualization_msgs::Marker::ADD;
        corner_marker.pose.orientation.w = 1.0;
        corner_marker.scale.x = 0.2;
        corner_marker.scale.y = 0.2;
        corner_marker.scale.z = 0.2;
        corner_marker.color.r = bbox.r;
        corner_marker.color.g = 0.2;
        corner_marker.color.b = 0.5;
        corner_marker.color.a = 0.64;

        // Create a sphere for each corner and add it to the marker array

        corner_marker.pose.position = p1;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p2;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p3;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p4;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p5;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p6;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p7;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p8;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);
    }

    ros::Duration marker_lifetime(3.0);


    for (auto& marker : marker_array.markers) {
        marker.lifetime = marker_lifetime;
    }
    marker_pub_.publish(marker_array);
}


void ObjectDetection::check_zones_all_points_version2(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters) 
{

   int highest_warning_code = 4; // To store the highest severity warning code

    for (const auto& cluster : cloud_clusters) 
    {
        if (cluster->empty()) continue;

        bool zone_checked = false; // Flag to break out of the loop once a point in a zone is found

        for (const auto& point : cluster->points) {


            if (front_zone.contains(point)) {
                highest_warning_code = std::min(highest_warning_code, 1); // Obstacle detected in red zone
                zone_checked = true;
                // ROS_INFO("[WARNING 220] Obstacle detected in red zone.");
                break; // No need to check further points in this cluster
            } else if (front_zone_warning.contains(point)) {
                highest_warning_code = std::min(highest_warning_code, 2); // Obstacle detected in yellow zone
                zone_checked = true;
                // ROS_INFO("[WARNING 330] Obstacle detected in yellow zone.");
                break; // No need to check further points in this cluster
            }


        }

        if (zone_checked) {
            // Optional: Log information about the detected point in the zone
            continue; // Move to the next cluster
        }
    }

    if (highest_warning_code < 4) {
        warning_display(highest_warning_code);

        message_int_warning_value.data = highest_warning_code;
        publisher_warning_.publish(message_int_warning_value);

    } else {

        // ROS_INFO("[INFO 000] No obstacle detected in any zone.");

        warning_display(4);

        message_int_warning_value.data = 4;
        publisher_warning_.publish(message_int_warning_value);
    }


}

void ObjectDetection::warning_display(const int warning_code)
{
    visualization_msgs::Marker wall_marker;
    wall_marker.header.frame_id = "detector_box";
    wall_marker.header.stamp = ros::Time::now();
    wall_marker.ns = "wall";
    wall_marker.id = 0;
    wall_marker.type = visualization_msgs::Marker::CUBE;
    wall_marker.action = visualization_msgs::Marker::ADD;
    wall_marker.pose.position.x = 0.0;
    wall_marker.pose.position.y = 0.0;
    wall_marker.pose.position.z = 0.0;
    wall_marker.pose.orientation.x = 0.0;
    wall_marker.pose.orientation.y = 0.0;
    wall_marker.pose.orientation.z = 0.0;
    wall_marker.pose.orientation.w = 1.0;
    wall_marker.scale.x = 0.25;
    wall_marker.scale.y = 0.25;
    wall_marker.scale.z = 0.25;
    wall_marker.color.a = 0.7;

    switch (warning_code) {
        case 1:
            wall_marker.color.r = 1.0;
            wall_marker.color.g = 0.0;
            wall_marker.color.b = 0.0;
            break;
        case 2:
            wall_marker.color.r = 1.0;
            wall_marker.color.g = 1.0;
            wall_marker.color.b = 0.0;
            break;
        case 3:
            wall_marker.color.r = 0.0;
            wall_marker.color.g = 1.0;
            wall_marker.color.b = 0.0;
            break;
        default:
            wall_marker.color.r = 0.0;
            wall_marker.color.g = 1.0;
            wall_marker.color.b = 0.0;
    }

    wall_warning.publish(wall_marker);
}

void ObjectDetection::publish_zone(const Zone& zone1, const Zone& zone2) {
    visualization_msgs::MarkerArray marker_array;

    // Create and add markers for each zone
    visualization_msgs::Marker marker1 = create_zone_marker(zone1, 1000, "red");
    visualization_msgs::Marker marker2 = create_zone_marker(zone2, 1001, "yellow");

    marker_array.markers.push_back(marker1);
    marker_array.markers.push_back(marker2);

    // Publish the marker array
    zone_publisher_.publish(marker_array);
}

visualization_msgs::Marker ObjectDetection::create_zone_marker(const Zone& zone, int id, const std::string& color) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "zone_detector"; // Or your relevant frame
    marker.header.stamp = ros::Time::now();
    marker.ns = "zone";
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05; // Line width
    marker.color.a = 1.0; // Alpha (opacity)

    if (color == "yellow") {
        marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0; // Yellow
    } else {
        marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; // Red (default)
    }

    for (const auto& corner : zone.corners) {
        geometry_msgs::Point p;
        p.x = corner.x; p.y = corner.y; p.z = 0;
        marker.points.push_back(p);
    }
    marker.points.push_back(marker.points.front()); // Close the loop

    return marker;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_detection");
    ObjectDetection ObjectDetection;
    ros::spin();

    return 0;
}
