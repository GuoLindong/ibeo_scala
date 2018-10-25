//
// Created by guolindong on 18-10-12.
//

#ifndef PROJECT_SCALA_RVIZ_DISPLAY_H
#define PROJECT_SCALA_RVIZ_DISPLAY_H


#define Pi (3.14159265)

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_rviz_plugins/OverlayText.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/PolygonMesh.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>

#include <tiggo_msgs/Heading.h>
#include <ibeo_scala/ObjectArray.h>
#include <ibeo_scala/Object.h>

class ScalaRvizDisplay{


public:
    void Init();
    void PoseCallback(const geometry_msgs::PoseStamped &msg);
    void PointCloudCallback(const sensor_msgs::PointCloud2& msg);
    void ObjectsCallback(const ibeo_scala::ObjectArray& msg);
    void GpsCallback(const sensor_msgs::NavSatFix& msg);
    void HeadingCallback(const tiggo_msgs::Heading& msg);
    void VelocityCallback(const geometry_msgs::TwistWithCovarianceStamped& msg);
    void Gps2Meter(double lon, double lat, double &out_x, double &out_y);
    template <typename T> std::vector<T> EulerToQuaternion(T roll, T pitch, T yaw);
    template <typename T> std::vector<T> QuaternionToEuler(std::vector<T> q);
    template <typename T> void SetPosition(geometry_msgs::Pose &pose, T x, T y, T z);
    template <typename T> void SetQuaternion(geometry_msgs::Pose &pose, std::vector<T> q);
    double GetDistance(double x1, double y1, double x2, double y2);

    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;

    tf::StampedTransform transform_map;
    ros::NodeHandle nh;

    ros::Subscriber sub_pointcloud;
    ros::Subscriber sub_dynamic_objects;
    ros::Subscriber sub_fix;
    ros::Subscriber sub_heading;
    ros::Subscriber sub_velocity;
    ros::Subscriber sub_pose;

    ros::Publisher pub_track;
    ros::Publisher pub_trajectory;
    ros::Publisher pub_pose;
    ros::Publisher pub_marker_text;
    ros::Publisher pub_marker_arrow;
    ros::Publisher pub_dynamic_objects_pose;
    ros::Publisher pub_dynamic_objects_box;
    ros::Publisher pub_points;
    ros::Publisher pub_global_map;

    double heading;
    double current_velocity;
    tf::TransformListener* tf_listener_map;

    sensor_msgs::PointCloud2 map;

    geometry_msgs::PoseStamped current_pose;
    double vehicle_x, vehicle_y, vehicle_phi;
    std::ofstream road_file;
    nav_msgs::Path path;

};
#endif //PROJECT_SCALA_RVIZ_DISPLAY_H