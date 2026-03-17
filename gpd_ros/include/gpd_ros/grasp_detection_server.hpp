/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Andreas ten Pas
 *  All rights reserved.
 */

#ifndef GRASP_DETECTION_SERVER_H_
#define GRASP_DETECTION_SERVER_H_

// ROS2
#include <rclcpp/rclcpp.hpp>
// #include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>

#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// GPD
#include <gpd/grasp_detector.h>
#include <gpd/util/cloud.h>

// this project (services)
#include <gpd_ros/srv/detect_grasps.hpp>

// this project (messages)
#include <gpd_ros/msg/grasp_config.hpp>
#include <gpd_ros/msg/grasp_config_list.hpp>

// this project (headers)
#include <gpd_ros/grasp_messages.hpp>
#include <gpd_ros/grasp_plotter.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudPointNormal;


/**
 * ROS2 service server for grasp detection
 */
class GraspDetectionServer : public rclcpp::Node
{
public:
    /**
     * Constructor
     */
    GraspDetectionServer ();

    /**
     * Destructor
     */
    ~GraspDetectionServer ()
    {
        delete cloud_camera_;
        delete grasp_detector_;
        delete rviz_plotter_;
    }

    /**
     * Service callback
     */
    void detectGrasps (
        const std::shared_ptr<gpd_ros::srv::DetectGrasps::Request> req,
        std::shared_ptr<gpd_ros::srv::DetectGrasps::Response> res);

private:
    /** ROS2 publisher for grasp list */
    rclcpp::Publisher<gpd_ros::msg::GraspConfigList>::SharedPtr grasps_pub_;

    /** ROS2 service */
    rclcpp::Service<gpd_ros::srv::DetectGrasps>::SharedPtr service_;

    /** cloud header */
    std_msgs::msg::Header cloud_camera_header_;

    /** frame id */
    std::string frame_;

    /** GPD detector */
    gpd::GraspDetector* grasp_detector_;

    /** cloud storage */
    gpd::util::Cloud* cloud_camera_;

    /** RViz plotter */
    GraspPlotter* rviz_plotter_;

    /** visualization flag */
    bool use_rviz_;

    /** workspace limits */
    std::vector<double> workspace_;

    /** camera viewpoint */
    Eigen::Vector3d view_point_;
};

#endif