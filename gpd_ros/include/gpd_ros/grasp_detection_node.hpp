/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Andreas ten Pas
 *  All rights reserved.
 */

#ifndef GRASP_DETECTION_NODE_H_
#define GRASP_DETECTION_NODE_H_

// system
#include <algorithm>
#include <memory>
#include <vector>

// ROS2
#include <rclcpp/rclcpp.hpp>
// #include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// PCL
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// GPD
#include <gpd/grasp_detector.h>
#include <gpd/sequential_importance_sampling.h>
#include <gpd/util/cloud.h>

// this project (messages)
#include <gpd_ros/msg/cloud_indexed.hpp>
#include <gpd_ros/msg/cloud_samples.hpp>
#include <gpd_ros/msg/cloud_sources.hpp>
#include <gpd_ros/msg/grasp_config.hpp>
#include <gpd_ros/msg/grasp_config_list.hpp>
#include <gpd_ros/msg/samples_msg.hpp>

// this project (headers)
#include <gpd_ros/grasp_messages.hpp>
#include <gpd_ros/grasp_plotter.hpp>

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudPointNormal;


/**
 * GraspDetectionNode class
 *
 * ROS2 node that detects grasp poses in a point cloud.
 */
class GraspDetectionNode : public rclcpp::Node
{
public:
    /**
     * Constructor
     */
    GraspDetectionNode ();

    /**
     * Destructor
     */
    ~GraspDetectionNode ()
    {
        delete cloud_camera_;
        delete grasp_detector_;
        delete rviz_plotter_;
    }

    /**
     * Run node
     */
    void run ();

    /**
     * Detect grasp poses
     */
    std::vector<std::unique_ptr<gpd::candidate::Hand>> detectGraspPoses ();

private:
    /**
     * Find indices of points within a ball
     */
    std::vector<int> getSamplesInBall (
        const PointCloudRGBA::Ptr& cloud, const pcl::PointXYZRGBA& centroid, float radius);

    /**
     * Point cloud callback
     */
    void cloud_callback (const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /**
     * Cloud indexed callback
     */
    void cloud_indexed_callback (const gpd_ros::msg::CloudIndexed::SharedPtr msg);

    /**
     * Cloud samples callback
     */
    void cloud_samples_callback (const gpd_ros::msg::CloudSamples::SharedPtr msg);

    /**
     * Initialize cloud camera
     */
    void initCloudCamera (const gpd_ros::msg::CloudSources& msg);

    /**
     * Samples callback
     */
    void samples_callback (const gpd_ros::msg::SamplesMsg::SharedPtr msg);

    Eigen::Matrix3Xd fillMatrixFromFile (const std::string& filename, int num_normals);


    /** camera view point */
    Eigen::Vector3d view_point_;

    /** point cloud with camera info */
    gpd::util::Cloud* cloud_camera_;

    /** header of cloud */
    std_msgs::msg::Header cloud_camera_header_;

    int size_left_cloud_;

    bool has_cloud_;
    bool has_normals_;
    bool has_samples_;

    std::string frame_;

    /** ROS2 subscribers */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_pc2_;
    rclcpp::Subscription<gpd_ros::msg::CloudIndexed>::SharedPtr cloud_sub_cloud_indexed_;
    rclcpp::Subscription<gpd_ros::msg::CloudSamples>::SharedPtr cloud_sub_cloud_samples_;
    rclcpp::Subscription<gpd_ros::msg::SamplesMsg>::SharedPtr samples_sub_;

    /** ROS2 publishers */
    rclcpp::Publisher<gpd_ros::msg::GraspConfigList>::SharedPtr grasps_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grasps_rviz_pub_;

    bool use_importance_sampling_;
    bool use_rviz_;

    std::vector<double> workspace_;

    /** GPD detector */
    gpd::GraspDetector* grasp_detector_;

    /** RViz plotter */
    GraspPlotter* rviz_plotter_;

    /** constants for input types */
    static const int POINT_CLOUD_2;
    static const int CLOUD_INDEXED;
    static const int CLOUD_SAMPLES;
};

#endif