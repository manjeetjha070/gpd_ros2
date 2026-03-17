/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Andreas ten Pas
 *  All rights reserved.
 */

#include <gpd_ros/grasp_detection_server.hpp>

GraspDetectionServer::GraspDetectionServer()
: Node("detect_grasps_server")
{
  cloud_camera_ = nullptr;
  grasp_detector_ = nullptr;
  rviz_plotter_ = nullptr;
  use_rviz_ = false;

  // Declare parameters
  this->declare_parameter<std::vector<double>>("camera_position", {0.0,0.0,0.0});
  this->declare_parameter<std::string>("config_file", "");
  this->declare_parameter<std::string>("rviz_topic", "");
  this->declare_parameter<std::vector<double>>("workspace", {});

  // Camera viewpoint
  std::vector<double> camera_position;
  this->get_parameter("camera_position", camera_position);

  view_point_ << camera_position[0], camera_position[1], camera_position[2];

  // Config file
  std::string cfg_file;
  this->get_parameter("config_file", cfg_file);

  grasp_detector_ = new gpd::GraspDetector(cfg_file);

  // RViz visualization
  std::string rviz_topic;
  this->get_parameter("rviz_topic", rviz_topic);

  if (!rviz_topic.empty())
  {
    rviz_plotter_ = new GraspPlotter(
        shared_from_this(),
        grasp_detector_->getHandSearchParameters().hand_geometry_);
    use_rviz_ = true;
  }

  // Publisher
  grasps_pub_ =
      this->create_publisher<gpd_ros::msg::GraspConfigList>(
          "clustered_grasps", 10);

  this->get_parameter("workspace", workspace_);
}


void GraspDetectionServer::detectGrasps(
    const std::shared_ptr<gpd_ros::srv::DetectGrasps::Request> req,
    std::shared_ptr<gpd_ros::srv::DetectGrasps::Response> res)
{
  RCLCPP_INFO(this->get_logger(), "Received service request...");

  cloud_camera_ = nullptr;

  const auto& cloud_sources = req->cloud_indexed.cloud_sources;

  Eigen::Matrix3Xd view_points(3, cloud_sources.view_points.size());

  for (size_t i = 0; i < cloud_sources.view_points.size(); i++)
  {
    view_points.col(i) <<
        cloud_sources.view_points[i].x,
        cloud_sources.view_points[i].y,
        cloud_sources.view_points[i].z;
  }

  if (cloud_sources.cloud.fields.size() == 6 &&
      cloud_sources.cloud.fields[3].name == "normal_x" &&
      cloud_sources.cloud.fields[4].name == "normal_y" &&
      cloud_sources.cloud.fields[5].name == "normal_z")
  {
    PointCloudPointNormal::Ptr cloud(new PointCloudPointNormal);
    pcl::fromROSMsg(cloud_sources.cloud, *cloud);

    Eigen::MatrixXi camera_source =
        Eigen::MatrixXi::Zero(view_points.cols(), cloud->size());

    for (size_t i = 0; i < cloud_sources.camera_source.size(); i++)
    {
      camera_source(cloud_sources.camera_source[i].data, i) = 1;
    }

    cloud_camera_ =
        new gpd::util::Cloud(cloud, camera_source, view_points);
  }
  else
  {
    PointCloudRGBA::Ptr cloud(new PointCloudRGBA);
    pcl::fromROSMsg(cloud_sources.cloud, *cloud);

    Eigen::MatrixXi camera_source =
        Eigen::MatrixXi::Zero(view_points.cols(), cloud->size());

    for (size_t i = 0; i < cloud_sources.camera_source.size(); i++)
    {
      camera_source(cloud_sources.camera_source[i].data, i) = 1;
    }

    cloud_camera_ =
        new gpd::util::Cloud(cloud, camera_source, view_points);
  }

  std::vector<int> indices(req->cloud_indexed.indices.size());

  for (size_t i = 0; i < indices.size(); i++)
  {
    indices[i] = req->cloud_indexed.indices[i].data;
  }

  cloud_camera_->setSampleIndices(indices);

  frame_ = cloud_sources.cloud.header.frame_id;
  cloud_camera_header_ = cloud_sources.cloud.header;

  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Received cloud with "
      << cloud_camera_->getCloudProcessed()->size()
      << " points and "
      << indices.size()
      << " samples");

  grasp_detector_->preprocessPointCloud(*cloud_camera_);

  std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps =
      grasp_detector_->detectGrasps(*cloud_camera_);

  if (!grasps.empty())
  {
    if (use_rviz_)
    {
      rviz_plotter_->drawGrasps(grasps, frame_);
    }

    auto grasp_list_msg =
        GraspMessages::createGraspListMsg(
            grasps, cloud_camera_header_);

    res->grasp_configs = grasp_list_msg;

    grasps_pub_->publish(grasp_list_msg);

    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Detected "
        << grasp_list_msg.grasps.size()
        << " highest-scoring grasps.");
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "No grasps detected!");
  }
}

int main(int argc, char **argv)
{
  std::srand(std::time(nullptr));

  rclcpp::init(argc, argv);

  auto node = std::make_shared<GraspDetectionServer>();

  auto service = node->create_service<gpd_ros::srv::DetectGrasps>(
      "detect_grasps",
      std::bind(&GraspDetectionServer::detectGrasps,
                node.get(),
                std::placeholders::_1,
                std::placeholders::_2));

  RCLCPP_INFO(node->get_logger(),
              "Grasp detection service is waiting for a point cloud ...");

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}