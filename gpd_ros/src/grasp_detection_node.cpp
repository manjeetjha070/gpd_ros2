#include <gpd_ros/grasp_detection_node.hpp>
#include <rclcpp/rclcpp.hpp>


/** constants for input point cloud types */
const int GraspDetectionNode::POINT_CLOUD_2 = 0; ///< sensor_msgs/PointCloud2
const int GraspDetectionNode::CLOUD_INDEXED = 1; ///< cloud with indices
const int GraspDetectionNode::CLOUD_SAMPLES = 2; ///< cloud with (x,y,z) samples

GraspDetectionNode::GraspDetectionNode ()
    : Node ("detect_grasps"),
      size_left_cloud_ (0),
      has_cloud_ (false),
      has_normals_ (false),
      has_samples_ (true),
      frame_ (""),
      use_importance_sampling_ (false)
{
    RCLCPP_INFO (get_logger(), "Init...");

    cloud_camera_             = nullptr;

    // set camera viewpoint to default origin
    //  std::vector<double> camera_position;
    //  node.getParam("camera_position", camera_position);
    //  view_point_ << camera_position[0], camera_position[1], camera_position[2];

    // choose sampling method for grasp detection
    //  node.param("use_importance_sampling", use_importance_sampling_, false);

    //  if (use_importance_sampling_)
    //  {
    //    importance_sampling_ = new SequentialImportanceSampling(node);
    //  }


    // Read input cloud and sample ROS topics parameters.
    std::string cfg_file      = declare_parameter<std::string> ("config_file", "");
    int cloud_type            = declare_parameter<int> ("cloud_type", POINT_CLOUD_2);
    std::string cloud_topic   = declare_parameter<std::string> ("cloud_topic", "/camera/depth_registered/points");
    std::string samples_topic = declare_parameter<std::string> ("samples_topic", "");
    std::string rviz_topic    = declare_parameter<std::string> ("rviz_topic", "plot_grasps");

    workspace_                = declare_parameter<std::vector<double>> ("workspace", {-1, 1, -1, 1, -1, 1});
    grasp_detector_           = new gpd::GraspDetector (cfg_file);


    if (!rviz_topic.empty())
    {
        grasps_rviz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray> (rviz_topic, 10);
        use_rviz_        = true;
    }
    else
    {
        use_rviz_ = false;
    }

    // subscribe to input point cloud ROS topic
    if (cloud_type == POINT_CLOUD_2)
    {
        cloud_sub_pc2_ = create_subscription<sensor_msgs::msg::PointCloud2> (
            cloud_topic, 10, std::bind (&GraspDetectionNode::cloud_callback, this, std::placeholders::_1));
    }
    else if (cloud_type == CLOUD_INDEXED)
    {
        cloud_sub_cloud_indexed_ = create_subscription<gpd_ros::msg::CloudIndexed> (
            cloud_topic, 10, std::bind (&GraspDetectionNode::cloud_indexed_callback, this, std::placeholders::_1));
    }
    else if (cloud_type == CLOUD_SAMPLES)
    {
        cloud_sub_cloud_samples_ = create_subscription<gpd_ros::msg::CloudSamples> (
            cloud_topic, 10, std::bind (&GraspDetectionNode::cloud_samples_callback, this, std::placeholders::_1));

        //    grasp_detector_->setUseIncomingSamples(true);

        has_samples_ = false;
    }

    // subscribe to input samples ROS topic
    if (!samples_topic.empty())
    {
        samples_sub_ = create_subscription<gpd_ros::msg::SamplesMsg> (
            samples_topic, 10, std::bind (&GraspDetectionNode::samples_callback, this, std::placeholders::_1));

        has_samples_ = false;
    }

    // uses ROS topics to publish grasp candidates, antipodal grasps, and grasps after clustering
    grasps_pub_   = create_publisher<gpd_ros::msg::GraspConfigList> ("clustered_grasps", 10);

    rviz_plotter_ = new GraspPlotter (get_clock(), grasps_rviz_pub_, grasp_detector_->getHandSearchParameters().hand_geometry_);


    RCLCPP_INFO (get_logger(), "GPD initialized");
}

void GraspDetectionNode::run ()
{
    rclcpp::Rate rate (10);

    RCLCPP_INFO (get_logger(), "=================== Waiting for point cloud...");

    while (rclcpp::ok())
    {
        if (has_cloud_)
        {
            // Detect grasps in point cloud.
            auto grasps = detectGraspPoses();

            // Visualize the detected grasps in rviz.
            if (use_rviz_) rviz_plotter_->drawGrasps (grasps, frame_);

            has_cloud_   = false;
            has_samples_ = false;
            has_normals_ = false;

            RCLCPP_INFO (get_logger(), "====================== Waiting for point cloud...");
        }

        rclcpp::spin_some (get_node_base_interface());
        rate.sleep();
    }
}

std::vector<std::unique_ptr<gpd::candidate::Hand>> GraspDetectionNode::detectGraspPoses ()
{
    printf ("******************** detectGraspPoses\n");
    // detect grasp poses
    std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps;

    if (use_importance_sampling_)
    {
        //    cloud_camera_->filterWorkspace(workspace_);
        //    cloud_camera_->voxelizeCloud(0.003);
        //    cloud_camera_->calculateNormals(4);
        //    grasps = importance_sampling_->detectGrasps(*cloud_camera_);
        RCLCPP_ERROR (get_logger(), "Importance sampling not supported");
    }
    else
    {
        printf ("********************  grasp_detector_->preprocessPointCloud\n");
        // preprocess the point cloud
        grasp_detector_->preprocessPointCloud (*cloud_camera_);

        printf ("********************  grasp_detector_->detectGrasps\n");

        // detect grasps in the point cloud
        grasps = grasp_detector_->detectGrasps (*cloud_camera_);
        printf ("********************  grasp_detector_->detectGrasps     done\n");
    }

    auto msg = GraspMessages::createGraspListMsg (grasps, cloud_camera_header_);

    grasps_pub_->publish (msg);

    RCLCPP_INFO (get_logger(), "Published %ld grasps", msg.grasps.size());

    return grasps;
}

std::vector<int> GraspDetectionNode::getSamplesInBall (
    const PointCloudRGBA::Ptr& cloud, const pcl::PointXYZRGBA& centroid, float radius)
{
    std::vector<int> indices;
    std::vector<float> dists;
    pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
    kdtree.setInputCloud (cloud);
    kdtree.radiusSearch (centroid, radius, indices, dists);
    return indices;
}

void GraspDetectionNode::cloud_callback (const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    printf ("********************  cloud_callback\n");

    if (has_cloud_) return;

    //delete cloud_camera_;
    cloud_camera_ = nullptr;

    Eigen::Matrix3Xd view_points (3, 1);
    view_points.col (0) = view_point_;

    if (msg->fields.size() == 6 && msg->fields[3].name == "normal_x" && msg->fields[4].name == "normal_y"
        && msg->fields[5].name == "normal_z")
    {
        PointCloudPointNormal::Ptr cloud (new PointCloudPointNormal);
        pcl::fromROSMsg (*msg, *cloud);

        cloud_camera_ = new gpd::util::Cloud (cloud, 0, view_points);

        RCLCPP_INFO (get_logger(), "Cloud with normals: %ld points", cloud->size());
    }
    else
    {
        PointCloudRGBA::Ptr cloud (new PointCloudRGBA);
        pcl::fromROSMsg (*msg, *cloud);

        cloud_camera_ = new gpd::util::Cloud (cloud, 0, view_points);

        RCLCPP_INFO (get_logger(), "Cloud received: %ld points", cloud->size());
    }

    cloud_camera_header_ = msg->header;
    frame_               = msg->header.frame_id;
    has_cloud_           = true;
}

void GraspDetectionNode::cloud_indexed_callback (const gpd_ros::msg::CloudIndexed::SharedPtr msg)
{
    if (!has_cloud_)
    {
        initCloudCamera (msg->cloud_sources);

        std::vector<int> indices (msg->indices.size());

        for (size_t i = 0; i < indices.size(); i++)
        {
            indices[i] = msg->indices[i].data;
        }

        cloud_camera_->setSampleIndices (indices);

        has_cloud_ = true;
        frame_     = msg->cloud_sources.cloud.header.frame_id;

        RCLCPP_INFO_STREAM (
            get_logger(),
            "Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points and "
                                   << msg->indices.size() << " samples");
    }
}

void GraspDetectionNode::cloud_samples_callback (const gpd_ros::msg::CloudSamples::SharedPtr msg)
{
    if (!has_cloud_)
    {
        initCloudCamera (msg->cloud_sources);

        Eigen::Matrix3Xd samples (3, msg->samples.size());

        for (size_t i = 0; i < msg->samples.size(); i++)
        {
            samples.col (i) << msg->samples[i].x, msg->samples[i].y, msg->samples[i].z;
        }

        cloud_camera_->setSamples (samples);

        has_cloud_   = true;
        has_samples_ = true;

        frame_       = msg->cloud_sources.cloud.header.frame_id;

        RCLCPP_INFO_STREAM (
            get_logger(),
            "Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points and "
                                   << cloud_camera_->getSamples().cols() << " samples");
    }
}

void GraspDetectionNode::samples_callback (const gpd_ros::msg::SamplesMsg::SharedPtr msg)
{
    if (!has_samples_)
    {
        Eigen::Matrix3Xd samples (3, msg->samples.size());

        for (size_t i = 0; i < msg->samples.size(); i++)
        {
            samples.col (i) << msg->samples[i].x, msg->samples[i].y, msg->samples[i].z;
        }

        cloud_camera_->setSamples (samples);

        has_samples_ = true;

        RCLCPP_INFO_STREAM (get_logger(), "Received grasp samples message with " << msg->samples.size() << " samples");
    }
}

void GraspDetectionNode::initCloudCamera (const gpd_ros::msg::CloudSources& msg)
{
    delete cloud_camera_;
    cloud_camera_ = nullptr;

    // Set view points.
    Eigen::Matrix3Xd view_points (3, msg.view_points.size());

    for (size_t i = 0; i < msg.view_points.size(); i++)
    {
        view_points.col (i) << msg.view_points[i].x, msg.view_points[i].y, msg.view_points[i].z;
    }

    // Set point cloud.
    if (msg.cloud.fields.size() == 6 && msg.cloud.fields[3].name == "normal_x" && msg.cloud.fields[4].name == "normal_y"
        && msg.cloud.fields[5].name == "normal_z")
    {
        PointCloudPointNormal::Ptr cloud (new PointCloudPointNormal);
        pcl::fromROSMsg (msg.cloud, *cloud);

        Eigen::MatrixXi camera_source = Eigen::MatrixXi::Zero (view_points.cols(), cloud->size());

        for (size_t i = 0; i < msg.camera_source.size(); i++)
        {
            camera_source (msg.camera_source[i].data, i) = 1;
        }

        cloud_camera_ = new gpd::util::Cloud (cloud, camera_source, view_points);
    }
    else
    {
        PointCloudRGBA::Ptr cloud (new PointCloudRGBA);
        pcl::fromROSMsg (msg.cloud, *cloud);

        // TODO: multiple cameras can see the same point
        Eigen::MatrixXi camera_source = Eigen::MatrixXi::Zero (view_points.cols(), cloud->size());

        for (size_t i = 0; i < msg.camera_source.size(); i++)
        {
            camera_source (msg.camera_source[i].data, i) = 1;
        }

        cloud_camera_ = new gpd::util::Cloud (cloud, camera_source, view_points);

        std::cout << "view_points:\n" << view_points << std::endl;
    }
}

int main (int argc, char** argv)
{
    // seed the random number generator
    std::srand (std::time (0));

    rclcpp::init (argc, argv);

    auto node = std::make_shared<GraspDetectionNode>();

    node->run();

    rclcpp::shutdown();
    return 0;
}