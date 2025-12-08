#include <glim_ros/rviz_viewer.hpp>

#include <mutex>
#include <fstream>
#include <filesystem>
#include <spdlog/spdlog.h>
#include <rclcpp/clock.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#define GLIM_ROS2
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/config.hpp>
#include <glim/util/trajectory_manager.hpp>
#include <glim/util/ros_cloud_converter.hpp>
#include <glim_ros/rgb_colorizer_ros.hpp>

namespace glim {

RvizViewer::RvizViewer() : logger(create_module_logger("rviz")), rgb_map_updated(false) {
  const Config config(GlobalConfig::get_config_path("config_ros"));

  imu_frame_id = config.param<std::string>("glim_ros", "imu_frame_id", "imu");
  lidar_frame_id = config.param<std::string>("glim_ros", "lidar_frame_id", "lidar");
  base_frame_id = config.param<std::string>("glim_ros", "base_frame_id", "");
  if (base_frame_id.empty()) {
    base_frame_id = imu_frame_id;
  }

  odom_frame_id = config.param<std::string>("glim_ros", "odom_frame_id", "odom");
  map_frame_id = config.param<std::string>("glim_ros", "map_frame_id", "map");
  publish_imu2lidar = config.param<bool>("glim_ros", "publish_imu2lidar", true);
  tf_time_offset = config.param<double>("glim_ros", "tf_time_offset", 1e-6);

  // RGB colorizer integration
  rgb_colorizer_enabled = config.param<bool>("glim_ros", "rgb_colorizer_enabled", false);
  map_save_path = config.param<std::string>("glim_ros", "map_save_path", "");
  logger->info("RGB colorizer enabled: {}", rgb_colorizer_enabled);

  last_globalmap_pub_time = rclcpp::Clock(rcl_clock_type_t::RCL_ROS_TIME).now();
  trajectory.reset(new TrajectoryManager);

  set_callbacks();

  kill_switch = false;
  thread = std::thread([this] {
    while (!kill_switch) {
      const auto expected = std::chrono::milliseconds(10);
      const auto t1 = std::chrono::high_resolution_clock::now();
      spin_once();
      const auto t2 = std::chrono::high_resolution_clock::now();

      if (t2 - t1 < expected) {
        std::this_thread::sleep_for(expected - (t2 - t1));
      }
    }
  });
}

RvizViewer::~RvizViewer() {
  kill_switch = true;
  thread.join();
}

void RvizViewer::at_exit(const std::string& dump_path) {
  // Save map.pcd to the dump directory provided by GLIM
  save_map_pcd(dump_path);
}

void RvizViewer::save_map_pcd(const std::string& dump_path) {
  // Determine save path: use dump_path if provided, otherwise use config or default
  std::string save_path;
  if (!dump_path.empty()) {
    // Use GLIM's dump directory (e.g., glim_ros/Log/map_YYYYMMDD_HHMMSS/)
    save_path = dump_path + "/map.pcd";
  } else if (!map_save_path.empty()) {
    save_path = map_save_path;
  } else {
    // Fallback: save to current directory
    save_path = "map.pcd";
  }

  logger->info("Saving map to: {}", save_path);

  if (rgb_colorizer_enabled) {
    // Save RGB map from accumulated RGB points (with color)
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    std::lock_guard<std::mutex> lock(rgb_map_mutex);

    if (accumulated_rgb_points.empty()) {
      logger->warn("No RGB map points to save");
      return;
    }

    pcl_cloud.reserve(accumulated_rgb_points.size());
    for (size_t i = 0; i < accumulated_rgb_points.size(); i++) {
      pcl::PointXYZRGB pt;
      pt.x = static_cast<float>(accumulated_rgb_points[i].x());
      pt.y = static_cast<float>(accumulated_rgb_points[i].y());
      pt.z = static_cast<float>(accumulated_rgb_points[i].z());

      uint32_t rgb = accumulated_rgb_colors[i];
      pt.r = (rgb >> 16) & 0xFF;
      pt.g = (rgb >> 8) & 0xFF;
      pt.b = rgb & 0xFF;
      pcl_cloud.push_back(pt);
    }
    logger->info("Saving RGB map with {} points", pcl_cloud.size());

    try {
      pcl::io::savePCDFileBinary(save_path, pcl_cloud);
      logger->info("Map saved successfully: {} ({} points)", save_path, pcl_cloud.size());
    } catch (const std::exception& e) {
      logger->error("Failed to save map: {}", e.what());
    }
  } else {
    // Save non-RGB map from submaps (XYZ only, no color)
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

    if (submaps.empty()) {
      logger->warn("No submap points to save");
      return;
    }

    // Concatenate all submaps
    size_t total_points = 0;
    for (const auto& submap : submaps) {
      total_points += submap->size();
    }

    pcl_cloud.reserve(total_points);
    for (size_t i = 0; i < submaps.size(); i++) {
      const auto& submap = submaps[i];
      const auto& pose = submap_poses[i];

      for (int j = 0; j < submap->size(); j++) {
        Eigen::Vector4d p_world = pose * submap->points[j];
        pcl::PointXYZ pt;
        pt.x = static_cast<float>(p_world.x());
        pt.y = static_cast<float>(p_world.y());
        pt.z = static_cast<float>(p_world.z());
        pcl_cloud.push_back(pt);
      }
    }
    logger->info("Saving non-RGB map with {} points", pcl_cloud.size());

    try {
      pcl::io::savePCDFileBinary(save_path, pcl_cloud);
      logger->info("Map saved successfully: {} ({} points)", save_path, pcl_cloud.size());
    } catch (const std::exception& e) {
      logger->error("Failed to save map: {}", e.what());
    }
  }
}

std::vector<GenericTopicSubscription::Ptr> RvizViewer::create_subscriptions(rclcpp::Node& node) {
  tf_buffer = std::make_unique<tf2_ros::Buffer>(node.get_clock());
  tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);
  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);

  points_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>("~/points", 10);
  aligned_points_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>("~/aligned_points", 10);

  points_corrected_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>("~/points_corrected", 10);
  aligned_points_corrected_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>("~/aligned_points_corrected", 10);

  rmw_qos_profile_t map_qos_profile = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};
  rclcpp::QoS map_qos(rclcpp::QoSInitialization(map_qos_profile.history, map_qos_profile.depth), map_qos_profile);
  map_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>("~/map", map_qos);
  odom_pub = node.create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);
  pose_pub = node.create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 10);

  odom_corrected_pub = node.create_publisher<nav_msgs::msg::Odometry>("~/odom_corrected", 10);
  pose_corrected_pub = node.create_publisher<geometry_msgs::msg::PoseStamped>("~/pose_corrected", 10);

  path_pub = node.create_publisher<nav_msgs::msg::Path>("~/path", 10);
  path_msg.header.frame_id = map_frame_id;

  return {};
}

void RvizViewer::set_callbacks() {
  using std::placeholders::_1;
  OdometryEstimationCallbacks::on_new_frame.add([this](const EstimationFrame::ConstPtr& new_frame) { odometry_new_frame(new_frame, false); });
  OdometryEstimationCallbacks::on_update_new_frame.add([this](const EstimationFrame::ConstPtr& new_frame) { odometry_new_frame(new_frame, true); });
  GlobalMappingCallbacks::on_update_submaps.add(std::bind(&RvizViewer::globalmap_on_update_submaps, this, _1));

  if (rgb_colorizer_enabled) {
    // Subscribe to RGB colorizer callbacks for publishing colored point clouds
    RGBColorizerCallbacks::on_frame_colorized.add([this](const ColorizedPointCloud& colorized) {
      publish_colored_points(colorized);
    });

    // Subscribe to RGB map updates for /glim_ros/map
    RGBColorizerCallbacks::on_map_updated.add([this](const ColorizedMap& map) {
      on_rgb_map_updated(map);
    });

    logger->info("Subscribed to RGB colorizer callbacks");
  } else {
    logger->info("RGB colorizer disabled, using fallback mode for /glim_ros/points");
  }
}

void RvizViewer::odometry_new_frame(const EstimationFrame::ConstPtr& new_frame, bool corrected) {
  const Eigen::Isometry3d T_odom_imu = new_frame->T_world_imu;
  const Eigen::Quaterniond quat_odom_imu(T_odom_imu.linear());
  const Eigen::Vector3d v_odom_imu = new_frame->v_world_imu;

  const Eigen::Isometry3d T_lidar_imu = new_frame->T_lidar_imu;
  const Eigen::Quaterniond quat_lidar_imu(T_lidar_imu.linear());

  Eigen::Isometry3d T_world_odom;
  Eigen::Quaterniond quat_world_odom;

  Eigen::Isometry3d T_world_imu;
  Eigen::Quaterniond quat_world_imu;

  {
    // Transform the odometry frame to the global optimization-based world frame
    std::lock_guard<std::mutex> lock(trajectory_mutex);
    trajectory->add_odom(new_frame->stamp, new_frame->T_world_imu, 1);
    T_world_odom = trajectory->get_T_world_odom();
    quat_world_odom = Eigen::Quaterniond(T_world_odom.linear());

    T_world_imu = trajectory->odom2world(T_odom_imu);
    quat_world_imu = Eigen::Quaterniond(T_world_imu.linear());
  }

  // Publish transforms
  const auto stamp = from_sec(new_frame->stamp);
  const auto tf_stamp = from_sec(new_frame->stamp + tf_time_offset);

  const bool publish_tf = !corrected;
  if (publish_tf) {
    // Odom -> Base
    geometry_msgs::msg::TransformStamped trans;
    trans.header.stamp = tf_stamp;
    trans.header.frame_id = odom_frame_id;
    trans.child_frame_id = base_frame_id;

    if (base_frame_id == imu_frame_id) {
      trans.transform.translation.x = T_odom_imu.translation().x();
      trans.transform.translation.y = T_odom_imu.translation().y();
      trans.transform.translation.z = T_odom_imu.translation().z();
      trans.transform.rotation.x = quat_odom_imu.x();
      trans.transform.rotation.y = quat_odom_imu.y();
      trans.transform.rotation.z = quat_odom_imu.z();
      trans.transform.rotation.w = quat_odom_imu.w();
      tf_broadcaster->sendTransform(trans);
    } else {
      try {
        const auto trans_imu_base = tf_buffer->lookupTransform(imu_frame_id, base_frame_id, from_sec(new_frame->stamp));
        const auto& t = trans_imu_base.transform.translation;
        const auto& r = trans_imu_base.transform.rotation;

        Eigen::Isometry3d T_imu_base = Eigen::Isometry3d::Identity();
        T_imu_base.translation() << t.x, t.y, t.z;
        T_imu_base.linear() = Eigen::Quaterniond(r.w, r.x, r.y, r.z).toRotationMatrix();

        const Eigen::Isometry3d T_odom_base = T_odom_imu * T_imu_base;
        const Eigen::Quaterniond quat_odom_base(T_odom_base.linear());

        trans.transform.translation.x = T_odom_base.translation().x();
        trans.transform.translation.y = T_odom_base.translation().y();
        trans.transform.translation.z = T_odom_base.translation().z();
        trans.transform.rotation.x = quat_odom_base.x();
        trans.transform.rotation.y = quat_odom_base.y();
        trans.transform.rotation.z = quat_odom_base.z();
        trans.transform.rotation.w = quat_odom_base.w();
        tf_broadcaster->sendTransform(trans);
      } catch (const tf2::TransformException& e) {
        logger->warn("Failed to lookup transform from {} to {} (stamp={}.{}): {}", imu_frame_id, base_frame_id, stamp.sec, stamp.nanosec, e.what());
      }
    }

    // World -> Odom
    trans.header.frame_id = map_frame_id;
    trans.child_frame_id = odom_frame_id;
    trans.transform.translation.x = T_world_odom.translation().x();
    trans.transform.translation.y = T_world_odom.translation().y();
    trans.transform.translation.z = T_world_odom.translation().z();
    trans.transform.rotation.x = quat_world_odom.x();
    trans.transform.rotation.y = quat_world_odom.y();
    trans.transform.rotation.z = quat_world_odom.z();
    trans.transform.rotation.w = quat_world_odom.w();
    tf_broadcaster->sendTransform(trans);

    // IMU -> LiDAR
    if (publish_imu2lidar) {
      trans.header.frame_id = imu_frame_id;
      trans.child_frame_id = lidar_frame_id;
      trans.transform.translation.x = T_lidar_imu.translation().x();
      trans.transform.translation.y = T_lidar_imu.translation().y();
      trans.transform.translation.z = T_lidar_imu.translation().z();
      trans.transform.rotation.x = quat_lidar_imu.x();
      trans.transform.rotation.y = quat_lidar_imu.y();
      trans.transform.rotation.z = quat_lidar_imu.z();
      trans.transform.rotation.w = quat_lidar_imu.w();
      tf_broadcaster->sendTransform(trans);
    }
  }

  auto& odom_pub = !corrected ? this->odom_pub : this->odom_corrected_pub;
  if (odom_pub->get_subscription_count()) {
    // Publish sensor pose (without loop closure)
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id;
    odom.child_frame_id = imu_frame_id;
    odom.pose.pose.position.x = T_odom_imu.translation().x();
    odom.pose.pose.position.y = T_odom_imu.translation().y();
    odom.pose.pose.position.z = T_odom_imu.translation().z();
    odom.pose.pose.orientation.x = quat_odom_imu.x();
    odom.pose.pose.orientation.y = quat_odom_imu.y();
    odom.pose.pose.orientation.z = quat_odom_imu.z();
    odom.pose.pose.orientation.w = quat_odom_imu.w();

    odom.twist.twist.linear.x = v_odom_imu.x();
    odom.twist.twist.linear.y = v_odom_imu.y();
    odom.twist.twist.linear.z = v_odom_imu.z();

    odom_pub->publish(odom);

    logger->debug("published odom (stamp={})", new_frame->stamp);
  }

  auto& pose_pub = !corrected ? this->pose_pub : this->pose_corrected_pub;
  if (pose_pub->get_subscription_count()) {
    // Publish sensor pose (with loop closure)
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_id;
    pose.pose.position.x = T_world_imu.translation().x();
    pose.pose.position.y = T_world_imu.translation().y();
    pose.pose.position.z = T_world_imu.translation().z();
    pose.pose.orientation.x = quat_world_imu.x();
    pose.pose.orientation.y = quat_world_imu.y();
    pose.pose.orientation.z = quat_world_imu.z();
    pose.pose.orientation.w = quat_world_imu.w();
    pose_pub->publish(pose);
    logger->debug("published pose (stamp={})", new_frame->stamp);
  }

  // Publish path (simple accumulation of poses)
  if (!corrected && path_pub->get_subscription_count()) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_id;
    pose.pose.position.x = T_world_imu.translation().x();
    pose.pose.position.y = T_world_imu.translation().y();
    pose.pose.position.z = T_world_imu.translation().z();
    pose.pose.orientation.x = quat_world_imu.x();
    pose.pose.orientation.y = quat_world_imu.y();
    pose.pose.orientation.z = quat_world_imu.z();
    pose.pose.orientation.w = quat_world_imu.w();

    path_msg.poses.push_back(pose);
    path_msg.header.stamp = stamp;
    path_pub->publish(path_msg);
  }

  // ~/points publishing:
  // - If rgb_colorizer_enabled: handled by RGBColorizerCallbacks::on_frame_colorized
  // - If not enabled: publish non-colorized points directly here
  if (!rgb_colorizer_enabled && !corrected && points_pub->get_subscription_count()) {
    publish_non_colorized_points(new_frame);
  }

  auto& aligned_points_pub = !corrected ? this->aligned_points_pub : this->aligned_points_corrected_pub;
  if (aligned_points_pub->get_subscription_count()) {
    // Publish points aligned to the world frame to avoid some visualization issues in Rviz2
    std::vector<Eigen::Vector4d> transformed(new_frame->frame->size());
    for (int i = 0; i < new_frame->frame->size(); i++) {
      transformed[i] = new_frame->T_world_sensor() * new_frame->frame->points[i];
    }

    gtsam_points::PointCloud frame;
    frame.num_points = new_frame->frame->size();
    frame.points = transformed.data();
    frame.times = new_frame->frame->times;
    frame.intensities = new_frame->frame->intensities;

    auto points = frame_to_pointcloud2(map_frame_id, new_frame->stamp, frame);
    aligned_points_pub->publish(*points);

    logger->debug("published aligned_points (stamp={} num_points={})", new_frame->stamp, frame.size());
  }
}

void RvizViewer::publish_colored_points(const ColorizedPointCloud& colorized) {
  if (!points_pub || points_pub->get_subscription_count() == 0) {
    return;
  }

  if (colorized.points.empty()) {
    return;
  }

  // Map frame_id string to configured frame IDs
  std::string frame_id;
  if (colorized.frame_id == "lidar") {
    frame_id = lidar_frame_id;
  } else if (colorized.frame_id == "imu") {
    frame_id = imu_frame_id;
  } else {
    frame_id = map_frame_id;
  }

  // Create PointCloud2 message with RGB colors
  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  msg->header.frame_id = frame_id;
  msg->header.stamp.sec = static_cast<int32_t>(colorized.stamp);
  msg->header.stamp.nanosec = static_cast<uint32_t>((colorized.stamp - msg->header.stamp.sec) * 1e9);

  msg->height = 1;
  msg->width = colorized.points.size();
  msg->is_bigendian = false;
  msg->is_dense = false;

  // Define fields: x, y, z, rgb
  sensor_msgs::msg::PointField field_x, field_y, field_z, field_rgb;
  field_x.name = "x";
  field_x.offset = 0;
  field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_x.count = 1;

  field_y.name = "y";
  field_y.offset = 4;
  field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_y.count = 1;

  field_z.name = "z";
  field_z.offset = 8;
  field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_z.count = 1;

  field_rgb.name = "rgb";
  field_rgb.offset = 12;
  field_rgb.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_rgb.count = 1;

  msg->fields = {field_x, field_y, field_z, field_rgb};
  msg->point_step = 16;
  msg->row_step = msg->point_step * msg->width;
  msg->data.resize(msg->row_step * msg->height);

  // Fill data
  for (size_t i = 0; i < colorized.points.size(); i++) {
    float* ptr = reinterpret_cast<float*>(msg->data.data() + i * msg->point_step);
    ptr[0] = static_cast<float>(colorized.points[i].x());
    ptr[1] = static_cast<float>(colorized.points[i].y());
    ptr[2] = static_cast<float>(colorized.points[i].z());
    // Pack RGB as float
    uint32_t rgb = colorized.colors[i];
    memcpy(&ptr[3], &rgb, sizeof(float));
  }

  points_pub->publish(std::move(*msg));
  logger->debug("published colored points (stamp={} num_points={})", colorized.stamp, colorized.points.size());
}

void RvizViewer::globalmap_on_update_submaps(const std::vector<SubMap::Ptr>& submaps) {
  const SubMap::ConstPtr latest_submap = submaps.back();

  const double stamp_endpoint_R = latest_submap->odom_frames.back()->stamp;
  const Eigen::Isometry3d T_world_endpoint_R = latest_submap->T_world_origin * latest_submap->T_origin_endpoint_R;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex);
    trajectory->update_anchor(stamp_endpoint_R, T_world_endpoint_R);
  }

  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> new_submap_poses(submaps.size());
  for (size_t i = 0; i < submaps.size(); i++) {
    new_submap_poses[i] = submaps[i]->T_world_origin;
  }

  // Invoke a submap concatenation task in the RvizViewer thread
  invoke([this, latest_submap, new_submap_poses] {
    this->submaps.push_back(latest_submap->frame);
    this->submap_poses = new_submap_poses;  // Store for map.pcd saving

    // If RGB colorizer is enabled, map publishing is handled via RGB map callback
    if (rgb_colorizer_enabled) {
      return;
    }

    if (!map_pub->get_subscription_count()) {
      return;
    }

    const rclcpp::Time now = rclcpp::Clock(rcl_clock_type_t::RCL_ROS_TIME).now();

    // Publish global map every 10 seconds
    if (now - last_globalmap_pub_time < std::chrono::seconds(10)) {
      return;
    }
    last_globalmap_pub_time = now;

    logger->info("Publishing non-RGB global map ({} submaps)", this->submaps.size());

    int total_num_points = 0;
    for (const auto& submap : this->submaps) {
      total_num_points += submap->size();
    }

    // Concatenate all the submap points
    gtsam_points::PointCloudCPU::Ptr merged(new gtsam_points::PointCloudCPU);
    merged->num_points = total_num_points;
    merged->points_storage.resize(total_num_points);
    merged->points = merged->points_storage.data();

    int begin = 0;
    for (size_t i = 0; i < this->submaps.size(); i++) {
      const auto& submap = this->submaps[i];
      std::transform(submap->points, submap->points + submap->size(), merged->points + begin, [&](const Eigen::Vector4d& p) { return this->submap_poses[i] * p; });
      begin += submap->size();
    }

    auto points_msg = frame_to_pointcloud2(map_frame_id, now.seconds(), *merged);
    map_pub->publish(*points_msg);
  });
}

void RvizViewer::invoke(const std::function<void()>& task) {
  std::lock_guard<std::mutex> lock(invoke_queue_mutex);
  invoke_queue.push_back(task);
}

void RvizViewer::spin_once() {
  std::vector<std::function<void()>> invoke_queue;

  {
    std::lock_guard<std::mutex> lock(invoke_queue_mutex);
    invoke_queue.swap(this->invoke_queue);
  }

  for (const auto& task : invoke_queue) {
    task();
  }
}

sensor_msgs::msg::PointCloud2::UniquePtr RvizViewer::frame_to_colored_pointcloud2(
  const std::string& frame_id, double stamp,
  const gtsam_points::PointCloud& frame, const std::vector<uint32_t>& colors) {

  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  msg->header.frame_id = frame_id;
  msg->header.stamp = from_sec(stamp);

  msg->height = 1;
  msg->width = frame.size();
  msg->is_dense = false;
  msg->is_bigendian = false;

  // Fields: x, y, z, rgb
  sensor_msgs::msg::PointField field;
  field.name = "x";
  field.offset = 0;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  msg->fields.push_back(field);

  field.name = "y";
  field.offset = 4;
  msg->fields.push_back(field);

  field.name = "z";
  field.offset = 8;
  msg->fields.push_back(field);

  field.name = "rgb";
  field.offset = 12;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg->fields.push_back(field);

  msg->point_step = 16;
  msg->row_step = msg->point_step * msg->width;
  msg->data.resize(msg->row_step);

  uint8_t* ptr = msg->data.data();
  for (int i = 0; i < frame.size(); i++) {
    float x = static_cast<float>(frame.points[i].x());
    float y = static_cast<float>(frame.points[i].y());
    float z = static_cast<float>(frame.points[i].z());

    std::memcpy(ptr, &x, sizeof(float));
    std::memcpy(ptr + 4, &y, sizeof(float));
    std::memcpy(ptr + 8, &z, sizeof(float));

    // Pack RGB as float (RViz convention)
    uint32_t rgb = colors[i];
    float rgb_float;
    std::memcpy(&rgb_float, &rgb, sizeof(float));
    std::memcpy(ptr + 12, &rgb_float, sizeof(float));

    ptr += msg->point_step;
  }

  return msg;
}

void RvizViewer::publish_non_colorized_points(const EstimationFrame::ConstPtr& new_frame) {
  if (!new_frame->frame || new_frame->frame->size() == 0) {
    return;
  }

  // Use frame_to_pointcloud2 for non-colorized points
  auto points_msg = frame_to_pointcloud2(lidar_frame_id, new_frame->stamp, *new_frame->frame);
  points_pub->publish(*points_msg);
  logger->debug("published non-colorized points (stamp={} num_points={})", new_frame->stamp, new_frame->frame->size());
}

void RvizViewer::on_rgb_map_updated(const ColorizedMap& map) {
  // Store accumulated RGB map data
  {
    std::lock_guard<std::mutex> lock(rgb_map_mutex);
    accumulated_rgb_points = map.points;
    accumulated_rgb_colors = map.colors;
    rgb_map_updated = true;
  }

  // Schedule map publishing in the viewer thread
  invoke([this] {
    publish_rgb_map();
  });
}

void RvizViewer::publish_rgb_map() {
  if (!map_pub || !map_pub->get_subscription_count()) {
    return;
  }

  const rclcpp::Time now = rclcpp::Clock(rcl_clock_type_t::RCL_ROS_TIME).now();

  // Throttle map publishing to every 10 seconds
  if (now - last_globalmap_pub_time < std::chrono::seconds(10)) {
    return;
  }
  last_globalmap_pub_time = now;

  std::vector<Eigen::Vector4d> points;
  std::vector<uint32_t> colors;

  {
    std::lock_guard<std::mutex> lock(rgb_map_mutex);
    if (accumulated_rgb_points.empty() || !rgb_map_updated) {
      return;
    }
    points = accumulated_rgb_points;
    colors = accumulated_rgb_colors;
    rgb_map_updated = false;
  }

  logger->info("Publishing RGB map with {} points", points.size());

  // Create PointCloud2 message with RGB colors
  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  msg->header.frame_id = map_frame_id;
  msg->header.stamp = now;

  msg->height = 1;
  msg->width = points.size();
  msg->is_bigendian = false;
  msg->is_dense = false;

  // Define fields: x, y, z, rgb
  sensor_msgs::msg::PointField field_x, field_y, field_z, field_rgb;
  field_x.name = "x";
  field_x.offset = 0;
  field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_x.count = 1;

  field_y.name = "y";
  field_y.offset = 4;
  field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_y.count = 1;

  field_z.name = "z";
  field_z.offset = 8;
  field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_z.count = 1;

  field_rgb.name = "rgb";
  field_rgb.offset = 12;
  field_rgb.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_rgb.count = 1;

  msg->fields = {field_x, field_y, field_z, field_rgb};
  msg->point_step = 16;
  msg->row_step = msg->point_step * msg->width;
  msg->data.resize(msg->row_step * msg->height);

  // Fill data
  for (size_t i = 0; i < points.size(); i++) {
    float* ptr = reinterpret_cast<float*>(msg->data.data() + i * msg->point_step);
    ptr[0] = static_cast<float>(points[i].x());
    ptr[1] = static_cast<float>(points[i].y());
    ptr[2] = static_cast<float>(points[i].z());
    // Pack RGB as float
    uint32_t rgb = colors[i];
    memcpy(&ptr[3], &rgb, sizeof(float));
  }

  map_pub->publish(std::move(*msg));
}

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::RvizViewer();
}
