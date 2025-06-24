/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2018, Simbe Robotics, Inc.
 *  Copyright (c) 2021, Samsung Research America
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Simbe Robotics, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Steve Macenski (steven.macenski@simberobotics.com)
 *                         stevenmacenski@gmail.com
 *********************************************************************/

#include <string>
#include <unordered_map>
#include <memory>
#include <vector>
#include <filesystem>

#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_layer.hpp"

namespace spatio_temporal_voxel_layer
{

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using rcl_interfaces::msg::ParameterType;

/*****************************************************************************/
SpatioTemporalVoxelLayer::SpatioTemporalVoxelLayer(void)
/*****************************************************************************/
{
}

/*****************************************************************************/
SpatioTemporalVoxelLayer::~SpatioTemporalVoxelLayer(void)
/*****************************************************************************/
{
  _voxel_grid.reset();
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::onInitialize(void)
/*****************************************************************************/
{
  RCLCPP_INFO(
    logger_,
    "%s being initialized as SpatioTemporalVoxelLayer!", getName().c_str());

  // initialize parameters, grid, and sub/pubs
  _global_frame = std::string(layered_costmap_->getGlobalFrameID());
  RCLCPP_INFO(
    logger_, "%s's global frame is %s.",
    getName().c_str(), _global_frame.c_str());

  bool track_unknown_space;
  double transform_tolerance, map_save_time;
  int decay_model_int;
  // source names
  auto node = node_.lock();
  declareParameter("observation_sources", rclcpp::ParameterValue(std::string("")));
  node->get_parameter(name_ + ".observation_sources", _topics_string);
  // timeout in seconds for transforms
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.2));
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance);
  // whether to default on
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".enabled", _enabled);
  enabled_ = _enabled;
  // publish the voxel grid to visualize
  declareParameter("publish_voxel_map", rclcpp::ParameterValue(false));
  node->get_parameter(name_ + ".publish_voxel_map", _publish_voxels);
  // size of each voxel in meters
  declareParameter("voxel_size", rclcpp::ParameterValue(0.05));
  node->get_parameter(name_ + ".voxel_size", _voxel_size);
  // 1=takes highest in layers, 0=takes current layer
  declareParameter("combination_method", rclcpp::ParameterValue(1));
  node->get_parameter(name_ + ".combination_method", _combination_method);
  // number of voxels per vertical needed to have obstacle
  declareParameter("mark_threshold", rclcpp::ParameterValue(0));
  node->get_parameter(name_ + ".mark_threshold", _mark_threshold);
  // clear under robot footprint
  declareParameter("clear_costmap_under_footprint", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".clear_costmap_under_footprint", _clear_costmap_under_footprint);

  // clear grid under robot footprint
  declareParameter("clear_grid_under_footprint_in_manual_mode", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".clear_grid_under_footprint_in_manual_mode", _clear_grid_under_footprint_in_manual_mode);

  // clear under robot footprint
  declareParameter("auto_grid_clear_range",  rclcpp::ParameterValue(0.0));
  node->get_parameter(name_ + ".auto_grid_clear_range", _auto_grid_clear_range);

  // keep tabs on unknown space
  declareParameter(
    "track_unknown_space",
    rclcpp::ParameterValue(layered_costmap_->isTrackingUnknown()));
  node->get_parameter(name_ + ".track_unknown_space", track_unknown_space);
  declareParameter("decay_model", rclcpp::ParameterValue(0));
  node->get_parameter(name_ + ".decay_model", decay_model_int);
  _decay_model = static_cast<volume_grid::GlobalDecayModel>(decay_model_int);
  // decay param
  declareParameter("voxel_decay", rclcpp::ParameterValue(-1.0));
  node->get_parameter(name_ + ".voxel_decay", _voxel_decay);
  // whether to map or navigate
  declareParameter("mapping_mode", rclcpp::ParameterValue(false));
  node->get_parameter(name_ + ".mapping_mode", _mapping_mode);
  // enable autosaving of the map
  declareParameter("autosaving_enabled", rclcpp::ParameterValue(false));
  node->get_parameter(name_ + ".autosaving_enabled", _autosaving_enabled);
  // path to the stvl map path
  declareParameter("stvl_map_file", rclcpp::ParameterValue(std::string("")));
  node->get_parameter(name_ + ".stvl_map_file", _stvl_map_file);
  // enable loading of nav data
  declareParameter("should_load_navigation_data", rclcpp::ParameterValue(false));
  node->get_parameter(name_ + ".should_load_navigation_data", _should_load_navigation_data);
  // if mapping, how often to save a map for safety
  declareParameter("map_save_duration", rclcpp::ParameterValue(60.0));
  node->get_parameter(name_ + ".map_save_duration", map_save_time);
  RCLCPP_INFO(
    logger_,
    "%s loaded parameters from parameter server.", getName().c_str());
  if (_mapping_mode) {
    _map_save_duration = std::make_unique<rclcpp::Duration>(
      map_save_time, 0.0);
  }
  _last_map_save_time = node->now();

  if (track_unknown_space) {
    default_value_ = nav2_costmap_2d::NO_INFORMATION;
  } else {
    default_value_ = nav2_costmap_2d::FREE_SPACE;
  }

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;

  auto pub_opt = rclcpp::PublisherOptions();
  sub_opt.callback_group = callback_group_;

  _voxel_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    getName() + "/voxel_grid", rclcpp::QoS(1), pub_opt);

  auto save_grid_callback = std::bind(
    &SpatioTemporalVoxelLayer::SaveGridCallback, this, _1, _2, _3);
  _grid_saver = node->create_service<spatio_temporal_voxel_layer::srv::SaveGrid>(
    getName() + "/save_grid", save_grid_callback, rmw_qos_profile_services_default, callback_group_);

  auto clear_grid_around_pose_callback = std::bind(
    &SpatioTemporalVoxelLayer::ClearGridAroundPoseCallback, this, _1, _2, _3);
  _clear_grid_around_pose_srv = node->create_service<nav2_msgs::srv::ClearGridAroundPose>(
    getName() + "/clear_grid_around_pose", clear_grid_around_pose_callback, rmw_qos_profile_services_default, callback_group_);

  auto save_stvl_map_callback = std::bind(
    &SpatioTemporalVoxelLayer::SaveStvlMapCallback, this, _1, _2, _3);
  _save_stvl_map_srv = node->create_service<std_srvs::srv::Trigger>(
    getName() + "/spatiotemporal_voxel_grid/save_stvl_map", save_stvl_map_callback, rmw_qos_profile_services_default, callback_group_);

  auto erase_stvl_map_callback = std::bind(
    &SpatioTemporalVoxelLayer::EraseStvlMapCallback, this, _1, _2, _3);
  _erase_stvl_map_srv = node->create_service<std_srvs::srv::Trigger>(
    getName() + "/spatiotemporal_voxel_grid/erase_stvl_map", erase_stvl_map_callback, rmw_qos_profile_services_default, callback_group_);

  auto clear_entire_grid_callback = std::bind(
    &SpatioTemporalVoxelLayer::ClearEntireGridCallback, this, _1, _2, _3);
  _clear_entire_grid_srv = node->create_service<std_srvs::srv::Trigger>(
    getName() + "/spatiotemporal_voxel_grid/clear_entire_grid", clear_entire_grid_callback, rmw_qos_profile_services_default, callback_group_);

  if(_mapping_mode)
  {
    InitializeVoxelGrid(node->get_clock());
  }
  else
  {
    _voxel_grid = std::make_unique<volume_grid::SpatioTemporalVoxelGrid>(
      node->get_clock(), _voxel_size, static_cast<double>(default_value_), _decay_model,
      _voxel_decay, _publish_voxels, false);
  }

  matchSize();

  RCLCPP_INFO(logger_, "%s created underlying voxel grid.", getName().c_str());

  std::stringstream ss(_topics_string);
  std::string source;
  while (ss >> source) {
    // get the parameters for the specific topic
    double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
    double min_z, max_z, vFOV, vFOVPadding, base_length, base_width;
    double hFOV, decay_acceleration, obstacle_range;
    std::string topic, sensor_frame, data_type, filter_str;
    bool inf_is_valid = false, disable_decay_inside_frustum, clearing, marking;
    bool clear_after_reading, enabled;
    int voxel_min_points;
    buffer::Filters filter;

    declareParameter(source + "." + "topic", rclcpp::ParameterValue(std::string("")));
    declareParameter(source + "." + "sensor_frame", rclcpp::ParameterValue(std::string("")));
    declareParameter(source + "." + "observation_persistence", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "expected_update_rate", rclcpp::ParameterValue(0.0));
    declareParameter(
      source + "." + "data_type",
      rclcpp::ParameterValue(std::string("PointCloud2")));
    declareParameter(source + "." + "min_obstacle_height", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "max_obstacle_height", rclcpp::ParameterValue(3.0));
    declareParameter(source + "." + "inf_is_valid", rclcpp::ParameterValue(false));
    declareParameter(source + "." + "marking", rclcpp::ParameterValue(true));
    declareParameter(source + "." + "clearing", rclcpp::ParameterValue(false));
    declareParameter(source + "." + "obstacle_range", rclcpp::ParameterValue(2.5));

    declareParameter(source + "." + "min_z", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "max_z", rclcpp::ParameterValue(10.0));
    declareParameter(source + "." + "vertical_fov_angle", rclcpp::ParameterValue(0.7));
    declareParameter(source + "." + "vertical_fov_padding", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "horizontal_fov_angle", rclcpp::ParameterValue(1.04));
    declareParameter(source + "." + "base_length", rclcpp::ParameterValue(0.1));
    declareParameter(source + "." + "base_width", rclcpp::ParameterValue(0.1));
    declareParameter(source + "." + "decay_acceleration", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "disable_decay_inside_frustum", rclcpp::ParameterValue(false));
    declareParameter(source + "." + "filter", rclcpp::ParameterValue(std::string("passthrough")));
    declareParameter(source + "." + "voxel_min_points", rclcpp::ParameterValue(0));
    declareParameter(source + "." + "clear_after_reading", rclcpp::ParameterValue(false));
    declareParameter(source + "." + "enabled", rclcpp::ParameterValue(true));
    declareParameter(source + "." + "model_type", rclcpp::ParameterValue(0));

    node->get_parameter(name_ + "." + source + "." + "topic", topic);
    node->get_parameter(name_ + "." + source + "." + "sensor_frame", sensor_frame);
    node->get_parameter(
      name_ + "." + source + "." + "observation_persistence",
      observation_keep_time);
    node->get_parameter(
      name_ + "." + source + "." + "expected_update_rate",
      expected_update_rate);
    node->get_parameter(name_ + "." + source + "." + "data_type", data_type);
    node->get_parameter(name_ + "." + source + "." + "min_obstacle_height", min_obstacle_height);
    node->get_parameter(name_ + "." + source + "." + "max_obstacle_height", max_obstacle_height);
    node->get_parameter(name_ + "." + source + "." + "inf_is_valid", inf_is_valid);
    node->get_parameter(name_ + "." + source + "." + "marking", marking);
    node->get_parameter(name_ + "." + source + "." + "clearing", clearing);
    node->get_parameter(name_ + "." + source + "." + "obstacle_range", obstacle_range);

    // minimum distance from camera it can see
    node->get_parameter(name_ + "." + source + "." + "min_z", min_z);
    // maximum distance from camera it can see
    node->get_parameter(name_ + "." + source + "." + "max_z", max_z);
    // vertical FOV angle in rad
    node->get_parameter(name_ + "." + source + "." + "vertical_fov_angle", vFOV);
    // vertical FOV padding in meters (3D lidar frustum only)
    node->get_parameter(name_ + "." + source + "." + "vertical_fov_padding", vFOVPadding);
    // horizontal FOV angle in rad
    node->get_parameter(name_ + "." + source + "." + "horizontal_fov_angle", hFOV);
    // length of the base for virtual proximity shield
    node->get_parameter(name_ + "." + source + "." + "base_length", base_length);
    // width of the base for virtual proximity shield
    node->get_parameter(name_ + "." + source + "." + "base_width", base_width);
    // acceleration scales the model's decay in presence of readings
    node->get_parameter(name_ + "." + source + "." + "decay_acceleration", decay_acceleration);
    // makes the voxels within frustum stay persistent (not decay)
    node->get_parameter(name_ + "." + source + "." + "disable_decay_inside_frustum", disable_decay_inside_frustum);
    // performs an approximate voxel filter over the data to reduce
    node->get_parameter(name_ + "." + source + "." + "filter", filter_str);
    // minimum points per voxel for voxel filter
    node->get_parameter(name_ + "." + source + "." + "voxel_min_points", voxel_min_points);
    // clears measurement buffer after reading values from it
    node->get_parameter(name_ + "." + source + "." + "clear_after_reading", clear_after_reading);
    // Whether the frustum is enabled on startup. Can be toggled with service
    node->get_parameter(name_ + "." + source + "." + "enabled", enabled);
    // model type - default depth camera frustum model
    int model_type_int = 0;
    node->get_parameter(name_ + "." + source + "." + "model_type", model_type_int);
    ModelType model_type = static_cast<ModelType>(model_type_int);

    if (filter_str == "passthrough") {
      RCLCPP_INFO(logger_, "Passthough filter activated.");
      filter = buffer::Filters::PASSTHROUGH;
    } else if (filter_str == "voxel") {
      RCLCPP_INFO(logger_, "Voxel filter activated.");
      filter = buffer::Filters::VOXEL;
    } else {
      RCLCPP_INFO(logger_, "No filters activated.");
      filter = buffer::Filters::NONE;
    }

    if (!(data_type == "PointCloud2" || data_type == "LaserScan")) {
      throw std::runtime_error(
              "Only topics that use pointclouds or laser scans are supported.");
    }

    // create an observation buffer
    _observation_buffers.push_back(
      std::shared_ptr<buffer::MeasurementBuffer>(
        new buffer::MeasurementBuffer(
          source, topic,
          observation_keep_time, expected_update_rate, min_obstacle_height,
          max_obstacle_height, obstacle_range, *tf_, _global_frame, sensor_frame,
          transform_tolerance, min_z, max_z, vFOV, vFOVPadding, hFOV, base_length, base_width,
          decay_acceleration, disable_decay_inside_frustum, marking, clearing, _voxel_size,
          filter, voxel_min_points, enabled, clear_after_reading, model_type,
          node->get_clock(), node->get_logger())));

    // Add buffer to marking observation buffers
    if (marking) {
      _marking_buffers.push_back(_observation_buffers.back());
    }

    // Add buffer to clearing observation buffers
    if (clearing) {
      _clearing_buffers.push_back(_observation_buffers.back());
    }

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = 50;

    // create a callback for the topic
    if (data_type == "LaserScan") {
      auto sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
          rclcpp_lifecycle::LifecycleNode>>(node, topic, custom_qos_profile, sub_opt);

      std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>
      > filter(new tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>(
          *sub, *tf_, _global_frame, 50,
                 node->get_node_logging_interface(),
                 node->get_node_clock_interface(),
                 tf2::durationFromSec(transform_tolerance)));

      if (inf_is_valid) {
        filter->registerCallback(
          std::bind(
            &SpatioTemporalVoxelLayer::LaserScanValidInfCallback,
            this, _1, _observation_buffers.back()));
      } else {
        filter->registerCallback(
          std::bind(
            &SpatioTemporalVoxelLayer::LaserScanCallback,
            this, _1, _observation_buffers.back()));
      }

      _observation_subscribers.push_back(sub);
      _observation_notifiers.push_back(filter);

      _observation_notifiers.back()->setTolerance(rclcpp::Duration::from_seconds(0.05));
    } else if (data_type == "PointCloud2") {
      auto sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2,
          rclcpp_lifecycle::LifecycleNode>>(node, topic, custom_qos_profile, sub_opt);

      std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>
      > filter(new tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>(
          *sub, *tf_, _global_frame, 50,
                 node->get_node_logging_interface(),
                 node->get_node_clock_interface(),
                 tf2::durationFromSec(transform_tolerance)));
      filter->registerCallback(
        std::bind(
          &SpatioTemporalVoxelLayer::PointCloud2Callback, this, _1,
          _observation_buffers.back()));

      _observation_subscribers.push_back(sub);
      _observation_notifiers.push_back(filter);
    }

    std::function<void(const std::shared_ptr<rmw_request_id_t>,
      std_srvs::srv::SetBool::Request::SharedPtr,
      std_srvs::srv::SetBool::Response::SharedPtr)> toggle_srv_callback;

    toggle_srv_callback = std::bind(
      &SpatioTemporalVoxelLayer::BufferEnablerCallback, this,
      _1, _2, _3, _observation_buffers.back(),
      _observation_subscribers.back());
    std::string toggle_topic = source + "/toggle_enabled";
    auto server = node->create_service<std_srvs::srv::SetBool>(
      toggle_topic, toggle_srv_callback, rmw_qos_profile_services_default, callback_group_);

    _buffer_enabler_servers.push_back(server);

    if (sensor_frame != "") {
      std::vector<std::string> target_frames;
      target_frames.reserve(2);
      target_frames.push_back(_global_frame);
      target_frames.push_back(sensor_frame);
      _observation_notifiers.back()->setTargetFrames(target_frames);
    }
  }

  current_ = true;
  was_reset_ = false;

  RCLCPP_INFO(logger_, "%s initialization complete!", getName().c_str());
}

void SpatioTemporalVoxelLayer::InitializeVoxelGrid(const std::shared_ptr<rclcpp::Clock> & clock)
{
  bool should_load_navigation_data = _should_load_navigation_data;
  bool stvl_ready = false;
  while (!stvl_ready)
  {
    try
    {
    	_voxel_grid.reset();
      
      if (should_load_navigation_data)
      {
        RCLCPP_INFO(logger_, "Attempting to load STVL map.");
        _voxel_grid = std::make_unique<volume_grid::SpatioTemporalVoxelGrid>(clock, \
                                                                           _voxel_size, \
                                                                           (double)default_value_, \
                                                                           _decay_model, \
                                                                           _voxel_decay, \
                                                                           _publish_voxels, \
                                                                           true, \
                                                                           _stvl_map_file);
      }
      else
      {
        RCLCPP_INFO(logger_, "STVL starting clean.");
        _voxel_grid = std::make_unique<volume_grid::SpatioTemporalVoxelGrid>(clock, \
                                                                           _voxel_size, \
                                                                           (double)default_value_, \
                                                                           _decay_model, \
                                                                           _voxel_decay, \
                                                                           _publish_voxels, \
                                                                           false);
      }

      stvl_ready = true;
    }
    catch (const std::exception& e)
    {
      RCLCPP_INFO_STREAM(logger_, "Failed to start STVL: " << e.what()
                        << "\n\nWill retry without loading navigation data.");
      should_load_navigation_data = false;
    }
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::LaserScanCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr message,
  const std::shared_ptr<buffer::MeasurementBuffer> & buffer)
/*****************************************************************************/
{
  if (!buffer->IsEnabled()) {
    return;
  }
  // laser scan where infinity is invalid callback function
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = message->header;
  try {
    _laser_projector.transformLaserScanToPointCloud(
      message->header.frame_id, *message, cloud, *tf_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      logger_,
      "TF returned a transform exception to frame %s: %s",
      _global_frame.c_str(), ex.what());
    _laser_projector.projectLaser(*message, cloud);
  }
  // buffer the point cloud
  buffer->Lock();
  buffer->BufferROSCloud(cloud);
  buffer->Unlock();
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::LaserScanValidInfCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr raw_message,
  const std::shared_ptr<buffer::MeasurementBuffer> & buffer)
/*****************************************************************************/
{
  if (!buffer->IsEnabled()) {
    return;
  }
  // Filter infinity to max_range
  float epsilon = 0.0001;
  sensor_msgs::msg::LaserScan message = *raw_message;
  for (size_t i = 0; i < message.ranges.size(); i++) {
    float range = message.ranges[i];
    if (!std::isfinite(range) && range > 0) {
      message.ranges[i] = message.range_max - epsilon;
    }
  }
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = message.header;
  try {
    _laser_projector.transformLaserScanToPointCloud(
      message.header.frame_id, message, cloud, *tf_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      logger_,
      "TF returned a transform exception to frame %s: %s",
      _global_frame.c_str(), ex.what());
    _laser_projector.projectLaser(message, cloud);
  }
  // buffer the point cloud
  buffer->Lock();
  buffer->BufferROSCloud(cloud);
  buffer->Unlock();
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::PointCloud2Callback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr message,
  const std::shared_ptr<buffer::MeasurementBuffer> & buffer)
/*****************************************************************************/
{
  if (!buffer->IsEnabled()) {
    return;
  }
  // buffer the point cloud
  buffer->Lock();
  buffer->BufferROSCloud(*message);
  buffer->Unlock();
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::BufferEnablerCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response,
  const std::shared_ptr<buffer::MeasurementBuffer> buffer,
  const std::shared_ptr<message_filters::SubscriberBase<rclcpp_lifecycle::LifecycleNode>> &subcriber
  )
/*****************************************************************************/
{
  buffer->Lock();
  if (buffer->IsEnabled() != request->data) {
    buffer->SetEnabled(request->data);
    if (request->data) {
      subcriber->subscribe();
      buffer->ResetLastUpdatedTime();
      response->message = "Enabling sensor";
    } else if (subcriber) {
      subcriber->unsubscribe();
      response->message = "Disabling sensor";
    }
  } else {
    response->message = "Sensor already in the required state doing nothing";
  }
  buffer->Unlock();
  response->success = true;
}


/*****************************************************************************/
bool SpatioTemporalVoxelLayer::GetMarkingObservations(
  std::vector<observation::MeasurementReading> & marking_observations) const
/*****************************************************************************/
{
  // get marking observations and static marked areas
  bool current = true;

  for (unsigned int i = 0; i != _marking_buffers.size(); ++i) {
    _marking_buffers[i]->Lock();
    _marking_buffers[i]->GetReadings(marking_observations);
    current = _marking_buffers[i]->UpdatedAtExpectedRate();
    _marking_buffers[i]->Unlock();
  }
  marking_observations.insert(
    marking_observations.end(),
    _static_observations.begin(), _static_observations.end());
  return current;
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayer::GetClearingObservations(
  std::vector<observation::MeasurementReading> & clearing_observations) const
/*****************************************************************************/
{
  // get clearing observations
  bool current = true;
  for (unsigned int i = 0; i != _clearing_buffers.size(); ++i) {
    _clearing_buffers[i]->Lock();
    _clearing_buffers[i]->GetReadings(clearing_observations);
    current = _clearing_buffers[i]->UpdatedAtExpectedRate();
    _clearing_buffers[i]->Unlock();
  }
  return current;
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::ObservationsResetAfterReading() const
/*****************************************************************************/
{
  for (unsigned int i = 0; i != _clearing_buffers.size(); ++i) {
    _clearing_buffers[i]->Lock();
    if (_clearing_buffers[i]->ClearAfterReading()) {
      _clearing_buffers[i]->ResetAllMeasurements();
    }
    _clearing_buffers[i]->Unlock();
  }

  for (unsigned int i = 0; i != _marking_buffers.size(); ++i) {
    _marking_buffers[i]->Lock();
    if (_marking_buffers[i]->ClearAfterReading()) {
      _marking_buffers[i]->ResetAllMeasurements();
    }
    _marking_buffers[i]->Unlock();
  }
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y, double * max_x, double * max_y)
/*****************************************************************************/
{
  nav2_costmap_2d::transformFootprint(
    robot_x, robot_y, robot_yaw,
    getFootprint(), _transformed_footprint);
  for (unsigned int i = 0; i < _transformed_footprint.size(); i++) {
    touch(
      _transformed_footprint[i].x, _transformed_footprint[i].y,
      min_x, min_y, max_x, max_y);
  }

  return true;
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::activate(void)
/*****************************************************************************/
{
  // subscribe and place info in buffers from sensor sources
  RCLCPP_INFO(logger_, "%s was activated.", getName().c_str());

  observation_subscribers_iter sub_it = _observation_subscribers.begin();
  for (; sub_it != _observation_subscribers.end(); ++sub_it) {
    (*sub_it)->subscribe();
  }

  observation_buffers_iter buf_it = _observation_buffers.begin();
  for (; buf_it != _observation_buffers.end(); ++buf_it) {
    (*buf_it)->ResetLastUpdatedTime();
  }
  
  auto node = node_.lock();

  if (_clear_grid_under_footprint_in_manual_mode) {
    is_in_manual_mode_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      "/robo_cart/is_in_manual_mode", rclcpp::SystemDefaultsQoS(),
      std::bind(&SpatioTemporalVoxelLayer::isInManualModeCb, this, _1));
  }

  // Add callback for dynamic parametrs
  dyn_params_handler = node->add_on_set_parameters_callback(
    std::bind(&SpatioTemporalVoxelLayer::dynamicParametersCallback, this, _1));
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::deactivate(void)
/*****************************************************************************/
{
  // unsubscribe from all sensor sources
  RCLCPP_INFO(logger_, "%s was deactivated.", getName().c_str());

  observation_subscribers_iter sub_it = _observation_subscribers.begin();
  for (; sub_it != _observation_subscribers.end(); ++sub_it) {
    if (*sub_it != nullptr) {
      (*sub_it)->unsubscribe();
    }
  }
  dyn_params_handler.reset();
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::reset(void)
/*****************************************************************************/
{
  boost::recursive_mutex::scoped_lock lock(_voxel_grid_lock);
  // reset layer
  Costmap2D::resetMaps();
  // Removed to stop the global voxel grid from being deleted
  // this->ResetGrid();

  current_ = false;
  was_reset_ = true;

  // Removed to stop the global voxel grid from being deleted
  // observation_buffers_iter it = _observation_buffers.begin();
  // for (; it != _observation_buffers.end(); ++it) {
  //   (*it)->ResetLastUpdatedTime();
  // }
  is_in_manual_mode_ = true;
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayer::AddStaticObservations(
  const observation::MeasurementReading & obs)
/*****************************************************************************/
{
  // observations to always be added to the map each update cycle not marked
  RCLCPP_INFO(
    logger_,
    "%s: Adding static observation to map.", getName().c_str());

  try {
    _static_observations.push_back(obs);
    return true;
  } catch (...) {
    RCLCPP_WARN(
      logger_,
      "Could not add static observations to voxel layer");
    return false;
  }
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayer::RemoveStaticObservations(void)
/*****************************************************************************/
{
  // kill all static observations added to each update cycle
  RCLCPP_INFO(
    logger_,
    "%s: Removing static observations to map.", getName().c_str());

  try {
    _static_observations.clear();
    return true;
  } catch (...) {
    RCLCPP_WARN(
      logger_,
      "Couldn't remove static observations from %s.", getName().c_str());
    return false;
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::ResetGrid(void)
/*****************************************************************************/
{
  if (!_voxel_grid->ResetGrid()) {
    RCLCPP_WARN(logger_, "Did not clear level set in %s!", getName().c_str());
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::matchSize(void)
/*****************************************************************************/
{
  // match the master costmap size, volume_grid maintains full w/ expiration.
  CostmapLayer::matchSize();
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
/*****************************************************************************/
{
  // update costs in master_grid with costmap_
  if (!_enabled) {
    return;
  }

  // if not current due to reset, set current now after clearing
  if (!current_ && was_reset_) {
    was_reset_ = false;
    current_ = true;
  }

  if (_clear_costmap_under_footprint) {
    setConvexPolygonCost(_transformed_footprint, nav2_costmap_2d::FREE_SPACE);
  }

  switch (_combination_method) {
    case 0:
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:
      break;
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::UpdateROSCostmap(
  double * min_x, double * min_y, double * max_x, double * max_y,
  std::unordered_set<volume_grid::occupany_cell> & cleared_cells)
/*****************************************************************************/
{
  // grabs map of occupied cells from grid and adds to costmap_
  Costmap2D::resetMaps();

  std::unordered_map<volume_grid::occupany_cell, uint>::iterator it;
  for (it = _voxel_grid->GetFlattenedCostmap()->begin();
    it != _voxel_grid->GetFlattenedCostmap()->end(); ++it)
  {
    uint map_x, map_y;
    if (static_cast<int>(it->second) >= _mark_threshold &&
      worldToMap(it->first.x, it->first.y, map_x, map_y))
    {
      costmap_[getIndex(map_x, map_y)] = nav2_costmap_2d::LETHAL_OBSTACLE;
      touch(it->first.x, it->first.y, min_x, min_y, max_x, max_y);
    }
  }

  std::unordered_set<volume_grid::occupany_cell>::iterator cell;
  for (cell = cleared_cells.begin(); cell != cleared_cells.end(); ++cell)
  {
    touch(cell->x, cell->y, min_x, min_y, max_x, max_y);
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
/*****************************************************************************/
{
  // grabs new max bounds for the costmap
  if (!_enabled) {
    return;
  }

  // Required because UpdateROSCostmap will also lock if AFTER we lock here voxel_grid_lock,
  // and if clearArea is called in between, we will have a deadlock
  boost::unique_lock<mutex_t> cm_lock(*getMutex());

  boost::recursive_mutex::scoped_lock lock(_voxel_grid_lock);

  // Steve's Note June 22, 2018
  // I dislike this necessity, I can't remove the master grid's knowledge about
  // STVL on the fly so I have play games with the API even though this isn't
  // really a rolling plugin implementation. It works, but isn't ideal.
  if (layered_costmap_->isRolling()) {
    updateOrigin(
      robot_x - getSizeInMetersX() / 2,
      robot_y - getSizeInMetersY() / 2);
  }

  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  std::vector<observation::MeasurementReading> marking_observations,
    clearing_observations;
  current = GetMarkingObservations(marking_observations) && current;
  current = GetClearingObservations(clearing_observations) && current;
  ObservationsResetAfterReading();
  current_ = current;

  std::unordered_set<volume_grid::occupany_cell> cleared_cells;

  // navigation mode: clear observations, mapping mode: save maps and publish
  bool should_save = false;
  auto node = node_.lock();
  if (_map_save_duration) {
    should_save = node->now() - _last_map_save_time > *_map_save_duration;
  }
  
  _voxel_grid->ClearFrustums(clearing_observations, cleared_cells);

  if (_mapping_mode && _autosaving_enabled && should_save) {    
    _last_map_save_time = node->now();

    auto response =
      std::make_shared<std_srvs::srv::Trigger::Response>();
    SaveStvlMapCallback(nullptr, nullptr, response);
  }

  // mark observations
  _voxel_grid->Mark(marking_observations);



  // update the ROS Layered Costmap
  UpdateROSCostmap(min_x, min_y, max_x, max_y, cleared_cells);

  // publish point cloud
  if (_publish_voxels) {
    std::unique_ptr<sensor_msgs::msg::PointCloud2> pc2 =
      std::make_unique<sensor_msgs::msg::PointCloud2>();
    _voxel_grid->GetOccupancyPointCloud(pc2);
    pc2->header.frame_id = _global_frame;
    pc2->header.stamp = node->now();
    _voxel_pub->publish(*pc2);
  }
  
  // update footprint
  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);

  if (is_in_manual_mode_ && _clear_grid_under_footprint_in_manual_mode) {
    clearVoxelGridInsidePolygon(_transformed_footprint);
  }

  if (_auto_grid_clear_range > 0) {
    clearSquareRegion(robot_x, robot_y, _auto_grid_clear_range);
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::SaveGridCallback(
  const std::shared_ptr<rmw_request_id_t>/*header*/,
  const std::shared_ptr<spatio_temporal_voxel_layer::srv::SaveGrid::Request> req,
  std::shared_ptr<spatio_temporal_voxel_layer::srv::SaveGrid::Response> resp)
/*****************************************************************************/
{
  boost::recursive_mutex::scoped_lock lock(_voxel_grid_lock);
  double map_size_bytes;

  if (_voxel_grid->SaveGrid(req->file_name, map_size_bytes) ) {
    RCLCPP_INFO(
      logger_,
      "SpatioTemporalVoxelLayer: Saved %s grid! Has memory footprint of %f bytes.",
      req->file_name.c_str(), map_size_bytes);
    resp->map_size_bytes = map_size_bytes;
    resp->status = true;
    return;
  }

  RCLCPP_WARN(logger_, "SpatioTemporalVoxelLayer: Failed to save grid.");
  resp->status = false;
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::ClearGridAroundPoseCallback(
  const std::shared_ptr<rmw_request_id_t>/*header*/,
  std::shared_ptr<nav2_msgs::srv::ClearGridAroundPose::Request> req,
  std::shared_ptr<nav2_msgs::srv::ClearGridAroundPose::Response> resp)
/*****************************************************************************/
{
  boost::recursive_mutex::scoped_lock lock(_voxel_grid_lock);
  try
  {
    clearCostmapLayerAroundPose(req->pose.pose.position.x, req->pose.pose.position.y, req->reset_distance);
    resp->status = true;
    return;
  }
  catch(const std::exception& e)
  {
    RCLCPP_WARN_STREAM(logger_, "SpatioTemporalVoxelLayer: Failed to remove grid around pose with exception: " << e.what());
  }
  resp->status = false;
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::SaveStvlMapCallback(
  const std::shared_ptr<rmw_request_id_t>/*header*/,
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
/*****************************************************************************/
{
  if(!_enabled)
  {
    resp->message = "STVL layer disabled, skipping";
    resp->success = true;
    return;
  }

  if(!_mapping_mode)
  {
    resp->message = "STVL not in mapping mode, no map to save, skipping";
    resp->success = true;
    return;
  }
  
  boost::recursive_mutex::scoped_lock lock(_voxel_grid_lock);
  double map_size_bytes;

  if( _voxel_grid->SaveGrid(_stvl_map_file, map_size_bytes) )
  {
    RCLCPP_INFO(       
      logger_,
      "SpatioTemporalVoxelGrid: Saved %s grid! Has memory footprint of %f bytes.",
      _stvl_map_file.c_str(), map_size_bytes);
    resp->message = "STVL map saved";
    resp->success = true;
    return;
  }

  resp->message = "STVL failed to save map";
  resp->success = false;
  return;
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::EraseStvlMapCallback(
  const std::shared_ptr<rmw_request_id_t>/*header*/,
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
/*****************************************************************************/
{
  if(!_mapping_mode)
  {
    resp->message = "STVL not in mapping mode, no map to erase, skipping";
    resp->success = true;
    return;
  }
  
  try
  {
    if (std::filesystem::remove(_stvl_map_file))
    {
      resp->success = true;
      resp->message = "STVL map file deleted";
    }
    else
    {
      // If map file doesn't exist it is still success - there is no map
      resp->success = true;
      resp->message = "Can't delete stvl map file, maybe it doesn't exist?";
    }
  }
  catch (const std::filesystem::filesystem_error& e)
  {
    resp->success = false;
    resp->message = std::string("Can't delete map, exception occurred: ") + e.what();
    return;
  }

  if (!_voxel_grid->ResetGrid())
  {
    resp->success = false;
    resp->message = "Failed to clear current stvl map";
  }

  return;
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::ClearEntireGridCallback(
  const std::shared_ptr<rmw_request_id_t>/*header*/,
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
/*****************************************************************************/
{
  if (!_voxel_grid->ResetGrid())
  {
    resp->success = false;
    resp->message = "Failed to clear current stvl map";
    RCLCPP_ERROR(       
      logger_,
      "SpatioTemporalVoxelGrid: ClearEntireGridCallback: %s",
      resp->message.c_str());
    return;
  }
  resp->success = true;
  resp->message = "Stvl grid cleared successfully";
  RCLCPP_INFO(       
    logger_,
    "SpatioTemporalVoxelGrid: ClearEntireGridCallback: %s ",
    resp->message.c_str());
  return;
}


rcl_interfaces::msg::SetParametersResult
SpatioTemporalVoxelLayer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    std::stringstream ss(_topics_string);
    std::string source;
    while (ss >> source) {
      if (type == ParameterType::PARAMETER_DOUBLE) {
        if (name == name_ + "." + source + "." + "min_obstacle_height") {
          for (auto & buffer : _observation_buffers) {
            if (buffer->GetSourceName() == source) {
              buffer->Lock();
              buffer->SetMinObstacleHeight(parameter.as_double());
              buffer->Unlock();
            }
          }
        } else if (name == name_ + "." + source + "." + "max_obstacle_height") {
          for (auto & buffer : _observation_buffers) {
            if (buffer->GetSourceName() == source) {
              buffer->Lock();
              buffer->SetMaxObstacleHeight(parameter.as_double());
              buffer->Unlock();
            }
          }
        } else if (name == name_ + "." + source + "." + "min_z") {
          for (auto & buffer : _observation_buffers) {
            if (buffer->GetSourceName() == source) {
              buffer->Lock();
              buffer->SetMinZ(parameter.as_double());
              buffer->Unlock();
            }
          }
        } else if (name == name_ + "." + source + "." + "max_z") {
          for (auto & buffer : _observation_buffers) {
            if (buffer->GetSourceName() == source) {
              buffer->Lock();
              buffer->SetMaxZ(parameter.as_double());
              buffer->Unlock();
            }
          }
        } else if (name == name_ + "." + source + "." + "vertical_fov_angle") {
          for (auto & buffer : _observation_buffers) {
            if (buffer->GetSourceName() == source) {
              buffer->Lock();
              buffer->SetVerticalFovAngle(parameter.as_double());
              buffer->Unlock();
            }
          }
        } else if (name == name_ + "." + source + "." + "vertical_fov_padding") {
          for (auto & buffer : _observation_buffers) {
            if (buffer->GetSourceName() == source) {
              buffer->Lock();
              buffer->SetVerticalFovPadding(parameter.as_double());
              buffer->Unlock();
            }
          }
        } else if (name == name_ + "." + source + "." + "horizontal_fov_angle") {
          for (auto & buffer : _observation_buffers) {
            if (buffer->GetSourceName() == source) {
              buffer->Lock();
              buffer->SetHorizontalFovAngle(parameter.as_double());
              buffer->Unlock();
            }
          }
        } else if (name == name_ + "." + source + "." + "base_length") {
          for (auto & buffer : _observation_buffers) {
            if (buffer->GetSourceName() == source) {
              buffer->Lock();
              buffer->SetBaseLength(parameter.as_double());
              buffer->Unlock();
            }
          }
        } else if (name == name_ + "." + source + "." + "base_width") {
          for (auto & buffer : _observation_buffers) {
            if (buffer->GetSourceName() == source) {
              buffer->Lock();
              buffer->SetBaseWidth(parameter.as_double());
              buffer->Unlock();
            }
          }
        }
      }
    }

    if (type == ParameterType::PARAMETER_BOOL) {
      if (name == name_ + "." + "disable_decay_inside_frustum") {
          for (auto & buffer : _observation_buffers) {
            if (buffer->GetSourceName() == source) {
              buffer->Lock();
              buffer->SetFrustumPersistent(parameter.as_bool());
              buffer->Unlock();
            }
          }
      }
      if (name == name_ + "." + "enabled") {
        bool enable = parameter.as_bool();
        if (enabled_ != enable) {
          if (enable) {
            observation_subscribers_iter sub_it = _observation_subscribers.begin();
            for (; sub_it != _observation_subscribers.end(); ++sub_it) {
              if (*sub_it != nullptr) {
                (*sub_it)->subscribe();
              }
            }

            observation_buffers_iter buf_it = _observation_buffers.begin();
            for (; buf_it != _observation_buffers.end(); ++buf_it) {
              if (*buf_it != nullptr) {
                (*buf_it)->ResetLastUpdatedTime();
              }
            }
          } else {
            observation_subscribers_iter sub_it = _observation_subscribers.begin();
            for (; sub_it != _observation_subscribers.end(); ++sub_it) {
              if (*sub_it != nullptr) {
                (*sub_it)->unsubscribe();
              }
            }
          }
        }
        enabled_ = enable;
      }
    }

    if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == name_ + "." + "mark_threshold") {
        _mark_threshold = parameter.as_int();
      }
    }
  }

  result.successful = true;
  return result;
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::clearArea(
  int start_x, int start_y, int end_x, int end_y, bool invert_area)
/*****************************************************************************/
{
  // convert map coords to world coords
  volume_grid::occupany_cell start_world(0, 0);
  volume_grid::occupany_cell end_world(0, 0);
  mapToWorld(start_x, start_y, start_world.x, start_world.y);
  mapToWorld(end_x, end_y, end_world.x, end_world.y);

  boost::recursive_mutex::scoped_lock lock(_voxel_grid_lock);
  _voxel_grid->ResetGridArea(start_world, end_world, invert_area);
  CostmapLayer::clearArea(start_x, start_y, end_x, end_y, invert_area);
}

void SpatioTemporalVoxelLayer::clearCostmapLayerAroundPose(
  double pose_x, double pose_y, double reset_distance)
{
  double start_point_x = pose_x - reset_distance / 2;
  double start_point_y = pose_y - reset_distance / 2;
  double end_point_x = start_point_x + reset_distance;
  double end_point_y = start_point_y + reset_distance;

  int start_x, start_y, end_x, end_y;
  this->worldToMapEnforceBounds(start_point_x, start_point_y, start_x, start_y);
  this->worldToMapEnforceBounds(end_point_x, end_point_y, end_x, end_y);

  // Clear area is called with invert=true as it esnures only the area inside is cleared
  clearArea(start_x, start_y, end_x, end_y, true);

  double ox = this->getOriginX(), oy = this->getOriginY();
  double width = this->getSizeInMetersX(), height = this->getSizeInMetersY();
  this->addExtraBounds(ox, oy, ox + width, oy + height);
}

void SpatioTemporalVoxelLayer::clearVoxelGridInsidePolygon(
  const std::vector<geometry_msgs::msg::Point> &polygon)
{
  if (polygon.empty()) {
      return;
  }

  std::vector<volume_grid::occupany_cell> polygon_occupancy_cell;
  for (const auto& point : polygon) {
    volume_grid::occupany_cell p{point.x, point.y};
    polygon_occupancy_cell.push_back(p);
  }
  
  // Reset the grid area defined by start and end occupancy cells
  // Using invert_area = true to clear the area strictly inside the defined bounding box
  _voxel_grid->ResetGridArea(polygon_occupancy_cell, true);
}

void SpatioTemporalVoxelLayer::isInManualModeCb(const std_msgs::msg::Bool::UniquePtr& msg)
{
  is_in_manual_mode_ = msg->data;
}

void SpatioTemporalVoxelLayer::clearSquareRegion(double robot_x, double robot_y, double clear_range)
{
    // Define the min and max bounds of the square region to clear
    double min_x = robot_x - clear_range;
    double max_x = robot_x + clear_range;
    double min_y = robot_y - clear_range;
    double max_y = robot_y + clear_range;

    // Create occupancy cells for the bounding box corners
    volume_grid::occupany_cell start{min_x, min_y};
    volume_grid::occupany_cell end{max_x, max_y};

    // Reset the grid area defined by start and end occupancy cells
    // Using invert_area = false, as we want to clear everything outside the square
    _voxel_grid->ResetGridArea(start, end, false);
}

}  // namespace spatio_temporal_voxel_layer

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  spatio_temporal_voxel_layer::SpatioTemporalVoxelLayer,
  nav2_costmap_2d::Layer)
