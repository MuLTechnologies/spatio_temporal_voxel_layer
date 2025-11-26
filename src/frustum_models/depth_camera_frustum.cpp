/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2018, Simbe Robotics, Inc.
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
 *********************************************************************/

#include <vector>
#include "spatio_temporal_voxel_layer/frustum_models/depth_camera_frustum.hpp"

namespace geometry
{

/*****************************************************************************/
DepthCameraFrustum::DepthCameraFrustum(
  const double & vFOV, const double & hFOV, const double & min_dist,
  const double & max_dist, const double & frustum_padding, const std::string & frustum_name, const bool & visualize_frustum)
: _vFOV(vFOV), _hFOV(hFOV), _min_d(min_dist), _max_d(max_dist), _frustum_padding(frustum_padding), _frustum_name(frustum_name), _visualize_frustum(visualize_frustum)
/*****************************************************************************/
{
  _valid_frustum = false;
  // Substract _frustum_padding from the _max_d to account for the moved origin
  _max_d = _max_d - _frustum_padding;
  this->ComputePlaneNormals();

  if(_visualize_frustum) {
    _node = std::make_shared<rclcpp::Node>("frustum_publisher");
    _frustum_pub = _node->create_publisher<visualization_msgs::msg::MarkerArray>("frustum", 10);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
}

/*****************************************************************************/
DepthCameraFrustum::~DepthCameraFrustum(void)
/*****************************************************************************/
{
}

/*****************************************************************************/
void DepthCameraFrustum::ComputePlaneNormals(void)
/*****************************************************************************/
{
  // give ability to construct with bogus values
  if (_vFOV == 0 && _hFOV == 0) {
      _valid_frustum = false;
      return;
  }

  // Define frustum origin with the _frustum_padding
  // The padding is applied by transposing the input frustum FOV forward (away from the camera in z axis),
  Eigen::Vector3d frustum_origin(0.0, 0.0, _frustum_padding);

  // Create frustum vertices
  std::vector<Eigen::Vector3d> pt_;
  pt_.reserve(8);

  Eigen::Vector3d vertex_pt(tan(_hFOV/2), tan(_vFOV/2), 1);
  pt_.push_back(vertex_pt * _min_d + frustum_origin);
  pt_.push_back(vertex_pt * _max_d + frustum_origin);

  vertex_pt = Eigen::Vector3d(-tan(_hFOV/2), tan(_vFOV/2), 1);
  pt_.push_back(vertex_pt * _min_d + frustum_origin);
  pt_.push_back(vertex_pt * _max_d + frustum_origin);

  vertex_pt = Eigen::Vector3d(-tan(_hFOV/2), -tan(_vFOV/2), 1);
  pt_.push_back(vertex_pt * _min_d + frustum_origin);
  pt_.push_back(vertex_pt * _max_d + frustum_origin);

  vertex_pt = Eigen::Vector3d(tan(_hFOV/2), -tan(_vFOV/2), 1);
  pt_.push_back(vertex_pt * _min_d + frustum_origin);
  pt_.push_back(vertex_pt * _max_d + frustum_origin);

  // cross each plane and get normals
  const Eigen::Vector3d v_01(pt_[1][0] - pt_[0][0],
    pt_[1][1] - pt_[0][1], pt_[1][2] - pt_[0][2]);
  const Eigen::Vector3d v_13(pt_[3][0] - pt_[1][0],
    pt_[3][1] - pt_[1][1], pt_[3][2] - pt_[1][2]);
  Eigen::Vector3d T_n(v_13.cross(v_01));
  T_n.normalize();
  _plane_normals.push_back(VectorWithPt3D(T_n[0], T_n[1], T_n[2], pt_[0]));

  const Eigen::Vector3d v_23(pt_[3][0] - pt_[2][0],
    pt_[3][1] - pt_[2][1], pt_[3][2] - pt_[2][2]);
  const Eigen::Vector3d v_35(pt_[5][0] - pt_[3][0],
    pt_[5][1] - pt_[3][1], pt_[5][2] - pt_[3][2]);
  Eigen::Vector3d T_l(v_35.cross(v_23));
  T_l.normalize();
  _plane_normals.push_back(VectorWithPt3D(T_l[0], T_l[1], T_l[2], pt_[2]));

  const Eigen::Vector3d v_45(pt_[5][0] - pt_[4][0],
    pt_[5][1] - pt_[4][1], pt_[5][2] - pt_[4][2]);
  const Eigen::Vector3d v_57(pt_[7][0] - pt_[5][0],
    pt_[7][1] - pt_[5][1], pt_[7][2] - pt_[5][2]);
  Eigen::Vector3d T_b(v_57.cross(v_45));
  T_b.normalize();
  _plane_normals.push_back(VectorWithPt3D(T_b[0], T_b[1], T_b[2], pt_[4]));

  const Eigen::Vector3d v_67(pt_[7][0] - pt_[6][0],
    pt_[7][1] - pt_[6][1], pt_[7][2] - pt_[6][2]);
  const Eigen::Vector3d v_71(pt_[1][0] - pt_[7][0],
    pt_[1][1] - pt_[7][1], pt_[1][2] - pt_[7][2]);
  Eigen::Vector3d T_r(v_71.cross(v_67));
  T_r.normalize();
  _plane_normals.push_back(VectorWithPt3D(T_r[0], T_r[1], T_r[2], pt_[6]));

  // far plane
  Eigen::Vector3d T_1(v_57.cross(v_71));
  T_1.normalize();
  _plane_normals.push_back(VectorWithPt3D(T_1[0], T_1[1], T_1[2], pt_[7]));

  // near plane
  _plane_normals.push_back(
    VectorWithPt3D(
      T_1[0], T_1[1], T_1[2], pt_[2]) * -1);

  _frustum_pts = pt_;

  assert(_plane_normals.size() == 6);
  _valid_frustum = true;

  // Store this initial state for future transformations
  _precomputed_plane_normals = _plane_normals;
}

/*****************************************************************************/
void DepthCameraFrustum::TransformModel()
/*****************************************************************************/
{
  if (!_valid_frustum) {
    return;
  }

  std::lock_guard<std::mutex> lock(_transform_mutex);

  // Reset the calculations to precomputed initial state
  _plane_normals = _precomputed_plane_normals;

  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.pretranslate(_orientation.inverse() * _position);
  T.prerotate(_orientation);

  std::vector<VectorWithPt3D>::iterator it;
  for (it = _plane_normals.begin(); it != _plane_normals.end(); ++it) {
    it->TransformFrames(T);
  }

  if (_visualize_frustum) {
    VisualizeFrustum();
  }
}

void DepthCameraFrustum::VisualizeFrustum() {
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.pretranslate(_orientation.inverse() * _position);
  T.prerotate(_orientation);

  visualization_msgs::msg::MarkerArray msg_list;
  visualization_msgs::msg::Marker msg;

  // frustum lines
  msg.header.frame_id = std::string("odom");  // Use global_frame of costmap
  msg.type = visualization_msgs::msg::Marker::LINE_LIST;
  msg.scale.x = 0.02;   // line width
  msg.pose.orientation.w = 1.0;
  msg.pose.position.x = 0;
  msg.pose.position.y = 0;
  msg.pose.position.z = 0;
  msg.header.stamp = _node->now();

  if (_frustum_padding != 0.0) {
    msg.color.b = 1.0f;
  } else { 
    msg.color.g = 1.0f;
  }

  msg.color.a = 1.0; 

  static const std::vector<std::vector<int>> v_t = \
  {{0,2}, {2,4}, {4,6}, {6,0}, {1,3}, {3,5}, {5,7}, {7,1}, {0,1}, {2,3}, {4,5}, {6,7}};

  for (uint i = 0; i != v_t.size(); i++) {
    for (uint j = 0; j != v_t[i].size(); j++) {
      Eigen::Vector3d T_pt = T * _frustum_pts.at(v_t[i][j]);
      geometry_msgs::msg::Point point;
      point.x = T_pt[0];
      point.y = T_pt[1];
      point.z = T_pt[2];
      msg.points.push_back(point);
    }
  }

  // Set namespace for all points
  msg.ns = _frustum_name;

  // Add the single marker to our list of markers
  msg_list.markers.push_back(msg);

  _frustum_pub->publish(msg_list);
}

/*****************************************************************************/
bool DepthCameraFrustum::IsInside(const openvdb::Vec3d & pt)
/*****************************************************************************/
{
  if (!_valid_frustum) {
    return false;
  }
  std::lock_guard<std::mutex> lock(_transform_mutex);

  std::vector<VectorWithPt3D>::iterator it;
  for (it = _plane_normals.begin(); it != _plane_normals.end(); ++it) {
    Eigen::Vector3d p_delta(pt[0] - it->initial_point[0],
      pt[1] - it->initial_point[1], pt[2] - it->initial_point[2]);
    p_delta.normalize();

    if (Dot(*it, p_delta) > 0.) {
      return false;
    }
  }
  return true;
}

/*****************************************************************************/
void DepthCameraFrustum::SetPosition(const geometry_msgs::msg::Point & origin)
/*****************************************************************************/
{
  std::lock_guard<std::mutex> lock(_transform_mutex);
  _position = Eigen::Vector3d(origin.x, origin.y, origin.z);
}

/*****************************************************************************/
void DepthCameraFrustum::SetOrientation(
  const geometry_msgs::msg::Quaternion & quat)
/*****************************************************************************/
{
  std::lock_guard<std::mutex> lock(_transform_mutex);
  _orientation = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
}

/*****************************************************************************/
double DepthCameraFrustum::Dot(
  const VectorWithPt3D & plane_pt, const openvdb::Vec3d & query_pt) const
/*****************************************************************************/
{
  return plane_pt.x * query_pt[0] +
         plane_pt.y * query_pt[1] +
         plane_pt.z * query_pt[2];
}

/*****************************************************************************/
double DepthCameraFrustum::Dot(
  const VectorWithPt3D & plane_pt, const Eigen::Vector3d & query_pt) const
/*****************************************************************************/
{
  return plane_pt.x * query_pt[0] +
         plane_pt.y * query_pt[1] +
         plane_pt.z * query_pt[2];
}

}  // namespace geometry
