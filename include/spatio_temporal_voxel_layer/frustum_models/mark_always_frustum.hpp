#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__FRUSTUM_MODELS__MARK_ALWAYS_FRUSTUM_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER__FRUSTUM_MODELS__MARK_ALWAYS_FRUSTUM_HPP_

// STL
#include <vector>
#include <mutex>
// STVL
#include "spatio_temporal_voxel_layer/frustum_models/frustum.hpp"

namespace geometry
{

// A class to model a always marking frustum
class MarkAlwaysFrustum : public Frustum
{
public:
  MarkAlwaysFrustum();
  virtual ~MarkAlwaysFrustum(void);
  virtual void TransformModel() {return;};
  virtual void VisualizeFrustum() {return;};
  virtual bool IsInside(const openvdb::Vec3d & pt) {return true;};
  virtual void SetPosition(const geometry_msgs::msg::Point & origin) {return;};
  virtual void SetOrientation(const geometry_msgs::msg::Quaternion & quat) {return;};
};

}  // namespace geometry

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__FRUSTUM_MODELS__MARK_ALWAYS_FRUSTUM_HPP_
