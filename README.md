# Spatio-Temporal Voxel Layer

This is a drop in replacement for the voxel_grid voxel representation of the environment. This package does a number of things to improve on the voxel grid package and extend the capabilities offered to the users, under a LGPL v2.1 license. Originally developed and maintained by [Steven Macenski](https://www.linkedin.com/in/steven-macenski-41a985101/) at [Simbe Robotics](http://www.simberobotics.com/). This fork is maintained by [MuL Technologies](https://www.multechnologies.com/) / [Husarion](https://husarion.com/) with additional features for the RCP (Robo Cart Platform).

This package sits on top of [OpenVDB](http://www.openvdb.org/), an open-source C++ library built by Dreamworks "comprising a novel hierarchical data structure and a suite of tools for the efficient storage and manipulation of sparse volumetric data discretized on three-dimensional grids. It is developed and maintained by DreamWorks Animation for use in volumetric applications typically encountered in feature film production."

Leveraging OpenVDB, we have the ability to efficiently maintain a 3 dimensional voxel-representative world space. We wrap this with ROS 2 tools and interfaces to [Nav2](https://docs.nav2.org/) to allow for use of this layer in standard ROS 2 configurations. It is certainly possible to utilize this package without ROS 2/Nav2 and I invite other competing methodologies to develop here and create interfaces.


Sample videos are shown below of a robot using **7 depth cameras** with less than 50% of a core, and another robot using a **VLP-16**.

7 Depth Cameras      |  VLP-16 LIDAR
:-------------------------:|:-------------------------:
![ezgif com-video-to-gif](https://user-images.githubusercontent.com/14944147/37010885-b18fe1f8-20bb-11e8-8c28-5b31e65f2844.gif) | ![vlp16](https://github.com/nickovaras/gifs/blob/master/follow.gif?raw=true)

We found in experimental trials with **6** 7hz dense stereo RGBD cameras we ran the major navigation process at **20-50%** nominal from **80-110%** on a 5th gen i7 CPU in the global costmap updating using the existing `voxel_layer`.

We've received feedback from users and have robots operating in the following environments with STVL:
- Retail
- Warehouses
- Factories
- Libraries
- Hospitals
- Hospitality
- RoboCup@Home
- Oil and Gas

Steve spoke at ROSCon 2018 about STVL and his presentation is [linked here](https://vimeo.com/292699571) (or click on image).

[![IMAGE ALT TEXT](https://user-images.githubusercontent.com/14944147/46768837-987c9280-cc9e-11e8-99ea-788d3d590dd8.png)](https://vimeo.com/292699571)

### Cite This Work

You can find this work [here](https://journals.sagepub.com/doi/10.1177/1729881420910530).

```
@article{doi:10.1177/1729881420910530,
    author = {Steve Macenski and David Tsai and Max Feinberg},
    title ={Spatio-temporal voxel layer: A view on robot perception for the dynamic world},
    journal = {International Journal of Advanced Robotic Systems},
    volume = {17},
    number = {2},
    year = {2020},
    doi = {10.1177/1729881420910530},
    URL = {https://doi.org/10.1177/1729881420910530}
}
```

## **Spatio**-
The Spatio in this package is the representation of the environment in a configurable `voxel_size` voxel grid stored and searched by OpenVDB.

In addition, buffered measurement readings have the option to run an approximate voxel grid filter, parameterizable at runtime in the configuration yamls. It is incredibly useful to reduce spikes in navigation cpu due to dense measurement readings when getting close to objects (i.e. more points), but does increase the overhead very slightly (1-2%) for nominal operations. It's a trade off but I recommend using it.

Below is an example a size of map that is **trivial** for the Spatio-Temportal Voxel Grid to maintain and render. This accounts for a 60,000 sq.ft. retail store with 710,765 voxels at a 0.05m resolution, with a size in memory of a mere 6.45MB.

![full_sore](https://user-images.githubusercontent.com/14944147/37013097-11e4f782-20c6-11e8-8212-6fca6e54331c.png)

## -**Temporal**
The Temporal in this package is the novel concept of `voxel_decay` whereas we have configurable functions that expire voxels and their occupation over time. Infrastructure was created to store times in each voxel after which the voxel will disappear from the map. This is combined with checking inclusion of voxels in current measurement frustums to accelerate the decay of those voxels that do not have measurements but should if still in the scene and remain marked. This is done rather than simply clearing them naively or via costly raytracing. The time it takes to clear depends on the configured functions and acceleration factors.

Voxel acceleration uses given FOV to compute the frustum geometry. Depth cameras (e.g. Intel Realsense) are modeled with traditional 6-planed cubical frustums. 3D lidars (e.g. Velodyne VLP 16) are modeled with their hourglass-shaped FOV. Although many 3D lidars have 360 degree horizontal FOV, it is possible to use a narrower angle for the clearing frustum by setting the hFOV parameter accordingly.

Future extensions will also to query a static map and determine which connected components belong to the map, not in the map, or moving. Each of these three classes of blobs will have configurable models to control the time they persist, and if these things are reported to the user.

Below is an example of instantaneous decay, where readings in frustum are accelerated and decayed at each iteration. The models provided can be tuned to do this, or persist through linear or exponential equations. The second example has the accelerated frustum with tuned decay times and acceleration factors in navigation mode.

![ezgif com-video-to-gif 1](https://user-images.githubusercontent.com/14944147/37063574-d0923d24-2167-11e8-850c-18b6aed61634.gif)

![ezgif com-video-to-gif 3](https://user-images.githubusercontent.com/14944147/37127014-cacc1d1c-2241-11e8-8c2e-6ff7341333c9.gif)

## Local Costmap
This package utilizes all of the information coming in from the robot before the decay time for the local costmap. Rather than having a defined, discrete spatial barrier for the local planner to operate in, it instead relies on the user configuration of the layer to have a short decay time of voxels (1-30 seconds) so that you only plan in relevant space. This was a conscious design requirement since frequently the local planner should operate with more information than other times when the speed is greater or slower. This natively implements dynamic costmap scaling for speed.

It is the user's responsibility to chose a decay time that makes sense for your robot's local planner. 5-15 seconds I have found to be nominally good for most open-sourced local planner plugins. I do not recommend using this for planar lidars, 2D raytracing for professional grade lidars is sufficiently efficient and effective.

## Global Costmap
Similar to the local costmap, the amount of information you want to store due to entropy in your scenes depend on your use-case. It is certainly possible to **not** decay the voxels in the global map at all. However, in practical application, I find a time 15-45 seconds to be a good balance due to things moving in the scene (i.e. store, warehouse, construction zone, office, etc). Permanent voxels set decay to -1. I do not recommend using this for planar lidars, 2D raytracing for professional grade lidars is sufficiently efficient and effective.

## Mapping

As the images above suggest, you can use this to map an environment in 3D in realtime if you choose. If you enable mapping mode, then it will maintain the entire voxel grid and you can save the map using the services provided. At the moment, I support mapping but there is no probabilistic (yet!) marking framework, so what the sensor sees is what the map gets. This is likely to change in the near to middle term future as 3D localization becomes more interesting to the enterprise robotics community.

You can run multiple instances of the layer one to map and other to navigate if you want to navigate while mapping the environment. Mapping will also save incremental maps in the launch directory. Maps may be visualized using `vdb_viewer`. The costmap and occupancy point clouds will not generate in this mode from this layer. Utility functions are provided so you don't need to learn anything about vdb files to convert to a pcl pointcloud in `vdb2pc.hpp`.

If you would like to be involved in this work, I would gladly take contributors and coauthors.

## Installation

### Install from source

Required dependencies: ROS 2 Humble, Nav2, OpenVDB, TBB.

```bash
sudo rosdep init && rosdep update && rosdep install --from-paths src --ignore-src -r -y
```

Build with colcon:
```bash
colcon build --packages-select spatio_temporal_voxel_layer
```

## Configuration and Running

### Plugin Registration

Add the STVL plugin to your costmap configuration in your Nav2 params YAML:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins:
        - "stvl_layer"
      stvl_layer:
        plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
        # ... layer params (see below)
```

### Layer-Level Parameters

| Parameter | Default | Type | Description |
|-----------|---------|------|-------------|
| `enabled` | `true` | bool | Enable/disable the entire layer |
| `voxel_size` | `0.05` | double | Size of each voxel in meters |
| `voxel_decay` | `-1.0` | double | Seconds if linear, e^n if exponential, -1 for persistent |
| `decay_model` | `0` | int | 0=linear, 1=exponential, -1=persistent |
| `track_unknown_space` | inherited | bool | Track unknown space or default to free |
| `mark_threshold` | `0` | int | Voxel height threshold for marking obstacles |
| `combination_method` | `1` | int | 1=max, 0=override |
| `origin_z` | `0.0` | double | Origin Z offset in meters |
| `publish_voxel_map` | `false` | bool | Publish voxel grid as PointCloud2 for visualization |
| `transform_tolerance` | `0.2` | double | TF transform timeout in seconds |
| `observation_sources` | `""` | string | Space-separated list of observation source names |
| `mapping_mode` | `false` | bool | Enable mapping mode (saves maps, not for navigation) |
| `map_save_duration` | `60.0` | double | Auto-save interval in seconds (mapping mode) |
| `autosaving_enabled` | `false` | bool | Enable automatic periodic map saving |
| `stvl_map_file` | `""` | string | Path to STVL map file for loading/saving |
| `should_load_navigation_data` | `false` | bool | Load navigation data from map file on startup |
| `marking_frustum_offset` | `0.0` | double | Offset for the marking frustum relative to the sensor (meters). Positive values extend the marking frustum forward. **(MuL custom)** |
| `visualize_frustum` | `false` | bool | Publish frustum boundaries as visualization markers for debugging. **(MuL custom)** |
| `clear_costmap_under_footprint` | `true` | bool | Clear the 2D costmap under the robot footprint |
| `clear_grid_under_footprint_in_manual_mode` | `true` | bool | Clear voxel grid under footprint only when in manual mode (subscribes to `/robo_cart/is_in_manual_mode`). **(MuL custom)** |
| `auto_grid_clear_range` | `0.0` | double | Auto-clear a square region (meters) around the robot. 0 = disabled. **(MuL custom)** |

### Per-Observation-Source Parameters

Each observation source is configured under the layer with its name as a key. For example, if `observation_sources: rgbd_front rgbd_rear`, then each has its own parameter block:

| Parameter | Default | Type | Description |
|-----------|---------|------|-------------|
| `topic` | `""` | string | ROS 2 topic name for sensor data |
| `sensor_frame` | `""` | string | TF frame of the sensor (optional, inferred from message if empty) |
| `data_type` | `"PointCloud2"` | string | `"PointCloud2"` or `"LaserScan"` |
| `marking` | `true` | bool | Use this source for marking obstacles |
| `clearing` | `false` | bool | Use this source for clearing obstacles |
| `enabled` | `true` | bool | Enable/disable this source |
| `obstacle_range` | `2.5` | double | Max range for obstacle detection (meters) |
| `min_obstacle_height` | `0.0` | double | Min height of obstacles (meters) |
| `max_obstacle_height` | `3.0` | double | Max height of obstacles (meters) |
| `min_z` | `0.0` | double | Min distance from sensor (meters) |
| `max_z` | `10.0` | double | Max distance from sensor (meters) |
| `vertical_fov_angle` | `0.7` | double | Vertical field of view (radians) |
| `horizontal_fov_angle` | `1.04` | double | Horizontal field of view (radians) |
| `decay_acceleration` | `0.0` | double | Acceleration for voxel decay within frustum (1/s^2). Must be 0 for laser scanners |
| `observation_persistence` | `0.0` | double | How long to keep measurements (seconds). 0 = use only latest |
| `expected_update_rate` | `0.0` | double | Expected sensor update rate (Hz). 0 = no checking |
| `inf_is_valid` | `false` | bool | Treat infinite range readings as valid (for laser scans) |
| `clear_after_reading` | `false` | bool | Clear measurement buffer after processing |
| `filter` | `"passthrough"` | string | Point cloud filter: `"voxel"`, `"passthrough"`, or `"off"` |
| `voxel_min_points` | `0` | int | Min points per voxel for voxel filter |
| `model_type` | `0` | int | Frustum model: 0=depth camera, 1=3D lidar, **2=proximity shield (MuL custom)** |
| `vertical_fov_padding` | `0.0` | double | Padding for vertical FOV in 3D lidar model (radians). **(MuL custom)** |
| `base_length` | `0.1` | double | Longitudinal base dimension for proximity shield frustum (meters). **(MuL custom, model_type=2 only)** |
| `base_width` | `0.1` | double | Lateral base dimension for proximity shield frustum (meters). **(MuL custom, model_type=2 only)** |
| `disable_decay_inside_frustum` | `false` | bool | Keep voxels persistent (no decay) inside sensor frustum. **(MuL custom)** |

### Frustum Model Types

- **0 — Depth Camera**: Cone/pyramid-shaped frustum originating from a single point (e.g. Intel RealSense)
- **1 — 3D Lidar**: Hourglass-shaped frustum (e.g. Velodyne VLP-16). Supports `vertical_fov_padding`
- **2 — Proximity Shield** **(MuL custom)**: Rectangle-based frustum defined by `base_length` and `base_width` instead of a point origin. Useful for modeling virtual proximity sensor zones

### Example Configuration (Depth Camera)

```yaml
stvl_layer:
  plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
  enabled: true
  voxel_decay: 0.25
  decay_model: -1
  voxel_size: 0.05
  marking_frustum_offset: 0.45
  visualize_frustum: false
  track_unknown_space: true
  unknown_threshold: 15
  mark_threshold: 0
  clear_costmap_under_footprint: false
  clear_grid_under_footprint_in_manual_mode: true
  auto_grid_clear_range: 7.0
  combination_method: 1
  origin_z: 0.0
  publish_voxel_map: true
  transform_tolerance: 0.5
  mapping_mode: false
  map_save_duration: 60.0
  observation_sources: rgbd_front rgbd_rear
  rgbd_front:
    data_type: PointCloud2
    topic: /camera_front/depth/points
    marking: true
    clearing: true
    obstacle_range: 4.45
    min_obstacle_height: -0.055
    max_obstacle_height: 1.45
    expected_update_rate: 0.5
    observation_persistence: 0.0
    clear_after_reading: true
    filter: "voxel"
    voxel_min_points: 0
    min_z: 0.2
    max_z: 5.0
    vertical_fov_angle: 0.977
    horizontal_fov_angle: 1.466
    decay_acceleration: 0.0
    model_type: 0
  rgbd_rear:
    data_type: PointCloud2
    topic: /camera_rear/depth/points
    marking: true
    clearing: true
    obstacle_range: 4.45
    min_obstacle_height: -0.055
    max_obstacle_height: 1.45
    expected_update_rate: 0.5
    observation_persistence: 0.0
    clear_after_reading: true
    filter: "voxel"
    voxel_min_points: 0
    min_z: 0.2
    max_z: 5.0
    vertical_fov_angle: 0.977
    horizontal_fov_angle: 1.466
    decay_acceleration: 0.0
    model_type: 0
```

### Running

```bash
ros2 launch [navigation_pkg] navigation.launch.py
```

### Enabling/Disabling Observation Sources at Runtime

Each observation source exposes a service to toggle it on/off:

```
~stvl_layer/{source_name}/toggle_enabled (std_srvs/srv/SetBool)
```

Example:
```bash
ros2 service call /local_costmap/stvl_layer/rgbd_front/toggle_enabled std_srvs/srv/SetBool "{data: true}"
ros2 service call /global_costmap/stvl_layer/rgbd_rear/toggle_enabled std_srvs/srv/SetBool "{data: false}"
```

### Services

| Service | Type | Description |
|---------|------|-------------|
| `~{layer}/save_grid` | `spatio_temporal_voxel_layer/srv/SaveGrid` | Save voxel grid to OpenVDB file |
| `~{layer}/clear_grid_around_pose` | `nav2_msgs/srv/ClearGridAroundPose` | Clear voxel grid in radius around a specified pose |
| `~{layer}/clear_grid_around_robot_footprint` | `nav2_msgs/srv/ClearGridAroundPose` | Clear grid around robot footprint (not square). **(MuL custom)** |
| `~{layer}/spatiotemporal_voxel_grid/save_stvl_map` | `std_srvs/srv/Trigger` | Save STVL map (mapping mode). **(MuL custom)** |
| `~{layer}/spatiotemporal_voxel_grid/erase_stvl_map` | `std_srvs/srv/Trigger` | Erase/clear STVL map. **(MuL custom)** |
| `~{layer}/spatiotemporal_voxel_grid/clear_entire_grid` | `std_srvs/srv/Trigger` | Clear all voxels in grid. **(MuL custom)** |

### Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `~{layer}/voxel_grid` | `sensor_msgs/msg/PointCloud2` | Published | Voxel grid occupancy cloud (when `publish_voxel_map: true`) |
| `/robo_cart/is_in_manual_mode` | `std_msgs/msg/Bool` | Subscribed | Manual mode status (when `clear_grid_under_footprint_in_manual_mode: true`). **(MuL custom)** |

### Dynamically Reconfigurable Parameters

The following parameters can be changed at runtime via `ros2 param set`:

- `enabled` (global)
- `marking_frustum_offset` (global)
- `visualize_frustum` (global)
- `min_obstacle_height`, `max_obstacle_height` (per-source)
- `min_z`, `max_z` (per-source)
- `vertical_fov_angle`, `vertical_fov_padding` (per-source)
- `horizontal_fov_angle` (per-source)
- `base_length`, `base_width` (per-source)
- `disable_decay_inside_frustum` (per-source)

Changing any frustum-related parameter triggers automatic frustum regeneration.

### Debug and Model Fitting

Frustum transformations are available for visualization and debugging. Enable them with the `visualize_frustum` parameter (can be dynamically toggled at runtime).

This can also be used for situations where you do not know your camera's proper frustum FOVs. It is possible to enable it and tweak the FOVs until you get the appropriate coverage of the space your sensor carves out in the global space. You should only do this with one sensor at a time or else your frustum in RViz might jitter around.

### Interesting side note

We are able to iterate over very large grids for voxel decay, however there is clearly for every frequency (running at 1, 5, 10, 100hz) an upper limit. In the image below, we don't actually hit the limit of the data structure, but iterating at 2hz, we hit the limit of ROS' ability to publish a sufficiently large point cloud in that time period, we are still running but you can see the robot at the end of an aisle without occupancy points, but still costmap marking from the underlying grid.

To counter this I include a service to save the grid in the .vdb format for later visualization, and for this reason I do not recommend visualizing the grid during nominal operations unless your decay time is relatively low (0-15 seconds) or else the layer may not meet its frequency requirements due to publishing this massive pointcloud.

![openvdb2](https://user-images.githubusercontent.com/14944147/37010656-8ce4ff4c-20ba-11e8-9c35-1ce3e3039f77.png)

# MuL / Husarion Customizations

This fork adds several features to the upstream STVL layer for the RCP platform. All custom parameters are marked with **(MuL custom)** in the tables above.

## Marking vs Clearing Frustum Split

The marking and clearing frustums are separated, allowing independent control of obstacle detection and clearing behavior. The `marking_frustum_offset` parameter adjusts the marking frustum forward or backward relative to the sensor without affecting the clearing frustum.

## Proximity Shield (model_type: 2)

A custom frustum model for creating a virtual proximity shield. Unlike the depth camera frustum (which propagates a pyramid from a single point), the proximity shield uses a rectangular base defined by `base_length` and `base_width`. The origin coordinates are taken from the input message frame.

Combined with `disable_decay_inside_frustum: true`, this creates reversed STVL behavior: obstacles within the frustum remain persistent while everything outside decays normally. This is designed for range sensors whose deadzones need persistent obstacle marking.

Example configuration:

```yaml
proximity_layer:
  plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
  enabled: true
  voxel_decay: 0.0
  decay_model: 0
  voxel_size: 0.01
  track_unknown_space: true
  unknown_threshold: 15
  observation_persistence: 0.0
  max_obstacle_height: 2.0
  mark_threshold: 0
  clear_costmap_under_footprint: false
  clear_grid_under_footprint_in_manual_mode: true
  auto_grid_clear_range: 0.0
  combination_method: 0
  origin_z: 0.0
  publish_voxel_map: true
  transform_tolerance: 0.2
  mapping_mode: false
  map_save_duration: 60.0
  observation_sources: proximity_point_cloud
  proximity_point_cloud:
    data_type: PointCloud2
    topic: /robo_cart/proximity_sensor_pointcloud
    marking: true
    clearing: true
    obstacle_range: 3.0
    min_z: 0.0
    max_z: 2.0
    base_length: 0.77       # longitudinal distance between proximity sensors
    base_width: 0.42        # lateral distance between proximity sensors
    vertical_fov_angle: 2.38
    horizontal_fov_angle: 2.34
    min_obstacle_height: 0.0
    max_obstacle_height: 2.0
    expected_update_rate: 0.0
    observation_persistence: 0.0
    inf_is_valid: false
    clear_after_reading: true
    decay_acceleration: 0.0
    disable_decay_inside_frustum: true
    model_type: 2
```

## Manual Mode Grid Clearing

The `clear_grid_under_footprint_in_manual_mode` parameter subscribes to the `/robo_cart/is_in_manual_mode` topic and only clears the voxel grid under the robot footprint when the robot is being manually controlled.

## Auto Grid Clear Range

The `auto_grid_clear_range` parameter automatically clears a square region of the given size (in meters) around the robot at each update cycle. Set to `0.0` to disable.

## Footprint-Based Grid Clearing Service

The `clear_grid_around_robot_footprint` service clears voxels using the actual robot footprint polygon, rather than a simple square region.
