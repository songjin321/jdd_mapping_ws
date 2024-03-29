-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = true,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
-- new added
MAX_3D_RANGE = 50
TRAJECTORY_BUILDER_3D.min_range=5
TRAJECTORY_BUILDER_3D.max_range=MAX_3D_RANGE
-- TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.15
TRAJECTORY_BUILDER_3D.submaps.num_range_data=100
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.hit_probability=0.56 -- 55
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.miss_probability=0.45 -- 49
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.15
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.5
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 30 -- 20
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.num_threads = 6
-- TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.05
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 7

POSE_GRAPH.optimization_problem.huber_scale = 5e1
POSE_GRAPH.optimize_every_n_nodes = 100 -- 320
POSE_GRAPH.constraint_builder.sampling_ratio = 0.01 -- 0.3
POSE_GRAPH.global_sampling_ratio = 0.3--0.003
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.min_score = 0.55 --0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55
-- new added
POSE_GRAPH.constraint_builder.log_matches = true
POSE_GRAPH.constraint_builder.max_constraint_distance = 15
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 15
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 15
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 5
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_low_resolution_score = 0.2
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(10.)

-- for map localization
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 5e1 --1e1
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 5e2 --1e2
return options
