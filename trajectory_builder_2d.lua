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

-- This file has been modified from its original form to:
--   * increase the maximum range to 80m, to match the Velodyne HDL-32E
--   * increase the number of accumulated range data packets to 90, to match
--     the number of packets emitted in a half rotation of a Velodyne HDL-32E
--   * increase the minimum range to 1m, to match "trajectory_builder_3d.lua"
--   * invert min_ and max_z, because the NCLT dataset is provided in the neu
--     coordinate frame

include "trajectory_builder_3d.lua"

TRAJECTORY_BUILDER_2D = {
  use_imu_data = true,
  min_range = MIN_RANGE,
  max_range = MAX_RANGE,
  min_z = -2.,
  max_z = 0.8,
  missing_data_ray_length = 5.,
  num_accumulated_range_data = 90,
  voxel_filter_size = 0.025,

  adaptive_voxel_filter = {
    max_length = 0.5,
    min_num_points = 200,
    max_range = 50.,
  },

  loop_closure_adaptive_voxel_filter = {
    max_length = 0.9,
    min_num_points = 100,
    max_range = 50.,
  },

  use_online_correlative_scan_matching = false,
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.1,
    angular_search_window = math.rad(20.),
    translation_delta_cost_weight = 1e-1,
    rotation_delta_cost_weight = 1e-1,
  },

  ceres_scan_matcher = {
    occupied_space_weight = 1.,
    translation_weight = 10.,
    rotation_weight = 4e3,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 20,
      num_threads = 1,
    },
  },

  motion_filter = {
    max_time_seconds = 5.,
    max_distance_meters = 0.2,
    max_angle_radians = math.rad(1.),
  },

  imu_gravity_time_constant = 10.,

  submaps = {
    num_range_data = 90,
    grid_options_2d = {
      grid_type = "PROBABILITY_GRID",
      resolution = 0.05,
    },
    range_data_inserter = {
      range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
      probability_grid_range_data_inserter = {
        insert_free_space = true,
        hit_probability = 0.55,
        miss_probability = 0.49,
      },
      tsdf_range_data_inserter = {
        truncation_distance = 0.3,
        maximum_weight = 10.,
        update_free_space = false,
        normal_estimation_options = {
          num_normal_samples = 4,
          sample_radius = 0.5,
        },
        project_sdf_distance_to_scan_normal = true,
        update_weight_range_exponent = 0,
        update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0.5,
        update_weight_distance_cell_to_hit_kernel_bandwidth = 0.5,
      },
    },
  },
}
