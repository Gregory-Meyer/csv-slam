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

-- This file has been modified from its original form to omit setting the map
-- builder and to allow specifying the Velodyne and IMU sensor IDs.

include "pose_graph.lua"
include "trajectory_builder.lua"

NUM_THREADS = 16

POSE_GRAPH
  .constraint_builder
  .ceres_scan_matcher
  .ceres_solver_options
  .num_threads = NUM_THREADS
POSE_GRAPH
  .constraint_builder
  .ceres_scan_matcher_3d
  .ceres_solver_options
  .num_threads = NUM_THREADS
POSE_GRAPH
  .optimization_problem
  .ceres_solver_options
  .num_threads = NUM_THREADS

TRAJECTORY_BUILDER
    .trajectory_builder_2d
    .ceres_scan_matcher
    .ceres_solver_options
    .num_threads = NUM_THREADS
TRAJECTORY_BUILDER
    .trajectory_builder_3d
    .ceres_scan_matcher
    .ceres_solver_options
    .num_threads = NUM_THREADS

MAP_BUILDER = {
  use_trajectory_builder_2d = false,
  use_trajectory_builder_3d = false,
  num_background_threads = NUM_THREADS,
  pose_graph = POSE_GRAPH,
  collate_by_trajectory = false,
}

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
}
