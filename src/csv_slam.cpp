// Copyright (C) 2020 Chao Chen, Kevin Han, Gregory Meyer, and Sumukha Udupa
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "cartographer_state.h"
#include "csv_writer.h"
#include "imu_reader.h"
#include "odometry_reader.h"
#include "sensor_callback.h"
#include "sensor_callback_queue.h"
#include "util.h"
#include "velodyne_reader.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <optional>
#include <queue>
#include <string>

#include <cartographer/common/time.h>
#include <cartographer/mapping/map_builder.h>
#include <cartographer/mapping/pose_extrapolator.h>
#include <cartographer/mapping/trajectory_builder_interface.h>
#include <cartographer/sensor/range_data.h>
#include <cartographer/transform/rigid_transform.h>

#include <Eigen/Core>

using std::atomic_bool;
using std::bad_alloc;
using std::exception;
using std::function;
using std::greater;
using std::mutex;
using std::optional;
using std::priority_queue;
using std::scoped_lock;
using std::string;
using std::unique_ptr;

namespace chrono = std::chrono;
using chrono::high_resolution_clock;
using chrono::milliseconds;
using chrono::nanoseconds;
using chrono::seconds;

namespace common = cartographer::common;
using common::Duration;
using common::Time;

namespace mapping = cartographer::mapping;
using mapping::MapBuilder;
using mapping::PoseGraphInterface;
using mapping::TrajectoryBuilderInterface;
using InsertionResult = TrajectoryBuilderInterface::InsertionResult;
using LocalSlamResultCallback =
    TrajectoryBuilderInterface::LocalSlamResultCallback;
using mapping::PoseExtrapolator;
using mapping::TrajectoryNodePose;

using cartographer::sensor::ImuData;
using cartographer::sensor::OdometryData;
using cartographer::sensor::RangeData;
using cartographer::sensor::TimedPointCloudData;

using cartographer::transform::Rigid3d;

struct LocalSlamData {
  Time time;
  Rigid3d pose;
};

constexpr double operator""_deg(long double degrees) {
  return double(degrees * EIGEN_PI / 180);
}

extern "C" void sigint_handler(int) noexcept;

atomic_bool keep_running = true;

const Rigid3d BODY_TO_VEL =
    xyz_rpy(0.002, -0.004, -0.957, 0.807_deg, 0.166_deg, -90.703_deg);
const Rigid3d BODY_TO_IMU =
    xyz_rpy(-0.11, -0.18, -0.71, 0.0_deg, 0.0_deg, 0.0_deg);
const Rigid3d VEL_TO_BODY = BODY_TO_VEL.inverse();
const Rigid3d VEL_TO_IMU = BODY_TO_IMU * VEL_TO_BODY;

int main(int argc, const char *argv[]) try {
  google::InitGoogleLogging(argv[0]);

  if (argc < 2) {
    std::cerr << "error: missing argument CONFIG\n";

    return EXIT_FAILURE;
  } else if (argc < 3) {
    std::cerr << "error: missing argument VEL\n";

    return EXIT_FAILURE;
  } else if (argc < 4) {
    std::cerr << "error: missing argument IMU\n";

    return EXIT_FAILURE;
  } else if (argc < 5) {
    std::cerr << "error: missing argument OUTPUT\n";

    return EXIT_FAILURE;
  }

  mutex local_slam_mutex;
  optional<LocalSlamData> maybe_local_slam_data;

  const char *const config_filename = argv[1];
  const auto [map_builder_ptr, trajectory_id] =
      CartographerState::from_config_filename_and_callback(
          config_filename, [&local_slam_mutex, &maybe_local_slam_data](
                               int, Time time, Rigid3d pose, RangeData,
                               unique_ptr<const InsertionResult>) {
            const scoped_lock lock(local_slam_mutex);
            maybe_local_slam_data = LocalSlamData{time, pose};
          });

  MapBuilder &map_builder = *map_builder_ptr;

  TrajectoryBuilderInterface *const trajectory_builder_ptr =
      map_builder.GetTrajectoryBuilder(trajectory_id);
  assert(trajectory_builder_ptr);
  TrajectoryBuilderInterface &trajectory_builder = *trajectory_builder_ptr;

  PoseGraphInterface *const pose_graph_ptr = map_builder.pose_graph();
  assert(pose_graph_ptr);
  PoseGraphInterface &pose_graph = *pose_graph_ptr;

  PoseExtrapolator pose_extrapolator(
      common::FromMilliseconds(1),
      pose_graph.GetTrajectoryData()[trajectory_id].gravity_constant);

  const char *const vel_input_filename = argv[2];
  VelodyneReader vel_reader(vel_input_filename, VEL_TO_IMU);

  const char *const imu_input_filename = argv[3];
  ImuReader imu_reader(imu_input_filename);

  optional<OdometryReader> maybe_odom_reader =
      [argc, argv]() -> optional<OdometryReader> {
    if (argc > 5) {
      return OdometryReader(argv[4], BODY_TO_IMU);
    } else {
      return std::nullopt;
    }
  }();

  const char *const output_filename = [argc, argv] {
    if (argc > 5) {
      return argv[5];
    } else {
      return argv[4];
    }
  }();

  CsvWriter csv_writer(output_filename);

  sensor_callback_queue::Args scq_args;
  scq_args.trajectory_builder_ptr = &trajectory_builder;
  scq_args.pose_extrapolator_ptr = &pose_extrapolator;
  scq_args.vel_ptr = &vel_reader;
  scq_args.imu_ptr = &imu_reader;

  if (maybe_odom_reader) {
    scq_args.odom_ptr = &*maybe_odom_reader;
  }

  SensorCallbackQueue callbacks(std::move(scq_args));

  if (callbacks.empty()) {
    std::cerr << "error: all sensor input files are empty\n";

    return EXIT_FAILURE;
  }

  const Time simulation_start = *callbacks.pop();
  Time simulation_time;

  std::signal(SIGINT, sigint_handler);

  Time last_pose_output_time = simulation_start;
  const high_resolution_clock::time_point start = high_resolution_clock::now();
  high_resolution_clock::time_point last_print = start;
  while (keep_running.load(std::memory_order_relaxed) && !callbacks.empty()) {
    simulation_time = *callbacks.pop();
    const Duration simulation_elapsed = simulation_time - simulation_start;

    const high_resolution_clock::time_point now = high_resolution_clock::now();
    const high_resolution_clock::duration elapsed_since_start = now - start;
    const high_resolution_clock::duration elapsed_since_last_print =
        now - last_print;

    if (elapsed_since_last_print >= seconds(1)) {
      last_print = now;

      std::cout << "processed "
                << chrono::duration_cast<seconds>(simulation_elapsed).count()
                << "s of data in "
                << chrono::duration_cast<seconds>(elapsed_since_start).count()
                << "s ("
                << double(nanoseconds(simulation_elapsed).count()) /
                       double(nanoseconds(elapsed_since_start).count())
                << "x speedup)\n";
    }

    const Duration elapsed_since_last_output =
        simulation_time - last_pose_output_time;

    if (elapsed_since_last_output < milliseconds(100)) {
      continue;
    }

    last_pose_output_time = simulation_time;

    LocalSlamData local_slam_data;
    {
      const scoped_lock lock(local_slam_mutex);

      if (!maybe_local_slam_data) {
        continue;
      }

      local_slam_data = *maybe_local_slam_data;
    }

    const Rigid3d local_to_global =
        pose_graph.GetLocalToGlobalTransform(trajectory_id);

    if (local_slam_data.time != pose_extrapolator.GetLastPoseTime()) {
      pose_extrapolator.AddPose(local_slam_data.time, local_slam_data.pose);
    }

    const Time simulation_now =
        std::max(simulation_time, pose_extrapolator.GetLastExtrapolatedTime());
    const Rigid3d tracking_to_local =
        pose_extrapolator.ExtrapolatePose(simulation_now);
    const Rigid3d tracking_to_map = local_to_global * tracking_to_local;

    csv_writer.append(tracking_to_map, simulation_now);
  }

  std::signal(SIGINT, SIG_DFL);

  const string state_filename = FORMAT(output_filename << ".pbstream");
  map_builder.SerializeStateToFile(true, state_filename);
  std::cout << "serialized unoptimized state to '" << state_filename << "'\n";

  map_builder.FinishTrajectory(trajectory_id);

  map_builder.SerializeStateToFile(true, state_filename);
  std::cout << "serialized optimized state to '" << state_filename << "'\n";
} catch (const bad_alloc &) {
  std::cerr << "error: out of memory\n";

  return EXIT_FAILURE;
} catch (const exception &e) {
  std::cerr << "error: " << e.what() << '\n';

  return EXIT_FAILURE;
} catch (...) {
  std::cerr << "error: unknown exception caught\n";

  return EXIT_FAILURE;
}

extern "C" void sigint_handler(int) noexcept {
  keep_running.store(false, std::memory_order_relaxed);
}
