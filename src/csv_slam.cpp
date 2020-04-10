#include "cartographer_state.h"
#include "csv_writer.h"
#include "imu_reader.h"
#include "velodyne_reader.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <optional>

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
using std::mutex;
using std::optional;
using std::scoped_lock;
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

using cartographer::sensor::RangeData;

using cartographer::transform::Rigid3d;

using Eigen::Vector3f;

struct LocalSlamData {
  Time time;
  Rigid3d pose;
};

extern "C" void sigint_handler(int) noexcept;

atomic_bool keep_running = true;

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
  const auto [map_builder_ptr, trajectory_id, vel_sensor_id, imu_sensor_id] =
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
  const Vector3f origin = {0.0f, 0.0f, 0.0f};
  VelodyneReader vel_reader(vel_input_filename, origin);
  auto maybe_vel_data = vel_reader.deserialize_packet();

  if (!maybe_vel_data) {
    std::cerr << "error: velodyne binary file \"" << vel_input_filename
              << "\" is empty\n";

    return EXIT_FAILURE;
  }

  const char *const imu_input_filename = argv[3];
  ImuReader imu_reader(imu_input_filename);
  auto maybe_imu_data = imu_reader.deserialize_measurement();

  if (!maybe_imu_data) {
    std::cerr << "error: imu csv file \"" << vel_input_filename
              << "\" is empty\n";

    return EXIT_FAILURE;
  }

  const char *const output_filename = argv[4];
  CsvWriter csv_writer(output_filename);

  const Time simulation_start =
      std::min(maybe_imu_data->time, maybe_vel_data->time);
  Time simulation_time;

  const auto pop_imu = [&imu_reader, &imu_sensor_id = imu_sensor_id,
                        &trajectory_builder, &pose_extrapolator,
                        &maybe_imu_data, &simulation_time] {
    assert(maybe_imu_data);

    trajectory_builder.AddSensorData(imu_sensor_id, *maybe_imu_data);
    pose_extrapolator.AddImuData(*maybe_imu_data);
    simulation_time = maybe_imu_data->time;
    maybe_imu_data = imu_reader.deserialize_measurement();
  };

  const auto pop_vel = [&vel_reader, &vel_sensor_id = vel_sensor_id,
                        &trajectory_builder, &maybe_vel_data,
                        &simulation_time] {
    assert(maybe_vel_data);

    trajectory_builder.AddSensorData(vel_sensor_id, *maybe_vel_data);
    simulation_time = maybe_vel_data->time;
    maybe_vel_data = vel_reader.deserialize_packet();
  };

  std::signal(SIGINT, sigint_handler);

  Time last_pose_output_time = simulation_start;
  const high_resolution_clock::time_point start = high_resolution_clock::now();
  high_resolution_clock::time_point last_print = start;
  while (keep_running.load(std::memory_order_relaxed) &&
         (maybe_imu_data || maybe_vel_data)) {
    if (maybe_imu_data && maybe_vel_data) {
      if (maybe_imu_data->time < maybe_vel_data->time) {
        pop_imu();
      } else {
        pop_vel();
      }
    } else if (maybe_vel_data) {
      pop_vel();
    } else {
      assert(maybe_imu_data);

      pop_imu();
    }

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

  map_builder.FinishTrajectory(trajectory_id);
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
