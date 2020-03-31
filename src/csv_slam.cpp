#include "cartographer_state.h"
#include "csv_writer.h"
#include "imu_reader.h"
#include "velodyne_reader.h"

#include <cstdlib>
#include <exception>
#include <iostream>
#include <new>

#include <cartographer/mapping/map_builder.h>
#include <cartographer/mapping/trajectory_builder_interface.h>
#include <cartographer/sensor/timed_point_cloud_data.h>

#include <Eigen/Core>

using std::bad_alloc;
using std::exception;
using std::optional;

namespace common = cartographer::common;
using common::Time;

namespace mapping = cartographer::mapping;
using mapping::MapBuilder;
using mapping::PoseGraphInterface;
using mapping::TrajectoryBuilderInterface;
using mapping::TrajectoryNodePose;

using Eigen::Vector3f;

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

  const char *const config_filename = argv[1];
  const auto [map_builder_ptr, trajectory_id, vel_sensor_id, imu_sensor_id] =
      CartographerState::from_config_filename(config_filename);
  MapBuilder &map_builder = *map_builder_ptr;

  TrajectoryBuilderInterface *const trajectory_builder_ptr =
      map_builder.GetTrajectoryBuilder(trajectory_id);
  assert(trajectory_builder_ptr);
  TrajectoryBuilderInterface &trajectory_builder = *trajectory_builder_ptr;

  PoseGraphInterface *const pose_graph_ptr = map_builder.pose_graph();
  assert(pose_graph_ptr);
  PoseGraphInterface &pose_graph = *pose_graph_ptr;

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
                        &trajectory_builder, &maybe_imu_data,
                        &simulation_time] {
    assert(maybe_imu_data);

    trajectory_builder.AddSensorData(imu_sensor_id, *maybe_imu_data);
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

  optional<int> last_output_node_index;

  while (maybe_imu_data || maybe_vel_data) {
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

    const auto poses = pose_graph.GetTrajectoryNodePoses();
    const auto first = poses.BeginOfTrajectory(trajectory_id);
    const auto last = poses.EndOfTrajectory(trajectory_id);

    if (first == last) {
      continue;
    }

    const auto &back = *std::prev(last);
    const int back_node_index = back.id.node_index;

    if (last_output_node_index && *last_output_node_index >= back_node_index) {
      continue;
    }

    last_output_node_index = back_node_index;
    const TrajectoryNodePose &pose = back.data;

    csv_writer.append(pose, simulation_time);
  }

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
