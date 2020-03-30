#include "hit_packet_reader.h"
#include "imu_reader.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <cartographer/mapping/map_builder.h>
#include <cartographer/mapping/trajectory_builder_interface.h>
#include <cartographer/sensor/timed_point_cloud_data.h>

using std::exception;
using std::ifstream;
using std::istreambuf_iterator;
using std::logic_error;
using std::ofstream;
using std::optional;
using std::ostringstream;
using std::pair;
using std::runtime_error;
using std::set;
using std::string;
using std::string_view;
using std::unique_ptr;
using std::vector;

namespace chrono = std::chrono;
using chrono::microseconds;
using chrono::milliseconds;
using chrono::nanoseconds;
using chrono::seconds;

namespace common = cartographer::common;
using common::FileResolver;
using common::LuaParameterDictionary;
using common::Time;

namespace mapping = cartographer::mapping;
using mapping::MapBuilder;
using SensorId = mapping::TrajectoryBuilderInterface::SensorId;
using SensorType = SensorId::SensorType;
using mapping::TrajectoryBuilderInterface;
using mapping::TrajectoryNodePose;

namespace sensor = cartographer::sensor;
using sensor::TimedPointCloudData;

namespace transform = cartographer::transform;
using transform::Rigid3d;

using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector3f;

class NullFileResolver : public FileResolver {
public:
  virtual ~NullFileResolver() = default;

  string GetFullPathOrDie(const string &) override {
    throw logic_error("NullFileResolver::GetFullPathOrDie: not implemented");
  }

  string GetFileContentOrDie(const string &) override {
    throw logic_error("NullFileResolver::GetFileContentOrDie: not implemented");
  }
};

static string read_file(const char *filename);
static pair<unique_ptr<MapBuilder>, int>
make_map_builder(const string &config_code, string_view vel_sensor_id,
                 string_view imu_sensor_id);

int main(int argc, const char *argv[]) try {
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
  ifstream config_file(config_filename);

  if (!config_file.is_open()) {
    std::cerr << "error: couldn't open '" << config_filename
              << "' for reading\n";

    return EXIT_FAILURE;
  }

  const string config_code = read_file(config_filename);
  const string vel_sensor_id("vel");
  const string imu_sensor_id("imu");

  auto [map_builder_ptr, trajectory_id] =
      make_map_builder(config_code, vel_sensor_id, imu_sensor_id);
  MapBuilder &map_builder = *map_builder_ptr;

  TrajectoryBuilderInterface *const trajectory_builder_ptr =
      map_builder.GetTrajectoryBuilder(trajectory_id);
  assert(trajectory_builder_ptr);
  TrajectoryBuilderInterface &trajectory_builder = *trajectory_builder_ptr;
  const auto &pose_graph = *map_builder.pose_graph();

  const char *const vel_input_filename = argv[2];
  ifstream vel_input_file(vel_input_filename);

  if (!vel_input_file.is_open()) {
    std::cerr << "error: couldn't open '" << vel_input_filename
              << "' for reading\n";

    return EXIT_FAILURE;
  }

  const Vector3f origin{};
  HitPacketReader hit_packet_reader(vel_input_file, origin);

  const char *const imu_input_filename = argv[3];
  ifstream imu_input_file(imu_input_filename);

  if (!imu_input_file.is_open()) {
    std::cerr << "error: couldn't open '" << imu_input_filename
              << "' for reading\n";

    return EXIT_FAILURE;
  }

  ImuReader imu_reader(imu_input_file);

  const char *const output_filename = argv[4];
  ofstream output_file(output_filename);

  if (!output_file.is_open()) {
    std::cerr << "error: couldn't open '" << output_filename
              << "' for writing\n";

    return EXIT_FAILURE;
  }

  if (!(output_file << "utime,x,y,z,phi,theta,psi\n").flush()) {
    std::cerr << "error: couldn't write to '" << output_filename << '\n';

    return EXIT_FAILURE;
  }

  auto maybe_imu_data = imu_reader.deserialize_measurement();
  auto maybe_timed_point_cloud_data = hit_packet_reader.deserialize_packet();

  Time simulation_start;

  if (maybe_imu_data && maybe_timed_point_cloud_data) {
    simulation_start =
        std::min(maybe_imu_data->time, maybe_timed_point_cloud_data->time);
  } else if (maybe_imu_data) {
    simulation_start = maybe_imu_data->time;
  } else {
    assert(maybe_timed_point_cloud_data);
    simulation_start = maybe_timed_point_cloud_data->time;
  }

  Time simulation_time;

  const auto consume_imu = [&imu_reader, &imu_sensor_id, &trajectory_builder,
                            &maybe_imu_data, &simulation_time] {
    assert(maybe_imu_data);

    trajectory_builder.AddSensorData(imu_sensor_id, *maybe_imu_data);
    simulation_time = maybe_imu_data->time;
    maybe_imu_data = imu_reader.deserialize_measurement();
  };

  const auto consume_vel = [&hit_packet_reader, &vel_sensor_id,
                            &trajectory_builder, &maybe_timed_point_cloud_data,
                            &simulation_time] {
    assert(maybe_timed_point_cloud_data);

    trajectory_builder.AddSensorData(vel_sensor_id,
                                     *maybe_timed_point_cloud_data);
    simulation_time = maybe_timed_point_cloud_data->time;
    maybe_timed_point_cloud_data = hit_packet_reader.deserialize_packet();
  };

  optional<int> last_output_node_index;

  while (maybe_imu_data || maybe_timed_point_cloud_data) {
    if (maybe_imu_data && maybe_timed_point_cloud_data) {
      if (maybe_imu_data->time < maybe_timed_point_cloud_data->time) {
        consume_imu();
      } else {
        consume_vel();
      }
    } else if (maybe_imu_data) {
      consume_imu();
    } else {
      assert(maybe_timed_point_cloud_data);

      consume_vel();
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

    const auto utime = chrono::duration_cast<microseconds>(
        simulation_time.time_since_epoch() -
        seconds(common::kUtsEpochOffsetFromUnixEpochInSeconds));

    const Rigid3d &transform = pose.global_pose;
    const Vector3d &translation = transform.translation();
    const Quaterniond &rotation = transform.rotation();

    const Vector3d rpy = rotation.toRotationMatrix().eulerAngles(2, 1, 0);

    if (!(output_file << utime.count() << ',' << translation.x() << ','
                      << translation.y() << ',' << translation.z() << ','
                      << rpy.x() << ',' << rpy.y() << ',' << rpy.z() << '\n')
             .flush()) {
      std::cerr << "error: couldn't write to '" << output_filename << '\n';

      return EXIT_FAILURE;
    }
  }

  map_builder.FinishTrajectory(trajectory_id);
} catch (const exception &e) {
  std::cerr << "error: " << e.what() << '\n';

  return EXIT_FAILURE;
} catch (...) {
  std::cerr << "error: unknown exception caught\n";

  return EXIT_FAILURE;
}

static string read_file(const char *filename) {
  ifstream config_file(filename);

  if (!config_file.is_open()) {
    ostringstream oss;
    oss << "read_file: couldn't open '" << filename << "' for reading";

    throw runtime_error(oss.str());
  }

  return string(istreambuf_iterator(config_file.rdbuf()), {});
}

static pair<unique_ptr<MapBuilder>, int>
make_map_builder(const string &config_code, string_view vel_sensor_id,
                 string_view imu_sensor_id) {
  LuaParameterDictionary config(config_code,
                                std::make_unique<NullFileResolver>());

  const auto map_builder_options = mapping::CreateMapBuilderOptions(
      config.GetDictionary("map_builder").get());
  pair<unique_ptr<MapBuilder>, int> result = {
      std::make_unique<MapBuilder>(map_builder_options), 0};
  auto &[map_builder_ptr, trajectory_id] = result;
  MapBuilder &map_builder = *map_builder_ptr;

  const auto trajectory_builder_options =
      mapping::CreateTrajectoryBuilderOptions(
          config.GetDictionary("trajectory_builder").get());
  trajectory_id = map_builder.AddTrajectoryBuilder(
      set<SensorId>{SensorId{SensorType::RANGE, string(vel_sensor_id)},
                    SensorId{SensorType::IMU, string(imu_sensor_id)}},
      trajectory_builder_options, {});

  return result;
}
