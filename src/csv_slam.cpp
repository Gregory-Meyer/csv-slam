#include "hit_packet_reader.h"

#include <chrono>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

#include <cartographer/mapping/map_builder.h>
#include <cartographer/mapping/trajectory_builder_interface.h>
#include <cartographer/sensor/timed_point_cloud_data.h>

using std::exception;
using std::ifstream;
using std::istreambuf_iterator;
using std::logic_error;
using std::optional;
using std::ostringstream;
using std::pair;
using std::runtime_error;
using std::set;
using std::string;
using std::string_view;
using std::unique_ptr;

namespace chrono = std::chrono;
using chrono::microseconds;
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

namespace sensor = cartographer::sensor;
using sensor::TimedPointCloudData;

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
    std::cerr << "error: missing argument INPUT\n";

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

  const char *const input_filename = argv[2];
  ifstream input_file(input_filename);

  if (!input_file.is_open()) {
    std::cerr << "error: couldn't open '" << input_filename
              << "' for reading\n";

    return EXIT_FAILURE;
  }

  const Vector3f origin{};
  HitPacketReader hit_packet_reader(input_file, origin);

  for (auto maybe_timed_point_cloud_data =
           hit_packet_reader.deserialize_packet();
       maybe_timed_point_cloud_data;
       maybe_timed_point_cloud_data = hit_packet_reader.deserialize_packet()) {
    auto &timed_point_cloud_data = *maybe_timed_point_cloud_data;

    trajectory_builder.AddSensorData(vel_sensor_id, timed_point_cloud_data);
  }

  // TODO: read the IMU csv

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

  const auto map_builder_options = mapping::CreateMapBuilderOptions(&config);
  pair<unique_ptr<MapBuilder>, int> result = {
      std::make_unique<MapBuilder>(map_builder_options), 0};
  auto &[map_builder_ptr, trajectory_id] = result;
  MapBuilder &map_builder = *map_builder_ptr;

  const auto trajectory_builder_options =
      mapping::CreateTrajectoryBuilderOptions(&config);
  trajectory_id = map_builder.AddTrajectoryBuilder(
      set<SensorId>{SensorId{SensorType::RANGE, string(vel_sensor_id)},
                    SensorId{SensorType::IMU, string(imu_sensor_id)}},
      trajectory_builder_options, {});

  return result;
}
