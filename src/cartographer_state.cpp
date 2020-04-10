#include "cartographer_state.h"

#include "util.h"

#include <fstream>
#include <iterator>
#include <set>
#include <stdexcept>

#include <cartographer/common/lua_parameter_dictionary.h>

using std::ifstream;
using std::istreambuf_iterator;
using std::logic_error;
using std::runtime_error;
using std::set;
using std::string;

namespace common = cartographer::common;
using common::FileResolver;
using common::LuaParameterDictionary;

namespace mapping = cartographer::mapping;
using mapping::MapBuilder;
using SensorId = mapping::TrajectoryBuilderInterface::SensorId;
using SensorType = SensorId::SensorType;
using LocalSlamResultCallback =
    mapping::TrajectoryBuilderInterface::LocalSlamResultCallback;

namespace {

class NullFileResolver : public FileResolver {
public:
  virtual ~NullFileResolver() = default;
  string GetFullPathOrDie(const string &) override;
  string GetFileContentOrDie(const string &) override;
};

string read_file(const char *config_filename);

} // namespace

CartographerState CartographerState::from_config_filename_and_callback(
    const char *config_filename, LocalSlamResultCallback callback) {
  static const string MAP_BUILDER = "map_builder";
  static const string TRAJECTORY_BUILDER = "trajectory_builder";
  static const string VEL_SENSOR_ID = "vel_sensor_id";
  static const string IMU_SENSOR_ID = "imu_sensor_id";

  const string config_code = read_file(config_filename);
  LuaParameterDictionary config(config_code,
                                std::make_unique<NullFileResolver>());

  const auto map_builder_options =
      mapping::CreateMapBuilderOptions(config.GetDictionary(MAP_BUILDER).get());

  auto map_builder_ptr = std::make_unique<MapBuilder>(map_builder_options);
  MapBuilder &map_builder = *map_builder_ptr;

  const auto trajectory_builder_options =
      mapping::CreateTrajectoryBuilderOptions(
          config.GetDictionary(TRAJECTORY_BUILDER).get());

  string vel_sensor_id = config.GetString(VEL_SENSOR_ID);
  string imu_sensor_id = config.GetString(IMU_SENSOR_ID);

  const int trajectory_id = map_builder.AddTrajectoryBuilder(
      set<SensorId>{SensorId{SensorType::RANGE, vel_sensor_id},
                    SensorId{SensorType::IMU, imu_sensor_id}},
      trajectory_builder_options, std::move(callback));

  return CartographerState{std::move(map_builder_ptr), trajectory_id,
                           std::move(vel_sensor_id), std::move(imu_sensor_id)};
}

namespace {

constexpr auto &ERR_NULL_FILE_RESOLVER_NOT_IMPLEMENTED =
    "NullFileResolver::GetFullPathOrDie: not implemented";

string NullFileResolver::GetFullPathOrDie(const string &) {
  throw logic_error(ERR_NULL_FILE_RESOLVER_NOT_IMPLEMENTED);
}

string NullFileResolver::GetFileContentOrDie(const string &) {
  throw logic_error(ERR_NULL_FILE_RESOLVER_NOT_IMPLEMENTED);
}

string read_file(const char *filename) {
  assert(config_filename);

  ifstream file(filename);

  if (!file.is_open()) {
    throw runtime_error(
        FORMAT("couldn't open \"" << filename << "\" for reading"));
  }

  const string contents(istreambuf_iterator(file.rdbuf()), {});
  file.ignore();

  if (file.bad()) {
    throw runtime_error(FORMAT("couldn't read from \"" << filename << "\"\n"));
  }

  return contents;
}

} // namespace
