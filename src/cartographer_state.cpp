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

#include "util.h"

#include <fstream>
#include <iterator>
#include <set>
#include <stdexcept>
#include <vector>

#include <cartographer/common/configuration_file_resolver.h>
#include <cartographer/common/lua_parameter_dictionary.h>

using std::ifstream;
using std::istreambuf_iterator;
using std::logic_error;
using std::runtime_error;
using std::set;
using std::string;
using std::vector;

namespace common = cartographer::common;
using common::ConfigurationFileResolver;
using common::LuaParameterDictionary;

namespace mapping = cartographer::mapping;
using mapping::MapBuilder;
using SensorId = mapping::TrajectoryBuilderInterface::SensorId;
using SensorType = SensorId::SensorType;
using LocalSlamResultCallback =
    mapping::TrajectoryBuilderInterface::LocalSlamResultCallback;

namespace {

string read_file(const char *config_filename);

} // namespace

CartographerState CartographerState::from_config_filename_and_callback(
    const char *config_filename, LocalSlamResultCallback callback) {
  static const string MAP_BUILDER = "map_builder";
  static const string TRAJECTORY_BUILDER = "trajectory_builder";

  const string config_code = read_file(config_filename);
  LuaParameterDictionary config(
      config_code,
      std::make_unique<ConfigurationFileResolver>(vector<string>{".."}));

  const auto map_builder_options =
      mapping::CreateMapBuilderOptions(config.GetDictionary(MAP_BUILDER).get());

  auto map_builder_ptr = std::make_unique<MapBuilder>(map_builder_options);
  MapBuilder &map_builder = *map_builder_ptr;

  const auto trajectory_builder_options =
      mapping::CreateTrajectoryBuilderOptions(
          config.GetDictionary(TRAJECTORY_BUILDER).get());

  const int trajectory_id = map_builder.AddTrajectoryBuilder(
      set<SensorId>{SensorId{SensorType::RANGE, "vel"},
                    SensorId{SensorType::IMU, "imu"},
                    SensorId{SensorType::ODOMETRY, "odom"}},
      trajectory_builder_options, std::move(callback));

  return CartographerState{
      std::move(map_builder_ptr),
      trajectory_id,
  };
}

namespace {

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
