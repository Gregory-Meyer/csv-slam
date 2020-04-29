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

#ifndef STARTUP_H
#define STARTUP_H

#include <memory>
#include <string>

#include <cartographer/mapping/map_builder.h>
#include <cartographer/mapping/trajectory_builder_interface.h>

struct CartographerState {
  std::unique_ptr<cartographer::mapping::MapBuilder> map_builder_ptr;
  int trajectory_id;

  static CartographerState from_config_filename_and_callback(
      const char *config_filename,
      cartographer::mapping::TrajectoryBuilderInterface::LocalSlamResultCallback
          callback);
};

#endif
