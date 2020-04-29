// Copyright (C) 2020 Chao Chen, Yutian Han, Gregory Meyer, and Sumukha Udupa
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

#ifndef VELODYNE_READER_H
#define VELODYNE_READER_H

#include <fstream>
#include <optional>
#include <string>

#include <cartographer/sensor/timed_point_cloud_data.h>
#include <cartographer/transform/rigid_transform.h>

class VelodyneReader {
public:
  VelodyneReader(const char *filename,
                 const cartographer::transform::Rigid3d &vel_to_imu);

  std::optional<cartographer::sensor::TimedPointCloudData> deserialize_packet();

private:
  std::string filename_;
  std::ifstream file_;
  cartographer::transform::Rigid3d vel_to_imu_;
};

#endif
