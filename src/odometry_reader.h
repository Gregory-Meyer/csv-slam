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

#ifndef ODOMETRY_READER_H
#define ODOMETRY_READER_H

#include <fstream>
#include <optional>
#include <string>

#include <cartographer/sensor/odometry_data.h>
#include <cartographer/transform/rigid_transform.h>

class OdometryReader {
public:
  OdometryReader(const char *filename,
                 const cartographer::transform::Rigid3d &odom_to_imu);

  std::optional<cartographer::sensor::OdometryData> deserialize_measurement();

private:
  std::string filename_;
  std::ifstream file_;
  cartographer::transform::Rigid3d odom_to_imu_;
};

#endif
