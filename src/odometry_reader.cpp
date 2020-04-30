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

#include "odometry_reader.h"

#include "util.h"

#include <cassert>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cartographer/common/time.h>

#include <Eigen/Core>

using std::istream;
using std::optional;
using std::runtime_error;
using std::string;
using std::uint64_t;

namespace chrono = std::chrono;
using std::chrono::microseconds;
using std::chrono::seconds;

namespace common = cartographer::common;
using common::Time;

namespace sensor = cartographer::sensor;
using sensor::OdometryData;

using cartographer::transform::Rigid3d;

using Eigen::AngleAxisd;
using Eigen::Quaterniond;
using Eigen::Vector3d;

OdometryReader::OdometryReader(const char *filename, const Rigid3d &odom_to_imu)
    : filename_(filename), file_(filename_), odom_to_imu_(odom_to_imu) {
  if (!file_.is_open()) {
    throw runtime_error(
        FORMAT("couldn't open \"" << filename_ << "\" for reading"));
  }
}

optional<OdometryData> OdometryReader::deserialize_measurement() {
  optional<OdometryData> maybe_data;

  string line;
  if (std::getline(file_, line)) {
    uint64_t utime_us;
    double x;
    double y;
    double z;
    double roll;  // roll around x-axis
    double pitch; // pitch around y-axis
    double yaw;   // yaw around z-axis

    if (std::sscanf(line.c_str(), "%" SCNu64 ",%lf,%lf,%lf,%lf,%lf,%lf",
                    &utime_us, &x, &y, &z, &roll, &pitch, &yaw) != 7) {
      throw runtime_error(
          FORMAT("missing or malformed fields in odometry csv file \""
                 << filename_ << '"'));
    }

    const microseconds utime(utime_us);
    const Time uts = to_uts(utime);

    const Rigid3d world_to_odom = xyz_rpy(x, y, z, roll, pitch, yaw);
    const Rigid3d world_to_imu = odom_to_imu_ * world_to_odom;

    maybe_data = OdometryData{uts, world_to_imu};
  } else if (file_.bad()) {
    throw runtime_error(FORMAT("couldn't read from \"" << filename_ << '"'));
  }

  return maybe_data;
}
