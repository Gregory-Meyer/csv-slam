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

#include "util.h"

#include <Eigen/Core>

namespace chrono = std::chrono;
using chrono::microseconds;
using chrono::seconds;

namespace common = cartographer::common;
using common::Time;

using cartographer::transform::Rigid3d;

using Eigen::AngleAxisd;
using Eigen::Quaterniond;
using Eigen::Vector3d;

Time to_uts(microseconds utime) {
  return Time(utime + seconds(common::kUtsEpochOffsetFromUnixEpochInSeconds));
}

microseconds to_utime(Time uts) {
  return chrono::duration_cast<microseconds>(
      uts.time_since_epoch() -
      seconds(common::kUtsEpochOffsetFromUnixEpochInSeconds));
}

Rigid3d xyz_rpy(double x, double y, double z, double phi, double theta,
                double psi) noexcept {
  const Vector3d transform(x, y, z);
  const Quaterniond rotation(AngleAxisd(phi, Vector3d::UnitX()) *
                             AngleAxisd(theta, Vector3d::UnitY()) *
                             AngleAxisd(psi, Vector3d::UnitZ()));

  return Rigid3d(transform, rotation);
}
