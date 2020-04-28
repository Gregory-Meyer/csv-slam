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

#include "csv_writer.h"

#include "util.h"

#include <chrono>
#include <stdexcept>
#include <string_view>

#include <Eigen/Core>

using std::runtime_error;
using std::string_view;
using std::chrono::microseconds;

namespace transform = cartographer::transform;
using transform::Rigid3d;

using cartographer::common::Time;

using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector3f;

namespace {

runtime_error err_unwritable(string_view filename);

} // namespace

CsvWriter::CsvWriter(const char *filename)
    : filename_(filename), file_(filename) {
  if (!file_.is_open()) {
    throw runtime_error(
        FORMAT("couldn't open \"" << filename_ << "\" for writing"));
  }

  if (!(file_ << "utime,x,y,z,phi,theta,psi\n")) {
    throw err_unwritable(filename_);
  }
}

void CsvWriter::append(const Rigid3d &pose, Time uts) {
  const microseconds utime = to_utime(uts);

  const Vector3d &translation = pose.translation();
  const Quaterniond &rotation = pose.rotation();

  const Vector3d rpy = rotation.toRotationMatrix().eulerAngles(2, 1, 0);

  if (!(file_ << utime.count() << ',' << translation.x() << ','
              << translation.y() << ',' << translation.z() << ',' << rpy.x()
              << ',' << rpy.y() << ',' << rpy.z() << '\n')) {
    throw err_unwritable(filename_);
  }
}

namespace {

runtime_error err_unwritable(string_view filename) {
  return runtime_error(FORMAT("couldn't write to \"" << filename << '"'));
}

} // namespace
