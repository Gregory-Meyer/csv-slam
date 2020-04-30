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

#ifndef UTIL_H
#define UTIL_H

#include <chrono>
#include <sstream>

#include <cartographer/common/time.h>
#include <cartographer/transform/rigid_transform.h>

#define FORMAT(...)                                                            \
  (static_cast<std::ostringstream &>(std::ostringstream().flush()              \
                                     << __VA_ARGS__)                           \
       .str())
#define FORMATLN(...)                                                          \
  (static_cast<std::ostringstream &>(std::ostringstream().flush()              \
                                     << __VA_ARGS__ << '\n')                   \
       .str())

cartographer::common::Time to_uts(std::chrono::microseconds utime);
std::chrono::microseconds to_utime(cartographer::common::Time uts);
cartographer::transform::Rigid3d xyz_rpy(double x, double y, double z,
                                         double phi, double theta,
                                         double psi) noexcept;

#endif
