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

#ifndef CSV_WRITER_H
#define CSV_WRITER_H

#include <fstream>
#include <string>

#include <cartographer/common/time.h>
#include <cartographer/transform/rigid_transform.h>

class CsvWriter {
public:
  explicit CsvWriter(const char *filename);
  void append(const cartographer::transform::Rigid3d &pose,
              cartographer::common::Time uts);

private:
  std::string filename_;
  std::ofstream file_;
};

#endif
