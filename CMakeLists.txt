# Copyright (C) 2020 Chao Chen, Kevin Han, Gregory Meyer, and Sumukha Udupa
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.11 FATAL_ERROR)

project(csv-slam LANGUAGES CXX VERSION 0.1.0)

find_package(Boost 1.58 REQUIRED COMPONENTS iostreams)
find_package(cartographer REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(csv-slam
    src/cartographer_state.cpp
    src/csv_slam.cpp
    src/csv_writer.cpp
    src/imu_reader.cpp
    src/odometry_reader.cpp
    src/sensor_callback.cpp
    src/sensor_callback_queue.cpp
    src/util.cpp
    src/velodyne_reader.cpp
)
target_compile_features(csv-slam PRIVATE cxx_std_17)
set_target_properties(csv-slam PROPERTIES
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)
target_link_libraries(csv-slam PRIVATE
    Boost::headers
    Boost::iostreams
    cartographer
    Eigen3::Eigen
)
