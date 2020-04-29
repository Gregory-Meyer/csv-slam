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

#ifndef SENSOR_CALLBACK_QUEUE_H
#define SENSOR_CALLBACK_QUEUE_H

#include "imu_reader.h"
#include "odometry_reader.h"
#include "sensor_callback.h"
#include "velodyne_reader.h"

#include <optional>
#include <queue>
#include <vector>

#include <cartographer/common/time.h>
#include <cartographer/mapping/pose_extrapolator.h>
#include <cartographer/mapping/trajectory_builder_interface.h>

namespace sensor_callback_queue {

struct Args {
  cartographer::mapping::TrajectoryBuilderInterface *trajectory_builder_ptr =
      nullptr;
  cartographer::mapping::PoseExtrapolator *pose_extrapolator_ptr = nullptr;

  VelodyneReader *vel_ptr = nullptr;
  ImuReader *imu_ptr = nullptr;
  OdometryReader *odom_ptr = nullptr;
};

} // namespace sensor_callback_queue

class SensorCallbackQueue {
public:
  SensorCallbackQueue(sensor_callback_queue::Args &&args);

  bool empty() const noexcept;
  std::optional<cartographer::common::Time> pop();

private:
  std::vector<SensorCallbackPtr> callbacks_;
};

#endif
