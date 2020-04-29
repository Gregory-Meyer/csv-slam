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

#include "sensor_callback_queue.h"

#include <algorithm>
#include <cassert>
#include <memory>
#include <utility>

using sensor_callback_queue::Args;

using std::optional;

using cartographer::common::Time;

namespace {

bool callback_ptr_greater(const SensorCallbackPtr &lhs,
                          const SensorCallbackPtr &rhs) noexcept;

} // namespace

SensorCallbackQueue::SensorCallbackQueue(Args &&args) {
  assert(args.trajectory_builder_ptr);
  assert(args.pose_extrapolator_ptr);
  assert(args.vel_ptr);
  assert(args.imu_ptr);

  if (auto vel = args.vel_ptr->deserialize_packet()) {
    callbacks_.push_back(std::make_unique<VelodyneSensorCallback>(
        std::move(*vel), *args.vel_ptr, *args.trajectory_builder_ptr,
        *args.pose_extrapolator_ptr));
  }

  if (auto imu = args.imu_ptr->deserialize_measurement()) {
    callbacks_.push_back(std::make_unique<ImuSensorCallback>(
        std::move(*imu), *args.imu_ptr, *args.trajectory_builder_ptr,
        *args.pose_extrapolator_ptr));
  }

  if (args.odom_ptr) {
    if (auto odom = args.odom_ptr->deserialize_measurement()) {
      callbacks_.push_back(std::make_unique<OdometrySensorCallback>(
          std::move(*odom), *args.odom_ptr, *args.trajectory_builder_ptr,
          *args.pose_extrapolator_ptr));
    }
  }

  std::make_heap(callbacks_.begin(), callbacks_.end(), callback_ptr_greater);
}

bool SensorCallbackQueue::empty() const noexcept { return callbacks_.empty(); }

optional<Time> SensorCallbackQueue::pop() {
  if (callbacks_.empty()) {
    return std::nullopt;
  }

  std::pop_heap(callbacks_.begin(), callbacks_.end(), callback_ptr_greater);

  SensorCallbackPtr elem_ptr = std::move(callbacks_.back());
  callbacks_.pop_back();

  if (SensorCallbackPtr new_elem_ptr = (*elem_ptr)()) {
    callbacks_.push_back(std::move(new_elem_ptr));
    std::push_heap(callbacks_.begin(), callbacks_.end(), callback_ptr_greater);
  }

  return elem_ptr->time();
}

namespace {

bool callback_ptr_greater(const SensorCallbackPtr &lhs,
                          const SensorCallbackPtr &rhs) noexcept {
  assert(lhs);
  assert(rhs);

  return *lhs > *rhs;
}

} // namespace
