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

#include "sensor_callback.h"

#include <cassert>
#include <utility>

using std::string;
using std::unique_ptr;

using cartographer::common::Time;

using cartographer::sensor::ImuData;
using cartographer::sensor::OdometryData;
using cartographer::sensor::TimedPointCloudData;

using cartographer::mapping::PoseExtrapolator;
using cartographer::mapping::TrajectoryBuilderInterface;

Time SensorCallback::time() const noexcept { return time_; }

bool operator==(const SensorCallback &lhs, const SensorCallback &rhs) noexcept {
  return lhs.time_ == rhs.time_;
}

bool operator!=(const SensorCallback &lhs, const SensorCallback &rhs) noexcept {
  return lhs.time_ != rhs.time_;
}

bool operator<(const SensorCallback &lhs, const SensorCallback &rhs) noexcept {
  return lhs.time_ < rhs.time_;
}

bool operator<=(const SensorCallback &lhs, const SensorCallback &rhs) noexcept {
  return lhs.time_ <= rhs.time_;
}

bool operator>(const SensorCallback &lhs, const SensorCallback &rhs) noexcept {
  return lhs.time_ > rhs.time_;
}

bool operator>=(const SensorCallback &lhs, const SensorCallback &rhs) noexcept {
  return lhs.time_ >= rhs.time_;
}

SensorCallback::SensorCallback(Time time,
                               TrajectoryBuilderInterface &trajectory_builder,
                               PoseExtrapolator &pose_extrapolator) noexcept
    : time_(time), trajectory_builder_ptr_(&trajectory_builder),
      pose_extrapolator_ptr_(&pose_extrapolator) {}

SensorCallback::SensorCallback(Time time,
                               const SensorCallback &previous) noexcept
    : time_(time), trajectory_builder_ptr_(previous.trajectory_builder_ptr_),
      pose_extrapolator_ptr_(previous.pose_extrapolator_ptr_) {}

TrajectoryBuilderInterface &
SensorCallback::trajectory_builder() const noexcept {
  assert(trajectory_builder_ptr_);

  return *trajectory_builder_ptr_;
}

PoseExtrapolator &SensorCallback::pose_extrapolator() const noexcept {
  assert(pose_extrapolator_ptr_);

  return *pose_extrapolator_ptr_;
}

VelodyneSensorCallback::VelodyneSensorCallback(
    TimedPointCloudData &&data, VelodyneReader &reader,
    TrajectoryBuilderInterface &trajectory_builder,
    PoseExtrapolator &pose_extrapolator) noexcept
    : SensorCallback(data.time, trajectory_builder, pose_extrapolator),
      data_(std::move(data)), reader_ptr_(&reader) {}

VelodyneSensorCallback::VelodyneSensorCallback(
    TimedPointCloudData &&data, const VelodyneSensorCallback &previous) noexcept
    : SensorCallback(data.time, previous), data_(std::move(data)),
      reader_ptr_(previous.reader_ptr_) {}

unique_ptr<SensorCallback> VelodyneSensorCallback::operator()() const {
  trajectory_builder().AddSensorData("vel", data_);

  if (auto new_data = reader().deserialize_packet()) {
    return std::make_unique<VelodyneSensorCallback>(std::move(*new_data),
                                                    *this);
  }

  return nullptr;
}

VelodyneReader &VelodyneSensorCallback::reader() const noexcept {
  assert(reader_ptr_);

  return *reader_ptr_;
}

ImuSensorCallback::ImuSensorCallback(
    ImuData &&data, ImuReader &reader,
    TrajectoryBuilderInterface &trajectory_builder,
    PoseExtrapolator &pose_extrapolator) noexcept
    : SensorCallback(data.time, trajectory_builder, pose_extrapolator),
      data_(std::move(data)), reader_ptr_(&reader) {}

ImuSensorCallback::ImuSensorCallback(ImuData &&data,
                                     const ImuSensorCallback &previous) noexcept
    : SensorCallback(data.time, previous), data_(std::move(data)),
      reader_ptr_(previous.reader_ptr_) {}

unique_ptr<SensorCallback> ImuSensorCallback::operator()() const {
  trajectory_builder().AddSensorData("imu", data_);
  pose_extrapolator().AddImuData(data_);

  if (auto new_data = reader().deserialize_measurement()) {
    return std::make_unique<ImuSensorCallback>(std::move(*new_data), *this);
  }

  return nullptr;
}

ImuReader &ImuSensorCallback::reader() const noexcept {
  assert(reader_ptr_);

  return *reader_ptr_;
}

OdometrySensorCallback::OdometrySensorCallback(
    OdometryData &&data, OdometryReader &reader,
    TrajectoryBuilderInterface &trajectory_builder,
    PoseExtrapolator &pose_extrapolator) noexcept
    : SensorCallback(data.time, trajectory_builder, pose_extrapolator),
      data_(std::move(data)), reader_ptr_(&reader) {}

OdometrySensorCallback::OdometrySensorCallback(
    OdometryData &&data, const OdometrySensorCallback &previous) noexcept
    : SensorCallback(data.time, previous), data_(std::move(data)),
      reader_ptr_(previous.reader_ptr_) {}

unique_ptr<SensorCallback> OdometrySensorCallback::operator()() const {
  trajectory_builder().AddSensorData("odom", data_);
  pose_extrapolator().AddOdometryData(data_);

  if (auto new_data = reader().deserialize_measurement()) {
    return std::make_unique<OdometrySensorCallback>(std::move(*new_data),
                                                    *this);
  }

  return nullptr;
}

OdometryReader &OdometrySensorCallback::reader() const noexcept {
  assert(reader_ptr_);

  return *reader_ptr_;
}
