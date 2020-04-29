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

#ifndef SENSOR_CALLBACK_H
#define SENSOR_CALLBACK_H

#include "imu_reader.h"
#include "odometry_reader.h"
#include "velodyne_reader.h"

#include <memory>
#include <string>

#include <cartographer/common/time.h>
#include <cartographer/mapping/pose_extrapolator.h>
#include <cartographer/mapping/trajectory_builder_interface.h>
#include <cartographer/sensor/imu_data.h>
#include <cartographer/sensor/odometry_data.h>
#include <cartographer/sensor/timed_point_cloud_data.h>

class SensorCallback;

using SensorCallbackPtr = std::unique_ptr<SensorCallback>;

class SensorCallback {
public:
  virtual ~SensorCallback() = default;

  cartographer::common::Time time() const noexcept;
  virtual SensorCallbackPtr operator()() const = 0;

  friend bool operator==(const SensorCallback &lhs,
                         const SensorCallback &rhs) noexcept;
  friend bool operator!=(const SensorCallback &lhs,
                         const SensorCallback &rhs) noexcept;
  friend bool operator<(const SensorCallback &lhs,
                        const SensorCallback &rhs) noexcept;
  friend bool operator<=(const SensorCallback &lhs,
                         const SensorCallback &rhs) noexcept;
  friend bool operator>(const SensorCallback &lhs,
                        const SensorCallback &rhs) noexcept;
  friend bool operator>=(const SensorCallback &lhs,
                         const SensorCallback &rhs) noexcept;

protected:
  SensorCallback(
      cartographer::common::Time time,
      cartographer::mapping::TrajectoryBuilderInterface &trajectory_builder,
      cartographer::mapping::PoseExtrapolator &pose_extrapolator) noexcept;
  SensorCallback(cartographer::common::Time time,
                 const SensorCallback &previous) noexcept;

  cartographer::mapping::TrajectoryBuilderInterface &
  trajectory_builder() const noexcept;

  cartographer::mapping::PoseExtrapolator &pose_extrapolator() const noexcept;

private:
  cartographer::common::Time time_;
  cartographer::mapping::TrajectoryBuilderInterface *trajectory_builder_ptr_;
  cartographer::mapping::PoseExtrapolator *pose_extrapolator_ptr_;
};

class VelodyneSensorCallback : public SensorCallback {
public:
  VelodyneSensorCallback(
      cartographer::sensor::TimedPointCloudData &&data, VelodyneReader &reader,
      cartographer::mapping::TrajectoryBuilderInterface &trajectory_builder,
      cartographer::mapping::PoseExtrapolator &pose_extrapolator) noexcept;
  VelodyneSensorCallback(cartographer::sensor::TimedPointCloudData &&data,
                         const VelodyneSensorCallback &previous) noexcept;

  virtual ~VelodyneSensorCallback() = default;

  SensorCallbackPtr operator()() const override;

private:
  VelodyneReader &reader() const noexcept;

  cartographer::sensor::TimedPointCloudData data_;
  VelodyneReader *reader_ptr_;
};

class ImuSensorCallback : public SensorCallback {
public:
  ImuSensorCallback(
      cartographer::sensor::ImuData &&data, ImuReader &reader,
      cartographer::mapping::TrajectoryBuilderInterface &trajectory_builder,
      cartographer::mapping::PoseExtrapolator &pose_extrapolator) noexcept;
  ImuSensorCallback(cartographer::sensor::ImuData &&data,
                    const ImuSensorCallback &previous) noexcept;

  virtual ~ImuSensorCallback() = default;

  SensorCallbackPtr operator()() const override;

private:
  ImuReader &reader() const noexcept;

  cartographer::sensor::ImuData data_;
  ImuReader *reader_ptr_;
};

class OdometrySensorCallback : public SensorCallback {
public:
  OdometrySensorCallback(
      cartographer::sensor::OdometryData &&data, OdometryReader &reader,
      cartographer::mapping::TrajectoryBuilderInterface &trajectory_builder,
      cartographer::mapping::PoseExtrapolator &pose_extrapolator) noexcept;
  OdometrySensorCallback(cartographer::sensor::OdometryData &&data,
                         const OdometrySensorCallback &previous) noexcept;

  virtual ~OdometrySensorCallback() = default;

  SensorCallbackPtr operator()() const override;

private:
  OdometryReader &reader() const noexcept;

  cartographer::sensor::OdometryData data_;
  OdometryReader *reader_ptr_;
};

#endif
