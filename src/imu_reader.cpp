#include "imu_reader.h"

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
using sensor::ImuData;

using Eigen::Vector3d;

ImuReader::ImuReader(istream &is) noexcept : is_ptr_(&is) {}

optional<ImuData> ImuReader::deserialize_measurement() {
  assert(is_ptr_);

  optional<ImuData> maybe_data;

  string line;
  if (std::getline(*is_ptr_, line)) {
    uint64_t utime_us;
    double Gx;
    double Gy;
    double Gz;
    double ax;
    double ay;
    double az;
    double omegax;
    double omegay;
    double omegaz;

    if (std::sscanf(line.c_str(),
                    "%" SCNu64 ",%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                    &utime_us, &Gx, &Gy, &Gz, &ax, &ay, &az, &omegax, &omegay,
                    &omegaz) != 10) {
      throw runtime_error("ImuReader::deserialize_measurement: missing or "
                          "malformed fields in CSV");
    }

    const microseconds since_unix_epoch(utime_us);
    const Time time(since_unix_epoch +
                    seconds(common::kUtsEpochOffsetFromUnixEpochInSeconds));

    const Vector3d linear_acceleration = {ax, -ay, -az};
    const Vector3d angular_velocity = {omegax, -omegay, -omegaz};

    maybe_data = ImuData{time, linear_acceleration, angular_velocity};
  }

  return maybe_data;
}
