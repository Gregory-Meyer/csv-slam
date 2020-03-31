#include "imu_reader.h"

#include "util.h"

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

ImuReader::ImuReader(const char *filename)
    : filename_(filename), file_(filename_) {
  if (!file_.is_open()) {
    throw runtime_error(
        FORMAT("couldn't open \"" << filename_ << "\" for reading"));
  }
}

optional<ImuData> ImuReader::deserialize_measurement() {
  optional<ImuData> maybe_data;

  string line;
  if (std::getline(file_, line)) {
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
      throw runtime_error(
          FORMAT("missing or malformed fields in imu csv file \"" << filename_
                                                                  << '"'));
    }

    const microseconds utime(utime_us);
    const Time uts = to_uts(utime);

    const Vector3d linear_acceleration = {ax, -ay, -az};
    const Vector3d angular_velocity = {omegax, -omegay, -omegaz};

    maybe_data = ImuData{uts, linear_acceleration, angular_velocity};
  } else if (file_.bad()) {
    throw runtime_error(FORMAT("couldn't read from \"" << filename_ << '"'));
  }

  return maybe_data;
}
