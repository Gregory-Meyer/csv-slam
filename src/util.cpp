#include "util.h"

namespace chrono = std::chrono;
using chrono::microseconds;
using chrono::seconds;

namespace common = cartographer::common;
using common::Time;

Time to_uts(microseconds utime) {
  return Time(utime + seconds(common::kUtsEpochOffsetFromUnixEpochInSeconds));
}

microseconds to_utime(Time uts) {
  return chrono::duration_cast<microseconds>(
      uts.time_since_epoch() -
      seconds(common::kUtsEpochOffsetFromUnixEpochInSeconds));
}
