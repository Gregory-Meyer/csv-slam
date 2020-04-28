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

#include "velodyne_reader.h"

#include "util.h"

#include <array>
#include <cassert>
#include <climits>
#include <iomanip>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

#include <boost/endian/conversion.hpp>

#include <cartographer/common/time.h>

using std::array;
using std::ifstream;
using std::istream;
using std::optional;
using std::ostream;
using std::pair;
using std::runtime_error;
using std::streamsize;
using std::string_view;
using std::uint16_t;
using std::uint32_t;
using std::uint64_t;
using std::uint8_t;
using std::vector;

namespace chrono = std::chrono;
using chrono::microseconds;

namespace endian = boost::endian;

namespace common = cartographer::common;
using common::Time;

namespace sensor = cartographer::sensor;
using sensor::PointCloudWithIntensities;
using sensor::TimedPointCloudData;
using sensor::TimedRangefinderPoint;

using Eigen::Vector3f;

namespace {

class Impl {
public:
  Impl(std::string_view filename, ifstream &file,
       const Eigen::Vector3f &origin) noexcept;
  optional<TimedPointCloudData> deserialize_packet();

private:
  template <typename T> T deserialize_integer();
  float deserialize_position();
  bool read(char *data, streamsize n);
  void ignore(streamsize n);

  runtime_error err_unreadable() const;
  runtime_error err_incomplete() const;

  std::string_view filename_;
  ifstream &file_;
  const Eigen::Vector3f &origin_;
};

} // namespace

VelodyneReader::VelodyneReader(const char *filename, const Vector3f &origin)
    : filename_(filename), file_(filename_), origin_(origin) {
  if (!file_.is_open()) {
    throw runtime_error(
        FORMAT("couldn't open \"" << filename_ << "\" for reading"));
  }
}

optional<TimedPointCloudData> VelodyneReader::deserialize_packet() {
  assert(file_.is_open());

  return Impl(filename_, file_, origin_).deserialize_packet();
}

namespace {

using MagicBuffer = array<uint8_t, 8>;

ostream &operator<<(ostream &os, const MagicBuffer &magic);
istream &operator>>(istream &is, MagicBuffer &magic);

Impl::Impl(std::string_view filename, ifstream &file,
           const Eigen::Vector3f &origin) noexcept
    : filename_(filename), file_(file), origin_(origin) {}

optional<TimedPointCloudData> Impl::deserialize_packet() {
  static_assert(sizeof(uint8_t) == 1,
                "std::uint8_t must have the same size as char");
  static_assert(CHAR_BIT == 8, "char must be 8 bits");

  if (!file_) {
    return std::nullopt;
  }

  static constexpr array<uint8_t, 8> HEADER_MAGIC = {
      {0x9C, 0xAD, 0x9C, 0xAD, 0x9C, 0xAD, 0x9C, 0xAD}};
  array<uint8_t, HEADER_MAGIC.size()> magic;

  if (!read(reinterpret_cast<char *>(magic.data()),
            static_cast<streamsize>(magic.size()))) {
    if (file_.eof() && file_.gcount() > 0) {
      throw err_incomplete();
    }

    return std::nullopt;
  }

  if (magic != HEADER_MAGIC) {
    throw runtime_error(FORMAT(
        "deserialized incorrect velodyne packet header magic number; expected "
        << HEADER_MAGIC << ", got " << magic));
  }

  static constexpr uint32_t MAX_NUM_HITS = 384;
  const auto num_hits = deserialize_integer<uint32_t>();

  if (num_hits > MAX_NUM_HITS) {
    throw runtime_error(FORMAT("velodyne packet header indicated "
                               << num_hits << " hits; expected no more than "
                               << MAX_NUM_HITS << " hits"));
  }

  const microseconds utime(deserialize_integer<uint64_t>());
  const Time time = to_uts(utime);

  static constexpr streamsize HEADER_PADDING_SIZE = 4;
  ignore(HEADER_PADDING_SIZE);

  using Ranges = vector<TimedRangefinderPoint>;
  Ranges ranges;
  ranges.reserve(static_cast<Ranges::size_type>(num_hits));

  for (uint32_t i = 0; i < num_hits; ++i) {
    static constexpr uint8_t MAX_ID = 31;
    static constexpr streamsize INTENSITY_SIZE = 1;

    // velodyne coordinate frame is x-right, y-back, z-down
    // convert to ENU coordinate frame: x-forward, y-left, z-up
    const float y = -deserialize_position();
    const float x = -deserialize_position();
    const float z = -deserialize_position();
    ignore(INTENSITY_SIZE);
    const auto id = deserialize_integer<uint8_t>();

    if (id > MAX_ID) {
      throw runtime_error(FORMAT("packet return indicated ID "
                                 << id << "; expected an ID no more than "
                                 << MAX_ID));
    }

    const TimedRangefinderPoint point = {Vector3f{x, y, z}, 0.0f};
    ranges.push_back(point);
  }

  return TimedPointCloudData{time, origin_, std::move(ranges)};
}

ostream &operator<<(ostream &os, const MagicBuffer &magic) {
  if (!(os << "{0x" << std::setfill('0') << std::setw(2) << std::right
           << std::hex << int(magic[0]))) {
    return os;
  }

  for (auto iter = magic.cbegin() + 1; iter != magic.cend(); ++iter) {
    if (!(os << ", 0x" << std::setfill('0') << std::setw(2) << std::right
             << std::hex << int(*iter))) {
      break;
    }
  }

  return os << '}';
}

istream &operator>>(istream &is, MagicBuffer &magic) {
  return is.read(reinterpret_cast<char *>(magic.data()),
                 static_cast<streamsize>(magic.size()));
}

template <typename T> T Impl::deserialize_integer() {
  T integer;

  if (!read(reinterpret_cast<char *>(&integer),
            static_cast<streamsize>(sizeof(T)))) {
    throw err_incomplete();
  }

  endian::little_to_native_inplace(integer);

  return integer;
}

float Impl::deserialize_position() {
  static constexpr uint16_t MAX_POSITION = 40000;
  static constexpr double RESOLUTION = 0.005; // meters
  static constexpr double OFFSET = 100.0;     // meters

  const auto as_fixed_point = deserialize_integer<uint16_t>();

  if (as_fixed_point > MAX_POSITION) {
    throw runtime_error(
        "VelodyneReader::deserialize_packet: hit position out of range");
  }

  return static_cast<float>((static_cast<double>(as_fixed_point) * RESOLUTION) -
                            OFFSET);
}

bool Impl::read(char *data, streamsize n) {
  if (!file_.read(data, n)) {
    if (file_.bad()) {
      throw err_unreadable();
    }

    return false;
  }

  return true;
}

void Impl::ignore(streamsize n) {
  if (!file_.ignore(n)) {
    if (file_.bad()) {
      throw err_unreadable();
    } else {
      throw err_incomplete();
    }
  }
}

runtime_error Impl::err_unreadable() const {
  return runtime_error(FORMAT("couldn't read from \"" << filename_ << '"'));
}

runtime_error Impl::err_incomplete() const {
  return runtime_error(
      FORMAT("read an incomplete velodyne packet from \"" << filename_ << '"'));
}

} // namespace
