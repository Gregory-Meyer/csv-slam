#include "hit_packet_reader.h"

#include <array>
#include <cartographer/common/time.h>
#include <cassert>
#include <climits>
#include <istream>
#include <stdexcept>
#include <utility>

#include <boost/endian/conversion.hpp>

using std::array;
using std::istream;
using std::optional;
using std::pair;
using std::runtime_error;
using std::streamsize;
using std::uint16_t;
using std::uint32_t;
using std::uint64_t;
using std::uint8_t;

namespace chrono = std::chrono;
using chrono::microseconds;
using chrono::seconds;

namespace endian = boost::endian;

namespace common = cartographer::common;
using common::Time;

namespace sensor = cartographer::sensor;
using sensor::PointCloudWithIntensities;
using sensor::TimedPointCloudData;
using sensor::TimedRangefinderPoint;

using Eigen::Vector3f;

namespace {

constexpr auto &ERR_INCOMPLETE_PACKET =
    "HitPacketReader::deserialize_packet: incomplete packet";

struct Impl {
  optional<TimedPointCloudData> deserialize_packet() {
    static_assert(sizeof(uint8_t) == 1,
                  "std::uint8_t must have the same size as char");
    static_assert(CHAR_BIT == 8, "char must be 8 bits");

    if (!is) {
      return std::nullopt;
    }

    static constexpr array<uint8_t, 8> HEADER_MAGIC = {
        {0x9C, 0xAD, 0x9C, 0xAD, 0x9C, 0xAD, 0x9C, 0xAD}};
    array<uint8_t, HEADER_MAGIC.size()> magic;

    if (!is.read(reinterpret_cast<char *>(magic.data()),
                 static_cast<streamsize>(magic.size()))) {
      if (is.gcount() > 0) {
        throw runtime_error(ERR_INCOMPLETE_PACKET);
      }

      return std::nullopt;
    }

    if (magic != HEADER_MAGIC) {
      throw runtime_error("HitPacketReader::deserialize_packet: packet header "
                          "magic number did not match");
    }

    static constexpr uint32_t MAX_NUM_HITS = 384;
    const auto num_hits = deserialize_integer<uint32_t>();

    if (num_hits > MAX_NUM_HITS) {
      throw runtime_error("HitPacketReader::deserialize_packet: packet "
                          "header indicated more than 384 hits");
    }

    auto result = std::make_optional<TimedPointCloudData>();
    auto &[time, origin, ranges] = *result;

    // microseconds
    const microseconds since_unix_epoch(deserialize_integer<uint64_t>());
    time = Time(since_unix_epoch +
                seconds(common::kUtsEpochOffsetFromUnixEpochInSeconds));

    static constexpr streamsize HEADER_PADDING_SIZE = 4;
    ignore(HEADER_PADDING_SIZE);

    ranges.reserve(static_cast<decltype(ranges)::size_type>(num_hits));

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
        throw runtime_error(
            "HitPacketReader::deserialize_packet: hit id out of range");
      }

      ranges.push_back(TimedRangefinderPoint{Vector3f{x, y, z}, 0.0f});
    }

    origin = this->origin;

    return result;
  }

  istream &is;
  const Eigen::Vector3f &origin;

private:
  template <typename T> T deserialize_integer() {
    T integer;

    if (!is.read(reinterpret_cast<char *>(&integer),
                 static_cast<streamsize>(sizeof(T)))) {
      throw runtime_error(ERR_INCOMPLETE_PACKET);
    }

    endian::little_to_native_inplace(integer);

    return integer;
  }

  float deserialize_position() {
    static constexpr uint16_t MAX_POSITION = 40000;
    static constexpr double RESOLUTION = 0.005; // meters
    static constexpr double OFFSET = 100.0;     // meters

    const auto as_fixed_point = deserialize_integer<uint16_t>();

    if (as_fixed_point > MAX_POSITION) {
      throw runtime_error(
          "HitPacketReader::deserialize_packet: hit position out of range");
    }

    return static_cast<float>(
        (static_cast<double>(as_fixed_point) * RESOLUTION) - OFFSET);
  }

  void ignore(streamsize n) {
    if (!is.ignore(n)) {
      throw runtime_error(ERR_INCOMPLETE_PACKET);
    }
  }
};

} // namespace

HitPacketReader::HitPacketReader(istream &is, const Vector3f &origin)
    : is_ptr_(&is), origin_(origin) {}

optional<TimedPointCloudData> HitPacketReader::deserialize_packet() {
  assert(is_ptr_);

  return Impl{*is_ptr_, origin_}.deserialize_packet();
}
