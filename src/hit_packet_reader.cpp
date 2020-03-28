#include "hit_packet_reader.h"

#include <cassert>
#include <climits>
#include <istream>
#include <stdexcept>
#include <utility>

#include <boost/endian/conversion.hpp>

HitPacketReader::HitPacketReader(std::istream &istream)
    : istream_ptr_(&istream) {}

hit_packet_reader::Iterator HitPacketReader::begin() noexcept {
  return hit_packet_reader::Iterator(*this);
}

hit_packet_reader::Iterator HitPacketReader::end() noexcept {
  return hit_packet_reader::Iterator();
}

namespace hit_packet_reader {

Iterator::reference Iterator::operator*() const {
  if (!parent_ptr_) {
    throw std::logic_error("hit_packet_reader::Iterator::operator*: cannot "
                           "dereference a past-the-end-iterator");
  }

  return current_;
}

Iterator::pointer Iterator::operator->() const {
  if (!parent_ptr_) {
    throw std::logic_error("hit_packet_reader::Iterator::operator->: cannot "
                           "dereference a past-the-end-iterator");
  }

  return &current_;
}

Iterator &Iterator::operator++() {
  if (!parent_ptr_) {
    throw std::logic_error("hit_packet_reader::Iterator::operator++(): cannot "
                           "increment a past-the-end-iterator");
  }

  deserialize_packet();

  return *this;
}

Iterator Iterator::operator++(int) {
  if (!parent_ptr_) {
    throw std::logic_error(
        "hit_packet_reader::Iterator::operator++(int): cannot "
        "increment a past-the-end-iterator");
  }

  const Iterator ret(MoveTag(), *this);

  ++*this;

  return ret;
}

bool operator==(const Iterator &lhs, const Iterator &rhs) noexcept {
  return lhs.parent_ptr_ == rhs.parent_ptr_;
}

bool operator!=(const Iterator &lhs, const Iterator &rhs) noexcept {
  return lhs.parent_ptr_ != rhs.parent_ptr_;
}

Iterator::Iterator(HitPacketReader &parent) noexcept : parent_ptr_(&parent) {
  deserialize_packet();
}

Iterator::Iterator(MoveTag, Iterator &iterator) noexcept
    : parent_ptr_(iterator.parent_ptr_),
      current_(std::move(iterator.current_)) {}

constexpr auto &ERR_INCOMPLETE_PACKET =
    "hit_packet_reader::Iterator::deserialize_packet: incomplete packet";

template <typename T> static T deserialize_integer(std::istream &istream) {
  T integer;

  if (!istream.read(reinterpret_cast<char *>(&integer),
                    static_cast<std::streamsize>(sizeof(T)))) {
    throw std::runtime_error(ERR_INCOMPLETE_PACKET);
  }

  boost::endian::little_to_native_inplace(integer);

  return integer;
}

static float deserialize_position(std::istream &istream) {
  static constexpr std::uint16_t MAX_POSITION = 40000;

  const auto as_fixed_point = deserialize_integer<std::uint16_t>(istream);

  if (as_fixed_point > MAX_POSITION) {
    throw std::runtime_error("hit_packet_reader::Iterator::deserialize_packet: "
                             "hit position out of range");
  }

  return static_cast<float>((static_cast<double>(as_fixed_point) / 200.0) -
                            100.0);
}

void Iterator::deserialize_packet() try {
  static_assert(sizeof(std::uint8_t) == 1,
                "uint8_t must have the same size as char");
  static_assert(CHAR_BIT == 8, "char must be 8 bits");

  static constexpr auto &HEADER_MAGIC = "\x9C\xAD\x9C\xAD\x9C\xAD\x9C\xAD";

  assert(parent_ptr_);

  std::uint8_t magic[sizeof(HEADER_MAGIC)];

  if (!parent_ptr_->istream_ptr_->read(
          &reinterpret_cast<char &>(magic),
          static_cast<std::streamsize>(sizeof(magic)))) {
    if (parent_ptr_->istream_ptr_->gcount() > 0) {
      throw std::runtime_error(ERR_INCOMPLETE_PACKET);
    }

    parent_ptr_ = nullptr;
    current_ = value_type();

    return;
  }

  if (!std::equal(std::cbegin(magic), std::cend(magic), HEADER_MAGIC)) {
    throw std::runtime_error("hit_packet_reader::Iterator::deserialize_packet: "
                             "packet header magic number did not match");
  }

  const auto num_hits =
      deserialize_integer<std::uint32_t>(*parent_ptr_->istream_ptr_);

  if (num_hits > 384) {
    throw std::runtime_error("hit_packet_reader::Iterator::deserialize_packet: "
                             "packet header indicated more than 384 hits");
  }

  // microseconds
  const auto utime =
      deserialize_integer<std::uint64_t>(*parent_ptr_->istream_ptr_);

  if (!parent_ptr_->istream_ptr_->ignore(4)) {
    throw std::runtime_error(ERR_INCOMPLETE_PACKET);
  }

  current_.points.clear();
  current_.intensities.clear();

  current_.points.reserve(
      static_cast<decltype(current_.points)::size_type>(num_hits));
  current_.intensities.reserve(
      static_cast<decltype(current_.intensities)::size_type>(num_hits));

  for (std::uint32_t i = 0; i < num_hits; ++i) {
    static constexpr std::uint8_t MAX_ID = 31;

    const float x = deserialize_position(*parent_ptr_->istream_ptr_);
    const float y = deserialize_position(*parent_ptr_->istream_ptr_);
    const float z = deserialize_position(*parent_ptr_->istream_ptr_);
    const float intensity =
        static_cast<float>(
            deserialize_integer<std::uint8_t>(*parent_ptr_->istream_ptr_)) /
        static_cast<float>(UINT8_MAX);

    const auto id =
        deserialize_integer<std::uint8_t>(*parent_ptr_->istream_ptr_);

    if (id > MAX_ID) {
      throw std::logic_error("hit_packet_reader::Iterator::deserialize_packet: "
                             "hit id out of range");
    }

    current_.points.push_back(
        cartographer::sensor::TimedRangefinderPoint{{x, y, z}, 0.0f});
    current_.intensities.push_back(intensity);
  }
} catch (...) {
  parent_ptr_ = nullptr;
  current_ = value_type();

  throw;
}

} // namespace hit_packet_reader
