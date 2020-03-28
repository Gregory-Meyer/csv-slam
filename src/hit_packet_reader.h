#ifndef HIT_PACKET_READER_H
#define HIT_PACKET_READER_H

#include <iosfwd>
#include <iterator>

#include <cartographer/sensor/point_cloud.h>

namespace hit_packet_reader {

class Iterator;

} // namespace hit_packet_reader

class HitPacketReader {
public:
  friend hit_packet_reader::Iterator;

  HitPacketReader(std::istream &istream);

  hit_packet_reader::Iterator begin() noexcept;
  hit_packet_reader::Iterator end() noexcept;

private:
  std::istream *istream_ptr_;
};

namespace hit_packet_reader {

class Iterator {
public:
  using difference_type = std::ptrdiff_t;
  using value_type = cartographer::sensor::PointCloudWithIntensities;
  using pointer = const value_type *;
  using reference = const value_type &;
  using iterator_category = std::input_iterator_tag;

  friend HitPacketReader;

  Iterator() = default;

  reference operator*() const;
  pointer operator->() const;
  Iterator &operator++();
  Iterator operator++(int);

  friend bool operator==(const Iterator &lhs, const Iterator &rhs) noexcept;
  friend bool operator!=(const Iterator &lhs, const Iterator &rhs) noexcept;

private:
  struct MoveTag {};

  explicit Iterator(HitPacketReader &parent) noexcept;
  Iterator(MoveTag, Iterator &iterator) noexcept;

  void deserialize_packet();

  HitPacketReader *parent_ptr_ = nullptr;
  value_type current_;
};

} // namespace hit_packet_reader

#endif
