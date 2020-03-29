#ifndef HIT_PACKET_READER_H
#define HIT_PACKET_READER_H

#include <iosfwd>
#include <optional>

#include <cartographer/sensor/timed_point_cloud_data.h>

#include <Eigen/Core>

class HitPacketReader {
public:
  HitPacketReader(std::istream &istream, const Eigen::Vector3f &origin);

  std::optional<cartographer::sensor::TimedPointCloudData> deserialize_packet();

private:
  std::istream *is_ptr_;
  Eigen::Vector3f origin_;
};

#endif
