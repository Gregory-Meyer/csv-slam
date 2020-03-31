#ifndef VELODYNE_READER_H
#define VELODYNE_READER_H

#include <fstream>
#include <optional>
#include <string>

#include <cartographer/sensor/timed_point_cloud_data.h>

#include <Eigen/Core>

class VelodyneReader {
public:
  VelodyneReader(const char *filename, const Eigen::Vector3f &origin);

  std::optional<cartographer::sensor::TimedPointCloudData> deserialize_packet();

private:
  std::string filename_;
  std::ifstream file_;
  Eigen::Vector3f origin_;
};

#endif
