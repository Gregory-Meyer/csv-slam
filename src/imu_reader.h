#ifndef IMU_READER_H
#define IMU_READER_H

#include <fstream>
#include <optional>
#include <string>

#include <cartographer/sensor/imu_data.h>

class ImuReader {
public:
  explicit ImuReader(const char *filename);

  std::optional<cartographer::sensor::ImuData> deserialize_measurement();

private:
  std::string filename_;
  std::ifstream file_;
};

#endif
