#ifndef IMU_READER_H
#define IMU_READER_H

#include <iosfwd>
#include <optional>

#include <cartographer/sensor/imu_data.h>

class ImuReader {
public:
  explicit ImuReader(std::istream &is) noexcept;

  std::optional<cartographer::sensor::ImuData> deserialize_measurement();

private:
  std::istream *is_ptr_;
};

#endif
