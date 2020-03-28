#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

#include <boost/endian/conversion.hpp>

#include <cartographer/sensor/point_cloud.h>
#include <cartographer/sensor/rangefinder_point.h>

#include <Eigen/Dense>

struct Hit {
  std::uint16_t x;
  std::uint16_t y;
  std::uint16_t z;
  std::uint8_t intensity;
  std::uint8_t laser_id;
};

struct Packet {
  std::uint64_t utime_us;
  std::vector<Hit> hits;
};

int main() { std::cout << "Hello, world!\n"; }
