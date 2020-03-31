#ifndef STARTUP_H
#define STARTUP_H

#include <memory>
#include <string>

#include <cartographer/mapping/map_builder.h>

struct CartographerState {
  std::unique_ptr<cartographer::mapping::MapBuilder> map_builder_ptr;
  int trajectory_id;
  std::string vel_sensor_id;
  std::string imu_sensor_id;

  static CartographerState from_config_filename(const char *config_filename);
};

#endif
