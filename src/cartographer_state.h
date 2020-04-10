#ifndef STARTUP_H
#define STARTUP_H

#include <memory>
#include <string>

#include <cartographer/mapping/map_builder.h>
#include <cartographer/mapping/trajectory_builder_interface.h>

struct CartographerState {
  std::unique_ptr<cartographer::mapping::MapBuilder> map_builder_ptr;
  int trajectory_id;
  std::string vel_sensor_id;
  std::string imu_sensor_id;

  static CartographerState from_config_filename_and_callback(
      const char *config_filename,
      cartographer::mapping::TrajectoryBuilderInterface::LocalSlamResultCallback
          callback);
};

#endif
