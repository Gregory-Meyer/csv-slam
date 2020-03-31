#ifndef CSV_WRITER_H
#define CSV_WRITER_H

#include <fstream>
#include <string>

#include <cartographer/common/time.h>
#include <cartographer/mapping/trajectory_node.h>

class CsvWriter {
public:
  explicit CsvWriter(const char *filename);
  void append(const cartographer::mapping::TrajectoryNodePose &pose,
              cartographer::common::Time uts);

private:
  std::string filename_;
  std::ofstream file_;
};

#endif
