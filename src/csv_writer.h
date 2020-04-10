#ifndef CSV_WRITER_H
#define CSV_WRITER_H

#include <fstream>
#include <string>

#include <cartographer/common/time.h>
#include <cartographer/transform/rigid_transform.h>

class CsvWriter {
public:
  explicit CsvWriter(const char *filename);
  void append(const cartographer::transform::Rigid3d &pose,
              cartographer::common::Time uts);

private:
  std::string filename_;
  std::ofstream file_;
};

#endif
