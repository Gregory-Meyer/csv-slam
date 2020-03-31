#ifndef UTIL_H
#define UTIL_H

#include <chrono>
#include <sstream>

#include <cartographer/common/time.h>

#define FORMAT(...)                                                            \
  (static_cast<std::ostringstream &>(std::ostringstream().flush()              \
                                     << __VA_ARGS__)                           \
       .str())
#define FORMATLN(...)                                                          \
  (static_cast<std::ostringstream &>(std::ostringstream().flush()              \
                                     << __VA_ARGS__ << '\n')                   \
       .str())

cartographer::common::Time to_uts(std::chrono::microseconds utime);
std::chrono::microseconds to_utime(cartographer::common::Time uts);

#endif
