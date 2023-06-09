#ifndef __TOFBF_H_
#define __TOFBF_H_

#include <stdint.h>

#include <vector>

#include "lipkg.h"

class Tofbf {
 private:
  const int kIntensityLow = 15;  // Low intensity threshold
  const int kIntensitySingle = 220;  // Discrete points require higher intensity
  const int kScanFrequency = 4500;  // Default scan frequency, to change, read
                                    // according to radar protocol
  double curr_speed_;
  Tofbf() = delete;
  Tofbf(const Tofbf &) = delete;
  Tofbf &operator=(const Tofbf &) = delete;

 public:
  Tofbf(int speed);
  std::vector<PointData> NearFilter(const std::vector<PointData> &tmp) const;
  ~Tofbf();
};

#endif  //__TOFBF_H_