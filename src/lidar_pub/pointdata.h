#ifndef _POINT_DATA_H_
#define _POINT_DATA_H_

#include <stdint.h>

#include <iostream>
#include <vector>

struct PointData {
  // Polar coordinate representation
  float angle;         // Angle ranges from 0 to 359 degrees
  uint16_t distance;   // Distance is measured in millimeters
  uint8_t intensity;  // Intensity is 0 to 255
  // Cartesian coordinate representation
  
  double x;          // LD TOFLiDAR no support x an y param
  double y;
  PointData(float angle, uint16_t distance, uint8_t intensity, double x = 0,
            double y = 0) {
    this->angle = angle;
    this->distance = distance;
    this->intensity = intensity;
    this->x = x;
    this->y = y;
  }
  PointData() {}
  friend std::ostream &operator<<(std::ostream &os, const PointData &data) {
    os << data.angle << " " << data.distance << " " << (int)data.intensity
       << " " << data.x << " " << data.y;
    return os;
  }
};

typedef std::vector<PointData> Points2D;

#define ANGLE_TO_RADIAN(angle) ((angle)*3141.59 / 180000)
#define RADIAN_TO_ANGLED(angle) ((angle)*180000 / 3141.59)

#endif  // _POINT_DATA_H_