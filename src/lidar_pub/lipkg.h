#ifndef __LIPKG_H
#define __LIPKG_H

#include <sensor_msgs/LaserScan.h>
#include <stdint.h>

#include <array>
#include <iostream>
#include <vector>

#include "pointdata.h"

enum {
  PKG_HEADER = 0x54,
  PKG_VER_LEN = 0x2C,
  POINT_PER_PACK = 12,
};

typedef struct __attribute__((packed)) {
  uint16_t distance;
  uint8_t intensity;
} LidarPointStructDef;

typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t ver_len;
  uint16_t speed;
  uint16_t start_angle;
  LidarPointStructDef point[POINT_PER_PACK];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
} LiDARFrameTypeDef;

class LiPkg {
 public:
  LiPkg(std::string frame_id, bool laser_scan_dir, bool enable_angle_crop_func,
  double angle_crop_min, double angle_crop_max);
  // get Lidar spin speed (Hz)
  double GetSpeed(void); 
  // get time stamp of the packet
  uint16_t GetTimestamp(void) { return timestamp_; }  
  // a packet is ready                                              
  bool IsPkgReady(void) { return is_pkg_ready_; }  
  // Get lidar data frame ready flag  
  bool IsFrameReady(void) { return is_frame_ready_; }  
  // Lidar data frame readiness flag reset
  void ResetFrameReady(void) { is_frame_ready_ = false; }
  // the number of errors in parser process of lidar data frame
  long GetErrorTimes(void) { return error_times_; } 
  // Get original Lidar data package
  const std::array<PointData, POINT_PER_PACK>& GetPkgData(void);  
  // parse single packet
  bool AnalysisOne(uint8_t byte);
  bool Parse(const uint8_t* data, long len);  
  // combine stantard data into data frames and calibrate
  bool AssemblePacket();  

  // Get sensor_msg/LaserScan type data
  sensor_msgs::LaserScan GetLaserScan() { return output_; }

 private:
  const int kPointFrequence = 4500;
  std::string frame_id_;
  uint16_t timestamp_;
  double speed_;
  long error_times_;
  bool is_pkg_ready_;
  bool is_frame_ready_;
  bool laser_scan_dir_;
  bool enable_angle_crop_func_;
  double angle_crop_min_;
  double angle_crop_max_;
  LiDARFrameTypeDef pkg;
  std::vector<uint8_t> data_tmp_;
  std::array<PointData, POINT_PER_PACK> one_pkg_;
  std::vector<PointData> frame_tmp_;
  sensor_msgs::LaserScan output_;
  // Lidar frame data tranfrom to  sensor_msg/LaserScan type data
  void ToLaserscan(std::vector<PointData> src);
};




#endif  //__LIPKG_H