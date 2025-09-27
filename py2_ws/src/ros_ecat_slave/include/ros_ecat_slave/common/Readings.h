/**
 * @file Readings.h
 * @author Keten (2863861004@qq.com)
 * @brief 从站读取的数据基类
 * @version 0.1
 * @date 2025-05-04
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once
/* c++-lib */
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace ros_ecat_slave {

// 读取的时间戳
using ReadingTimePoint = std::chrono::time_point<
    std::chrono::high_resolution_clock>;  // 声明ReadingTimePoint
// 表示高精度的时间戳
inline double getTime(ReadingTimePoint stamp) {
  return static_cast<double>(std::chrono::duration_cast<std::chrono::seconds>(
                                 stamp.time_since_epoch())
                                 .count()) +
         1e-9 * static_cast<double>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        stamp.time_since_epoch())
                        .count() %
                    1000000000);
}

class Reading {
 public:
  Reading();
  virtual ~Reading() = default;
  const ReadingTimePoint& getStamp() const { return stamp_; }
  void setStamp(const ReadingTimePoint& stamp) { stamp_ = stamp; }

 protected:
  ReadingTimePoint stamp_;
};

}  // namespace ros_ecat_slave
