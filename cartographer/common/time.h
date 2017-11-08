/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_COMMON_TIME_H_
#define CARTOGRAPHER_COMMON_TIME_H_

#include <chrono>
#include <ostream>
#include <ratio>

#include "cartographer/common/port.h"

namespace cartographer {
namespace common {
/**
 * @brief 0001年1月1日到1970年1月1日所经历的秒数
 * 
 */
constexpr int64 kUtsEpochOffsetFromUnixEpochInSeconds =
    (719162ll * 24ll * 60ll * 60ll);

struct UniversalTimeScaleClock {
  using rep = int64;
  /**
   * @brief 周期为 100ns
   * 
   */
  using period = std::ratio<1, 10000000>;
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
  static constexpr bool is_steady = true;
};

// Represents Universal Time Scale durations and timestamps which are 64-bit
// integers representing the 100 nanosecond ticks since the Epoch which is
// January 1, 1 at the start of day in UTC.
/**
 * @brief 时间间隔，单位:100ns
 * 
 */
using Duration = UniversalTimeScaleClock::duration;
/**
 * @brief 时刻，单位:100ns
 * 
 */
using Time = UniversalTimeScaleClock::time_point;

// Convenience functions to create common::Durations.
Duration FromSeconds(double seconds);
Duration FromMilliseconds(int64 milliseconds);

// Returns the given duration in seconds.
double ToSeconds(Duration duration);

// Creates a time from a Universal Time Scale.
/**
 * @brief 时间转换
 * 
 * @param ticks 单位:100ns
 * @return Time Time对象
 */
Time FromUniversal(int64 ticks);

// Outputs the Universal Time Scale timestamp for a given Time.
/**
 * @brief 时间转换
 * 
 * @param time 单位:100ns
 * @return int64 Time对象
 */
int64 ToUniversal(Time time);

// For logging and unit tests, outputs the timestamp integer.
std::ostream& operator<<(std::ostream& os, Time time);

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_TIME_H_
