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

#ifndef CARTOGRAPHER_COMMON_RATE_TIMER_H_
#define CARTOGRAPHER_COMMON_RATE_TIMER_H_

#include <chrono>
#include <deque>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"

namespace cartographer {
namespace common {

// Computes the rate at which pulses come in.
/**
 * @brief 采样器类,统计数据频率和处理频率,一般是1:1
 * 
 * @tparam std::chrono::steady_clock 
 */
template <typename ClockType = std::chrono::steady_clock>
class RateTimer {
 public:
  // Computes the rate at which pulses come in over 'window_duration' in wall
  // time.
  /**
   * @brief 构造函数
   * 
   * @param window_duration 采样事件窗口时间间隔(上限)
   */
  explicit RateTimer(const common::Duration window_duration)
      : window_duration_(window_duration) {}
  ~RateTimer() {}

  RateTimer(const RateTimer&) = delete;
  RateTimer& operator=(const RateTimer&) = delete;

  // Returns the pulse rate in Hz.
  /**
   * @brief 计算数据包中固有的采样频率,单位:Hz
   * 
   * @return double 
   */
  double ComputeRate() const {
    if (events_.empty()) {
      return 0.;
    }
    return static_cast<double>(events_.size() - 1) /
           common::ToSeconds((events_.back().time - events_.front().time));
  }

  // Returns the ratio of the pulse rate (with supplied times) to the wall time
  // rate. For example, if a sensor produces pulses at 10 Hz, but we call Pulse
  // at 20 Hz wall time, this will return 2.
  /**
   * @brief 计算将数据包交付上层的频率(处理频率)和传感器数据频率之比
   * 
   * @return double 
   */
  double ComputeWallTimeRateRatio() const {
    if (events_.empty()) {
      return 0.;
    }
    return common::ToSeconds((events_.back().time - events_.front().time)) /
           std::chrono::duration_cast<std::chrono::duration<double>>(
               events_.back().wall_time - events_.front().wall_time)
               .count();
  }

  // Records an event that will contribute to the computed rate.
  /**
   * @brief 记录采样事件(sensor_time,wall_time)
   * 
   * @param time 
   */
  void Pulse(common::Time time) {
    events_.push_back(Event{time, ClockType::now()});
    while (events_.size() > 2 &&
           (events_.back().wall_time - events_.front().wall_time) >
           // 这样写是有问题的,因为这里的大于号比较的只是两个Duration的tick值.
           // 例如,当你使用默认时钟的时候,大于号左侧的表示有几个1ns,右侧是有几个100ns
           // 这样比较相当于把设定的窗口增加了
               window_duration_) {
    //  std::cout << "loop ";
      events_.pop_front();
    }
    /*
    printf("delta_wall_time:%ld delta_pack_time:%ld win:%ld cur_time:%ld event_num:%ld\n",
           (events_.back().wall_time - events_.front().wall_time).count(),
           (events_.back().time - events_.front().time).count(),
           window_duration_.count(),
           ClockType::now().time_since_epoch().count(),
           events_.size()           
    );
    */
  }

  // Returns a debug string representation.
  string DebugString() const {
    if (events_.size() < 2) {
      return "unknown";
    }
    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << ComputeRate() << " Hz "
        << DeltasDebugString() << " (pulsed at "
        << ComputeWallTimeRateRatio() * 100. << "% real time)";
    return out.str();
  }

 private:
  struct Event {
    common::Time time;
    typename ClockType::time_point wall_time;
  };

  // Computes all differences in seconds between consecutive pulses.
  /**
   * @brief 计算采样的时间间隔
   * 
   * @return std::vector<double> 
   */
  std::vector<double> ComputeDeltasInSeconds() const {
    CHECK_GT(events_.size(), 1);
    const size_t count = events_.size() - 1;
    std::vector<double> result;
    result.reserve(count);
    for (size_t i = 0; i != count; ++i) {
      result.push_back(
          common::ToSeconds(events_[i + 1].time - events_[i].time));
    }
    return result;
  }

  // Returns the average and standard deviation of the deltas.
  string DeltasDebugString() const {
    const auto deltas = ComputeDeltasInSeconds();
    const double sum = std::accumulate(deltas.begin(), deltas.end(), 0.);
    const double mean = sum / deltas.size();

    double squared_sum = 0.;
    for (const double x : deltas) {
      squared_sum += common::Pow2(x - mean);
    }
    const double sigma = std::sqrt(squared_sum / (deltas.size() - 1));

    std::ostringstream out;
    out << std::scientific << std::setprecision(2) << mean << " s +/- " << sigma
        << " s";
    return out.str();
  }

  std::deque<Event> events_;
  const common::Duration window_duration_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_RATE_TIMER_H_
