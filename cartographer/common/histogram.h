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

#ifndef CARTOGRAPHER_COMMON_HISTOGRAM_H_
#define CARTOGRAPHER_COMMON_HISTOGRAM_H_

#include <string>
#include <vector>

#include "cartographer/common/port.h"

namespace cartographer {
namespace common {
/**
 * @brief 统计数据，用直方图显示
 * 
 */
class Histogram {
 public:
  /**
   * @brief 添加数据
   * 
   * @param value 
   */
  void Add(float value);
  /**
   * @brief 通过字符串用直方图显示数据
   * 
   * @param buckets 直方图的直方条数，即把数据分为几级
   * @return string 
   */
  string ToString(int buckets) const;

 private:
  std::vector<float> values_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_HISTOGRAM_H_
