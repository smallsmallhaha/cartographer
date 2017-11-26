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

#ifndef CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_
#define CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_

#include <cmath>
#include <vector>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

/**
 * @brief Odds函数
 * 
 * @param probability 
 * @return float 
 */
inline float Odds(float probability) {
  return probability / (1.f - probability);
}

/**
 * @brief Odds的反函数
 * 
 * @param odds 
 * @return float 
 */
inline float ProbabilityFromOdds(const float odds) {
  return odds / (odds + 1.f);
}

constexpr float kMinProbability = 0.1f;
constexpr float kMaxProbability = 1.f - kMinProbability;

// Clamps probability to be in the range [kMinProbability, kMaxProbability].
/**
 * @brief 紧缩函数,将probability调整至正常区间
 * 
 * @param probability 
 * @return float 
 */
inline float ClampProbability(const float probability) {
  return common::Clamp(probability, kMinProbability, kMaxProbability);
}

constexpr uint16 kUnknownProbabilityValue = 0;
/**
 * @brief 更新标志位,最高位为1
 * 
 */
constexpr uint16 kUpdateMarker = 1u << 15;

// Converts a probability to a uint16 in the [1, 32767] range.
/**
 * @brief 线性映射 [0.1,0.9] -> [1,2^15-1]
 * 
 * @param probability 
 * @return uint16 
 */
inline uint16 ProbabilityToValue(const float probability) {
  const int value =
      common::RoundToInt((ClampProbability(probability) - kMinProbability) *
                         (32766.f / (kMaxProbability - kMinProbability))) +
      1;
  // DCHECK for performance.
  DCHECK_GE(value, 1);
  DCHECK_LE(value, 32767);
  return value;
}

extern const std::vector<float>* const kValueToProbability;

// Converts a uint16 (which may or may not have the update marker set) to a
// probability in the range [kMinProbability, kMaxProbability].
/**
 * @brief 概率类型变换,可能有标志位
 * 
 * @param value 
 * @return float 
 */
inline float ValueToProbability(const uint16 value) {
  return (*kValueToProbability)[value];
}

/**
 * @brief 根据给定的odds值计算查找表,为应用该odds值做准备
 * 
 * 查找表结构: 32768位的数组
 * 第一位为 ProbabilityToValue(ProbabilityFromOdds(odds))+kUpdateMarker
 * 后面都是 ProbabilityToValue(ProbabilityFromOdds(odds * Odds(kValueToProbability[index])))+kUpdateMarker
 * 注意 kUpdateMarker 为标志位,以最高位是否为1标志是否经过更新
 * 
 * 公式: M_new = OddsInv( odds * Odds( M_old ) )
 * 详情见论文
 * 
 * @param odds 
 * @return std::vector<uint16> 
 */
std::vector<uint16> ComputeLookupTableToApplyOdds(float odds);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_
