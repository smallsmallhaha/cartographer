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

#include "cartographer/mapping/probability_values.h"

namespace cartographer {
namespace mapping {

namespace {

// 0 is unknown, [1, 32767] maps to [kMinProbability, kMaxProbability].
/**
 * @brief 线性映射 [1, 2^15-1] -> [kMinProbability, kMaxProbability]
 * 
 * @param value 
 * @return float 
 */
float SlowValueToProbability(const uint16 value) {
  CHECK_GE(value, 0);
  CHECK_LE(value, 32767);
  if (value == kUnknownProbabilityValue) {
    // Unknown cells have kMinProbability.
    return kMinProbability;
  }
  const float kScale = (kMaxProbability - kMinProbability) / 32766.f;
  return value * kScale + (kMinProbability - kScale);
}

/**
 * @brief 预先计算概率格网值和概率值的映射表,只被调用一次
 * 
 * 映射表结构: 两个相同的浮点型数组
 * 每个数组都表示了 [0,2^15-1] 到 [0.1,0.9] 的线性映射关系
 * 
 * @return const std::vector<float>* 
 */
const std::vector<float>* PrecomputeValueToProbability() {
  std::vector<float>* result = new std::vector<float>;
  // Repeat two times, so that both values with and without the update marker
  // can be converted to a probability.
  for (int repeat = 0; repeat != 2; ++repeat) {
    for (int value = 0; value != 32768; ++value) {
      result->push_back(SlowValueToProbability(value));
    }
  }
  return result;
}

}  // namespace

/**
 * @brief 概率格网值和概率值的映射表
 * 
 * 映射表结构: 两个相同的浮点型数组
 * 每个数组都表示了 [0,2^15-1] 到 [0.1,0.9] 的线性映射关系
 * 
 * 使用两个表是为了使用更新标志位kUpdateMarker时,仍然能直接使用该表完成[0,2^16-1]的映射
 * 
 */
const std::vector<float>* const kValueToProbability =
    PrecomputeValueToProbability();

std::vector<uint16> ComputeLookupTableToApplyOdds(const float odds) {
  std::vector<uint16> result;
  result.push_back(ProbabilityToValue(ProbabilityFromOdds(odds)) +
                   kUpdateMarker);
  for (int cell = 1; cell != 32768; ++cell) {
    result.push_back(ProbabilityToValue(ProbabilityFromOdds(
                         odds * Odds((*kValueToProbability)[cell]))) +
                     kUpdateMarker);
  }
  return result;
}

}  // namespace mapping
}  // namespace cartographer
