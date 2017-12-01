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

#ifndef CARTOGRAPHER_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_
#define CARTOGRAPHER_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_

#include <vector>

#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace transform {

// A time-ordered buffer of transforms that supports interpolated lookups.
/**
 * @brief 坐标变换内插器(使用缓冲区实现)
 * 
 */
class TransformInterpolationBuffer {
 public:
  TransformInterpolationBuffer() = default;
  explicit TransformInterpolationBuffer(
      const mapping::proto::Trajectory& trajectory);

  // Adds a new transform to the buffer and removes the oldest transform if the
  // buffer size limit is exceeded.
  // 向缓冲区挺安静新的坐标变换
  void Push(common::Time time, const transform::Rigid3d& transform);

  // Returns true if an interpolated transfrom can be computed at 'time'.
  // 是否能计算时间点time的坐标变换
  bool Has(common::Time time) const;

  // Returns an interpolated transform at 'time'. CHECK()s that a transform at
  // 'time' is available.
  // 查找时间点time的坐标变换
  transform::Rigid3d Lookup(common::Time time) const;

  // Returns the timestamp of the earliest transform in the buffer or 0 if the
  // buffer is empty.
  // 返回缓冲区的最晚时间,若缓冲区为空,返回0
  common::Time earliest_time() const;

  // Returns the timestamp of the earliest transform in the buffer or 0 if the
  // buffer is empty.
  // 返回缓冲区的最早时间,若缓冲区为空,返回0
  common::Time latest_time() const;

  // Returns true if the buffer is empty.
  // 缓冲区是否为空
  bool empty() const;

 private:
  /**
   * @brief 主要的数据结构，记录时间和变换矩阵的对应关系对
   * 
   */
  struct TimestampedTransform {
    common::Time time;
    transform::Rigid3d transform;
  };

  std::vector<TimestampedTransform> timestamped_transforms_;
};

}  // namespace transform
}  // namespace cartographer

#endif  // CARTOGRAPHER_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_
