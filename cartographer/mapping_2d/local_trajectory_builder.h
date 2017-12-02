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

#ifndef CARTOGRAPHER_MAPPING_2D_LOCAL_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_2D_LOCAL_TRAJECTORY_BUILDER_H_

#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/pose_estimate.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer/mapping_2d/proto/local_trajectory_builder_options.pb.h"
#include "cartographer/mapping_2d/scan_matching/ceres_scan_matcher.h"
#include "cartographer/mapping_2d/scan_matching/real_time_correlative_scan_matcher.h"
#include "cartographer/mapping_2d/submaps.h"
#include "cartographer/mapping_3d/motion_filter.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/sensor/voxel_filter.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping_2d {

// Wires up the local SLAM stack (i.e. pose extrapolator, scan matching, etc.)
// without loop closure.
class LocalTrajectoryBuilder {
 public:
  // RangeData插入submap的结果,含有插入的新/旧子图和TrajectoryNode::Data数据
  // TrajectoryNode::Data数据结构:时间,重力对齐方向,重力对齐后的点云,经过scan-to-map-match但未优化的位姿
  struct InsertionResult {
    std::shared_ptr<const mapping::TrajectoryNode::Data> constant_data;
    std::vector<std::shared_ptr<const Submap>> insertion_submaps;
  };

  explicit LocalTrajectoryBuilder(
      const proto::LocalTrajectoryBuilderOptions& options);
  ~LocalTrajectoryBuilder();

  LocalTrajectoryBuilder(const LocalTrajectoryBuilder&) = delete;
  LocalTrajectoryBuilder& operator=(const LocalTrajectoryBuilder&) = delete;

  const mapping::PoseEstimate& pose_estimate() const;

  // Range data must be approximately horizontal for 2D SLAM.
  /**
   * @brief 添加RangeData
   * 
   * 该函数以num_accumulated_为调用周期处理数据,每个周期的处理流程如下:
   * 
   * 1. 使用首次被调用时的位姿为标准,
   *    调用num_accumulated_次后得到一个由num_accumulated_个RangeData数据构成的点云集合
   * 2. 将点云集合投影到水平面上,
   *    通过位姿外推器提供的初值使用ScanMatch对RangeData和旧的子图进行匹配,
   *    向旧的子图和新的子图中插入匹配完成的RangeData,
   *    返回匹配的结果
   * 
   * @param common::Time 
   * @param range_data 
   * @return std::unique_ptr<InsertionResult> 
   */
  std::unique_ptr<InsertionResult> AddRangeData(
      common::Time, const sensor::RangeData& range_data);
  void AddImuData(const sensor::ImuData& imu_data);
  void AddOdometerData(const sensor::OdometryData& odometry_data);

 private:
  std::unique_ptr<InsertionResult> AddAccumulatedRangeData(
      common::Time time, const sensor::RangeData& range_data);
  sensor::RangeData TransformAndFilterRangeData(
      const transform::Rigid3f& gravity_alignment,
      const sensor::RangeData& range_data) const;

  // Scan matches 'gravity_aligned_range_data' and fill in the
  // 'pose_observation' with the result.
  void ScanMatch(common::Time time, const transform::Rigid2d& pose_prediction,
                 const sensor::RangeData& gravity_aligned_range_data,
                 transform::Rigid2d* pose_observation);

  // Lazily constructs a PoseExtrapolator.
  void InitializeExtrapolator(common::Time time);

  const proto::LocalTrajectoryBuilderOptions options_;
  ActiveSubmaps active_submaps_;

  mapping::PoseEstimate last_pose_estimate_;

  mapping_3d::MotionFilter motion_filter_;
  scan_matching::RealTimeCorrelativeScanMatcher
      real_time_correlative_scan_matcher_;
  scan_matching::CeresScanMatcher ceres_scan_matcher_;

  std::unique_ptr<mapping::PoseExtrapolator> extrapolator_;

  int num_accumulated_ = 0;
  /**
   * @brief 每接收num_accumulated_个RangeData数据包中第一个数据包的位姿
   * 
   */
  transform::Rigid3f first_pose_estimate_ = transform::Rigid3f::Identity();
  sensor::RangeData accumulated_range_data_;
};

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_LOCAL_TRAJECTORY_BUILDER_H_
