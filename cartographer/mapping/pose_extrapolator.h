/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
#define CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_

#include <deque>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/imu_tracker.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

// Keep poses for a certain duration to estimate linear and angular velocity.
// Uses the velocities to extrapolate motion. Uses IMU and/or odometry data if
// available to improve the extrapolation.
/**
 * @brief 位姿外推器
 * 
 * 在cartographer中PoseExtrapolator被用到了两个地方:
 * 一个是在cartographer/mapping_2d/local_trajectory_builder.h中,
 *   主要在AddRangeData()和AddAccumulatedRangeData()中,用于推算物体的运动轨迹
 *   该外推器与定位是同时的,用于构建子图
 * 
 * 另一个在cartographer_ros/node.h中,
 *   主要在PublishTrajectoryStates()中,用于向ROS发送路径信息
 *   该外推器与制图是同时的,用于显示地图
 * 
 */
class PoseExtrapolator {
 public:
  explicit PoseExtrapolator(common::Duration pose_queue_duration,
                            double imu_gravity_time_constant);

  PoseExtrapolator(const PoseExtrapolator&) = delete;
  PoseExtrapolator& operator=(const PoseExtrapolator&) = delete;

  /**
   * @brief 通过IMU观测数据初始化一个位姿外推器
   * 
   * @param pose_queue_duration 
   * @param imu_gravity_time_constant 
   * @param imu_data 
   * @return std::unique_ptr<PoseExtrapolator> 
   */
  static std::unique_ptr<PoseExtrapolator> InitializeWithImu(
      common::Duration pose_queue_duration, double imu_gravity_time_constant,
      const sensor::ImuData& imu_data);

  // Returns the time of the last added pose or Time::min() if no pose was added
  // yet.
  /**
   * @brief 最近一次位姿时间
   * 
   * @return common::Time GetLastPoseTime 
   */
  common::Time GetLastPoseTime() const;

  /**
   * @brief 添加pose数据
   * 
   * 1. 若无imu_tracker_,实例化一个
   * 2. 添加pose到timed_pose_queue_
   * 3. 修剪timed_pose_queue_,删掉一些比较旧的数据
   * 4. 刷新linear_velocity_from_poses_,angular_velocity_from_poses_和imu_tracker_
   * 5. 修剪IMU和Odometry数据
   * 
   * @param time 
   * @param pose 
   */
  void AddPose(common::Time time, const transform::Rigid3d& pose);
  /**
   * @brief 添加IMU数据
   * 
   * @param imu_data 
   */
  void AddImuData(const sensor::ImuData& imu_data);
  /**
   * @brief 添加Odometry数据,刷新linear_velocity_from_odometry_,angular_velocity_from_odometry_
   * 
   * @param odometry_data 
   */
  void AddOdometryData(const sensor::OdometryData& odometry_data);
  /**
   * @brief 外推处时刻time的位姿信息
   * 
   * @param time 
   * @return transform::Rigid3d 
   */
  transform::Rigid3d ExtrapolatePose(common::Time time);

  // Gravity alignment estimate.
  /**
   * @brief 重力对齐估计,返回重力的方向
   * 
   * @param time 
   * @return Eigen::Quaterniond 
   */
  Eigen::Quaterniond EstimateGravityOrientation(common::Time time);

 private:
  /**
   * @brief 刷新linear_velocity_from_poses_,angular_velocity_from_poses_
   * 
   */
  void UpdateVelocitiesFromPoses();
  /**
   * @brief 修剪IMU数据,直到所有的IMU数据(除第一条)时间都在timed_pose_queue_之后
   * 
   */
  void TrimImuData();
  /**
   * @brief 修剪数据,直到所有Odometry数据(除前两条)时间都在timed_pose_queue_之后
   * 
   */
  void TrimOdometryData();
  /**
   * @brief 高级IMUTracker,可以提高位姿估计的稳定性
   * 
   * 若没有IMU数据,就使用pose或odometry数据作为角速度观测,
   * 一直指向下方的重力作为加速度观测值
   * 
   * @param time 
   * @param imu_tracker 
   */
  void AdvanceImuTracker(common::Time time, ImuTracker* imu_tracker);
  Eigen::Quaterniond ExtrapolateRotation(common::Time time);
  Eigen::Vector3d ExtrapolateTranslation(common::Time time);

  const common::Duration pose_queue_duration_;
  /**
   * @brief 时间和pose
   * 
   */
  struct TimedPose {
    common::Time time;
    transform::Rigid3d pose;
  };
  std::deque<TimedPose> timed_pose_queue_;
  Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();

  const double gravity_time_constant_;
  std::deque<sensor::ImuData> imu_data_;
  /**
   * @brief IMU追踪器,跟踪IMU的方向和重力方向
   * 
   */
  std::unique_ptr<ImuTracker> imu_tracker_;

  std::deque<sensor::OdometryData> odometry_data_;
  Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
