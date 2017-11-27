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

#ifndef CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
#define CARTOGRAPHER_MAPPING_IMU_TRACKER_H_

#include "Eigen/Geometry"
#include "cartographer/common/time.h"

namespace cartographer {
namespace mapping {

// Keeps track of the orientation using angular velocities and linear
// accelerations from an IMU. Because averaged linear acceleration (assuming
// slow movement) is a direct measurement of gravity, roll/pitch does not drift,
// though yaw does.
/**
 * @brief 利用IMU测量的加速度和角速度追踪(重力)方向
 * 
 */
class ImuTracker {
 public:
  ImuTracker(double imu_gravity_time_constant, common::Time time);

  // Advances to the given 'time' and updates the orientation to reflect this.
  /**
   * @brief 更新时间time_到当前时间,同时更新IMU方向和重力方向
   * 
   * @param time 
   */
  void Advance(common::Time time);

  // Updates from an IMU reading (in the IMU frame).
  /**
   * @brief 添加加速度观测值
   * 
   * 使用指数平均获得重力新的重力方向
   * 指数平均简介: gv_new=(1-a)*gv_new+a*gv_old (gv=gravity_vector)
   * 当a->0时,gv_new~=gv_new
   * 
   * 感觉这个指数平均除了能保证取新的重力方向为重力方向,并没有什么太大的用处啊
   * 
   * @param imu_linear_acceleration 
   */
  void AddImuLinearAccelerationObservation(
      const Eigen::Vector3d& imu_linear_acceleration);
  /**
   * @brief 添加角速度观测值
   * 
   * @param imu_angular_velocity 
   */
  void AddImuAngularVelocityObservation(
      const Eigen::Vector3d& imu_angular_velocity);

  // Query the current time.
  /**
   * @brief 返回当前时间
   * 
   * @return common::Time time 
   */
  common::Time time() const { return time_; }

  // Query the current orientation estimate.
  /**
   * @brief 返回当前的IMU方向估计值
   * 
   * @return Eigen::Quaterniond orientation 
   */
  Eigen::Quaterniond orientation() const { return orientation_; }

 private:
  // 重力常数
  const double imu_gravity_time_constant_;
  // 当前时间
  common::Time time_;
  // 上一次获得线性加速度的时间
  common::Time last_linear_acceleration_time_;
  // IMU方向,初始时方向为Eigen::Quaterniond::I
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d gravity_vector_;
  Eigen::Vector3d imu_angular_velocity_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
