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

#ifndef CARTOGRAPHER_SENSOR_POINT_CLOUD_H_
#define CARTOGRAPHER_SENSOR_POINT_CLOUD_H_

#include <vector>

#include "Eigen/Core"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {
/**
 * @brief 点云，即三维坐标数组
 * 
 */
typedef std::vector<Eigen::Vector3f> PointCloud;
/**
 * @brief 含有反射强度信息的点云
 * 
 */
struct PointCloudWithIntensities {
  PointCloud points;
  std::vector<float> intensities;

  // For each item in 'points', contains the time delta of when it was acquired
  // after points[0], i.e. the first entry is always 0.f. If timing
  // information is not available all entries will be 0.f.
  /**
   * @brief 点points[n-1]与点points[0]的时间偏移
   * 
   */
  std::vector<float> offset_seconds;
};
/**
 * @brief 点云坐标变换
 * 
 * @param point_cloud 
 * @param transform 
 * @return PointCloud 
 */
// Transforms 'point_cloud' according to 'transform'.
PointCloud TransformPointCloud(const PointCloud& point_cloud,
                               const transform::Rigid3f& transform);

// Returns a new point cloud without points that fall outside the region defined
// by 'min_z' and 'max_z'.
/**
 * @brief 剪切点云
 * 
 * @param point_cloud 
 * @param min_z 
 * @param max_z 
 * @return PointCloud 
 */
PointCloud Crop(const PointCloud& point_cloud, float min_z, float max_z);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_POINT_CLOUD_H_
