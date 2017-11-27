-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.


-- cartographer处理消息的过程:
-- 1. 接收到激光数据
--      将每个激光帧的数据平均分成num_subdivisions_per_laser_scan个RangeData
--      将每scans_per_accumulation个RangeData的数据按照第一个RangeData的pose组合到一个坐标系下
--      重力对齐
--      修剪z异常的值
--      使用大小为voxel_filter_size的过滤器过滤[1]
--      使用ScanMatch将RangeData与旧子图的概率格网做匹配[3]
--      将完成匹配的RangeData插入新的和旧的子图
--      返回插入结果(含过滤点云[2])
-- 
-- 2. 接收到IMU数据
--      数据会被传输给位姿外推器,用于估计位姿,没有IMU的话,路径将会被视为在一个平面上
--       
--       


TRAJECTORY_BUILDER_2D = {
  -- 是否使用IMU数据
  use_imu_data = true,
  -- 最小距离,低于此距离的测量数据直接被丢弃
  min_range = 0.,
  -- 最大距离,高于此距离的为miss,详见论文
  -- 注: 只有处于两个距离之间的才能作为hit
  max_range = 30.,
  -- 经过组合和重力对齐后的RangeData的合法范围
  -- 修改后的RangeData再经过过滤后可用于匹配和构建新的子图
  min_z = -0.8,
  max_z = 2.,
  -- 当距离测量值大于max_range,通过下面公式计算miss坐标:
  -- miss=origin+missing_data_ray_length/range*(miss-origin)
  -- 其中,origin为原点坐标,miss为miss点的坐标,range为距离
  missing_data_ray_length = 5.,
  -- 将每scans_per_accumulation个RangeData组合成一个RangeData再重力对齐和匹配
  scans_per_accumulation = 1,
  -- 过滤后再匹配
  voxel_filter_size = 0.025,

  -- 用于[1]的过滤器
  adaptive_voxel_filter = {
    max_length = 0.5,
    min_num_points = 200,
    max_range = 50.,
  },

  -- 用于[2]的过滤器
  loop_closure_adaptive_voxel_filter = {
    max_length = 0.9,
    min_num_points = 100,
    max_range = 50.,
  },

  use_online_correlative_scan_matching = false,
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.1,
    angular_search_window = math.rad(20.),
    translation_delta_cost_weight = 1e-1,
    rotation_delta_cost_weight = 1e-1,
  },

  ceres_scan_matcher = {
    occupied_space_weight = 1.,
    translation_weight = 10.,
    rotation_weight = 40.,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 20,
      num_threads = 1,
    },
  },

  -- 动作过滤器,若连续两次的位姿变换高于给定值,则[3]后面的步骤不再做了
  motion_filter = {
    max_time_seconds = 5.,
    max_distance_meters = 0.2,
    max_angle_radians = math.rad(1.),
  },

  -- 注意,这里不是重力加速度,这个值没啥用,大于10就行了
  imu_gravity_time_constant = 10.,

  submaps = {
    -- 子图分辨率,单位m,主要结构为概率格网,概率表示了该格网处为实体的概率
    resolution = 0.05,
    -- 构成每张子图的RangeData数量
    num_range_data = 90,
    range_data_inserter = {
      -- 是否处理空区域和miss区域,若选择false则子图中只有少量hit点的信息
      insert_free_space = true,
      -- hit概率,p_hit越大(大于0.5),则p_new=odds_inv(odds(p_hit)*odds(p_old))的速度越快
      hit_probability = 0.55,
      -- hit概率,p_miss越小(小于0.5),则p_new=odds_inv(odds(p_miss)*odds(p_old))的速度越快
      miss_probability = 0.49,
    },
  },
}
