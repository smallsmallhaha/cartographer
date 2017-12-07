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

SPARSE_POSE_GRAPH = {
  -- 每optimize_every_n_scans个RangeData进行一次优化计算
  optimize_every_n_scans = 90,
  -- 约束构造器
  constraint_builder = {
    -- 添加约束的采样率,越高越好,计算量也越大
    sampling_ratio = 0.3,
    -- 约束初始位姿的平移量上限
    max_constraint_distance = 15.,
    -- 计算约束步骤中的FCMS分数下限,越过则丢弃约束
    min_score = 0.55,
    -- 全局匹配(只是不同trajectory间的匹配)的最低分数
    global_localization_min_score = 0.6,
    -- 闭环优化的平移量权重(实际上是计算得到的约束的权重,用于闭环优化)
    loop_closure_translation_weight = 1.1e4,
    -- 闭环优化的旋转量权重(实际上是计算得到的约束的权重,用于闭环优化)
    loop_closure_rotation_weight = 1e5,
    -- 是否输出匹配成功的数据
    -- 格式: "Node (0, 121) with 101 points on submap (0, 1) differs by...with score 60.1%."
    log_matches = true,
    fast_correlative_scan_matcher = {
      linear_search_window = 7.,
      angular_search_window = math.rad(30.),
      branch_and_bound_depth = 7,
    },
    ceres_scan_matcher = {
      occupied_space_weight = 20.,
      translation_weight = 10.,
      rotation_weight = 1.,
      ceres_solver_options = {
        use_nonmonotonic_steps = true,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
    fast_correlative_scan_matcher_3d = {
      branch_and_bound_depth = 8,
      full_resolution_depth = 3,
      min_rotational_score = 0.77,
      min_low_resolution_score = 0.55,
      linear_xy_search_window = 5.,
      linear_z_search_window = 1.,
      angular_search_window = math.rad(15.),
    },
    ceres_scan_matcher_3d = {
      occupied_space_weight_0 = 5.,
      occupied_space_weight_1 = 30.,
      translation_weight = 10.,
      rotation_weight = 1.,
      only_optimize_yaw = false,
      ceres_solver_options = {
        use_nonmonotonic_steps = false,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
  },
  matcher_translation_weight = 5e2,
  matcher_rotation_weight = 1.6e3,
  optimization_problem = {
    huber_scale = 1e1,
    acceleration_weight = 1e3,
    rotation_weight = 3e5,
    consecutive_scan_translation_penalty_factor = 1e5,
    consecutive_scan_rotation_penalty_factor = 1e5,
    fixed_frame_pose_translation_weight = 1e1,
    fixed_frame_pose_rotation_weight = 1e2,
    log_solver_summary = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 50,
      num_threads = 7,
    },
  },
  max_num_final_iterations = 200,
  global_sampling_ratio = 0.003,
  log_residual_histograms = true,
  global_constraint_search_after_n_seconds = 10.,
}
