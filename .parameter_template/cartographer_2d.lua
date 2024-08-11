include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",  -- 地图坐标系的名称
  tracking_frame = "base_link",  -- 跟踪坐标系的名称，通常是机器人底盘
  published_frame = "base_link",  -- 发布的坐标系名称，通常是机器人底盘
  odom_frame = "odom",  -- 里程计坐标系的名称
  provide_odom_frame = true,  -- 是否提供里程计坐标系
  publish_frame_projected_to_2d = true,  -- 发布坐标系投影到2D平面
  use_odometry = true,  -- 是否使用里程计数据
  use_nav_sat = false,  -- 是否用卫星导航数据
  use_landmarks = false,  -- 是否使用地标
  num_laser_scans = 1,  -- 使用一个激光扫描仪
  num_multi_echo_laser_scans = 0,  -- 不使用多回波激光扫描仪
  num_subdivisions_per_laser_scan = 1,  -- 每次激光扫描的细分数量
  num_point_clouds = 0,  -- 不使用点云
  lookup_transform_timeout_sec = 0.2,  -- 查找坐标变换的超时时间（秒）
  submap_publish_period_sec = 0.5,  -- 子地图发布周期（秒），适应10Hz的激光雷达刷新率
  pose_publish_period_sec = 0.1,  -- 位姿发布周期（秒），适应10Hz的里程计刷新率
  trajectory_publish_period_sec = 0.5,  -- 轨迹发布周期（秒），适应10Hz的激光雷达刷新率
  rangefinder_sampling_ratio = 1.0,  -- 测距仪采样比例
  odometry_sampling_ratio = 1.0,  -- 里程计采样比例
  fixed_frame_pose_sampling_ratio = 1.0,  -- 固定坐标系位姿采样比例
  imu_sampling_ratio = 0.0,  -- IMU 采样比例
  landmarks_sampling_ratio = 1.0,  -- 地标采样比例
}

MAP_BUILDER.use_trajectory_builder_2d = true  -- 使用2D轨迹构建器

TRAJECTORY_BUILDER_2D.min_range = 0.2  -- 激光雷达的最小测量范围（米）
TRAJECTORY_BUILDER_2D.max_range = 10.0  -- 激光雷达的最大测量范围（米），兼顾计算量和质量
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.0  -- 缺失数据射线的长度（米）
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- 使用IMU数据
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- 使用在线相关扫描匹配
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.15  -- 运动过滤器的最大距离变化（米），适中
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.15)  -- 运动过滤器的最大角度变化（弧度），适中
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45  -- 每个子地图中的激光雷达数据数量，适应10Hz的刷新率
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05  -- 子地图分辨率，单位米

-- 优化扫描匹配和定位的参数
POSE_GRAPH.constraint_builder.min_score = 0.6  -- 约束生成器的最小得分，兼顾质量和计算量
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65  -- 全局定位的最小得分，兼顾质量和计算量
POSE_GRAPH.optimize_every_n_nodes = 60  -- 每60个节点进行一次全局优化
POSE_GRAPH.constraint_builder.sampling_ratio = 0.75  -- 约束生成器的采样比例，兼顾质量和计算量
POSE_GRAPH.constraint_builder.max_constraint_distance = 12.0  -- 最大约束距离，提高匹配的稳定性

return options
