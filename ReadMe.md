
# 启动键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard
 
# 查看launch参数
ros2 launch --show-args nav2_bringup bringup_launch.py
ros2 launch --show-args nav2_bringup navigation_launch.py
ros2 launch --show-args slam_toolbox online_async_launch.py
ros2 launch --show-args slam_toolbox localization_launch.py

# GitHub地址
https://github.com/ros-navigation/navigation2
https://github.com/SteveMacenski/slam_toolbox
https://github.com/introlab/rtabmap_ros

# 保存地图
ros2 run nav2_map_server map_saver_cli -t map -f '/root/agrobot_ws/src/agrobot_car_mapping/mapping_dir/map'
 
# 查看帮助
ros2 run tf2_ros static_transform_publisher --help

# 输出TF树
ros2 run tf2_tools view_frames

# 雷神激光雷达
colcon build
source install/setup.sh
ros2 launch lslidar_driver lsn10p_launch.py
 
# 建图  
colcon build --cmake-clean-cache
colcon build --packages-select agrobot_car_mapping agrobot_car_bringup lslidar_driver
source install/setup.sh
clear
ros2 launch agrobot_car_bringup mapping_launch.py
 
# 导航
colcon build --cmake-clean-cache
colcon build --packages-select agrobot_car_navigation agrobot_car_bringup
source install/setup.sh
clear
ros2 launch agrobot_car_bringup navigation_launch.py

# 定位
colcon build --packages-select agrobot_car_localization agrobot_car_bringup
source install/setup.sh
clear 
ros2 launch agrobot_car_bringup localization_launch.py

# 清除
rm -rf build install log

source install/setup.sh
clear
ros2 launch agrobot_car_bringup static_tf_launch.py


# 初始化位姿
ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "header:
  frame_id: 'map'
pose:
  pose:
    position:
      x: 7.631
      y: -5.072
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.7069657693523876
      w: 0.707247764905614
" --once


colcon build --packages-select agrobot_car_navigation
source install/setup.sh
clear
ros2 launch agrobot_car_navigation nav2_launch.py

colcon build
source install/setup.sh
clear
ros2 launch agrobot_car_mapping slam_toolbox_mapping_launch.py

 
apt autoremove