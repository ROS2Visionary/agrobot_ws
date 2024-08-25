
# bringup 建图  
colcon build
source install/setup.sh
clear
ros2 launch agrobot_car_bringup mapping_launch.py


# bringup 定位  
colcon build
source install/setup.sh
clear
ros2 launch agrobot_car_bringup localization_launch.py


# bringup 导航 slam  
colcon build
source install/setup.sh
clear
ros2 launch agrobot_car_bringup nav_slam_launch.py


# bringup 导航 slam  &  filter
colcon build
source install/setup.sh
clear
ros2 launch agrobot_car_bringup nav_slam_filter_launch.py


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


# 手动保存地图
ros2 run nav2_map_server map_saver_cli -t map -f '/root/agrobot_ws/src/agrobot_car_mapping/mapping_dir/map'
 
# 查看帮助
ros2 run tf2_ros static_transform_publisher --help

# 输出TF树
ros2 run tf2_tools view_frames

# 雷神激光雷达
colcon build
source install/setup.sh
ros2 launch lslidar_driver lsn10p_launch.py
 

