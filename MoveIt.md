agrobot_arm_controller

moveit_fk_demo
colcon build --cmake-clean-cache

# 启动MoveIt Setup Assistant，利用它配置和生成MoveIt的配置文件
colcon build
source install/setup.sh
ros2 run moveit_setup_assistant moveit_setup_assistant

# 启动MoveIt Setup Assistant生成的demo
colcon build 
source install/setup.sh
ros2 launch agrobot_arm_config demo.launch.py

ros2 launch agrobot_arm_config demo.launch.py


# MoveIt自带示例
source install/setup.sh
ros2 launch panda_moveit_config demo.launch.py


# 启动rviz - 机械臂
colcon build
source install/setup.sh
ros2 launch agrobot_arm_description agrobot_arm_launch.py


# 新建文件夹
rm -rf /ros2_projects/robot_ws/src/agrobot_arm_config
mkdir src/agrobot_arm_config

colcon mixin add default  https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml

source ~/ws_moveit2/install/setup.bash

sudo apt install ros-humble-joint-state-controller
