# Panther_light_main

基于 ROS 2 Humble、MoveIt 2 和 ros2_control 的六轴机械臂控制工程。

## 启动 MoveIt

在工程根目录执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch panther_moveit_config demo.launch.py
```

该命令会启动 RViz、MoveIt 和机械臂控制器。在 RViz 的 **MotionPlanning** 面板设置目标位姿，点击 **Plan** 规划，再点击 **Execute** 执行。

**当前配置使用真实硬件，启动时会使能电机。** 启动前请连接机械臂，确认串口 `/dev/ttyACM0` 可访问，波特率为 `921600`。

MoveIt 使用 `panther_arm_controller` 控制六个关节；速度控制器和直接位置控制器默认不激活。
