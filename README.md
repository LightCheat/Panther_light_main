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

## 阻尼最小二乘 IK 测试

求解器：`src/kinematics_application/src/ik_solver_dls.cpp`；测试程序：同目录的 `ik_dls_demo.cpp`。

每个新终端先进入工程根目录，并加载上述 ROS 2 和工作空间环境。

仅验证 IK，不发送电机命令：

```bash
ros2 run kinematics_application ik_dls_demo --ros-args -p dry_run:=true
```

实机测试时，先在一个终端运行上述 MoveIt 启动命令，再在另一个终端切换控制器并运行 demo：

```bash
ros2 control switch_controllers \
  --deactivate panther_arm_controller \
  --activate panther_position_controller \
  --strict

ros2 run kinematics_application ik_dls_demo
```

**demo 以六个关节全零时的位姿为基准，使末端 `link6` 沿世界 Z 轴上移 5 cm，保持姿态不变。** 它直接发送目标关节位置，不以当前位置为起点，也不做轨迹插值。

按 `Ctrl+C` 停止 demo 后，切回 MoveIt 控制器：

```bash
ros2 control switch_controllers \
  --deactivate panther_position_controller \
  --activate panther_arm_controller \
  --strict
```
