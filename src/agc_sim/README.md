# agc_sim（W2 · 仿真世界与模型）

单一世界 + 车/目标模型 + `ros_gz` 桥。分两段交付：**W2-min**（供 M0）与 **W2-full**（传感器链路，待补）。

## 目录

```
agc_sim/
├── worlds/agc_world.sdf                    # 单一世界（地面/光照/静态障碍内联）
├── models/agc_car/agc_car.sdf.xacro        # 地面车（差速两轮 + 2 caster + DiffDrive）
├── models/intruder_uav/intruder_uav.sdf.xacro  # 目标被动模型（失重、无插件）
├── config/{car,target}.yaml                # 模型与 spawn 参数（唯一来源）
├── config/bridge.yaml                      # ros_gz_bridge 声明式配置
└── launch/world_bringup.launch.py          # 世界 + 桥 + spawn 车/目标
```

## 运行

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select agc_sim
source install/setup.bash
ros2 launch agc_sim world_bringup.launch.py            # GUI
ros2 launch agc_sim world_bringup.launch.py gz_args:='-s -r -v 3 <world>'   # 无头
```

## 桥类型映射与 QoS 标定（Nav2 吸收点：sensor QoS / 类型映射）

| ROS 话题 | GZ 话题 | ROS 类型 | GZ 类型 | 方向 | QoS |
|---|---|---|---|---|---|
| `/clock` | `/clock` | `rosgraph_msgs/msg/Clock` | `gz.msgs.Clock` | GZ→ROS | 默认（reliable） |
| `/agc/sim/ground_odom_raw` | `/model/agc_car/odometry` | `nav_msgs/msg/Odometry` | `gz.msgs.Odometry` | GZ→ROS | 默认 |
| `/model/agc_car/cmd_vel` | `/model/agc_car/cmd_vel` | `geometry_msgs/msg/Twist` | `gz.msgs.Twist` | ROS→GZ | 默认（reliable） |

**服务**（CLI 形式桥接，见 `config/bridge.yaml` 注释）：

| ROS 服务 | GZ 服务 | ROS 类型 | 方向 |
|---|---|---|---|
| `/world/agc_world/set_pose` | `/world/agc_world/set_pose` | `ros_gz_interfaces/srv/SetEntityPose` | ROS→GZ |

> W2-min 各话题暂用桥默认 QoS；**sensor 类话题（点云等）的显式 QoS 标定随 W2-full 落地**（对齐 Nav2 sensor QoS）。
> 世界真值位姿旁路（gt）不在 W2-min；gz `dynamic_pose/info` 的 Pose_V→TF 在本机桥版本不写 frame 名，真值旁路属 W4（专用适配节点）。

## 传感器链路与 costmap 输入端（W2-full）

**传感器定义**：`agc_car` 挂 `gpu_lidar`（Velodyne 语义，16 线 × 360°/10 Hz，量程 0.2–20 m），
link `car/lidar_link`，`<topic>lidar/points</topic>`。（旧树激光在己方无人机 `iris_3d_lidar` 上；
本次按"costmap 消费方在地面/W5"将激光先行挂车体，己方无人机激光随 W3。）

**costmap 输入端数据契约（供 W4 / W5 消费）**：

| 数据 | 类型 | frame | 用途 |
|---|---|---|---|
| 3D 点云 | `sensor_msgs/PointCloud2`（ROS 侧 `/agc/sim/car_points`） | `world` | 障碍层 / voxel 层 / collision_monitor 输入 |
| 占据栅格 | `nav_msgs/OccupancyGrid` | `world` | static / inflation 层（由 W4 生成） |

**sensor QoS 标定**：点云与 sensor 类话题使用 `SENSOR_DATA`（best-effort、KeepLast）profile；
TF/clock 等用默认 reliable。桥条目见 `config/bridge.yaml`。

**已知限制（本环境，须在 W4 前解决）**：GZ Sim 的 `Sensors` 系统依赖 ogre2 渲染上下文；
本机在 world 内声明 `gz-sim-sensors-system(ogre2)` 后 **server 阻塞、world 加载不完**
（`/world/agc_world/create` 一直不可用），且默认 `server.config` 又**不含** Sensors 系统，
故：
- **默认不启用**传感器系统，`gz sim`（含 GUI）与 M0 不受影响；
- 点云的实际生成待渲染环境解决（候选：`GZ_RENDERING_PLUGIN=ogre2`、为 Sensors 配置 EGL/离屏渲染、
  或改用软件渲染），届时按实际 gz 话题名核对 `bridge.yaml` 的点云条目。
- `world` 内传感器链路的**模型定义（`car/lidar_link` 的 `gpu_lidar`）与桥条目已就位**，仅缺渲染上下文。

## 物理与驱动

- 物理引擎 `dart`（GZ Sim 8/Harmonic 不支持 ODE）；`max_step_size=0.004`、`real_time_update_rate=250`，对齐旧树步长行为。
- 地面车为**真实物理**驱动（`gz-sim-diff-drive-system`，见 `docs/adr/0010-ground-drive-real-physics.md`）；桥仅传命令，位姿由引擎产生。
- `set_pose` 为唯一允许写模型位姿的通道（供场景复位；须运行期门控）。

## 只读旧树边界

世界布局 / spawn 位 / 碰撞盒口径取自旧树对齐**行为事实**（WHAT）；实现为 GZ Sim 全新重写（HOW 不继承），未复制旧代码或资产。