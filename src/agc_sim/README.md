# agc_sim（W2 · 仿真世界与模型）

单一世界 + 车/目标模型 + `ros_gz` 桥。分两段交付：**W2-min**（供 M0）与 **W2-full**（传感器链路，待补）。
> **W5 打补丁（平台换装，[ADR-0021](../../docs/adr/0021-ground-safety-thresholds-and-platform-recalibration.md)）**：地面车由 TB3 级小车换装为 **Velodyne 语义差速 UGV**，基座取 Gazebo Fuel 官方模型 **Clearpath Robotics Husky A200**（来源 / 许可见 [第三方资产与许可](../../docs/conventions/第三方资产与许可.md)；保形换装：仅换视觉 mesh + 碰撞/惯量并 1:1 移植尺度，驱动内核仍为 gz-sim `DiffDrive`）。不变量保持：`car/base_link`、`car/lidar_link`（`/agc/car_lidar`）、`car/capture_net`、`car/jammer`、桥入/出、`config/car.yaml` 唯一参数源。

## 目录

```
agc_sim/
├── worlds/agc_world.sdf                    # 单一世界（地面/光照/静态障碍内联）
├── models/agc_car/agc_car.sdf.xacro        # 地面平台（Husky 级 4 轮差速 UGV + DiffDrive；W5 换装）
├── models/agc_car/meshes/*.stl             # Fuel 官方网格（来源 / 许可见 docs/conventions/第三方资产与许可.md）
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
| `/model/agc_car/cmd_vel` | `/agc/sim/cmd_intent` | `geometry_msgs/msg/Twist` | `gz.msgs.Twist` | ROS→GZ | 默认（reliable） |

> R-009（[ADR-0025](../../docs/adr/0025-gz-side-terminal-failstop-deadman-plugin.md)）：ROS 侧话题名不变
> （`/model/agc_car/cmd_vel`），但 gz 侧改桥到**意图话题** `/agc/sim/cmd_intent`；GZ 侧守门人插件
> `agc_cmd_deadman` 订阅该意图（fresh→转发、stale→归零），再写 `DiffDrive` 默认订阅的 gz 话题
> `/model/agc_car/cmd_vel`（该话题的唯一写入者是插件）。

**服务**（CLI 形式桥接，见 `config/bridge.yaml` 注释）：

| ROS 服务 | GZ 服务 | ROS 类型 | 方向 |
|---|---|---|---|
| `/world/agc_world/set_pose` | `/world/agc_world/set_pose` | `ros_gz_interfaces/srv/SetEntityPose` | ROS→GZ |

> W2-min 各话题暂用桥默认 QoS；**sensor 类话题（点云等）的显式 QoS 标定随 W2-full 落地**（对齐 Nav2 sensor QoS）。
> 世界真值位姿旁路（gt）不在 W2-min；gz `dynamic_pose/info` 的 Pose_V→TF 在本机桥版本不写 frame 名，真值旁路属 W4（专用适配节点）。

## 传感器链路与 costmap 输入端（W2-full）

**传感器定义**：`agc_car` 挂 `gpu_lidar`（Velodyne 语义，16 线 × 360°/10 Hz，量程 0.2–20 m），
link `car/lidar_link`，`<topic>/agc/car_lidar</topic>`（点云发布于 `/agc/car_lidar/points`）。（旧树激光在己方无人机 `iris_3d_lidar` 上；
本次按"costmap 消费方在地面/W5"将激光先行挂车体，己方无人机激光随 W3。）

**costmap 输入端数据契约（供 W4 / W5 消费）**：

| 数据 | 类型 | frame | 用途 |
|---|---|---|---|
| 3D 点云 | `sensor_msgs/PointCloud2`（ROS 侧 `/agc/sim/car_points`） | `world` | 障碍层 / voxel 层 / collision_monitor 输入 |
| 占据栅格 | `nav_msgs/OccupancyGrid` | `world` | static / inflation 层（由 W4 生成） |

**sensor QoS 标定**：点云与 sensor 类话题使用 `SENSOR_DATA`（best-effort、KeepLast）profile；
TF/clock 等用默认 reliable。桥条目见 `config/bridge.yaml`。

**实现要点（已打通）**：world 内 **显式全量声明系统插件且按顺序**（`Physics → UserCommands →
SceneBroadcaster → Sensors`），与 Nav2 官方 `tb3_sandbox` 写法一致。默认 `server.config` 只含前三个；
**若仅额外声明 Sensors、其余靠默认，会导致插件加载顺序错位、server 在加载阶段挂死**（本机曾因此被
误判为"渲染不可用"，实为顺序问题）。sensor 的 `<topic>` 取绝对名 `/agc/car_lidar` → gz 点云话题
`/agc/car_lidar/points`（另有 `/agc/car_lidar` 的 `LaserScan`，未桥）。

**实测**：`/agc/sim/car_points`（`sensor_msgs/PointCloud2`）@ **~10 Hz**，GUI 与无头均正常；M0 不受影响。

## 拦截相关模型（W2-interception）

- **可控目标**（`intruder_uav`，R9）：模型改**双模 xacro**（`controlled` 参）——W6 前置 P 起由**第二 PX4 SITL 实例** attach 并驱动（`agc_task/intruder_driver` 经 uXRCE / `px4_msgs`）；桥 `/agc/sim/target_cmd_vel`（gz `VelocityControl` 通道）**已退役**（`.trae/specs/w6-task-eval/spec.md` 前置 P；ADR-0016）。
- **地面拦截载荷**（`agc_car`，R10）：`car/capture_net`（直径 2 m 圆面，法线 `+x`，供 W6 物理捕获"穿越判据"）与 `car/jammer`（行为级干扰发射器）。二者均为**轻量、无 collision** 附件，参数（`capture_net_radius` / `payload_mass`）落 `config/car.yaml` 并由 launch 单一设置；**捕获裁决与干扰行为属 W6**（ADR-0015）。

## 物理与驱动

- 物理引擎 `dart`（GZ Sim 8/Harmonic 不支持 ODE）；`max_step_size=0.004`、`real_time_update_rate=250`，对齐旧树步长行为。
- 地面车为**真实物理**驱动（`gz-sim-diff-drive-system`，见 `docs/adr/0010-ground-drive-real-physics.md`）；桥仅传命令，位姿由引擎产生。
- `set_pose` 为唯一允许写模型位姿的通道（供场景复位；须运行期门控）。

## 只读旧树边界

世界布局 / spawn 位 / 碰撞盒口径取自旧树对齐**行为事实**（WHAT）；实现为 GZ Sim 全新重写（HOW 不继承），未复制旧代码或资产。

## W3 打补丁（回退 W2）

为供 W3 空中链路挂入本世界，`agc_sim` 做最小补丁（[ADR-0019](../../docs/adr/0019-uav-trajectory-and-tracking-parameter-calibration.md)）：

- `agc_world.sdf` 补 PX4 必需系统插件（`Contact`/`Imu`/`AirPressure`/`ApplyLinkWrench`/`NavSat`）+ `spherical_coordinates`，插件顺序对齐 PX4 官方 `Tools/simulation/gz/worlds/default.sdf`。
- 作业区放大到 **80×80 m**：`agc_car` / `intruder_uav` spawn 分离 **~30 m**（`car.yaml` / `target.yaml`）；`pole_1`(0,-6) / `pole_2`(6,6) 重排入作业区（供 W3 自避障验证）。
- 地面仍为 200×200，未改；物理步长 / 更新率不变。