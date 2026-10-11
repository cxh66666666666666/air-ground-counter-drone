# AGC 空地协同（Anti-Drone AGC）

空地协同反无人机仿真项目的领域语言。本文只收录本项目特有的概念，不含通用编程术语。定义用中文表述，术语键保留原始形式。

## 语言

**AGC**:
空地协同（Air-Ground Collaboration）——在同一世界、同一时钟下，无人机与地面车围绕统一任务定义构成的协同系统。
_Avoid_: 空地系统、飞机地面

**选择性吸收（Selective Absorption）**:
ROS2 重构中对待外部既有能力的既定方式：优先复用/吸收成熟方法（如 Nav2）的架构范式与实现经验来优化本仓库原代码，而非整包照搬或另起炉灶。
_Avoid_: 替代、迁移、照搬

**模式全覆盖·进程按域选用（Pattern-Complete, Process-Scoped）**:
把"Nav2 覆盖所有部分"落成的可执行表述——地面域真跑 Nav2 本体；空中域不引入 Nav2 运行时进程，但全面采纳其架构范式（costmap 分层语义、lifecycle 节点、planner/controller server 抽象、行为树）。
_Avoid_: Nav2 全覆盖（含义不清）

**契约真源（Contract Source of Truth）**:
不可变约束的唯一归属地——《交接文档》，承载坐标系、唯一执行口、硬约束、终态语义、接口语义。其它文档只能引用、不得复述。
_Avoid_: 合同、基线文档

**唯一执行口（Sole Execution Port）**:
运动中唯一可**命令车体运动**的通道，即 `/agc/ground/cmd_vel`；任何其它节点均不得命令车体运动。车体位姿由 GZ Sim 物理引擎产生，复位写入为唯一例外。语义与例外以契约真源为准。
_Avoid_: 执行入口（与"Runbook"混淆）

**安全层（Safety Layer）**:
地面域中位于控制输出与唯一执行口之间的兜底屏障，按配置区域对速度强制缩放或急停；默认由 Nav2 `collision_monitor` 实现，空中域由自研避障/滤波器承担。
_Avoid_: 保护层、shield（指标口径另见契约真源）

**无人机跟踪（UAV Standoff Tracking）**:
己方无人机对入侵目标的**常开**跟踪：保持**安全间距**（不紧贴），目的是**不丢失目标**并持续提供观测，同时负责自身飞行安全。与"编队跟随"不同。
_Avoid_: 跟随、伴飞、编队

**协同感知（Collaborative Perception）**:
无人机提供空中观测（全局视野），地面平台据此**自主规划**自身路径；二者不共享单一规划器。
_Avoid_: 跟随、编队、主从

**可控目标（Controllable Target）**:
入侵目标（`intruder_uav`）在仿真中**可被脚本化驱动**的能力——外部经速度指令改变其运动，用于 W6 的脚本化轨迹与干扰注入；与"被动刚体"相对。
_Avoid_: 主动目标、自主目标

**拦截载荷（Interception Payload）**:
地面平台（`agc_car`）搭载的拦截附件总称，含**捕获网面**与**干扰载荷**两类（ADR-0015）。
_Avoid_: 武器、装备

**捕获网面（Capture Net）**:
车载直径 2 m 的圆面，是 W6 **物理捕获"穿越判据"**的几何载体；车体模型层只提供几何与位姿。
_Avoid_: 网兜、拦截网

**干扰载荷（Jammer Payload）**:
车载**行为级电子干扰**发射器：以对目标行为的干预体现（EMI 不建模）；行为与接口属 W6。
_Avoid_: 电磁武器、EMI 设备

**Runbook（步骤文档）**:
唯一指导 agent 与人工执行重构流程的入口文档，即 `docs/ROS2迁移步骤.md`；只编排"现在做什么、如何验收"，不承载约束。
_Avoid_: 执行入口、执行口

**workstream**:
本项目的独立可验收工作单元，每个 workstream 对应 `.trae/specs/<ws>/` 下的 spec/tasks/checklist 三件套。
_Avoid_: 阶段、phase、P0–P6

**只读旧树边界（Read-Only Archive Boundary）**:
对归档旧树的读取纪律：只可读以对齐**行为 / 契约事实**，禁止作为格式、命名、目录、任务粒度的来源，禁止复制其代码。
_Avoid_: 参考旧项目、沿用旧实现
