# 项目节点语言选型：C++ 实时域 / Python RL 域

## Status
accepted

## 背景与决策
W0 的《目录与命名规范》已隐含"节点包用 `ament_cmake`（C++）、RL / 工具用 `ament_python`"，
但该划分**沿袭自旧树语言分布，未经显式论证**。本 ADR 将其**显式化并钉死**：

- **C++（`ament_cmake`）**：性能 / 实时敏感域——W3 空中链路（高频 uORB）、W4 感知融合
  （高频点云 / 图像）、W5 地面安全层（硬约束 C6 的实时逃逸能力）。理由：低延迟、可控内存、
  无 GIL、可给实时保证。
- **Python（`ament_python`）**：RL 与工具脚本（W7）——依赖 PyTorch / Stable-Baselines 等生态。
- **语言不构成接口耦合**：跨域通信只经 `agc_msgs`（topic / service / action），与实现语言无关。

## Considered Options
- 全域 Python —— 被否：W4 点云处理与 W5 安全层的实时性 / 吞吐不足。
- 全域 C++ —— 被否：RL 生态（训练 / 评估流水线）在 Python，重写代价高。

## Consequences
- 各包语言按域固定（[包边界](../conventions/包边界.md)）。
- M0 已落地的 C++ 节点（`agc_perception` / `agc_ground` / `agc_task`）符合本决策，**无需返工**。
- 若某域后续需换语言，须先复核本 ADR 与其性能假设。