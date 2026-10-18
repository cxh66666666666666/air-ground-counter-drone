"""agc_rl —— 地面 RL 最小版（W5-b）。

- `interface`：obs / action 空间冻结（唯一真源）。
- `policy`：策略加载（确定性策略，可加载 JSON 参数）。
- `inference_node`：推理节点（`ground_policy_rl`），只写 `cmd_vel_raw`。

训练产线整体留 W7（[ADR-0016](../../docs/adr/0016-execution-scope-exit-w6.md)）。
"""
