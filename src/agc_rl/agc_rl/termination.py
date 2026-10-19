"""episode 结束判定**单一来源**（W7-a · spec R1）。

本模块是 W7 的 episode 终止 / 预算语义**唯一真源**：**训练 adapter 与评估 adapter
（`scripts/episode_runner.py`）共用本模块**，不得各自另立判定（spec R1 判据：
"同一 episode 的评估摘要与训练 adapter 的 end / reason 一致"）。

映射（对齐 Gymnasium v0.26+ 语义，见 spec R1）：
  * **任务终态**（任意 `TERMINALS`）→ MDP **terminated**（含超时失败，契约 C4 超时一律失败）；
  * **外部截断**（墙钟看门狗耗尽 / 训练 horizon / 无终态）→ **truncated**（非任务终态，须 bootstrap）；
  * `INFRA_FAILURE` → 单列 `infra`，**不**吞入成功率（[ADR-0046](../../docs/adr/0046-runner-three-layer-acceptance-and-integrity-gate.md)）。

预算口径**复用** [ADR-0039](../../docs/adr/0039-time-budget-kinds-and-rtf-record.md) 三类
（`mission_sim_budget` / `episode_wall_watchdog` / `process_wall_duration`），**不产生第三套**。

约束类信息（超时失败 / infra 单列 / 缺失掩码）真源为契约
[#hard-constraints](../../docs/ROS2迁移交接文档.md#hard-constraints) 与
[#terminal-semantics](../../docs/ROS2迁移交接文档.md#terminal-semantics)；本模块只**引用**不复述。
"""

from agc_msgs.msg import AgcTaskState

# 任务终态集合（`/agc/task/state` 观测；task_manager 锁存，ADR-0037）。
TERMINALS = {
    AgcTaskState.TRACKING_SUCCESS, AgcTaskState.INTERCEPT_CONFIRMED,
    AgcTaskState.MISSION_FAILED, AgcTaskState.ABORTED, AgcTaskState.INFRA_FAILURE,
}
TERMINAL_NAMES = {
    AgcTaskState.TRACKING_SUCCESS: 'TRACKING_SUCCESS',
    AgcTaskState.INTERCEPT_CONFIRMED: 'INTERCEPT_CONFIRMED',
    AgcTaskState.MISSION_FAILED: 'MISSION_FAILED',
    AgcTaskState.ABORTED: 'ABORTED',
    AgcTaskState.INFRA_FAILURE: 'INFRA_FAILURE',
}

SUCCESS_TERMINALS = {'TRACKING_SUCCESS', 'INTERCEPT_CONFIRMED'}


# 三类时间预算口径标签（互不冒充）。clock 标注时钟来源；owner 标注归属节点 / 进程。
# 复用 ADR-0039，不新增第三套口径。
TIME_BUDGETS = {
    'mission_sim_budget': {
        'clock': 'ros_sim', 'owner': 'task_manager', 'param': 't_max_s',
        'note': 'C4 任务超时（仿真时钟 now-start_time）；由 task_manager 拥有，非 runner 墙钟',
    },
    'episode_wall_watchdog': {
        'clock': 'wall_monotonic', 'owner': 'episode_runner', 'param': 'wall_watchdog_s',
        'note': '外部看门狗：runner 墙钟等待 episode 结束的上限；超时判 infra、不算成功',
    },
    'process_wall_duration': {
        'clock': 'wall_steady', 'owner': 'run_logger', 'param': 'duration_s',
        'note': '进程时长（墙钟，含预热 / 关闭）；非 episode 预算',
    },
}


def episode_outcome_match(terminal_state, reason_code, expected_terminal, expected_reason):
    return (terminal_state == expected_terminal) and (not expected_reason or reason_code == expected_reason)


def classify_episode_outcome(terminal_state, reason_code, expected_terminal, expected_reason,
                             wall_elapsed_s, wall_watchdog_s):
    """episode 结束判定的**唯一来源**（eval adapter；train adapter 必须复用同一语义）。

    区分三类口径：mission_sim_budget（仿真时钟终态）/ episode_wall_watchdog（墙钟超时，
    判 infra、**不算成功**）/ 进程时长（logger，仅记录、不参与判定）。

    返回字段含 `is_task_terminal`（真 = MDP terminated）与 `is_truncation`（真 = 外部截断，
    非任务终态）——供 Gymnasium 适配层直接映射 `terminated / truncated`。
    """
    match = episode_outcome_match(terminal_state, reason_code, expected_terminal, expected_reason)
    if terminal_state == 'INFRA_FAILURE':
        return {'outcome_class': 'infra_failure', 'classification': 'infra', 'budget_kind': None,
                'counted_as_success': False, 'expected_match': bool(match),
                'is_task_terminal': True, 'is_truncation': False,
                'reason': reason_code or 'infra_failure'}
    if terminal_state in SUCCESS_TERMINALS:
        return {'outcome_class': 'mission_terminal', 'classification': 'mission',
                'budget_kind': 'mission_sim_budget', 'counted_as_success': bool(match),
                'expected_match': bool(match), 'is_task_terminal': True, 'is_truncation': False,
                'reason': reason_code}
    if terminal_state in ('MISSION_FAILED', 'ABORTED'):
        # 任务终态（**含超时失败**，契约 C4）→ MDP terminated，不 bootstrap。
        return {'outcome_class': 'mission_terminal', 'classification': 'mission',
                'budget_kind': 'mission_sim_budget', 'counted_as_success': False,
                'expected_match': bool(match), 'is_task_terminal': True, 'is_truncation': False,
                'reason': reason_code}
    # 未达终态：等待被墙钟看门狗停止（EPISODE_WALL_WATCHDOG 超时）→ 外部截断。
    return {'outcome_class': 'wall_watchdog_timeout', 'classification': 'infra',
            'budget_kind': 'episode_wall_watchdog', 'counted_as_success': False,
            'expected_match': False, 'is_task_terminal': False, 'is_truncation': True,
            'reason': 'episode_wall_watchdog exhausted'}


def classify_acceptance_layers(result):
    """三层验收判定——**三层全必过**方可 `pass`；`expected_match` 仅属命题层。

    ① 启动 / 运行完整性：`started` + 子进程退出确认 + `summary` 存在 + 同 run 关联。
    ② 终态命题：`expected_terminal` / `expected_reason`。
    ③ 指标有效性：C9 两指标为**有效数值**，**不**以 0 冒充。
    """
    integrity = bool(
        result.get('started')
        and result.get('proc_exit_confirmed')
        and result.get('summary_present')
        and result.get('same_run_verified')
        and result.get('state_run_verified'))
    proposition = bool(result.get('expected_match'))
    metrics_ok = (isinstance(result.get('shield_infeasible_rate'), (int, float))
                  and isinstance(result.get('reset_failure_rate'), (int, float)))
    return {
        'layer_integrity': integrity,
        'layer_proposition': proposition,
        'layer_metrics': metrics_ok,
        'pass': bool(integrity and proposition and metrics_ok),
    }


def summarize_metrics(episodes):
    """C9 指标汇总——**不**剔除缺 summary 的回合；如实报缺失计数 / 比例；空集合 → not_applicable。

    均值**仅基于有效样本**；缺失 / `not_applicable` 回合**不**以 0 冒充有效。
    """
    def _one(key):
        vals, missing = [], 0
        for e in episodes:
            v = e.get(key)
            if isinstance(v, (int, float)):
                vals.append(float(v))
            else:
                missing += 1
        return {
            'value': (sum(vals) / len(vals)) if vals else 'not_applicable',
            'valid_count': len(vals),
            'missing_count': missing,
            'missing_ratio': (missing / len(episodes)) if episodes else 0.0,
        }
    return {
        'shield_infeasible_rate': _one('shield_infeasible_rate'),
        'reset_failure_rate': _one('reset_failure_rate'),
    }


def compute_rtf(sim_start_s, sim_end_s, wall_start_s, wall_end_s):
    """RTF = 仿真时间增量 / 墙钟增量；仿真时钟缺席或墙钟增量 <= 0 → None。"""
    if sim_start_s is None or sim_end_s is None:
        return None
    wall = wall_end_s - wall_start_s
    if wall <= 0.0:
        return None
    return (sim_end_s - sim_start_s) / wall


def build_time_evidence(wall_start_s, wall_end_s, sim_start_s, sim_end_s):
    """同一 run 的起止事件 + RTF 证据（不同 RTF 的成功率 / 耗时可比）。"""
    sim_elapsed = None if (sim_start_s is None or sim_end_s is None) else (sim_end_s - sim_start_s)
    return {
        'wall_start_s': wall_start_s, 'wall_end_s': wall_end_s,
        'wall_elapsed_s': wall_end_s - wall_start_s,
        'sim_start_s': sim_start_s, 'sim_end_s': sim_end_s, 'sim_elapsed_s': sim_elapsed,
        'rtf': compute_rtf(sim_start_s, sim_end_s, wall_start_s, wall_end_s),
    }
