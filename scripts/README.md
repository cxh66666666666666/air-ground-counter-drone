# scripts/ 索引

> 本目录**只放长期工具**。一次性、按风险项（R-xxx）产出的驱动 / 台架 / 探针脚本已归档到 [archive/](./archive/)；验收 / 回归核查已迁到 [../tests/](../tests/)。
> 规则与家的划分见 [目录与命名规范](../docs/conventions/目录与命名规范.md)。

## 长期工具

| 文件 | 用途 |
|---|---|
| `episode_runner.py` | 批量跑 episode、三层验收判定、唯一 run 关联、完整性硬门 |
| `evidence_archive.py` | 证据不可变归档 + intake manifest（ADR-0064） |
| `dep_audit.py` | 包依赖审计 |
| `toolchain_pin_check.py` | 工具链 pin 清单核对（只读、退出码语义，ADR-0065） |
| `toolchain_root_check.py` | 工具链根注入 / preflight 校验（ADR-0067） |
| `third_party_asset_check.py` | 第三方资产 provenance / 许可 / 哈希核对（ADR-0068） |
| `lock_px4_failsafe_params.sh` | 注入 PX4 失效安全参数锁定（ADR-0034） |
| `w3_target_driver.py` | 目标驱动（直线 / 圆周）运行时驱动 |

## 新增脚本前先问一句

- 它是**长期工具**吗？→ 留 `scripts/`。
- 它是**一次性**（跑完即归档）吗？→ `scripts/archive/`。
- 它在**断言行为、有退出码语义**吗？→ `../tests/`。

不要往 `scripts/` 顶层平铺一次性脚本 —— 这正是本次 curation 要根治的问题。
