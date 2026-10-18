# tests/ 验收核查（acceptance checks）

> 本目录是**验收 / 回归核查**：每个脚本针对一个风险项 / 能力项**断言行为**，**退出码 0 = PASS、非 0 = FAIL**。
> 多为只读核查或自包含 harness；部分需拉起完整链路 / SITL，故**不接 CI**，手动按需运行。

从 `scripts/` 迁入（2026-10-18 curation）：**只改路径、不改名**。历史文档（ADR / checklist / 评审）中出现的 `scripts/<name>_check.py` 即本目录同名文件。

## 运行

```bash
python3 tests/run_all.py                       # 列出全部核查
python3 tests/run_all.py --run w6_r034_check.py
python3 tests/run_all.py --all --timeout 600   # 全跑（带超时）
```

## 命名

沿用原风险项编号：`w<ws>_r<nnn>_check.py`、`w<ws>_<name>_check.py`。
描述性改名留待**单独一轮**（避免与历史文档引用一次性大改；ADR 史料不重写）。
