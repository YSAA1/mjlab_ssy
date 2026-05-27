## Development Workflow

- Always use `uv run` instead of raw `python` when invoking project tooling.
- Before committing, run:
  - `make format`
  - `make type`
  - `make test` when the change is broad or user-facing
- Prefer targeted tests during iteration:
  - `uv run pytest tests/<file>.py`
  - `uv run pytest tests/`
- For lint/format/type checks:
  - `uv run ruff format`
  - `uv run ruff check --fix`
  - `uv run ty check`
  - `uv run pyright`

## Project Harness

- 当前恢复入口：`.harness/state.md`
- Harness 说明：`docs/agent/harness.md`
- G1 sim2sim / real-deploy 阶段研究已归档：
  `docs/research/archive/g1-sim2sim-2026-05-25.md`
- 长历史计划：
  `docs/plans/2026-05-22--g1-tracking-sim2real-hardening.md`

不要把临时任务状态、一次性结论或短期 TODO 写进 `AGENTS.md`。需要恢复
当前项目状态时先读 `.harness/state.md`；需要追溯旧 G1 研究证据时读归档
摘要，再用 `rg` 定点查询历史计划或日志路径。

## 避坑与思维重构铁律 (5-Attempt Hard Stop)

当对同一个问题连续尝试 **3~5次** 仍未成功时，必须执行“硬刹车”，严禁继续以类似的方法进行局部微调或反复打补丁。请遵循以下策略：

1. **破除沉没成本**：立刻停止修补当前方案。多次失败表明当前的设计方向、前置假设或根本逻辑存在底层偏差。
2. **重构思维模型**：重新审视问题本质，退后一步重新梳理链路，换个角度、框架或全新路径重新设计解决方案。
3. **善用外部灵感**：若没有新思路，应立即调用 **Web Search** 进行调研。你所面临的困难绝大多数并非首创，参考互联网上的成熟方案和避坑经验，能有效避免深陷思维死胡同。

## Project Map

- `pyproject.toml`: source of truth for console scripts, dependency groups, and `uv` behavior
- `src/mjlab/`: primary application code
- `src/mjlab/scripts/`: user-facing CLI entry points for training, play, export, motion processing, and debugging
- `src/mjlab/tasks/`: task registration and task-specific environment / RL configs
- `tests/`: regression and integration coverage; add focused tests for new task variants and CLI wrappers
- `docs/`: user and developer docs
- `data/`: local data assets used by docs, examples, and motion workflows
- `artifacts/`, `logs/`, `wandb/`: runtime outputs; do not commit generated contents

## Local Runtime Content

- Keep local tool state and generated dependency checkouts out of version control:
  - `.codex/`
  - `.omx/`
  - `.external/`
  - temporary stray files like `=0.6.1`
- Treat these as machine-local working directories, not source files.

## Commit Guidance

- Use concise commits with a clear “why” in the title.
- Preserve the baseline task behavior when adding specialized variants.
- For motion / tracking work, prefer adding new task IDs over mutating existing defaults.
- Do not commit code that fails `ruff` or relevant `pytest` coverage.
