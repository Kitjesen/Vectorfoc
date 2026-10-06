# 历史记录

以下记录保留历史正文；当前入口见 [固件 README](../../README.md)。

- [CODE_AUDIT_REPORT.md](CODE_AUDIT_REPORT.md)、[CODE_REVIEW.md](CODE_REVIEW.md)：审计。
- [FEATURE_ARCH_V1_1.md](FEATURE_ARCH_V1_1.md)：旧版设计。
- [OPTIMIZATION_ROADMAP.md](OPTIMIZATION_ROADMAP.md)、[COMPREHENSIVE_OPTIMIZATION_PLAN.md](COMPREHENSIVE_OPTIMIZATION_PLAN.md)：优化建议。
- [CHANGELOG.md](CHANGELOG.md)、[CHANGELOG_2026-02-08.md](CHANGELOG_2026-02-08.md)：变更记录。
- [ALGORITHM_NOTES.md](ALGORITHM_NOTES.md)：原算法说明。

旧路径定位：`src/control/` 已并入 `src/foc/`；`control.c`、`inner.c`、`outer.c` 分别并入/改名为 `control_dispatcher.c`、`control_dispatcher.c`、`outer_control.c`。
原 `src/ui/` 的参数归 `config`，错误记录归 `safety`，LED 归 `app`，VOFA 归 `comm`。
原 `Lib/` 归 `platform/`，`MDK-ARM/` 归 `platform/keil/`，`test/` 归 `tests/`，`Src/ALGO` 中间目录已撤销。
