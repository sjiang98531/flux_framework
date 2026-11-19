# 参数辨识当前状态与验收归档（2026-11-19）

## 仓库与冻结基线

`https://github.com/sjiang98531/flux_framework.git`
功能代码冻结于 commit `eddf075`（`FW/Src/identify.c` 无功能改动）。

raw URL：
- `https://raw.githubusercontent.com/sjiang98531/flux_framework/eddf075/FW/Src/identify.c`
- `https://raw.githubusercontent.com/sjiang98531/flux_framework/eddf075/FW/Inc/identify.h`
- `https://raw.githubusercontent.com/sjiang98531/flux_framework/eddf075/tests/ident_eval/PROTOCOL_V2.md`
- 最终盲测报告：`tests/ident_eval/BLIND_REPORT_547376626.md`

## 已完成的关键改进（`b4caac6` → `eddf075`）

1. **R1–R4：饱和拟合与阻断门控闭环**：
   - `acFit()` 3 次谐波描述函数修正统一采用实测谐波比并限幅，消除低 PWM 频段深饱和系统性高估；
   - `acCalDelay()` 非整数拍延迟失准拦截（`IDENT_WARN_AC_DRIFT`）；
   - `acFit()` 扫幅不足 × 深饱和降级为仅 L0（`lRatio < 0.70 && iTop < 0.85*iSwpMax`）。
2. **R5：小电感强饱和 `vac` 专项校准**：
   - 交叉饱和偏置外推限幅（最多 20% 修正，防止噪声放大）；
   - `acCheckAndUpdateFF()` 重测容差收紧至 15%，消除 2-DOF 欠驱动幅度偏低。
3. **R6：电感反增物理护栏**：
   - `acFit()` 中 `lTop > 1.10f * gIdent.lqAtH[0]` 置 `IDENT_WARN_AC_DRIFT` 阻断。
4. **R7：同频往复摆动（$2 f_0$ 正交电压解调）监测**：
   - `acFast()` / `acSegFinish()` 在 `AC_S_Q10 .. AC_S_Q10+3` 段解调 $2 f_0$ 正交电压 `vqAc2`，当 `vqAc2 > 0.033f * vAmp` 时置 `s_acMotion = 1u` 阻断，捕捉整周期平均 `vqPP` 抵消的大幅往复摆动。

## 最终全新随机种子盲测验收（`eddf075`, `seed=547376626`）

| 验收项 | 实测结果 | 验收线 | 判定 |
|---|---|---|---|
| 健康误拦 FP | 3/300 (1.00%) | ≤ 3% | ✅ PASS |
| 压力子集（±2%FS, `seed=547376627`, N=30）静默失败 | 0 (`safe_abort=10, blk=9`) | == 0 | ✅ PASS |
| 独立坏电机集（`seed=704910903`）召回 | 26/26 (100.0%) | ≥ 90% | ✅ PASS |
| 冻结坏电机集（`seed=20261188`）召回 | 25/25 (100.0%) | ≥ 90% | ✅ PASS |
| 高危静默失败（`SwpL > 25%` 且未阻断） | **0** | == 0 | ✅ PASS |
| `SwpL` 扣除 V/I 增益后纯算法残差 (p50/p95/max) | 1.52% / **7.16%** / 14.25% | p95 ≤ 8% | ✅ PASS |
| `SwpL` 绝对值（未扣增益, p50/p95/max） | 4.15% / 10.05% / 20.46% | 参考 | - |
| `RunL` 运行期外推指标（单列, p50/p95/max） | 4.33% / 13.27% / 37.03% | 单列报告 | - |

## 编译状态

- Keil 双配置 0 Error（`IDENT_TEST=0` Code=32600；`IDENT_TEST=1` Code=55616）
- `IDENT_TEST=0` SHA256 = `53453137678B75D54C9303AECD32137E74E46D275C2895517367499A17A6721E`（全程不变）
- GCC `-Wall -Wextra` 0 Warning，`FW/` 纯 CRLF

## 下一步：实机台架验证

- 限流电源 + `IDENT_I_SCALE ≈ 0.3` + 已知风机 + LCR/毫欧表对照，首轮先用 `IDENT_MODE_STATIC` 实测板级延迟 `acDlyTicks` 并固化。
