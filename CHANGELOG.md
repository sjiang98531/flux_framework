# Changelog

## [Unreleased] - 2026-11-18

### T1: HAL 三相电流重构饱和与辨识测试矢量峰值限幅
- **`FW/Src/hal_sc32r602.c`**：`acquireSamples()` 中 `gMeas.iabc.s16C` 改为 `int32_t` 求和后经 `QMath_Sat16()` 饱和，防止大电流/过流或零偏叠加时 `int16_t` 回绕翻号。
- **`FW/Inc/identify.h` / `FW/Inc/cfg_check.h` / `FW/Inc/cfg_check_list.h`**：`IDENT_I_SCALE_MAX` 保留 `SPD_PI_LIMIT / 2.5`，`I_MAX` 约束项改按 `seg 15` 合成矢量峰值 `IDENT_VEC_PEAK_K = 2.795085`（`sqrt(2.5^2 + 1.25^2)`）封顶至 `0.80 * I_MAX`；新增编译期/上电检查 `IDENT_VEC_PEAK_FITS`。
- **`FW/Src/identify.c`**：`acSegStart()` 中 `s_acSegB[seg] * biasX` 改用 `int64_t` 计算，并按 `amp^2 + bias^2 <= (0.85 * 32768)^2` 对幅值与偏置封顶，消除大 `biasX` 回绕风险（`biasX = 1` 时保持逐位一致）。

### T2: 交流阻抗质量判定前移、Param_Offer 门控与 RL 失败立即终止
- **`FW/Inc/identify.h`**：新增诊断告警位 `IDENT_WARN_LOW_SNR`（`1u << 5`）与 `IDENT_WARN_AC_FAIL`（`1u << 6`），在 `IdentResult_TypeDef` 中新增 `snrMin`（实测感抗压降相对死区电压最小比值）与 `acDlyTicks`（实测总延迟 `CTRL_INVPARK_ADVANCE - bestSh`，无论是否生效修正均记录，供台架标定）。
- **`FW/Src/identify.c`**：
  - `acSegFinish()` 在 `d <= 0` 时清零该段结果并置 `IDENT_WARN_AC_FAIL`；对小信号基频段（`seg 0` 与 `AC_S_Q04`）计算 `snr = |Im(Z)| * iAmp / deadVoltV`，更新 `gIdent.snrMin` 并在 `< 1.0` 时置 `IDENT_WARN_LOW_SNR`。
  - `Ident_Slow()` 在 `IDENT_PH_SAT_FIT` 阶段先计算 `trustedPre`（正值检查、`rlR/rlL > 0`、无 `AC_DRIFT | LOW_SNR | AC_FAIL`），再调用 `offerStill(trustedPre)`；不可信时仅向 `Param_Offer()` 提交 `Rs` 与死区（`mask 1|32`），不提交 `L / L(i)`（`mask 2|4`），同时置 `IDENT_WARN_AUTO_BLOCKED` 并阻止 `IDENT_MODE_AUTO` 自动重起动。
  - `Ident_Slow()` 在 `IDENT_PH_RL` 阶段若 `rlSlow()` 放弃（`rlR <= 0 || rlL <= 0`），立即置 `RL_TIMEOUT | AUTO_BLOCKED` 并终止于 `IDENT_PH_DONE`，不再进入后续大电流阶段。

### T3: 0.5·f0 等幅重测与 `SWING_CLAMP` 语义收敛
- **`FW/Src/identify.c`**：
  - 新增 `s_acGain[AC_NSEG]` 与 `s_acRetry[AC_NSEG]`；在 `acSegFinish()` 写入任何副作用前，对 `AC_S_Q04L` / `AC_S_Q10L` 若实测电流与对应 `f0` 段偏差 `> 5%` 且重测不足 2 次，按 `ref / iAmp`（限 `0.3 ~ 3.0`）缩放 `s_acGain` 并返回 `1`，由 `Ident_Slow()` 重测当前段以对齐实际电流幅值。
  - `acFit()` 中当理论摆动电感占比 `10 * km / (w25^2 * L0) < 1%` 时直接置 `k04 = k10 = 0` 且不告警；仅在重测后电流仍差 `> 15%` 回退或触发 `4 * km` 硬夹限时置 `IDENT_WARN_SWING_CLAMP`。
  - 同步更新 `identify.h` / `identify.c` 头部流程说明。

### T4: 加强版主机仿真验证与测试套件（测试脚本侧）
- 补充光滑饱和 `L0 * (0.5 + 0.5 / (1 + (i/I0)^2))`、高频涡流支路、d 轴二次交叉饱和及独立 IPM 电机模型。
- 新增 `[0, I_sweep_max]` 与 `[0, I_MAX]` 双口径误差评估、30 种子噪声及偏置/增益组合统计（`p50 / p95 / max`）、`ψf / J` 错填工况，以及“静默失败（误差 > 15% 且未阻断）”自动断言。
