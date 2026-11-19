# 参数辨识当前状态与待解决问题（2026-11-19）

## 仓库

`https://github.com/sjiang98531/flux_framework.git`
HEAD = `b4caac6`

raw URL：
- `https://raw.githubusercontent.com/sjiang98531/flux_framework/b4caac6/FW/Src/identify.c`
- `https://raw.githubusercontent.com/sjiang98531/flux_framework/b4caac6/FW/Src/startup.c`
- `https://raw.githubusercontent.com/sjiang98531/flux_framework/b4caac6/FW/Inc/identify.h`
- `https://raw.githubusercontent.com/sjiang98531/flux_framework/b4caac6/tests/ident_eval/PROTOCOL_V2.md`

## 背景

PMSM 驱动器参数自动辨识。换电机后上电自动测 Rs、Ld/Lq、L(i) 饱和曲线、死区、磁链、惯量。测准了用，测不了拦住。MCU SC32R602（Cortex-M3, Flash 128KB, RAM 8KB），PWM 16kHz/48kHz。`IDENT_TEST=1` 编译，`IDENT_TEST=0` 完全去除（二进制逐字节不变，SHA256 已验证）。

## 已完成的工作

1. **原理性重构**（从 Gemini 的 19 个补丁式提交开始）：
   - 无源对位阻尼（不依赖未标定 Rs）+ 颤振检测（幅值钳位 + Kd 自适应减半）
   - 2-DOF 交流注入（复阻抗前馈 + 保守带宽 PI，不放大电流指令）
   - 含 3 次谐波精确描述函数饱和拟合
   - 死区阶梯子窗静止监测 + 交流正交轴静止监测
   - IPD 后置（避免击飞低惯量转子）
   - lsMaxA 1.5 倍有界线性外推

2. **双向消融**：6 个组件逐一开关，拆除了零贡献的 R/L 升频复测

3. **冻结评估协议 v2.1**：运行期 Param_LqAt 模型，SwpL（辨识区间精度）为主指标，RunL（过载外推）单列

4. **Claude 审核后的修复**（当前 HEAD `b4caac6`）：
   - dtFit：零偏余量 + 分半斜率一致性 + n≥3 + 残差口径统一
   - acFast：过流判据改 id²+iq² + 编译器屏障
   - 阻尼检测器：大步长+2/小步长-1（不依赖反号奇偶）+ 减半下限修正 + 超时告警（不阻断）
   - 扫幅不足门控（初版，阈值待标定）

## 当前开发集验证结果

| 验收项 | 结果 | 验收线 | 判定 |
|---|---|---|---|
| 健康误拦 FP | 0-1 例（<1%） | ≤3% | ✅ |
| 压力子集（±2%FS）静默失败 | 0 | ==0 | ✅ |
| 坏电机门控召回 | 100% (25/25) | ≥90% | ✅ |
| 高危静默失败（SwpL>25% 未拦） | 3-5 例 | ==0 | ❌ |
| SwpL 残差 p95 | ~10% | ≤8% | ❌ |

## 待解决的核心问题

### 问题：低 PWM 频率 × 深饱和 → 高危静默失败

**现象**：vac48/vac16 类电机（16kHz PWM, 20-50μH, ksat=0.02-0.03, i0sat=5A）在盲测中出现 SwpL=23-43% 的静默失败（Rs 准确 <6%，但 L(i) 饱和曲线拟合偏差大）。warn=0x2（DLY_MISMATCH），未阻断。

**根因**：
1. 矢量峰值限幅（`IDENT_VEC_MAX_A = min(0.8*I_MAX, 0.9*BOARD_HOCP_A/1.15)`）把扫幅压低（如 vac48 ir_k=0.62 时扫幅顶仅 11.4A，而饱和拐点 i0sat=5A，深饱和区在 15-20A）
2. 扫幅不足导致描述函数拟合区间不够深，系统性低估饱和斜率
3. `DLY_MISMATCH`（0x4）不在 `trustedPre` 掩码里，不阻断（也不能简单加进去，因为 fan_dly0/dly2 正常电机也会触发 0x4）

**已尝试但未完全解决的方案**：
- 扫幅不足门控（`lTop/L0 < 0.70 && iTop < 0.85*swp_max`）：这些样本的 L 下降不够 30%（因为扫不到那么深的饱和区，反而看不出下降），所以门控没触发

**需要帮助的问题**：
1. 如何找到一个判别量，能区分"扫幅不足导致饱和拟合不可信"和"电机本身饱和不深"？
2. 失败样本的 warn=0x2（DLY_MISMATCH）是否可以作为辅助判据？延迟偏差大 + 扫幅受限 → 不可信？
3. 直接阻断 vs 降级为仅 L0（`s_l0Only`）哪个更合适？观测器在过载区用偏高的 L 是否安全？

### 失败样本特征数据（种子 661924030）

| 样本 | 电机 | L(mH) | ksat | i0sat | ir_k | SwpL% | Rs% | warn | dlySh | snrMin |
|---|---|---|---|---|---|---|---|---|---|---|
| h007 | vac48 | 0.066 | 0.010 | 5.0 | 1.10 | 30.6 | -5.4 | 0x0 | ? | ? |
| h032 | vac16 | 0.022 | 0.027 | 5.0 | 0.62 | 25.9 | +0.2 | 0x0 | ? | ? |
| h151 | vac48 | ? | ? | 5.0 | ? | 42.1 | -0.3 | 0x0 | ? | ? |
| h155 | vac16 | ? | ? | 5.0 | ? | 22.5 | -5.9 | 0x0 | ? | ? |

（开发集 b4caac6 上这些样本 warn 变成 0x2 = DLY_MISMATCH，因为超时告警改了之后更多样本能走到 acFit）

### 其他次要问题

1. **分半斜率阈值 1.5**：开发集标定显示健康电机 ratio p75=1.10, max=1.72, >1.5 只有 2/149。阈值合理但需要更多数据确认。
2. **`off_bound = min(0.02*I_MAX, 0.5*I0)`**：高 I_MAX 板（vac48 I_MAX=39A）的 2%FS=0.78A 远大于 I0=0.5A，已用 min 限制。实机需要核对实际零偏。
3. **延迟校准** `acCalDelay`：仿真里没有频变电阻所以成立，真电机铁损/涡流会让 Re(Z) 随频率上升被误当成延迟。实机建议先用纯电阻负载标板级延迟并固化。
4. **`kSwingModel`**：算出来后没参与任何判决，"实测远小于理论=转子卡住"目前只是注释。

## 编译状态

- Keil 双配置 0 Error 0 Warning
- `IDENT_TEST=0` SHA256 = `53453137678B75D54C9303AECD32137E74E46D275C2895517367499A17A6721E`（全程不变）
- `IDENT_TEST=1` Code=55068, RAM=3.19KB
- GCC `-Wall -Wextra` 0 警告，CRLF 纯净

## 下一步计划

1. **标定扫幅不足门控阈值**：提取失败样本的 lqAtH[0]/lqAtH[top]、iTop/swp_max、dlySh、perA，找判别量
2. **新种子盲测**（旧种子已作废）
3. **盲测通过后转实机**：限流电源 + IDENT_I_SCALE≈0.3 + 已知风机 + LCR/毫欧表对照，首轮只用 IDENT_MODE_STATIC
