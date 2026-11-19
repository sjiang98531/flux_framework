# 参数辨识审核交接文档

## 仓库

`https://github.com/sjiang98531/flux_framework.git`

分支 `master`，HEAD = `be7f1e4`

可以用以下 raw URL 直接读取关键文件（无需 clone）：

- 评估协议（验收线、已知限制、盲测结果）：
  `https://raw.githubusercontent.com/sjiang98531/flux_framework/be7f1e4/tests/ident_eval/PROTOCOL_V2.md`
- 辨识核心实现（1556 行 C）：
  `https://raw.githubusercontent.com/sjiang98531/flux_framework/be7f1e4/FW/Src/identify.c`
- 辨识定位阻尼（359 行 C，辨识相关段在 `#if IDENT_TEST` 内）：
  `https://raw.githubusercontent.com/sjiang98531/flux_framework/be7f1e4/FW/Src/startup.c`
- 辨识接口与告警位定义（173 行 C）：
  `https://raw.githubusercontent.com/sjiang98531/flux_framework/be7f1e4/FW/Inc/identify.h`
- 变更记录：
  `https://raw.githubusercontent.com/sjiang98531/flux_framework/be7f1e4/CHANGELOG.md`

额度有限的话优先读 `PROTOCOL_V2.md`（了解做了什么、验收线、已知限制）和 `identify.c` 的以下行范围：
- **2-DOF 注入控制**：`acSegStart`（549-644 行）、`acFast`（646-749 行）、`acCheckAndUpdateFF`（751-777 行）
- **死区拟合**：`dtFit`（137-190 行）
- **饱和描述函数拟合**：`dfStep`（1143-1192 行）
- **结果提交与门控**：`offerStill`（1194-1225 行）、`Ident_Slow` 的 `IDENT_PH_IPD` 段（1320-1366 行）

`startup.c` 的辨识定位阻尼在 326-359 行（`#if IDENT_TEST` 段内）。

## 项目目标

PMSM（永磁同步电机）驱动器的参数自动辨识功能。换电机后上电自动测 Rs、Ld/Lq、电感饱和曲线 L(i)、死区、磁链 ψf、惯量。测准了就用，测不了就报错拦住。

MCU 是 SC32R602（Cortex-M3，Flash 128KB / RAM 8KB），PWM 16kHz 或 48kHz。辨识代码在 `IDENT_TEST=1` 时编译、`IDENT_TEST=0` 时完全去除（二进制逐字节不变，已用 `fromelf --bin` 验证 SHA256 一致）。

## 起点

从 commit `205cd2ae`（Gemini 前序工作，19 个补丁式提交）开始。前序改动的问题：
1. 用"放大指令电流"对抗纯反馈 PI 跟踪衰减，死区和强饱和下打补丁越打越多
2. 定位阻尼扣除未标定的 Rs（含死区偏大），形成负虚拟电阻正反馈
3. 饱和拟合假定纯正弦电流，忽略 3 次谐波
4. 静止判据有盲区、依赖未标定的配置参数

## 做了什么（6 个组件，经双向消融验证）

### C1：无源对位阻尼（startup.c 326-359 行）
定位阶段不扣除未标定的 `Rs*iq`（粗测 Rs 含死区、远大于真实值），改用 `iq = -Kd * vq_ss`（纯虚拟电阻）。耗散功率 `(Rs + 1/Kd)*iq² + Vdt*iq ≥ 0` 对任意未知电机连续时间无源稳定。停稳判据 `|vq_filt| + |focKrs*iq|`（|eq| 保守上界）。

**离散颤振防护**：连续时间无源不等于离散稳定。极低磁链电机（ψ≈0.4 mWb）理论 Kd 极大，一阶滤波 + 采样延迟下特征根越过 -1，表现为 ~500 Hz 阻尼指令大幅交替。两级防护（均不依赖配置 ψ/J）：
1. 阻尼分量幅值钳位 `ALIGN_A/8`（约 0.1 倍额定电流）
2. 连续 4 拍大幅换向 → Kd 减半 + 清滤波（可多次衰减，下限 256）

### C2：2-DOF 交流注入控制（identify.c acSegStart/acFast/acCheckAndUpdateFF）
由"纯反馈 PI + 仿射放大电流指令"重构为"复阻抗电压前馈 + 保守带宽 PI + 1 步前馈闭环更新"。
- 注入轴：`vd = V_ff(sin(phi+dly), cos(phi+dly)) * env + PI_safe(I_ref*env*sin(phi) - id)`
- 前馈承担全部稳态阻抗与死区基波压降；反馈 PI 带宽 ≤ PWM/60（额外延迟时 PWM/80），即使强饱和使 L_inc 降至 0.20*L0 仍稳定
- 电流指令始终等于真实目标（不再放大）
- 瞬时峰值硬保护 `s_acIpkLim`，触顶时动态截断有效饱和点数 `s_acSatN`

### C3：含 3 次谐波的精确描述函数（identify.c dfStep, 1143-1192 行）
`acFast` 同步解调 3*phi 分量 `(h3s, h3c)`；`dfStep` 沿实测非正弦波形 `u = sin(θ) + h3s*sin(3θ) + h3c*cos(3θ)` 积分计算描述函数。消融验证：仅在 2-DOF 下有贡献（旧基线上无用），把 `20261119` SwpP95 从 9.11% 降到 7.31%。

### C5：静止监测与物理边界
- 死区阶梯：10×20 ms 子窗 vq 峰峰值 + 线性拟合残差校验
- 死区过渡带边界：拟合下限 2.6×INVERTER_DT_I0_A（含 1.3x 配置偏差），可辨识性边界 3.9×I0
- 交流零偏置段：整周期子窗正交轴 vqPP 与段间极差（去除了对恒定 DC 零偏敏感的 |vqMean|）
- 摆动系数上界：无配置依赖物理上界 `0.35 * L0 * ω0²`，触顶时同时置 SWING_CLAMP | AC_DRIFT（阻断）

### C6：IPD 后置
IPD 从"AC 之前"移到"AC/SAT_FIT 之后"（保持枚举数值不变以兼容 J-Scope）。理由：IPD 在前会把低惯量无凸极 vac48 击飞（静默失败 Swp=22.6%）；在后则安全（Swp=1.2%）。代价：IPM 被静摩擦卡在偏离定位轴 20°~30° 时选择安全阻断而不是测准，记为已知限制。

### 外推策略：lsMaxA 1.5 倍有界线性外推
允许运行期按拟合斜率外推至扫幅上限的 1.5 倍后再钳平。这是**工程判断而非数据推论**（按最坏真值标准胜出的是限流 6.9%，但牺牲过载能力）。已知限制：平滑饱和真值下所有外推策略卡在 ~15%（模型形状偏差）。

## 验证流程

1. **冻结评估口径**（Protocol v1 → v2.1）：运行期 Param_LqAt + lsMaxA 钳位模型，固定过载上限 I_run；主指标 SwpL（辨识区间精度），RunL（过载外推）单列
2. **双向消融**（11 组变体）：在困难集上逐一开关 6 个组件，拆除了零贡献的 C4（R/L 升频复测）
3. **扰动测试**：电流增益 ±5%、vbus ±5%、零偏 ±0.5%FS（主集）/ ±2%FS（压力子集）、死区 ±30%、R 温漂 +15%、齿槽+静摩擦、J×ψ 网格 15 点
4. **盲测**（N=300，种子一次性使用，组合扰动叠加每台电机）

## 盲测结果（种子 661924030，N=300，代码 be7f1e4）

| 验收项 | 结果 | 验收线 | 判定 |
|---|---|---|---|
| SwpL 残差 p95 | 8.37% | ≤8% | ❌ 差 0.37% |
| 高危静默失败（SwpL>25% 未拦） | 3 例 | ==0 | ❌ |
| 健康误拦 FP | 1.0% | ≤3% | ✅ |
| 压力子集真实静默失败 | 2 例 | ==0 | ❌ |
| 坏电机门控召回 | 100% (18/18) | ≥90% | ✅ |

### 失败样本分析

**3 例高危静默失败**（Rs 全部 <6%，L(i) 饱和曲线拟合偏差大）：
- 全是 vac48/vac16（低电感 16kHz PWM）+ 深饱和（ksat>0.02）
- 根因：矢量峰值限幅压低扫幅，描述函数拟合区间不足以覆盖饱和区
- Claude 建议：加门控识别"扫幅不足"并拦截，把静默失败变安全拦截

**2 例压力子集静默失败**（±2%FS 零偏下 Rs 被死区过渡带污染）：
- fan/fan48 额定电流低（0.75A × ir_k），±2%FS（±0.1A）残余零偏把阶梯推入过渡带
- ±0.5%FS 主集无此问题

## 编译状态

- Keil ARMCC V5.06 双配置 0 Error 0 Warning
- `IDENT_TEST=0`：Code=32600，二进制与重构前逐字节一致（SHA256 `53453137...A17A6721E`）
- `IDENT_TEST=1`：Code=54424, RAM=3.19KB（Flash 余量 ~74KB）
- GCC `-Wall -Wextra` 0 警告；`FW/` 下文件纯 CRLF

## 请审核的问题

1. `identify.c` 的 2-DOF 注入控制（`acFast` / `acSegStart` / `acCheckAndUpdateFF`）是否原理正确
2. 死区拟合 `dtFit` 的过渡带边界（2.6x/3.9x）是否合理（物理推导：死区误差在小电流段等效电阻 ≈ Vdt/I0，只有阶梯电流远大于 I0 时斜率才收敛到真实 Rs；2.6x = 2x × 1.3 配置偏差上界，3.9x = 3x × 1.3）
3. `startup.c` 的无源阻尼 + 颤振检测是否安全
4. 已知限制是否可接受，还是需要继续改进后再转实机

## 下一步建议

- Claude 建议：对"低 PWM × 深饱和"加门控（识别扫幅不足并拦截），把静默失败变安全拦截
- 转实机：限流电源 + IDENT_I_SCALE≈0.3 + 已知参数风机 + LCR 表/毫欧表对照
- 盲测只能说明在仿真电机分布内未过拟合，不等于实机通过
