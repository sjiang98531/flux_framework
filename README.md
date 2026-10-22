# flux_selfpll — 用磁链观测器替换原厂 SMO 的无感 FOC

> **接手先读这一份。** 读完你应该知道：项目要做什么、现在做到哪、代码怎么组织、哪些路已经走过不要再走、下一步做什么、怎么和用户协作。
> 逐轮实验的完整过程与数据在 **`实验记录.md`**；判读方法、工具与踩坑在 **`docs/方法与经验.md`**。
> 最后更新：2026-10-22，对应 **tag `test29` / 固件 `FLUXPLL_BUILD_TAG = 0xF10E000E`**（test29 = test28 瘦身版 + PLL 输出速度下限，已台架回归：9/9 冷启动一次成功）。

---

## 0. 一页现状

**目标**：在赛元原厂高压风机 FOC 工程上，**只把 SMO 观测器换成磁链观测器**，其余（启动流程、速度环、电流环、保护、参数）与原厂一致，并且性能优于 SMO。可以调的只有磁链观测器本身和自写 PLL（FluxPLL）的 Kp/Ki。

**结论：目标已达成（空载、正转、≤1953 rpm 范围内）。** 同口径对比（smo_baseline）后的分工是：

- **磁链版更好的**：**Iq 低 18–20%**（效率）；**低速启动明显更准**（100→293 rpm 跟踪标准差 3–4 vs 17.6 rpm）
  且 **9/9 一次成功、0 回退**（SMO 5 次里 2 次回退）
- **两者相当**：高速稳态转速波动、阶跃过冲、平台振荡周期 —— **这些是原厂速度环的特性，与观测器无关**

| 项 | 原厂 SMO | 现在（磁链 + FluxPLL） | 证据 |
|---|---|---|---|
| 架构 | 原厂 | **与原厂相同**：观测器从 Startup 第一拍自由运行，唯一交接点是 `Fast_Boot_StartUp()` 80 rpm | test22/23 |
| 冷启动 | **5 次里 2 次回退**（启动期 `filt` 最低 −94~−182 rpm） | **9/9 一次成功**（0 回退、0 卡死），进 Spin 0.46–1.03 s | `smo.csv` vs test29 |
| 启动斜坡 100→293 rpm 跟踪标准差 | **17.6 rpm** | **3.0–4.1 rpm** | smo_baseline vs test29 |
| 飞车启动（转子在转时上电） | 有 | **可用**：Calib→Torqueloop→Spin 0.43 s | test23 第 3 次 |
| 角度精度（Iq 最小法实测） | 影子模式下与磁链差 24°，靠最大 72° 经验补偿 | **−1.6° @1172 rpm，−1.0° @293 rpm**，无需补偿 | test24、test26 |
| **1172 rpm 空载 Iq**（各停 ≥15 s，取最后 5 s） | **0.351–0.366 A** | **0.292 A（−18%）** | `smo.csv` vs test29 |
| **1172 rpm 转速波动**（同口径，≥10 s 平台最后 5 s） | **±2.8–9.1 rpm（0.23–0.77%）** | **±2.9–10.2 rpm（0.25–0.87%）** | **相当**，`smo.csv` vs test29 |
| 阶跃到顶的过冲 / 欠调 | **+106~+109 / −75~−77 rpm** | **+105~+114 / −72~−79 rpm** | **一致**，都是速度环 |
| 1172 平台振荡周期（自相关） | **3.24–3.31 s（0.30–0.31 Hz）** | **3.04–3.08 s（0.32–0.33 Hz）** | 差 <8%，同一速度环 |
| ISR 峰值 | — | **1511 counts = 41.9 µs（67% 周期）**，`overBudget` = 0 | test29 |
| 慢环（速度斜坡实测/配置） | — | **193/200 rpm/s** | test27 |

**没测过**：反转、带载启动、2000 rpm 以上、长时间温升（见 §8）。

**今天（10-21）最重要的认知转变**（详见 `实验记录.md` 的「更正汇总」）：
1. 上午到下午花了 20 轮在"注入 + 250 rpm 释放 + 冻结"的自创启动方案上打补丁，**根因是假设"转子跟着开环角走"不成立**——0.5 A 强拖在 200 rpm/s 斜坡上约 110 rpm 就失步，释放时转子只有 30–130 rpm。所谓"释放后观测器跌速""14.3 Hz 观测器振荡"其实是**转子真的没跟上**和**冻结角度与转子之间的滑差拍频**。观测器本身一直是准的。
2. 回到原厂架构（关注入、原厂参数）后，磁链观测器**能从零速自己建立**，唯一的毛病（ψ 从 0 起步方向随机导致假启动）用一行"ψ 初值种在 θ̂ 上"解决。
3. ISR 计时 `2250 counts = 62.5 µs`（TIM3 是 36 MHz），原记录"50% 有余量"是错的——ISR 早就贴满整个 PWM 周期，这才是"慢环只有几百 Hz"的原因。删掉没人读的计算后 ISR 降到 71%。

---

## 1. 约束（违反会出事）

1. **`FOC_TypeDef` 字段布局不能改。** 闭源库按字节偏移访问它。原厂的 `SMO` 字段改名为 `FluxFeedback`（同类型 `SMOType`，偏移 0x70 不变），`FluxPLL` 复用 `FOC.sPLL` 的存储。要加字段只能加在自己的结构体里（`FluxObs_TypeDef` / `FluxDiag_TypeDef`）。
2. **闭源库 `MCI/Src/SOC_MCI_2R_V102.lib`**：`Fast_Boot_StartUp/Spin/Current_Controller`、`MCF_ADV_Protect`、`MCF_FlyStart_2R`、`Var_Filt` 等。`.lib` 内目标文件头被厂家改过，不能直接反汇编；**反汇编链接后的 `Output/SOC_FOC_SDK_V1_0.axf`** 即可。已核实它们读写的 FOC 偏移（§6.4）。
3. **原厂参数不动**：`IQ_STARTUP 0.5`、`ALIGN_TIME 0.001`、`SPEED_KP 8000`、`SPEED_KI 100`、`STARTUP_SPEED_RAMP 60`、`SPEED_RAMP_UP 200`、`STARTUP_TO_SPIN_SPEED 80`。用户明确要求"先和原厂对齐，只改磁链"。
4. **一次只改一个变量**，改前先说清楚：假设 / 改动 / 预期现象 / 证实与证伪判据。
5. **ISR 预算**：任何加进 `ADC_IRQHandler` 调用链（`PMSM_Obs`、`FluxObs_*`、`FluxPLL_*`）的代码先估时钟周期。M0+ **无硬件除法**，一次 `uint32` 除法 ≈ 100–200 周期；test25 就是在 ISR 里加了一次除法，导致慢环停摆、停机命令失效、只能断电。

---

## 2. 硬件与单位

| 项 | 值 |
|---|---|
| MCU | 赛元 SC32M156C7，Cortex-M0+ @72 MHz，无 FPU、无硬件除法 |
| 电机 | 5 对极；`Rs=14.7 Ω`、`Ls=46 mH`（`mc_customize.h`）；ψf 实测 **0.135–0.141 Wb**（铭牌 0.154 偏高，见 §6.1） |
| 母线 | 台架 310 V（`UDC_MAX=475` 只是归一化基准）；310 V 下理论最高约 2088 rpm |
| PWM / 快环 | 16 kHz（62.5 µs），中心对齐，ADC 序列结束中断 `ADC_IRQHandler` 里跑整个快环 |
| 慢环 | 主循环轮询 `bSlowLoopFlag`，配置 1 kHz；实际频率取决于 ISR 留下的 CPU 时间 |
| 死区 | 配置 `DEAD_TIME=1200 ns`；10-20 曾查到硬件寄存器实际装 24（≈667 ns），未复核 |
| 电流采样 | 2 电阻，`ic=-ia-ib` |
| 归一化 | 电压 475 V；磁链 `FLUX_PSI_N = 475/(2π·100) = 0.756 Wb`；速度 3200 rpm；`NOMINAL_Q15 = 6111`（0.141 Wb） |
| 电流换算 | **`FOC.sIdq.s16Q` counts × 7.964 / 32768 = A**（与 `IQ_STARTUP` 指令值吻合）。注意 `IQ_STARTUP` 指令本身是按 `I_MAX=3.93 A` 编码的（4168 counts = 0.5 A），两者差 2 倍的原因在闭源电流环里，未查清；**比较时只用同一口径** |
| 角度/速度 | `s32Speed` Q31，2³¹ ↔ 3200 rpm；`s32Theta` **Q16.16**（>>16 得 0..65535 一圈）；`SPEED_TO_THETA = 1092` |
| ISR 计时 | TIM3 ÷2 = **36 MHz**，1 count = 27.8 ns，**2250 counts = 62.5 µs = 一个 PWM 周期**（`FLUX_ISR_BUDGET`） |
| 状态码 | `runState`：0 Charge 1 Calib 2 Align 3 Startup 4 Spin 5 Torqueloop；`mainState`：0 Fault 1 Init 2 Stop 3 Run |

---

## 3. 当前架构

### 3.1 快环数据流（每 62.5 µs）

```
ADC_IRQHandler (User/Src/SC_it.c)
 ├─ FluxObs_Capture()                     记下上一拍的 PWM 比较值、母线、电流
 └─ s_STATE_FAST[mainState]()
     └─ Startup: M1_RunStartupFast()      / Spin: M1_RunSpinFast() → PMSM_Obs()
         ├─ FluxObs_UpdateFeedback()      (User/Src/FluxObs.c)
         │    └─ FluxObs_Run():
         │         va,vb ← 比较值×母线 重构（指令电压，不含死区）
         │         e = v − Rs·i（i 取前后两拍均值）      → s16Eal/s16Ebe（诊断）
         │         e −= MA(e)  （高通去直流，2^12 拍）
         │         ψs += 2·(e·KTs + NLFO 径向修正)      NLFO: corr ∝ ψr·(ψf²−|ψr|²)
         │         ψr = ψs − Ls·i                      → s16PsiRAl/Be
         │         每 16 拍算一次 |ψr|（诊断）
         ├─ err = ψrβ·cosθ̂ − ψrα·sinθ̂ = |ψr|·sin(θ−θ̂)   磁链叉积鉴相，不按转向翻符号
         ├─ Startup: FluxPLL_RunStartup(err, …, 12.8 rpm 下限)   Spin: FluxPLL_Run(err)
         └─ CmdTheta = θ̂ (+ ErrTheta) → 电流环 → SVPWM
```

### 3.2 启动流程（与原厂相同）

```
Charge 20 ms → Calib（MCF_FlyStart_2R 飞车检测 100 ms）
   ├─ 检测到转动 → Torqueloop → Spin                       （飞车启动）
   └─ 静止 → Align（原厂 1 ms）→ Startup → Fast_Boot_StartUp() 判 80 rpm 持续 10 拍 → Spin
Startup 入口 FluxObs_BeginStartup()：清观测器，ψ 初值 = ψf∠θ̂（test23，消除假启动）
Startup 期间：Iq = 0.5 A，PLL 带 12.8 rpm 积分下限往前推，观测器自由运行
```

### 3.3 与原厂的全部差异

| 位置 | 原厂 | 现在 | 原因 |
|---|---|---|---|
| `PMSM_Obs()` / `M1_RunStartupFast()` | `MLIB_SMO` + 反电势鉴相（按转向翻符号） | `FluxObs_UpdateFeedback` + 磁链叉积鉴相 | 换观测器本身 |
| PLL | `MLIB_PLL` / `MLIB_PLL1`，80 rpm 交接时切增益移位 | `FluxPLL_Run` / `FluxPLL_RunStartup`，全程一组增益 | 磁链幅值恒定，不需要按转速调增益 |
| 角度补偿 `s16ThetaComp` | `0.825·spd·2`，限幅到 72° | 两处直接写 0 | 磁链角无滤波滞后；实测加补偿反而偏 |
| `FluxObs_BeginStartup()` | 无 | 清观测器 + ψ 种在 θ̂ | 消除 ψ 从 0 起步导致的假启动 |
| **PLL 速度下限** | `MLIB_PLL1` 只给**积分器**加下限 | Startup 里给**积分器和输出转速都**加（`advance()` 的 `minSpeed` 参数，Spin 传 0） | 磁链鉴相在零速幅值是满的，`err` 能到 −336 counts，正好把比例项抵成 0；只加积分器会让角度冻住、转子不动（test28_2 → test29） |
| J-Scope 诊断量 | 无 | `s16Eal/Ebe`、`s16PsiRMag`、`gFluxDiag.*`、`gFluxPLLDiag.*` | 只写不读，不影响控制 |
| 其余（参数、速度环、电流环、保护、状态机分支） | — | **相同** | — |

### 3.4 文件

| 文件 | 说明 |
|---|---|
| `User/Src/FluxObs.c`、`User/Inc/FluxObs.h` | 磁链观测器（自写） |
| `User/Src/FluxPLL.c`、`User/Inc/FluxPLL.h` | 自写 PLL（Q16.16 角度、饱和加法、速度限幅） |
| `User/Inc/FluxConfig.h` | 观测器与 PLL 的全部常量 + **版本历史（`FLUXPLL_BUILD_TAG` 注释）** |
| `User/Src/FOC_PMSM.c` | `PMSM_Obs()`：Spin 阶段的观测器 + PLL 调用 |
| `User/Src/StateMachine.c` | 原厂状态机，改动见 §3.3 |
| `User/Src/SC_it.c` | 快环中断，`FluxObs_Capture` / `FluxObs_Timing` |
| `Main/Inc/mc_customize.h` | 电机/控制参数（全部原厂值） |
| `MCI/` | 原厂数学库头文件 + 闭源 `.lib` |
| `Output/FluxSelfPLL_v18_test28.jscope` | **当前固件对应的 J-Scope 工程**（test28 编译后生成；v17 只对 test27 固件有效；每次重新编译后必须重新生成） |
| `Data/` | 全部台架 CSV（test1–27、SMO 基线）与 SMO 基线固件 |

---

## 4. 关键参数（当前值）

| 宏 | 值 | 说明 |
|---|---|---|
| `FLUX_NOMINAL_WB` | 0.141f | ψf，NLFO 的幅值目标；`NOMINAL_Q15 = 6111` |
| `FLUX_NLFO_SH` | 9 | NLFO 增益移位，τ≈16–19 ms。**不要加大（变弱）** |
| `FLUX_NLFO_FQ_LIMIT` | 32768 | 只防溢出。**不要收紧**（test11 收到 8192 明显变差） |
| `FLUXPLL_KP_Q31_PER_ERR` / `KI` | 25583 / 106 | 设计 ωn=15 Hz、ζ=0.707；按实际 Kd≈6014 是 ωn≈14.4 Hz、ζ≈0.68 |
| `FLUX_BUS_MIN_V` | 100.0f | 母线低于此值观测器保持复位 |
| `FLUXPLL_BUILD_TAG` | 0xF10E000E | 每次实质改动 +1，历史写在 `FluxConfig.h` 注释里 |
| 原厂：`IQ_STARTUP` / `ALIGN_TIME` / `SPEED_KP` | 0.5 A / 0.001 s / 8000 | 原厂值。注意 `SPEED_KP` 注释里的 "15000" 是过时注释 |

---

## 5. 已放弃的路线（不要再走）

完整数据见 `实验记录.md` 对应轮次。

| # | 路线 | 结论 | 轮次 |
|---|---|---|---|
| 1 | **启动注入**：Startup 期间每拍把 ψ 写成 ψf∠θ_开环，250 rpm 释放 | 前提"转子跟着开环角走"不成立；释放时 ψ 带着负载角/失步误差，必然产生瞬态。原厂架构下根本不需要 | test1–21，test22 推翻 |
| 2 | 释放后冻结 PLL 速度 200 ms（`FluxPLL_Hold`） | 冻结期间电流矢量在转子上滑，转子反而再掉速；"14.3 Hz 振荡"就是这个滑差拍频 | test8–21 |
| 3 | 冻结电流环 PI 积分器、输出交叉淡出、积分器牵引、err 限幅、事件触发释放 | 都是在给路线 1 打补丁，各自数据都显示无效或更差 | test5–15 |
| 4 | **死区补偿 DTC**（按 1200 ns、sign(i) 模型） | 两次都更差。实际有效电压误差只有约 2 V（§6.1），按 7.9 V 补偿是过补约 4 倍并且反号 | test12、test16、10-20 |
| 5 | 收紧 NLFO 钳位 | NLFO 是唯一稳住幅值的东西，减弱只会让 \|ψ\| 跑更远 | test11 |
| 6 | NLFO 加径向积分项 | 幅值差是标称值错，不是控制误差 | test13 |
| 7 | 用 e/ω 当磁链修正参考 | 参考被电压误差旋转，修正有切向分量会把角度拉歪；改成 NLFO（纯径向）后才跑通 | test8→9 |
| 8 | 强拖段加大电流 / 加长对齐 / 提 Kp / 预置速度 PI | 在路线 1 下有效或部分有效，但回到原厂架构后都不需要，参数已全部恢复原厂 | test18–21 |
| 9 | 冻结结束时按 e 投影预置速度 PI | 那一刻转子匀速、投影≈0，不是后续需要的转矩电流，造成 −245 rpm 欠调 | test21 |
| 10 | 在 ISR 里做除法/每拍 SinCos | ISR 本来就贴满周期，一加就崩 | test6、test25 |

---

## 6. 已确认的事实（可直接引用）

### 6.1 电机与电压模型
1. **ψf ≈ 0.135–0.141 Wb**（铭牌 0.154 高约 10%）。`|e|/ω` 在 554/976/1465 rpm 为 0.1438/0.1407/0.1393，可拟合成 `ψ + V/ω`：ψ≈0.1353–0.1366 Wb、V≈2 V。拟合残差主要来自加速段与稳态段的差别（V 随电流变），所以 **ψ 与 V 的拆分不可靠，只有"有效电压误差约 2 V 量级（不是 7.5–7.9 V）"这个结论可靠**。
2. 电压重构用的是 `pwm_gen.CompA/B/C`（指令），**不含死区**。
3. 稳态 Id≈0 时，沿电流方向的电压误差只改变 |ψ|，不改变角度；NLFO 负责把幅值拉回。

### 6.2 观测器与 PLL
1. `s32Theta` 是 Q16.16，角度步进必须 `(speed>>15)*kTheta`（test3 定位，曾少除 2¹⁶ 导致 θ̂ 冻结）。
2. NLFO 修正必须**纯径向**（沿 ψr），否则会把角度拉歪（test8→9）。
3. 观测器**能从零速建立**：静止时靠极小的 e 积分 + NLFO 拉幅值；种好初值后，转子被 12.8 rpm 下限推动约 50 rpm 时就能锁住真实转子（test22/23 用 e 的旋转速度独立核对）。
4. **绝对角度误差 −1.6°（1172 rpm）、−1.0°（293 rpm）**，方法：电流坐标系加 0/±20/±40° 偏移，拟合 `Iq = a/cos(δ−δ0)`（test24/26）。
5. `controlComp`（原为导数支路 LPF 滞后的补偿）在 PLL 路径上不存在对应滞后，293 rpm 时会多加约 +3.9°，已置 0。
6. **磁链鉴相在零速时幅值是满的，PLL 的速度下限必须加在输出转速上。**
   `err` 一下就能到 `floorQ31/Kp = 8.59e6/25583 = 336 counts`，正好把比例项抵成 0。
   原厂 PLL1 把下限只加在积分器上，配 SMO（零速 `err ≈ 0`）够用，配磁链不够。
   而且钳位必须放在**算出 `s32Speed` 之后、推进 `s32Theta` 之前**——放在 `advance()` 返回之后是无效的：
   这一拍角度已按 0 速推完，下一拍速度又重算，结果只是 `filt` 显示 12.8 rpm，角度照样冻住。
   症状：Startup 里 `err` 停在 −280~−380、`filt` 掉到负值、转子全程不动（test28_2 → test29）。

### 6.3 状态机与原厂行为
1. `M1_RunSpinSlow()` 有两条回退：`Fast_Boot_Spin()`（轮询）和 `spdCmd·filt<0 && |filt|<50 rpm`（**即时**，去抖无效）。
2. `Fast_Boot_StartUp/Spin` 只看 `s16SpdFilt` 与 `s16SpdCmd`；`s16SpdFilt` 是 `Var_Filt()` 对 `FluxFeedback.s16Speed` 的滑动平均，即 **PLL 自己的输出**——不能拿它判断"观测器准不准"，要用 e 的旋转速度或 |e|。
3. 冷启动前转子必须静止，否则 `MCF_FlyStart_2R` 会走飞车启动路径（test23 第 3 次）。
4. 原厂速度环（Kp 8000、Ki 100）阻尼偏小：交接后有约 +80 rpm 超调、周期约 4 s 的衰减振荡。SMO 版同样存在，属于原厂特性，不在本项目范围。
   **293 rpm 处要 12 s 以上才稳得住**，进入平台后第 1–4 s 的 filt 标准差约 ±42 rpm，5–8 s 还有 ±24。
   ⇒ **所有稳态波动必须取"平台最后 5 s"**，短平台（<15 s）的数据一律不可信。
   **smo_baseline 用原厂 SMO 固件完整复现了这一点**：到顶过冲 +106~+109 rpm、平台振荡周期
   3.24–3.31 s、稳态波动 0.23–0.77%，与磁链版的 +105~+114 / 3.04–3.08 s / 0.25–0.87% 几乎一致。
   ⇒ **过冲与振荡是速度环的特性，与观测器无关**，换观测器不会改善它。

### 6.4 闭源库实际访问的 FOC 字段（反汇编 AXF，test27 期间）

`FluxFeedback` 起于 FOC+0x70：`s16Ealpha_est` +0x84、`s16Ebeta_est` +0x86、`s16Speed` +0x90。

| 函数 | 访问的 FOC 偏移 |
|---|---|
| `Fast_Boot_StartUp` / `Fast_Boot_Spin` | 0x52 SpdCmd、0x58 SpdFilt |
| `Fast_Boot_Current_Controller` | 0x1c–0x3c（电流、VMAX）、0xD0 起（CurID/CurIQ）；Startup 状态下会改 d 轴积分器 |
| `MCF_ADV_Protect` | 0x00–0x02、0x58、0x60、0x140 起（sProtect） |
| `MCF_FlyStart_2R` | 0x18–0x1a（Iαβ）、0x94（sPLL）、0xAC 起（sFly_Start） |
| `Var_Filt` | 0x40、0x46、**0x90 → 0x58** |

⇒ **`FluxFeedback.s16Ealpha_est/Ebeta_est` 没有任何读者**（test27 已删掉为它们服务的导数支路与混合计算）。

### 6.5 CPU 时间
1. `2250 counts = 62.5 µs`。test14 时代 `isrMax` 2263 = 62.9 µs，ISR 早已贴满周期；偶发超时不会丢中断（pending 立即重入），所以**台阶/斜坡间隔正常不能反证没超时**。
2. 精简后（test27）：`isrMax` 1608（44.7 µs，71%），`overBudget` 0，慢环斜坡 193/200 rpm/s。
3. 慢环停摆 ⇒ 停机命令、保护全部失效（test25）。看门狗在原厂代码里就是关的（`WDG_init()` 被注释）。

---

## 7. 代码瘦身（test28 已完成）

test28 把下列死代码全部删除，**行为与 test27 逐位相同**（主机上用同一组 40 万拍合成输入跑新旧 `FluxObs.c`+`FluxPLL.c`，含坏采样复位、饱和、Startup/Run 两种 PLL，全部状态量的哈希一致）：

- 注入：`injectFlux()`、`FluxObs_RearmInject()`、`FluxObs_OpenLoopAngle()`、坏采样分支里保留注入状态的代码、`MS_TICKS`，以及 `M1_SwitchRunSpinStartup()` 里的调用
- PLL 冻结：`FluxPLL_Hold()` / `FluxPLL_Holding()` / `sHoldTicks` / `gFluxPLLDiag.dLast`，以及 `M1_RunSpinFast()` 里"冻结电流环 PI"那段（恢复成原厂的一行 `FOC_Current_Controller(&FOC);`）
- 死区补偿：`signCurrent()`、`abs16()`、`voltage()` 里的 DTC 段与开关掩码
- `controlComp`：`PMSM_Obs()` 和 `M1_RunSpinSlow()` 里直接 `s16ThetaComp = 0`
- `FluxConfig.h`：只剩 `FLUX_BUS_MIN_V`、`FLUX_NOMINAL_WB`、`FLUX_NLFO_SH`、`FLUX_NLFO_FQ_LIMIT`、`FLUXPLL_KP/KI`、`FLUXPLL_BUILD_TAG`（版本历史保留）
- `FluxObs_TypeDef` / `FluxDiag_TypeDef`：删掉所有只写不读的字段；**v17 J-Scope 的 10 个通道全部保留**，但结构体布局变了，必须重新生成 jscope（v18）

被删代码的原文都在 `git show test27:User/Src/FluxObs.c` 等处，需要时从 git 取。

---

## 8. 待办（按建议顺序）

1. **补测未覆盖工况**（不改代码）。目前所有结论都来自**空载正转**，建议顺序：
   **反转启动 → 1500–2000 rpm → 带载启动**，中间穿插第 2 条的同口径 SMO 对比
   （它决定 §0 那张"优于 SMO"的表还站不站得住，比带载更急）。连续运行温升放最后。
   - 反转：`M1_RunSpinSlow` 的即时反向判据 `spdCmd·filt<0 && |filt|<50` 在反转时容易误触发
   - 1500–2000 rpm：310 V 母线下限约 2088 rpm，要看是电压饱和还是观测器先失效
   - 带载：`IQ_STARTUP = 0.5 A` 的拖拽力矩够不够是未知数
2. ~~同口径 SMO 对比~~ —— **已完成（`smo_baseline`，`Data/smo_baseline.csv`）**。
   1172 rpm 上：Iq 低 18%，波动 / 过冲 / 振荡周期与 SMO 相当。
   **977 与 1500 rpm 两个点不再补**（2026-10-22 决定）：1172 这一点已足以定性——
   三个关键指标与 SMO 几乎重合，多测两点不会改变结论。
3. ~~代码瘦身 + 回归~~（**test28**：瘦身，`test28`/`test28_2` 回归确认行为不变；**test29**：修启动卡死，9/9 冷启动一次成功，`overBudget` 0、`isrMax` 1511，稳态与 test27/28 一致）。
4. **调 FluxPLL Kp/Ki**（用户允许调整的范围）：现在 ωn≈14 Hz。提高看低速锁定与交接，降低看稳态 err。
5. ~~293 rpm 长驻留复测~~ —— **已用现有数据结案，不用再测。**
   把每个 293 rpm 平台从进入时刻起按 3 s 窗口算 filt 标准差（单位 rpm）：

   | 平台进入后 | 1–4 s | 3–6 s | 5–8 s | 8–11 s | 12–15 s |
   |---|---|---|---|---|---|
   | test28（25.6 s 平台） | 41.9 | 30.9 | 24.1 | 14.2 | 7.3 |
   | test28_2 各平台 | 41.6–45.2 | 30.7–33.6 | 23.9–26.1 | 14.6 | — |
   | test29 各平台 | 41.6–43.2 | 30.7–31.9 | 24.0–24.9 | 14.5 | — |

   三份的衰减曲线几乎完全重合。test28 平台够长，最后 5 s 是 1.8 rpm（0.62%），test27 是 1.4 rpm（0.45%）。
   ⇒ 所谓"293 rpm 波动从 0.5% 涨到 7%"，只是 test28_2/test29 的平台只有 8–13 s、最后 5 s 还没衰减完，
   **不是性能变差**。

   **但这里有一条真正值得记的**：原厂速度环在 293 rpm 要 **12 s 以上**才稳得住，开头标准差约 **±42 rpm**。
   这是原厂速度 PI 的特性，不在本项目改动范围，但**所有"短平台"的波动数据都不可信**，
   稳态一律以"平台最后 5 s"为准。
6. 待查的开放问题：
   - Iq 指令与实测差 2 倍的原因（闭源电流环内部换算？）
   - 硬件死区实际值（1200 vs 667 ns），以及剩下约 −1~−1.6° 固定角度偏差的来源
   - 原厂速度环超调（属原厂特性，需用户决定是否处理）

---

## 9. 协作方式与流程

### 9.1 分工
- **用户**：编译（Keil）、烧录、上台架、采 J-Scope CSV、执行 git。
- **分析型 AI（Claude）**：读代码与数据、提出单变量改动与判据、审查改动、分析 CSV。可以直接改文件（经用户同意）；本机命令行环境不可用时由用户/DeepSeek 执行 git 与删除。
- **操作型 AI（DeepSeek）**：按方案改代码、编译、生成 J-Scope 工程、提交 git、追加实验记录。
- 每轮：方案（假设/改动/预期/判据）→ 改代码 → `git commit` + `git tag -a testN` → 编译 → 生成新 jscope（单独提交）→ 上台架 → CSV 存 `Output/testN.csv` → 分析 → 追加 `实验记录.md`（单独提交）。

### 9.2 git
仓库在本目录，`master`，无远程。规则：
1. 改代码前先 commit；改完立刻 `git tag -a testN`，**再**编译烧录，保证标签指向被测源码。
2. 出问题先 `git diff testA..testB -- User/ Main/`，不要凭记忆。
3. 代码改动、jscope、实验记录分开提交。
4. commit 信息写"改了什么、为什么"，中文。不要用 `/tmp` 传信息文件（Git Bash 的 /tmp 原生 git 读不到），用 `.git/CMT.txt`。
5. Keil 每次打开都会改 `*.uvoptx`，已忽略。

已有标签：`baseline-20261021`、`test17`…`test28`（详见 `git tag -n`）。

### 9.3 编译
`E:\Keil_v5\UV4\UV4.exe -b Project\MotorControl_FOC.uvprojx -t MC_FOC_M15x -j0 -o Output\build.log`（返回 0 = 无错无警，1 = 有警告）。既有警告：`FOC_PMSM.c` 的 `s32Temp` 未使用。

### 9.4 J-Scope
- 每次重新编译后都必须重新生成工程（地址随镜像变化），新文件名递增（v18、v19…），打开后 File → 重选 ELF。
- 生成工具：`C:\Users\SOC\.workbuddy-ai\binaries\python\envs\default\Scripts\python.exe C:\Users\SOC\.workbuddy-ai\skills\jscope-project-gen\assets\make_jscope.py --axf <axf> --out <new.jscope> --template <上一个.jscope>`。**不许手工编 XML、不许用 .map 地址手算偏移。**
- 最多 10 个通道。**通道顺序改变会改变各通道之间的读取时刻**（实测约 0.4 ms），比较不同轮次的相位类量时必须保持通道顺序不变。
- 当前推荐通道（v17）：`gFluxDiag.runState`、`FOC.s16SpeedRamp`、`FOC.s16SpdFilt`、`gFluxPLLDiag.errLast`、`FOC.s16CmdTheta`、`sFluxObs.s16Eal`、`sFluxObs.s16Ebe`、`FOC.sIdq.s16Q`、`gFluxDiag.overBudget`、`gFluxDiag.isrMax`。

### 9.5 每份 CSV 的固定检查
1. `overBudget` 是否为 0；`isrMax` 是否 < 2250。
2. 慢环是否正常：`s16SpdFilt` 每秒更新次数（正常约 490/s）、斜坡实测斜率是否接近配置。
3. `runState` 4→3 回退次数；冷启动是否都走 2→3（走 5 说明转子没停稳）。
4. 用 e 独立核对转速：`|e|`（约 5.07 counts/rpm）和 `angle(e)` 的旋转速度，与 `s16SpdFilt` 比。
5. 稳态段 filt 均值±标准差、Iq 均值、`|err|` 最大值，与上一轮比较。

判读方法的细节、分析脚本写法、各种坑：`docs/方法与经验.md`。

---

## 10. 目录

```
README.md                 本文：接手入口
实验记录.md               10-20 前史 + test1–test27 全部过程、数据、更正
docs/方法与经验.md        判读方法、工具、踩坑与教训
docs/archive/             旧文档原文（只读，内容已并入上面三份）
docs/参考资料/            烧录工具手册、原厂 SMO 参数计算表
Data/                     台架 CSV（test1–27、smo_test、flux_test2）与 SMO 基线固件
Output/                   Keil 构建产物（不进库）+ 当前 J-Scope 工程
User/ Main/ MCI/ CMSIS/ Device/ Project/   源码与工程
```
