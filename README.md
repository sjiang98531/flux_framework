# flux_framework —— 基于磁链观测的无感 FOC 框架（SC32R602）

高压风机 PMSM 无位置传感器 FOC 固件。芯片 SC32R602（Cortex-M0+，72 MHz），Keil 工程 `Project/MotorControl_FOC.uvprojx`。

## 目录

| 目录 | 内容 |
|---|---|
| `FW/Inc`、`FW/Src` | 控制算法与配置（本框架核心代码） |
| `Board/` | 驱动板外设初始化（`SC_Init.c`）、引脚与 ADC 通道（`Hardware_config.h`） |
| `User/` | 芯片头文件入口（`includes.h`）等 |
| `Device/`、`CMSIS/` | 芯片外设驱动与 CMSIS（原样使用） |
| `Project/` | Keil 工程、链接脚本 `flux_framework.sct` |
| `Output/`、`List/` | 编译输出目录（编译时生成） |

## 配置（三层）

| 文件 | 层 | 内容 | 何时修改 |
|---|---|---|---|
| `FW/Inc/cfg_motor.h` | 电机 | 极对数、铭牌额定电流 / 转速 / 功率、Rs、Ld/Lq、psi_f、转矩 -> 加速度系数；可选电感饱和 L(i) | 更换电机时必须修改（Rs / L / 磁链等可先填估计值，辨识后回填） |
| `FW/Inc/cfg_app.h` | 应用 | 转向、起动方式、起停门限、加减速斜坡、电流上限（额定电流的倍数）；现场调试项：转速门限（rpm） | 按产品要求修改；调试项默认值一般可直接运行，按现场现象调整（文件中注明了各项的调整方向） |
| `FW/Inc/cfg_internal.h` | 内部 | 带宽、阻尼、时间常数、观测器、飞车检测、母线电压环等，全部由前两层与驱动板参数自动换算 | 无需修改 |
| `FW/Inc/cfg_board.h` | 驱动板 | 时钟、PWM 频率、采样电阻 / 运放增益、母线、死区 | 更换驱动板时修改 |

更换电机的步骤：填写 `cfg_motor.h` -> 按需修改 `cfg_app.h` 第一部分 -> 编译运行；若起动、切换或低速表现不理想，
再按 `cfg_app.h` 第二部分各项注释调整转速门限。与观测器有关的转速门限可按"反电势占额定母线的比例"估算：
n = k · UDC_RATE · 60 / (2π · 极对数 · psi_f)。

`FW/Inc/cfg_derived.h` 由以上四个文件自动推导全部定点增益，无需修改；`FW/Inc/cfg_check_list.h` 在编译期 / 上电时检查配置，
任一项不满足时 `gFluxDiag.cfgFailLine` 给出行号，电机不起动。

电机参数可自动辨识：`cfg_board.h` 中 `IDENT_TEST = 1` 编译，给出一个转速指令即可。先进行静止测量（电机不旋转，约 15 s：
R/L、Rs、死区、Ld/Lq、L(i)、IPD 极性对比度），完成后自动停机并重新起动进行运行测量（磁链、加速度系数），随后按指令转速继续运行。
`gIdent.state = 2` 表示完成，结果在 `gIdent`，并在本次上电期间生效（见 `param.h`、`identify.h`）。量产固件须设回 `IDENT_TEST = 0`。

## 控制结构

- **起动**：自举预充 -> 飞车检测（静止 / 顺风 / 逆风）-> 静止时按起动方式 `gStartup.mode`（上电默认 `STARTUP_MODE`）：IPD 测初始位置（快、无电流脉冲：高频注入测凸极轴向 + 直流偏置判极性，失败自动退回定位）、直接拖动（静音：不定位，低频匀速牵入后加速）或定位后立即加速（静音：定位约 0.9 s，不等停稳）-> I/F 拖动 -> 转速一致后转矩不变切闭环。
- **观测器**：电压模型磁链观测器 + 死区补偿 + 去直流（含相位补偿）+ NLFO 幅值修正 + 电感饱和 L(i)，磁链 PLL 锁角。
- **控制**：电流环（零极点对消 + dq 解耦）、速度环（S 形斜坡 + 轨迹前馈）、回馈减速母线电压控制、MTPA / 弱磁（默认关闭）。
- **保护**：硬件过流、母线过 / 欠压、缺相、超速、堵转（I/F 超时）、失锁重启。
- **实时结构**：快环 = ADC 中断 16 kHz（电流环、观测器），慢环 = PendSV 1 kHz（速度环、状态判断），主循环只处理非实时任务。

## 模块

| 文件 | 功能 |
|---|---|
| `supervisor` | 主状态机与子状态切换、故障汇总（`gSup`：状态、故障码） |
| `startup` / `ipd` / `flystart` | 预充、飞车检测分发、IPD / 直接拖动 / 定位 / 飞车检测 |
| `openloop` | I/F 开环拖动、开环与闭环之间的切换 |
| `spin` | 闭环运行、零电流跟踪、回退 I/F |
| `est` / `flux_obs` / `flux_pll` | 角度估计调度 / 磁链观测器 / 锁相环 |
| `foc_core` / `svpwm` | 电流环 / 空间矢量调制 |
| `speed_ctrl` / `id_ref` | 速度环与母线电压控制 / MTPA 与弱磁 |
| `meas` / `protect` | 测量滤波 / 运行保护 |
| `param` / `identify` | 运行时参数 / 参数辨识 |
| `diag` | 固件版本号与配置检查结果（`gFluxDiag`） |
| `hal` / `hal_sc32r602` | 硬件抽象层；移植其他芯片时只需提供新的 `hal_<芯片>.h/.c` 与 `Board/` |

## 运行

调试器中写 `gSup.speedCmd`（Q15，`rpm * 32768 / SPEED_MAX`）即可起停与调速：|指令| 超过 `STOP_TO_RUN_SPEED`（15 rpm）起动，
低于 `RUN_TO_STOP_SPEED`（12 rpm）停机。停机方式由 `cfg_app.h` 的 `STOP_MODE` 决定（运行中可改 `gSup.stopMode`）：
默认受控减速（按 `SPEED_RAMP_DOWN` 减速到 `TRACK_END_RPM` 后关闭 PWM），或滑行（电流降到 0 后靠负载自行减速）。

## 版本

固件版本见 `gFluxDiag.buildTag`（`FW/Inc/cfg_derived.h` 的 `BUILD_TAG`），当前为 `0xF20E0068`。
本仓库为发布版，与开发仓库 flux_framework 的同号版本控制逻辑一致（发布版不含任何调试 / 测量内容；可读取的状态仅有版本号、配置检查结果 `gFluxDiag` 与运行状态、故障码 `gSup`）。
各次修改见 git 提交记录。
