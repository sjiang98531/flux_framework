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

## 配置（换电机 / 换板子只改这三个文件）

| 文件 | 内容 |
|---|---|
| `FW/Inc/cfg_motor.h` | 电机参数：极对数、Rs、Ld/Lq、psi_f、转矩 -> 加速度系数、铭牌额定值、电感饱和 L(i) |
| `FW/Inc/cfg_board.h` | 驱动板参数：时钟、PWM 频率、采样电阻 / 运放增益、母线、死区、转向，以及诊断 / 测试开关 |
| `FW/Inc/cfg_ctrl.h` | 控制规格：带宽、阻尼、时间常数、斜坡、各类门限（文件头有完整的换电机检查清单） |

各增益按带宽、阻尼、时间常数由电机参数自动换算；以下参数直接以物理值给出，**更换电机时须按 `cfg_ctrl.h` 文件头的检查清单重新确定**：

- 转速门限（`*_RPM`，如切闭环、回退、飞车接管、失锁、零电流跟踪结束、启停门限）：与观测器有关的门限按
  "反电势占额定母线的比例"估算，n = k · UDC_RATE · 60 / (2π · 极对数 · psi_f)，各门限的 k 写在注释中（见 `cfg_ctrl.h`"转速门限"一节）；
- 电流上限（`SPD_PI_LIMIT`、`IBUS_LIMIT_A`、`WEAKEN_ID_MIN_A`）：按电机热极限、退磁电流与电源能力确定；
- 定位时间（`ALIGN_MIN_S` / `ALIGN_STILL_S` / `ALIGN_MAX_S`）：按定位摆周期 T 的 1.6 / 0.2 / 4 倍选取；
- 观测器最低母线 `FLUX_BUS_MIN_V`。

`FW/Inc/cfg_derived.h` 由以上三个文件自动推导全部定点增益，无需修改；`FW/Inc/cfg_check_list.h` 在编译期 / 上电时检查配置，
任一项不满足时 `gFluxDiag.cfgFailLine` 给出行号，电机不起动。

电机参数可用辨识模式测量：`cfg_board.h` 中 `IDENT_TEST = 1` 编译，给出指令后电机不旋转，约 8 s 完成 Rs、死区、Ld/Lq、L(i)、
IPD 对比度的测量，结果在 `gIdent`，并自动在本次上电期间生效（见 `param.h`）。量产固件须设回 `IDENT_TEST = 0`。

## 控制结构

- **起动**：自举预充 -> 飞车检测（静止 / 顺风 / 逆风）-> 静止时 IPD 测初始位置（失败自动退回定位）-> I/F 拖动 -> 转速一致后转矩不变切闭环。
- **观测器**：电压模型磁链观测器 + 死区补偿 + 去直流（含相位补偿）+ NLFO 幅值修正 + 电感饱和 L(i)，磁链 PLL 锁角。
- **控制**：电流环（零极点对消 + dq 解耦）、速度环（S 形斜坡 + 轨迹前馈）、回馈减速母线电压控制、MTPA / 弱磁（默认关闭）。
- **保护**：硬件过流、母线过 / 欠压、缺相、超速、堵转（I/F 超时）、失锁重启。
- **实时结构**：快环 = ADC 中断 16 kHz（电流环、观测器），慢环 = PendSV 1 kHz（速度环、状态判断），主循环只处理非实时任务。

## 模块

| 文件 | 功能 |
|---|---|
| `supervisor` | 主状态机与子状态切换、故障汇总、事件记录（`gSup`、`gSupLog`） |
| `startup` / `ipd` / `flystart` | 预充、IPD、定位、I/F 拖动 / 飞车检测 |
| `spin` | 闭环运行、零电流跟踪、回退 I/F |
| `est` / `flux_obs` / `flux_pll` | 角度估计调度 / 磁链观测器 / 锁相环 |
| `foc_core` / `svpwm` | 电流环 / 空间矢量调制 |
| `speed_ctrl` / `id_ref` | 速度环与母线电压控制 / MTPA 与弱磁 |
| `meas` / `protect` | 测量滤波 / 运行保护 |
| `param` / `identify` | 运行时参数 / 参数辨识 |
| `diag` | 诊断量（配置检查、慢环健康计数、观测器置信度、版本号） |
| `hal` / `hal_sc32r602` | 硬件抽象层；移植其他芯片时只需提供新的 `hal_<芯片>.h/.c` 与 `Board/` |

## 运行

调试器中写 `gSup.speedCmd`（Q15，`rpm * 32768 / SPEED_MAX`）即可起停与调速：|指令| 超过 `STOP_TO_RUN_SPEED`（15 rpm）起动，
低于 `RUN_TO_STOP_SPEED`（12 rpm）软停机。

## 版本

固件版本见 `gFluxDiag.buildTag`（`FW/Inc/cfg_derived.h` 的 `BUILD_TAG`），当前为 `0xF20E0059`。
本仓库为发布版，与开发仓库 flux_framework 的同号版本控制逻辑一致（发布版不含性能测试工具与诊断计时）。
各次修改见 git 提交记录。
