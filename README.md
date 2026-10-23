# flux_framework —— 基于磁链观测的可移植无感 FOC 框架（SC32R602）

高压风机 PMSM 无感 FOC。原厂 SMO 已换成磁链观测器 + 自写 PLL，然后按模块重构。

本工程由 flux_selfpll 的 test33 重构而来。test1~test33 的全部实验过程和数据都留在 flux_selfpll 里（`实验记录.md`、`README.md`、`Data/`），这里不再重复。

| 先读 | 内容 |
|---|---|
| `重构说明.md` | 目录、模块、调用链、旧→新对照、参数怎么改、验证结果、风险 |
| `台架验收清单.md` | 编译、核对常量、和 test33 同口径对比、弱磁/MTPA 怎么试 |
| `tools/sim/README.md` | 主机仿真：新旧逐位对比、新功能仿真、ARM 语法检查 |

## 一页速览

- **参数**：
  - 换电机改 `FW/Inc/cfg_motor.h`。
  - 调性能改 `FW/Inc/cfg_ctrl.h`，填带宽 Hz、阻尼。
  - 换驱动板改 `FW/Inc/cfg_board.h`、`Board/`、`FW/Src/hal_sc32r602.c`。
  - 定点增益在 `FW/Inc/fw_config.h` 里自动算。
- **参数按理论取值（0xF20E0003）**：
  - 电流环 300 Hz + dq 解耦
  - 速度环 0.5 Hz / ζ0.7
  - PLL 15 Hz / 0.707
  - 观测器去直流相位补偿
  - 自写防溢出 PI
- **退回原厂参数**：每项都写了原厂值，全部改回去（`tools/sim/legacy_cfg.sh`）就与 test33 逐位相同（仿真 45 组）。
- **默认关闭**：MTPA、弱磁、IPM 有效磁链。
- **固件标识**：`gFluxDiag.buildTag`。
  - 0xF20E0001：重构
  - 0xF20E0002：加 MTPA / 弱磁
  - 0xF20E0003：按理论取参
- **闭源库约束**：
  - `FOC_TypeDef / USER_TypeDef` 的字段布局不能改。
  - `FOC / USER / FastBoot_Para / pwm_gen / eM1_RunSubState` 这些名字不能改。
  - 详见 `FW/Inc/vendor_lib.h`。
- **快环**：ADC 中断，16 kHz，预算 62.5 µs，不许有除法和 sqrt。**慢环**：main 轮询，1 kHz。

## git tag

| tag | 内容 |
|---|---|
| `base-test33` | test33 原样导出 |
| `p1-refactor` | 模块化重构，行为不变 |
| `p2-mtpa-fw` | MTPA / 弱磁 / IPM 有效磁链（默认关），行为仍 = test33 |
| `p3-theory` | 按理论取参（0xF20E0003） |
