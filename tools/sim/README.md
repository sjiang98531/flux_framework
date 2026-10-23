# tools/sim —— 主机仿真与验证

在 PC 上把固件的快环/慢环、闭源库模型、电机模型接起来跑，用来：

1. **证明重构没改行为**：test33 原代码和"当前代码 + test33 参数"（`legacy_cfg.sh`）跑同一组输入，每拍状态哈希逐位比较。
2. **比较参数/功能的效果**：`eval.py` 测阶跃、加载、低速角度误差、高速；`startup_eval.py` 测 128 组冷启动。
3. **在上台架之前试新功能**：弱磁、MTPA、IPM 观测器。
4. **ARM 32 位语法检查**：clang 按 Cortex-M0+ 检查 Keil 工程里的全部 .c。

需要 Linux 或 WSL：gcc、clang、python3（画图要 matplotlib）、git。

## 用法

```bash
tools/sim/build.sh            # 编 sim_old（test33，从 tag base-test33 导出）、sim_new（当前）、
                              #    sim_legacy（当前代码 + test33 参数）、fz_*、gain_check(_legacy)
tools/sim/run_equiv.sh        # test33 vs sim_legacy 45 组逐位对比，最后一行 TOTAL 45 FAIL 0
tools/sim/build/gain_check_legacy   # test33 参数算出的增益 vs test33 常量（13 项 OK）
python3 tools/sim/eval.py tools/sim/build/sim_legacy tools/sim/build/sim_new      # 性能对比
python3 tools/sim/startup_eval.py tools/sim/build/sim_legacy tools/sim/build/sim_new
tools/sim/armchk/check.sh     # ARM 语法检查，最后一行 "checked 42 files, 0 with diagnostics"

# 单次仿真：sim_new <工况> <csv> <hash文件|""> <秒数> <IPM电机0/1>
tools/sim/build/sim_new 0 out.csv "" 28
# 工况 0：293 -> 1172 -> 600 -> -400 -> 停 -> 1000；1：1000 rpm 加载；2：1800 -> 2600；3：1000；4：启停循环；
#      5：±1000 硬换向；6：1172 -> 600 -> 1172；7：300/150/80 rpm 平台；8：1000 rpm 突加 0.5 N·m
# 环境变量改电机模型：SIM_TD（死区等效 s）SIM_TH0（初始机械角 rad）SIM_W0（初始转速 rpm，飞车）
#                     SIM_FRIC（摩擦 N·m）SIM_VBUS SIM_LOAD SIM_LOAD_END

# 改配置编变体（不改工作区）：
tools/sim/variant.sh fw "" 's/#define CTRL_FW_ENABLE          0/#define CTRL_FW_ENABLE          1/'
tools/sim/build/sim_fw 2 fw.csv "" 16
```

CSV 每 1 ms 一行，列含义：

| 列 | 含义 |
|---|---|
| t、main、run | 时间、主状态、子状态 |
| cmd_rpm、filt_rpm、pll_rpm | 指令、滤波转速、PLL 转速 |
| true_rpm | 电机模型的真实转速 |
| iq_A、id_A、iqcmd_A、idcmd_A | 实测与指令电流 |
| theta_err_deg | θ̂ 减去真实转子角 |
| psi_mag | |ψr|，Q15 |
| vd、vq | 电流环输出电压 |

## 文件

| 文件 | 内容 |
|---|---|
| vendor_lib.c、vendor_tables.h | 闭源库 SOC_MCI_2R_V102.lib 的 C 版本。按 test33 AXF 反汇编逐条翻译；正余弦表、开方表直接从 AXF 取。**只用于仿真**，固件仍链接原厂 .lib |
| plant.c/h | PMSM dq 模型 + 逆变器（板级相序、死区误差）+ ADC（量化、噪声、零点），默认参数 = 本风机（501 rpm/s/A 辨识值、风阻按 1172 rpm/0.292 A 标定） |
| sim_main.c | 主循环、工况、CSV、每拍哈希 |
| glue_old.c / glue_new.c | 接 test33 / 新代码 |
| fuzz.c | 观测器 + PLL 随机输入对比 |
| shim/ | 主机上代替器件头文件、armcc 关键字 |
| armchk/ | clang ARM 检查用的最小 libc 头和大小写转接 |

**局限**：

- 电机模型不是你的电机，冷启动成功率、低速打滑之类的绝对数字不能和台架比。
- 闭源库模型是反汇编翻译的。新旧对比两边用的是同一个模型，所以它不影响"是否一样"的结论。

## 压力测试开关（环境变量，默认都不开，不影响逐位对比）

| 变量 | 含义 |
|---|---|
| `SIM_PWMDLY=1` | 比较值晚一个 PWM 周期生效（影子寄存器在周期边界装载的硬件） |
| `SIM_NOISE=4` | ADC 噪声幅值，LSB（默认 1） |
| `SIM_TD=1.0e-6` | 等效死区，秒（默认 0.2 µs，板子名义 1.2 µs） |
| `SIM_KR` / `SIM_KL` / `SIM_KPSI` / `SIM_KJ` | 电机实际 Rs / L / ψf / J = 标称 × 系数（控制器仍按标称） |
