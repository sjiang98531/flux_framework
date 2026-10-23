# tools/sim —— 主机仿真与验证

在 PC 上把固件的快环/慢环、闭源库模型、电机模型接起来跑，用来：

1. **证明重构没改行为**：test33 原代码和当前代码跑同一组输入，每拍状态哈希逐位比较。
2. **在上台架之前试新功能**：弱磁、MTPA、IPM 观测器。
3. **ARM 32 位语法检查**：clang 按 Cortex-M0+ 检查 Keil 工程里的全部 .c。

需要 Linux 或 WSL：gcc、clang、python3（画图要 matplotlib）、git。

## 用法

```bash
tools/sim/build.sh            # 编 build/sim_old（test33，从 tag base-test33 导出）、sim_new、fz_*、gain_check
tools/sim/run_equiv.sh        # 45 组逐位对比，最后一行 TOTAL 45 FAIL 0
tools/sim/build/gain_check    # 物理规格算出的增益 vs test33 常量
tools/sim/armchk/check.sh     # ARM 语法检查，最后一行 "checked 42 files, 0 with diagnostics"

# 单次仿真：sim_new <工况> <csv> <hash文件|""> <秒数> <IPM电机0/1>
tools/sim/build/sim_new 0 out.csv "" 28
# 工况 0：293 -> 1172 -> 600 -> -400 -> 停 -> 1000；1：1000 rpm 加载；2：1800 -> 2600；3：1000；4：启停循环；5：±1000 硬换向
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
