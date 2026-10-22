#ifndef FLUX_CONFIG_H
#define FLUX_CONFIG_H
#include "mc_customize.h"
/* Flux observer + FluxPLL parameters only. Startup, speed loop and current
 * loop parameters stay in mc_customize.h at the vendor values.
 * Model bases (FLUX_V_BASE / FLUX_WN_HZ / FLUX_PSI_N) and FLUX_ISR_BUDGET are
 * in mc_customize.h. Experiment history: 实验记录.md. */

/* 固件标识。约定：每次实质改动 +1，接上 J-Scope 读 gFluxDiag.buildTag 就能确认板上是哪一版。
 * 0xF10E0001  首版起一直用到 test16（常量没变，那段时间区分不了版本）
 * 0xF10E0002  代码收敛 + 回退事件触发 + DTC 关闭（= test14 行为）
 * 0xF10E0003  test18：e 提前计算（诊断）+ ALIGN_TIME 1 ms -> 0.5 s
 * 0xF10E0004  test19：IQ_STARTUP 0.5 A -> 1.0 A
 * 0xF10E0005  test20：SPEED_KP 8000 -> 15000（误以为是原厂值，原厂实为 8000）
 * 0xF10E0006  test21：速度 PI 积分器改用实测 Iq 预置
 * 0xF10E0007  test22：回到原厂架构——关注入/释放/冻结，删 test21 预置；IQ_STARTUP 0.5、ALIGN_TIME 0.001、SPEED_KP 8000 回原厂值
 * 0xF10E0008  test23：BeginStartup 时 psi 初值设成 |psi_f|@theta_hat，消除假启动
 * 0xF10E0009  test24：稳态角度偏移扫描（临时）
 * 0xF10E000A  test25：controlComp 置 0 + 低速 300 rpm 复扫
 * 0xF10E000B  test26：扫描代码去掉 ISR 里的除法，300 rpm 重测
 * 0xF10E000C  test27：删扫描代码；删没人读的计算（导数支路、fluxToEmf 混合、normGainQ10、phaseError/valid）
 * 0xF10E000D  test28：代码瘦身，行为不变——删注入/PLL 冻结/DTC/controlComp 等死代码及未用宏和结构体字段
 * 0xF10E000E  test29：PLL 速度下限加在输出转速上（Startup 专用，修启动卡死）
 * 0xF10E000F  test30：速度斜坡加速度前馈（非原厂），SPEED_ACC_FF_IQ = 200/501 = 0.40 A，Spin 入口积分器预置扣掉前馈
 * 0xF10E0010  test31：Startup 期间上报给原厂的转速加 1000 rpm/s 加速度限制（防 PLL 追相冲高导致假切 Spin）
 * 0xF10E0011  test32：真对齐——FAST_BOOT 模式对齐改为 0.5 A、1.0 s（原 0.125 A、1 ms）；PLL/ψ 种子改在对齐电流矢量方向（CmdTheta±90°）
 * 0xF10E0012  test33：回到 test31（撤销 test32 的 1 s 对齐与 ±90° 种子），保留"鉴相用 PLL 当前角度"一行修正 */
#define FLUXPLL_BUILD_TAG 0xF10E0012UL

/* Below this bus voltage the voltage model is not run (observer held in reset). */
#define FLUX_BUS_MIN_V       100.0f

/* Rotor flux. Nameplate said 0.154; measured 0.141 (test13):
 *   |e|/omega at 978 rpm = 0.1409 Wb (instantaneous), integrated |psi| 0.1386 Wb.
 * Used for the NLFO target and the Startup seed. Re-measure for another motor. */
#define FLUX_NOMINAL_WB      0.141f

/* FluxPLL gains, "Q31 speed increment per err LSB" (see FluxPLL.c).
 * Designed for Kd = 6674 (0.154 Wb), wn = 15 Hz, zeta = 0.707; with the
 * measured 0.141 Wb the real loop is wn ~14 Hz, zeta ~0.68. Every successful
 * start used exactly these two constants; they are tunable. */
#define FLUXPLL_KP_Q31_PER_ERR 25583
#define FLUXPLL_KI_Q31_PER_ERR 106

/* NLFO 径向修正强度（借鉴 SguanFOC_Library Sguan_NLFO.c，MIT）。
 *   corr = psi_r * (|psi_f|^2 - |psi_r|^2) / |psi_f|^2 >> FLUX_NLFO_SH
 * 一阶收敛 tau：SH=8 约 8 ms，SH=9 约 16 ms（默认），SH=10 约 32 ms。 */
#define FLUX_NLFO_SH         9

/* NLFO 修正项饱和值（Q15）。32768 = 在 |psi| 偏离 41% 处饱和，实际只防溢出。
 * test11 收紧到 8192 明显更差（|psi| 摆幅 5016~10136 -> 0~14838），不要收紧。 */
#define FLUX_NLFO_FQ_LIMIT   32768
/* test31: acceleration limit on the speed reported to the vendor code during
 * Startup (StateMachine.c, M1_RunStartupFast). 0.5 A startup current gives about
 * 250 rpm/s; real starts in test28_2/test29 never needed more than this limit,
 * the PLL catch-up spikes of test30 were 5000-11000 rpm/s.
 * STEP is the Q31 speed change allowed per fast-loop tick. */
#define FLUX_STARTUP_ACC_LIMIT   1000.0f   /* rpm/s */
#define FLUX_STARTUP_ACC_STEP    ((int32_t)(FLUX_STARTUP_ACC_LIMIT / SPEED_MAX / PWM_FREQ * 2147483648.0f))
#endif
