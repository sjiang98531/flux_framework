/* ============================================================================
 * @file    svpwm.c
 * @brief   空间矢量脉宽调制，接口说明见 svpwm.h
 * ==========================================================================*/
#include "svpwm.h"
#include "compiler.h"

Svpwm_TypeDef gSvpwm;

/* 清除状态，记录 PWM 半周期计数（比较值满量程）。上电调用一次（HAL_InitParams） */
void Svpwm_Init(Svpwm_TypeDef *u, int16_t halfPeriod)
{
    u->Alpha = 0;
    u->Beta = 0;
    u->CompA = 0;
    u->CompB = 0;
    u->CompC = 0;
    u->Mode = SEVENMODE;
    u->Sector = 0;
    u->N_halfPeriod = halfPeriod;
}

/* 各扇区参数表（下标 1..6 = 扇区号；0 和 7 不使用）：
 *   v1/v2：t1、t2 取 {X, Y, Z} 中的哪一个，n1/n2 = 1 表示取负；
 *   c：A/B/C 相比较值取三个时间点中的第几个（七段式 {ta, tb, tc}，五段式 {t0, P - a2, P}，两种模式下标相同）；
 *   p1/p2：母线电流 = (i[p1]*t1 - i[p2]*t2) >> 15，i = {ia, ib, ic}。
 * 采用查表而非 switch：Keil 会将 switch 编译为调用 Flash 中的跳转辅助函数，快环放在 RAM 时会产生额外的 Flash 访问。 */
typedef struct { uint8_t v1, n1, v2, n2, c[3], p1, p2; } SvSector;
static const SvSector s_svTab[7] = {
    { 0, 0, 0, 0, { 0, 0, 0 }, 0, 0 },
    { 2, 0, 1, 0, { 1, 0, 2 }, 1, 2 },     /* 1：t1 = Z,  t2 = Y  */
    { 1, 0, 0, 1, { 0, 2, 1 }, 0, 1 },     /* 2：t1 = Y,  t2 = -X */
    { 2, 1, 0, 0, { 0, 1, 2 }, 0, 2 },     /* 3：t1 = -Z, t2 = X  */
    { 0, 1, 2, 0, { 2, 1, 0 }, 2, 0 },     /* 4：t1 = -X, t2 = Z  */
    { 0, 0, 1, 1, { 2, 0, 1 }, 1, 0 },     /* 5：t1 = X,  t2 = -Y */
    { 1, 1, 2, 1, { 1, 2, 0 }, 2, 1 },     /* 6：t1 = -Y, t2 = -Z */
};

/* αβ 电压 -> 三相比较值（快环每拍）。原理：
 *   六个有效电压矢量将平面分为 6 个扇区，目标电压矢量由所在扇区两侧的有效矢量按作用时间加权合成，
 *   剩余时间施加零矢量。t1、t2 为两个有效矢量各占半周期的比例，由 X/Y/Z 按扇区选取。
 * 母线电流重构：有效矢量作用期间，母线电流等于某一相（或某一相的负值）电流，按 t1、t2 加权即为本周期平均母线电流 */
RAMFUNC void Svpwm_Calc(Svpwm_TypeDef *u, const MCFLIB_3_ABC_T_S16 *iabc, int16_t *ibusRec)
{
    /* X = Vβ，Y = (Vβ + √3Vα)/2，Z = (Vβ − √3Vα)/2（28378 = √3/2 的 Q15） */
    int32_t r3 = (u->Alpha * 28378) >> 15;
    int32_t xyz[3], tt[3], i3[3];
    int32_t half, vb, vc, t1, t2, P, a1, a2, t0;
    const SvSector *k;
    uint8_t n = 0;

    xyz[0] = u->Beta; half = xyz[0] >> 1;
    xyz[1] = half + r3; xyz[2] = half - r3;
    vb = r3 - half; vc = -(r3 + half);              /* 另外两相的参考电压（只用其符号） */
    P = u->N_halfPeriod;

    /* 扇区号：由三个参考电压的符号组合成 1..6；0 / 7（三者同号，仅在零电压附近）时三相均输出半周期 */
    if (xyz[0] >= 0) n = 1;
    if (vb >= 0) n += 2;
    if (vc >= 0) n += 4;
    u->Sector = n;
    if (n == 0 || n == 7) {
        u->CompA = u->CompB = u->CompC = (int16_t)(P >> 1);
        if (u->Mode == SEVENMODE || u->Mode == FIVEMODE) *ibusRec = 0;
        return;
    }
    k = &s_svTab[n];

    /* 两个有效矢量的作用时间（占半周期的比例，Q15） */
    t1 = k->n1 ? -xyz[k->v1] : xyz[k->v1];
    t2 = k->n2 ? -xyz[k->v2] : xyz[k->v2];
    /* 换算为计数时四舍五入：截断会使两个有效矢量平均各短 0.5 count，实际输出电压系统性地小于指令
     * （低速时相对反电势不可忽略）。观测器直接使用指令电压（flux_obs.c），指令与实际输出必须无偏。
     * 电流环将 |V| 限制在 MAX_DUTY 电压圆内（t1 + t2 <= 0.97），进位不会使 t0 变负 */
    a1 = (P * t1 + 16384) >> 15;
    a2 = (P * t2 + 16384) >> 15;
    t0 = P - a1 - a2;               /* 零矢量时间 */

    /* 比较值按扇区分配到三相 */
    if (u->Mode == SEVENMODE) {
        tt[0] = t0 >> 1;            /* 七段式：零矢量两端平分 */
        tt[1] = tt[0] + a1;
        tt[2] = tt[1] + a2;
    } else if (u->Mode == FIVEMODE) {
        tt[0] = t0;                 /* 五段式：一相钳位在 P（不开关），零矢量时间全部放在另一端 */
        tt[1] = P - a2;
        tt[2] = P;
    } else {
        u->CompA = u->CompB = u->CompC = (int16_t)(P >> 1);
        return;
    }
    u->CompA = (int16_t)tt[k->c[0]]; u->CompB = (int16_t)tt[k->c[1]]; u->CompC = (int16_t)tt[k->c[2]];
    /* 母线电流 = 两个有效矢量各自对应的相电流按作用时间加权 */
    i3[0] = iabc->s16A; i3[1] = iabc->s16B; i3[2] = iabc->s16C;
    *ibusRec = (int16_t)((i3[k->p1] * t1 - i3[k->p2] * t2) >> 15);
}
