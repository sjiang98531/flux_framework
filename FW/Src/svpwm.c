/* 空间矢量调制，见 svpwm.h。 */
#include "svpwm.h"

Svpwm_TypeDef pwm_gen;

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

void Svpwm_Calc(Svpwm_TypeDef *u, const MCFLIB_3_ABC_T_S16 *iabc, int16_t *ibusRec)
{
    /* X = Vβ，Y = (Vβ + √3Vα)/2，Z = (Vβ − √3Vα)/2（28378 = √3/2 的 Q15） */
    int32_t r3 = (u->Alpha * 28378) >> 15;
    int32_t X = u->Beta, half = X >> 1;
    int32_t Y = half + r3, Z = half - r3;
    int32_t vb = r3 - half, vc = -(r3 + half);      /* 另外两相的参考电压（只用符号） */
    int32_t t1 = 0, t2 = 0, P, a1, a2, t0, ta, tb, tc;
    int32_t ia = iabc->s16A, ib = iabc->s16B, ic = iabc->s16C;
    uint8_t n = 0;

    /* 扇区号：三个参考电压的符号拼成 1..6 */
    if (X >= 0) n = 1;
    if (vb >= 0) n += 2;
    if (vc >= 0) n += 4;

    /* 两个有效矢量的作用时间（占半周期的比例，Q15） */
    switch (n) {
    case 1: t1 = Z;  t2 = Y;  break;
    case 2: t1 = Y;  t2 = -X; break;
    case 3: t1 = -Z; t2 = X;  break;
    case 4: t1 = -X; t2 = Z;  break;
    case 5: t1 = X;  t2 = -Y; break;
    case 6: t1 = -Y; t2 = -Z; break;
    default: break;
    }
    P  = u->N_halfPeriod;
    a1 = (P * t1) >> 15;
    a2 = (P * t2) >> 15;
    t0 = P - a1 - a2;               /* 零矢量时间 */
    ta = t0 >> 1;                   /* 七段式：零矢量两端平分 */
    tb = ta + a1;
    tc = tb + a2;

    /* 比较值按扇区分配到三相；母线电流 = 两个有效矢量各自对应的相电流按作用时间加权 */
    if (u->Mode == SEVENMODE) {
        switch (n) {
        case 1: u->CompA = tb; u->CompB = ta; u->CompC = tc; *ibusRec = (int16_t)((ib * t1 - ic * t2) >> 15); break;
        case 2: u->CompA = ta; u->CompB = tc; u->CompC = tb; *ibusRec = (int16_t)((ia * t1 - ib * t2) >> 15); break;
        case 3: u->CompA = ta; u->CompB = tb; u->CompC = tc; *ibusRec = (int16_t)((ia * t1 - ic * t2) >> 15); break;
        case 4: u->CompA = tc; u->CompB = tb; u->CompC = ta; *ibusRec = (int16_t)((ic * t1 - ia * t2) >> 15); break;
        case 5: u->CompA = tc; u->CompB = ta; u->CompC = tb; *ibusRec = (int16_t)((ib * t1 - ia * t2) >> 15); break;
        case 6: u->CompA = tb; u->CompB = tc; u->CompC = ta; *ibusRec = (int16_t)((ic * t1 - ib * t2) >> 15); break;
        default: u->CompA = u->CompB = u->CompC = (int16_t)(P >> 1); *ibusRec = 0; break;
        }
    } else if (u->Mode == FIVEMODE) {
        /* 五段式：一相钳在 P（不开关），零矢量时间全给另一端 */
        switch (n) {
        case 1: u->CompA = P - a2; u->CompB = t0;     u->CompC = P;      *ibusRec = (int16_t)((ib * t1 - ic * t2) >> 15); break;
        case 2: u->CompA = t0;     u->CompB = P;      u->CompC = P - a2; *ibusRec = (int16_t)((ia * t1 - ib * t2) >> 15); break;
        case 3: u->CompA = t0;     u->CompB = P - a2; u->CompC = P;      *ibusRec = (int16_t)((ia * t1 - ic * t2) >> 15); break;
        case 4: u->CompA = P;      u->CompB = P - a2; u->CompC = t0;     *ibusRec = (int16_t)((ic * t1 - ia * t2) >> 15); break;
        case 5: u->CompA = P;      u->CompB = t0;     u->CompC = P - a2; *ibusRec = (int16_t)((ib * t1 - ia * t2) >> 15); break;
        case 6: u->CompA = P - a2; u->CompB = P;      u->CompC = t0;     *ibusRec = (int16_t)((ic * t1 - ib * t2) >> 15); break;
        default: u->CompA = u->CompB = u->CompC = (int16_t)(P >> 1); *ibusRec = 0; break;
        }
    } else {
        u->CompA = u->CompB = u->CompC = (int16_t)(P >> 1);
    }
    u->Sector = n;
}
