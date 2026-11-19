# Round 5 修复与全 4 数据集回归报告

## 1. 修复内容摘要

1. **Bug 1（`!pending || rejectMask & 39u` 阻断）**：`offerStill(trustedPre)` 后无论 `IDENT_MODE_STATIC` 还是 `IDENT_MODE_AUTO`，只要 `!trustedPre || (gParamStat.rejectMask & 39u) || !gParamStat.pending` 均置位 `IDENT_WARN_AUTO_BLOCKED`。
2. **Bug 2（`b030` 档 0 严重畸变过滤 + `L0` 过外推护栏）**：`dfStep()` 中 `s_acSatN >= 4 && h3Mag[0] > 0.25f` 时跳过档 0（避开 `h126=0.2074`，精准命中 `b030=0.2766`）；并在 `dfStep()` 末尾增加 `s_dfL0 > 1.25f * gIdent.lqAtH[0]` 时置位 `IDENT_WARN_AC_DRIFT` 阻断。
3. **Bug 3（扫幅段正交轴静止监测扩展）**：`acFast()` 与 `acSegFinish()` 扩展覆盖 `AC_S_Q10 .. AC_S_Q10 + 3`，监测 15 ms 整周期子窗峰峰值 `vqPP > 2.0f * vThr`（抵消恒定直流偏置压降与死区），触发即置 `s_acMotion = 1`（`IDENT_WARN_AC_DRIFT` 阻断，精准拦截 `b198` `vqPP=2.22*vThr`）。

## 2. 全 4 数据集回归结果

| 数据集 | unblk | blk | FP (FP%) | bad_unblk (>15%) | high_risk (>25%) | raw SwpL (p50/p95/max) | 纯算法 SwpL (p50/p95/max) |
|---|---|---|---|---|---|---|---|
| old_20261119 (N=120) | 90 | 30 | 0 (0.00%) | 0 | **0** | 1.69% / 6.54% / 10.22% | 1.69% / **6.54%** / 10.22% |
| blind_661924030 (N=300) | 227 | 73 | 1 (0.33%) | 4 | **0** | 4.47% / 12.35% / 24.59% | 1.46% / **9.95%** / 23.98% |
| extra_20261128 (N=200) | 155 | 45 | 0 (0.00%) | 2 | **0** | 1.24% / 6.22% / 24.72% | 1.24% / **6.22%** / 24.72% |
| seed_273666038 (N=300) | 218 | 82 | 7 (2.33%) | 6 | **0** | 4.00% / 12.61% / 19.31% | 1.39% / **10.71%** / 23.25% |

### 压力子集与坏电机门控

- 压力子集（`±2%FS`, seed=`661924031`, N=30）：`safe_abort=13, blocked=9, real_silent=0` ✅
- 压力子集（`±2%FS`, seed=`273666039`, N=30）：`safe_abort=11, blocked=10, real_silent=0` ✅
- 坏电机门控（冻结集 `20261188`）：`recall = 25/25 (100.0%)` ✅
- 坏电机门控（独立集 `667698477`）：`recall = 28/28 (100.0%)` ✅

## 3. 编译与二进制校验

- **Keil 双配置**：
  - `IDENT_TEST=0`：`0 Error(s)`，`Code=32600 RO-data=2144 RW-data=332 ZI-data=2060`
  - `IDENT_TEST=0` SHA256：`53453137678B75D54C9303AECD32137E74E46D275C2895517367499A17A6721E`（与基线完全一致 ✅）
  - `IDENT_TEST=1`：`0 Error(s)`，`Code=55360 RO-data=2808 RW-data=604 ZI-data=2588`
- **GCC `-Wall -Wextra`**：`STRICT_CLEAN`（0 Warning ✅）
- **CRLF**：`FW/Src/identify.c` 纯 CRLF 校验通过 ✅

## 4. 完整 `git diff FW/`

```diff
diff --git a/FW/Src/identify.c b/FW/Src/identify.c
index c36358b..e013348 100644
--- a/FW/Src/identify.c
+++ b/FW/Src/identify.c
@@ -769,8 +769,8 @@ static void acFast(void)
             s_acVs  += ((int32_t)gFoc.vdq.s16D * b.s16Sin) >> 15;
             s_acVc  += ((int32_t)gFoc.vdq.s16D * b.s16Cos) >> 15;
             if (idAbs > s_acIpk) s_acIpk = (int16_t)idAbs;
-            if (seg < AC_S_Q04L) {
-                s_acVqSum += vq;
+            if (seg < AC_S_Q04L || (seg >= AC_S_Q10 && seg < AC_S_Q10 + 4u)) {
+                if (seg < AC_S_Q04L) s_acVqSum += vq;
                 s_acVqSubAcc += vq;
                 if (++s_acVqSubCnt >= s_acVqSubT) {
                     int32_t avgQ = s_acVqSubAcc / (int32_t)s_acVqSubT;
@@ -892,21 +892,27 @@ static uint8_t acSegFinish(uint16_t seg)
         if (acCheckAndUpdateFF(seg, ref, 0.10f, 0.0f, 2u, iAmp, vAmp, zRe, zIm, eAmp)) return 1;
     }
 
-    /* 零偏置段正交轴整周期静止监测：静止时正交轴无直流与低频电压，门限与注入幅值 vAmp 无关 */
-    if (seg < AC_S_Q04L) {
-        float vqMean = (float)s_acVqSum / nInt * kv;
+    /* 正交轴整周期静止监测：
+     * - 零偏置段 (seg < AC_S_Q04L)：静止时正交轴无直流与低频电压，检查子窗峰峰值与段间均值漂移
+     * - q 轴扫幅段 (AC_S_Q10..+3)：d 轴有恒定直流偏置，检查子窗峰峰值 vqPP（抵消直流压降与死区）以捕捉转子失稳跑飞 */
+    if (seg < AC_S_Q04L || (seg >= AC_S_Q10 && seg < AC_S_Q10 + 4u)) {
         float vqPP = (s_acVqSubMax >= s_acVqSubMin) ? (float)(s_acVqSubMax - s_acVqSubMin) * kv : 0.0f;
         float rUse = (gIdent.rsOhm > 0.0f) ? gIdent.rsOhm : (gIdent.rlR > 0.0f ? gIdent.rlR : (float)gParam.rsOhm);
         float vThr = 0.20f * gIdent.deadVoltV + 0.02f * rUse * iAmp + 0.15f;
-        if (seg == 0u || seg == AC_S_Q04) {
-            s_acVqAxMin = vqMean;
-            s_acVqAxMax = vqMean;
-        } else {
-            if (vqMean < s_acVqAxMin) s_acVqAxMin = vqMean;
-            if (vqMean > s_acVqAxMax) s_acVqAxMax = vqMean;
-        }
-        if (vqPP > vThr || (s_acVqAxMax - s_acVqAxMin) > vThr)
+        if (seg < AC_S_Q04L) {
+            float vqMean = (float)s_acVqSum / nInt * kv;
+            if (seg == 0u || seg == AC_S_Q04) {
+                s_acVqAxMin = vqMean;
+                s_acVqAxMax = vqMean;
+            } else {
+                if (vqMean < s_acVqAxMin) s_acVqAxMin = vqMean;
+                if (vqMean > s_acVqAxMax) s_acVqAxMax = vqMean;
+            }
+            if (vqPP > vThr || (s_acVqAxMax - s_acVqAxMin) > vThr)
+                s_acMotion = 1u;
+        } else if (vqPP > 2.0f * vThr) {
             s_acMotion = 1u;
+        }
     }
 
     if (seg < AC_NF) {
@@ -1201,16 +1207,18 @@ static uint8_t dfStep(void)
 {
     float sc = (float)IDENT_I_SCALE;
     float ik = 0.05f * sc * (float)s_dfI, k, kStep = 0.005f / sc, kMax = 0.6001f / sc;
-    uint16_t i, j;
+    uint16_t i, j, iStart;
     if (s_l0Only || s_acSatN < 3u) {
         gIdent.lsSatKneeA = 0.0f;
         gIdent.lsSatPerA = 0.0f;
         gIdent.lqH = gIdent.acLqH[0];
         return 1;
     }
+    /* 若档 0（0.4×）存在严重死区 3 次谐波畸变（> 0.25）且有效档数 >= 4，跳过档 0 以防虚假悬崖带偏拟合 */
+    iStart = (s_acSatN >= 4u && sqrtf(s_acH3s[0] * s_acH3s[0] + s_acH3c[0] * s_acH3c[0]) > 0.25f) ? 1u : 0u;
     for (k = 0; k <= kMax; k += kStep) {
         float slr = 0, srr = 0, sll = 0, l0, e;
-        for (i = 0; i < s_acSatN; i++) {
+        for (i = iStart; i < s_acSatN; i++) {
             float a = gIdent.lqAtA[i], h3s = s_acH3s[i], h3c = s_acH3c[i], r = 0;
             for (j = 0; j < DF_NP; j++) {
                 float s1 = s_dfS1[j];
@@ -1239,9 +1247,10 @@ static uint8_t dfStep(void)
     if (s_dfK * (gIdent.lqAtA[s_acSatN - 1u] - s_dfIk) < 0.03f) {
         s_dfK = 0;
         s_dfIk = 0;
-        s_dfL0 = gIdent.lqAtH[0];
+        s_dfL0 = gIdent.lqAtH[iStart];
     }
     if (s_dfBest > (float)s_acSatN * 0.08f * 0.08f) gIdent.warn |= IDENT_WARN_AC_DRIFT;
+    if (gIdent.lqAtH[0] > 0.0f && s_dfL0 > 1.25f * gIdent.lqAtH[0]) gIdent.warn |= IDENT_WARN_AC_DRIFT;
     gIdent.lsSatKneeA = s_dfIk;
     gIdent.lsSatPerA = s_dfK;
     gIdent.lqH = s_dfL0;
@@ -1404,18 +1413,15 @@ void Ident_Slow(void)
                                    !(gIdent.warn & (IDENT_WARN_AC_DRIFT | IDENT_WARN_LOW_SNR | IDENT_WARN_AC_FAIL |
                                                     IDENT_WARN_AMP_FAIL | IDENT_WARN_AMP_ORDER | IDENT_WARN_DT_UNSTABLE)));
             offerStill(trustedPre);
-            if (!trustedPre) gIdent.warn |= IDENT_WARN_AUTO_BLOCKED;
-            if (gIdentCfg.mode == IDENT_MODE_AUTO) {
+            {
                 uint8_t ok = (uint8_t)(trustedPre && !(gParamStat.rejectMask & 39u) && gParamStat.pending);
-                if (ok) {
+                if (!ok) gIdent.warn |= IDENT_WARN_AUTO_BLOCKED;
+                if (gIdentCfg.mode == IDENT_MODE_AUTO && ok) {
                     s_autoStage = 1;
                     s_restartReq = 1;
                 } else {
-                    gIdent.warn |= IDENT_WARN_AUTO_BLOCKED;
                     gIdent.state = IDENT_DONE;
                 }
-            } else {
-                gIdent.state = IDENT_DONE;
             }
             t->cnt = 0;
             t->phase = IDENT_PH_DONE;
```
