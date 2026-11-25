# Round 7 同频往复摆动（2·f0 正交电压解调）监测与全 6 数据集回归报告

> **历史记录（2026-11-25 注）**：本文针对 `05337d7` 之前的辨识实现（2-DOF 交流注入、`acFit` 饱和拟合、阻断门控），这些代码已删除，文中函数、行号与验收结论不适用于当前代码。当前实现与验证方式见仓库根目录 `REVIEW_STATUS.md`。

## 1. 全部 6 数据集（1520 台）正交轴静止判据分布对比

在 `AC_S_Q10 .. AC_S_Q10+3` 段统计 1093 台健康未阻断样本 vs `f231` (`mid16`, 36.53%)、`b198` (`mid48`)、`n158` (`fan`)：

| 特征量 | 健康 p50 | 健康 p95 | 健康 p99 | 健康 max | `f231` | `b198` | `n158` |
|---|---|---|---|---|---|---|---|
| `vqPP / vThr`（旧整周期均值峰峰值） | 0.0104 | 0.1529 | 0.3851 | 1.7885 | 0.9547 (漏) | 2.2166 | 1.1954 |
| `vqAc1 / vThr`（f0 基波解调） | 0.0997 | 0.5050 | 0.9081 | 2.1997 | 0.4275 | 0.1033 | 3.9561 |
| `vqAc1 / vAmp`（f0 基波 / 激励幅值） | 0.0058 | 0.0258 | 0.0410 | 0.0893 | 0.0115 | 0.0035 | 0.1058 |
| `vqAc2 / vThr`（2·f0 倍频 / 死区门限） | 0.1699 | 0.3483 | 0.4427 | 1.3819 | 1.3337 | 1.5201 | 0.5492 |
| **`vqAc2 / vAmp`（2·f0 倍频 / 激励幅值）** | **0.0067** | **0.0115** | **0.0162** | **0.0314** | **0.0357** | **0.0468** | 0.0121 |

- 物理解释：同频往复摆动时，反电势正交投影 $\omega_r \sin\theta$ 及交叉耦合调制在正交轴上表现为 **$2 f_0$ 倍频分量**（且与激励电压 `vAmp` 成比例），在整周期平均 `vqPP` 中被积分抵消。
- 判据选定：`vqAc2 > 0.033f * vAmp`，在 1093 台健康样本上**零新增误拦**，同时精准拦截 `f231` (0.0357) 与 `b198` (0.0468)。

## 2. 全 6 数据集回归结果（1520 台 + 120 压力 + 106 坏电机）

| 数据集 | unblk | blk | FP (FP%) | bad_unblk (>15%) | high_risk (>25%) | raw SwpL (p50/p95/max) | 纯算法 SwpL (p50/p95/max) |
|---|---|---|---|---|---|---|---|
| old_20261119 (N=120) | 90 | 30 | 0 (0.00%) | 0 | **0** | 1.69% / 6.54% / 10.22% | 1.69% / **6.54%** / 10.22% |
| blind_661924030 (N=300) | 227 | 73 | 1 (0.33%) | 4 | **0** | 4.47% / 12.35% / 24.59% | 1.46% / **9.95%** / 23.98% |
| extra_20261128 (N=200) | 155 | 45 | 0 (0.00%) | 2 | **0** | 1.24% / 6.22% / 24.72% | 1.24% / **6.22%** / 24.72% |
| seed_273666038 (N=300) | 218 | 82 | 7 (2.33%) | 6 | **0** | 4.00% / 12.61% / 19.31% | 1.39% / **10.71%** / 23.25% |
| seed_550745933 (N=300) | 215 | 85 | 1 (0.33%) | 4 | **0** | 3.91% / 12.79% / 24.79% | 1.41% / **7.99%** / 21.49% |
| blind_192012398 (N=300) | 225 | 75 | 3 (1.00%) | 1 | **0** | 4.30% / 10.09% / 21.45% | 1.49% / **6.94%** / 16.16% |

### 压力子集与坏电机门控

- 压力子集 `661924031` (N=30)：`safe_abort=13, blocked=9, real_silent=0` ✅
- 压力子集 `273666039` (N=30)：`safe_abort=11, blocked=10, real_silent=0` ✅
- 压力子集 `550745934` (N=30)：`safe_abort=7, blocked=10, real_silent=0` ✅
- 压力子集 `192012399` (N=30)：`safe_abort=14, blocked=5, real_silent=0` ✅
- 坏电机门控 `20261188`：`recall = 25/25 (100.0%)` ✅
- 坏电机门控 `667698477`：`recall = 28/28 (100.0%)` ✅
- 坏电机门控 `889340222`：`recall = 25/25 (100.0%)` ✅
- 坏电机门控 `372939614`：`recall = 28/28 (100.0%)` ✅

## 3. 编译与二进制校验

- **Keil 双配置**：
  - `IDENT_TEST=0`：`0 Error(s)`，`Code=32600 RO-data=2144 RW-data=332 ZI-data=2060`
  - `IDENT_TEST=0` SHA256：`53453137678B75D54C9303AECD32137E74E46D275C2895517367499A17A6721E`（与基线完全一致 ✅）
  - `IDENT_TEST=1`：`0 Error(s)`，`Code=55616 RO-data=2808 RW-data=612 ZI-data=2588`
- **GCC `-Wall -Wextra`**：`STRICT_CLEAN`（0 Warning ✅）
- **CRLF**：`FW/Src/identify.c` 纯 CRLF 校验通过 ✅

## 4. 完整 `git diff FW/`

```diff
diff --git a/FW/Src/identify.c b/FW/Src/identify.c
index fc7aba3..066ccd5 100644
--- a/FW/Src/identify.c
+++ b/FW/Src/identify.c
@@ -281,6 +281,7 @@ static int32_t  s_acVs, s_acVc, s_acIs, s_acIc, s_acEs, s_acEc;
 static int32_t  s_acI3s, s_acI3c;
 /* 零偏置段正交轴整周期子窗静止监测（消除注入频率耦合，无盲区捕捉风车） */
 static int32_t  s_acVqSum, s_acVqSubAcc, s_acVqSubMin, s_acVqSubMax;
+static int32_t  s_acVq2s, s_acVq2c;
 static uint32_t s_acVqSubT, s_acVqSubCnt;
 static float    s_acVqAxMin, s_acVqAxMax;
 static int16_t  s_acVph;
@@ -675,6 +676,7 @@ static void acSegStart(uint16_t seg, uint8_t isRetry)
     s_acOverI = 0;
     s_acVs = s_acVc = s_acIs = s_acIc = s_acEs = s_acEc = 0;
     s_acI3s = s_acI3c = 0;
+    s_acVq2s = s_acVq2c = 0;
     s_acVph = (int16_t)(gIdent.deadVoltV * (float)(32768.0 / UDC_MAX));
     COMPILER_BARRIER();    /* 编译器屏障：确保上面的非 volatile 写对快环可见后再清 s_acDone */
     s_acDone = 0;
@@ -770,7 +772,13 @@ static void acFast(void)
             s_acVc  += ((int32_t)gFoc.vdq.s16D * b.s16Cos) >> 15;
             if (idAbs > s_acIpk) s_acIpk = (int16_t)idAbs;
             if (seg < AC_S_Q04L || (seg >= AC_S_Q10 && seg < AC_S_Q10 + 4u)) {
-                if (seg < AC_S_Q04L) s_acVqSum += vq;
+                if (seg < AC_S_Q04L) {
+                    s_acVqSum += vq;
+                } else {
+                    SinCos_TypeDef b2 = FocMath_SinCos((int16_t)(((phiOld + s_acDly) * 2u) >> 16));
+                    s_acVq2s += (vq * b2.s16Sin) >> 15;
+                    s_acVq2c += (vq * b2.s16Cos) >> 15;
+                }
                 s_acVqSubAcc += vq;
                 if (++s_acVqSubCnt >= s_acVqSubT) {
                     int32_t avgQ = s_acVqSubAcc / (int32_t)s_acVqSubT;
@@ -910,8 +918,11 @@ static uint8_t acSegFinish(uint16_t seg)
             }
             if (vqPP > vThr || (s_acVqAxMax - s_acVqAxMin) > vThr)
                 s_acMotion = 1u;
-        } else if (vqPP > 2.0f * vThr) {
-            s_acMotion = 1u;
+        } else {
+            /* 同频往复摆动时 ω_r·sin(θ) 与互感调制在正交电压上产生 2·f0 分量，整周期平均 vqPP 会将其抵消 */
+            float vqAc2 = 2.0f * sqrtf((float)s_acVq2s * (float)s_acVq2s + (float)s_acVq2c * (float)s_acVq2c) / nInt * kv;
+            if (vqPP > 2.0f * vThr || vqAc2 > 0.033f * vAmp)
+                s_acMotion = 1u;
         }
     }
```
