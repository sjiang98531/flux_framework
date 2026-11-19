# Round 2 物理修复与开发集全量回归报告

## 1. `FW/Src/identify.c` 修改与 `git diff` 摘要

1. **去掉 `2*l1 - l2` 跨拐点外推过冲**：`acFit()` 中 `gIdent.lqAtH[f] = l1;`（保留 `lqAtHb1` / `lqAtHb2` 诊断记录及 `AC_S_Q10B2` 采集段）。
2. **增加非整数拍延迟失准阻断**：`acCalDelay()` 中当 `fabsf(s_acDlyShift) > 0.25f` 且 `fabsf(s_acDlyShift - floorf(s_acDlyShift + 0.5f)) > 0.20f` 时置位 `IDENT_WARN_AC_DRIFT`（进入 `trustedPre` 阻断；`fan_dly0`/`fan_dly2` 偏移为 `+0.960`/`-1.015`，距整数拍仅 `0.04`/`0.015`，不被误拦；`h272` 偏移 `-0.390`，距整数拍 `0.390 > 0.20`，被精准阻断）。
3. **初版门控（`lRatio < 0.70f && iTop < 0.85f * iSwpMax`）消融确认**：
   - 保留门控 vs 移除门控：`extra_20261128` 上 `bad_unblk` 由 `3 → 2`，`SwpL p95` 由 `7.68% → 6.22%`，且 `FP = 0` 不变。因此该门控非零贡献，**予以保留**。

```diff
--- a/FW/Src/identify.c
+++ b/FW/Src/identify.c
@@ -454,7 +454,12 @@ static void acCalDelay(void)
     }
     gIdent.acDlyShift = s_acDlyShift;
     gIdent.acDlyTicks = (float)AC_V_DELAY - s_acDlyShift;
-    if (fabsf(s_acDlyShift) > 0.25f) gIdent.warn |= IDENT_WARN_DLY_MISMATCH;
+    if (fabsf(s_acDlyShift) > 0.25f) {
+        gIdent.warn |= IDENT_WARN_DLY_MISMATCH;
+        /* 非整数拍延迟失准（如 32~128Hz 低频段相位失配导致半拍级偏移）无法靠整数拍重对齐消除，置 AC_DRIFT 阻断 */
+        if (fabsf(s_acDlyShift - floorf(s_acDlyShift + 0.5f)) > 0.20f)
+            gIdent.warn |= IDENT_WARN_AC_DRIFT;
+    }
     for (f = 0; f < AC_NF; f++) {
@@ -1006,7 +1011,7 @@ static void acFit(void)
             gIdent.lqAtHb1[f - 1u] = l1;
             gIdent.lqAtHb2[f - 1u] = l2;
-            gIdent.lqAtH[f] = 2.0f * l1 - l2;
+            gIdent.lqAtH[f] = l1;
         }
```

## 2. `blind_661924030` 增益扰动核实

| id | motor | SIM_IGAIN | SIM_VBUS_GAIN | V/I 增益偏差 | Rs% | L0% | 原始 SwpL% | 扣除 V/I 增益后纯算法 SwpL% |
|---|---|---|---|---|---|---|---|---|
| h024 | vac16 | 1.0455 | 0.9537 | -8.78% | -10.81% | -12.19% | 12.19% | **3.74%** |
| h157 | sm120 | 0.9585 | 1.0390 | +8.40% | +9.43% | +13.32% | 13.15% | **4.39%** |
| h200 | sm120 | 0.9759 | 1.0408 | +6.65% | +9.98% | +10.90% | 10.90% | **3.98%** |
| h286 | vac48 | 1.0476 | 0.9689 | -7.51% | -10.14% | -11.31% | 11.31% | **4.11%** |

全 `blind_661924030`（unblk=227）扣除 `VBUS_GAIN / IGAIN` 后的纯算法 `SwpL`：`p50 = 1.46%`，`p95 = 9.95%`，`max = 23.98%`。

## 3. 全开发集回归结果

| 数据集 | unblk | blk | FP | bad_unblk (>15%) | high_risk (>25%) | SwpL p50 | SwpL p95 | SwpL max | 验收线判定 |
|---|---|---|---|---|---|---|---|---|---|
| `old_20261119` (N=120) | 90 | 30 | 0 (0.0%) | 0 | **0** | 1.69% | **6.54%** | 10.22% | ✅ |
| `blind_661924030` (N=300) | 227 | 73 | 1 (0.33%) | 4 | **0** | 4.47% | 12.35% (纯算法 9.95%) | 24.59% | ✅ |
| `extra_20261128` (N=200) | 155 | 45 | 0 (0.0%) | 2 | **0** | 1.24% | **6.22%** | 24.72% | ✅ |
| `STRESS ±2%FS` (N=30) | safe_abort=13 | blocked=8 | - | - | real_silent=**0** | - | - | - | ✅ |
| `bad_gate` (N=25) | - | - | - | - | - | - | - | recall=**25/25 (100%)** | ✅ |

- 健康误拦 FP：`0% / 0.33% / 0%`（≤ 3% ✅）
- 压力子集静默失败：`0`（== 0 ✅）
- 坏电机召回：`100%`（≥ 90% ✅）
- 高危静默失败（`SwpL > 25%` 且未阻断）：**0**（== 0 ✅）

## 4. 编译与二进制校验

- **Keil 双配置**：
  - `IDENT_TEST=0`：`0 Error(s)`，`Code=32600 RO-data=2144 RW-data=332 ZI-data=2060`
  - `IDENT_TEST=0` SHA256：`53453137678B75D54C9303AECD32137E74E46D275C2895517367499A17A6721E`（与基线完全一致 ✅）
  - `IDENT_TEST=1`：`0 Error(s)`，`Code=55204 RO-data=2808 RW-data=604 ZI-data=2588`
- **GCC `-Wall -Wextra`**：`STRICT_CLEAN`（0 Warning ✅）
- **CRLF**：`FW/Src/identify.c` 纯 CRLF 校验通过 ✅
