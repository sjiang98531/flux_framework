# Round 6 电感反增物理护栏与全 5 数据集回归报告

## 1. 全部 5 数据集（1220 台）`lqAtH[top]/lqAtH[0]` 分布

- 健康未阻断样本（N=889）：`p50 = 0.8362`, `p95 = 0.9803`, `p99 = 1.0034`, **`max = 1.0653`**（仅 1 例 >1.05：`s20261119_047` = 1.0653）
- `n158` (`fan`, 28.36%)：`lqAtH[top]/lqAtH[0]` = **`1.2011`**（远高于健康上界 `1.0653`）
- `n116` (`fan48`, 20.47%)：`lqAtH[top]/lqAtH[0]` = `0.7728`（正常饱和，非反增）
- 因此在 `acFit()` 中设定 `lTop > 1.10f * gIdent.lqAtH[0]` 置位 `IDENT_WARN_AC_DRIFT` 阻断，健康误拦增量为 **0**。

## 2. 全 5 数据集回归结果

| 数据集 | unblk | blk | FP (FP%) | bad_unblk (>15%) | high_risk (>25%) | raw SwpL (p50/p95/max) | 纯算法 SwpL (p50/p95/max) |
|---|---|---|---|---|---|---|---|
| old_20261119 (N=120) | 90 | 30 | 0 (0.00%) | 0 | **0** | 1.69% / 6.54% / 10.22% | 1.69% / **6.54%** / 10.22% |
| blind_661924030 (N=300) | 227 | 73 | 1 (0.33%) | 4 | **0** | 4.47% / 12.35% / 24.59% | 1.46% / **9.95%** / 23.98% |
| extra_20261128 (N=200) | 155 | 45 | 0 (0.00%) | 2 | **0** | 1.24% / 6.22% / 24.72% | 1.24% / **6.22%** / 24.72% |
| seed_273666038 (N=300) | 218 | 82 | 7 (2.33%) | 6 | **0** | 4.00% / 12.61% / 19.31% | 1.39% / **10.71%** / 23.25% |
| seed_550745933 (N=300) | 215 | 85 | 1 (0.33%) | 4 | **0** | 3.91% / 12.79% / 24.79% | 1.41% / **7.99%** / 21.49% |

### 压力子集与坏电机门控

- 压力子集 `661924031` (N=30)：`safe_abort=13, blocked=9, real_silent=0` ✅
- 压力子集 `273666039` (N=30)：`safe_abort=11, blocked=10, real_silent=0` ✅
- 压力子集 `550745934` (N=30)：`safe_abort=7, blocked=10, real_silent=0` ✅
- 坏电机门控 `20261188`：`recall = 25/25 (100.0%)` ✅
- 坏电机门控 `667698477`：`recall = 28/28 (100.0%)` ✅
- 坏电机门控 `889340222`：`recall = 25/25 (100.0%)` ✅

## 3. 编译与二进制校验

- **Keil 双配置**：
  - `IDENT_TEST=0`：`0 Error(s)`，`Code=32600 RO-data=2144 RW-data=332 ZI-data=2060`
  - `IDENT_TEST=0` SHA256：`53453137678B75D54C9303AECD32137E74E46D275C2895517367499A17A6721E`（与基线完全一致 ✅）
  - `IDENT_TEST=1`：`0 Error(s)`，`Code=55408 RO-data=2808 RW-data=604 ZI-data=2588`
- **GCC `-Wall -Wextra`**：`STRICT_CLEAN`（0 Warning ✅）
- **CRLF**：`FW/Src/identify.c` 纯 CRLF 校验通过 ✅

## 4. 完整 `git diff FW/`

```diff
diff --git a/FW/Src/identify.c b/FW/Src/identify.c
index e013348..fc7aba3 100644
--- a/FW/Src/identify.c
+++ b/FW/Src/identify.c
@@ -1039,14 +1039,16 @@ static void acFit(void)
         gIdent.warn |= IDENT_WARN_AMP_ORDER;
     }
 
-    /* 扫幅不足 × 深饱和门控：当饱和拟合有效（s_acSatN >= 3）但顶档电感相对 L0 已明显下降，
-     * 且顶档电流未达到 2.5×IDENT_I_SCALE（说明矢量/电压限幅压低了扫幅范围），
-     * 说明拟合区间不足以覆盖真实饱和深度 -> 降级为仅 L0（阻断 L(i) 提交），防止外推偏差大的曲线静默放行 */
+    /* 扫幅不足 × 深饱和门控 & 电感反增物理护栏：
+     * 1. 铁芯饱和时电感随电流只可能非增，若顶档电感反超档 0 超过 10% 必为偏轴/失锁，置 AC_DRIFT 阻断
+     * 2. 当饱和拟合有效（s_acSatN >= 3）但顶档电感相对 L0 已明显下降且顶档电流未达 2.5×IDENT_I_SCALE，降级仅 L0 */
     if (!s_l0Only && s_acSatN >= 3u && gIdent.lqAtH[0] > 0.0f) {
         float lTop = gIdent.lqAtH[s_acSatN - 1u];
         float lRatio = (gIdent.lqAtH[0] > 1e-12f) ? lTop / gIdent.lqAtH[0] : 1.0f;
         float iTop = gIdent.lqAtA[s_acSatN - 1u];
         float iSwpMax = 2.5f * (float)IDENT_I_SCALE;
+        if (lTop > 1.10f * gIdent.lqAtH[0])
+            gIdent.warn |= IDENT_WARN_AC_DRIFT;
         if (lRatio < 0.70f && iTop < 0.85f * iSwpMax) {
             s_l0Only = 1u;
             gIdent.warn |= IDENT_WARN_AMP_ORDER;
```
