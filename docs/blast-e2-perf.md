# E2 分层性能：探针导出优化对照

> 日期：2026-09-13。Release / MSVC。同一 E1 圆柱。未改碰撞、未换求解器、未降体素精度。
> 数值路径与判据保持：完整/切口应力、Hold/Fail、未收敛但 >2S 仍收集候选。

## 这轮改了什么

1. `copyBondProbes()` 三次线性遍历（邻接首次命中、solver bond 首次映射胜出）。旧实现保留为 `copyBondProbesLegacy()`，补丁在 `third_party/nvblast/patches/apply_p1_extstress.py`。
2. 每 tick 只导出一次；`bondMeta[sdk]` 做线性统计与候选；着色复用同一快照。
3. `commitStructureSplits` 应用候选快照，不再 `applyHardThreshold` 二次导出。

## 命令

```
cmake --build build --config Release --target blast_probe_export_tests blast_e2_perf vulkan_engine_voxel
build\Release\blast_probe_export_tests.exe
build\Release\blast_e2_perf.exe
build\Release\vulkan_engine_voxel.exe --e2-perf
```

## 正确性

`blast_probe_export_tests`：完整、断一条未 split、split 后（7 actors / 208 键）新旧导出逐项相同（下标、端点、health、拉压剪、力、力矩）。空输出 / 截断容量 / 满容量一致。Hold 无候选，Fail 候选集与旧 `collectOverstressed` 一致。每 tick `exports=1`。

E1 / E2 / P1 / P3 / graph regression 仍过。

## 头测（6660 键完整 / 5961 键切口）

生产路径 `probe + stats + candidates`（一次导出）：

| | 优化前 median | 优化后 median | p95 | max |
|---|---:|---:|---:|---:|
| 完整 200，copyBondProbes | 49.4 | **0.067** | 0.17 | 0.17 |
| 完整 200，导出+统计+候选 | ~60–105 | **0.55**（隔离）/ **0.08**（solveInstance） | | |
| 切口 200，同上 | ~54–116 | **0.53** / **0.12** | | |
| exports / tick | 2（断裂开） | **1** | | |

完整 200 生产 tick：probe 0.048 + stats 0.010 + cand 0.021 = **0.079 ms**，低于 1–2 ms 验收。隔离线性统计仍自己建表（0.47 ms）；生产用 `bondMeta` 后统计 0.01 ms。

A（求解）未改：完整收敛 ~0.3–0.5 ms；切口未收敛 8–27 ms，现为下一项主因。

## 引擎 `--e2-perf`（200 iters，不画 DDA）

| 阶段 | A | B probe | C stats | C cand | D | struct-cb | update | exports |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 完整站立（前） | 0.32 | 36.5 | 5.8 | 0 | 4.9 | 44.2 | 49.7 | |
| 完整站立（后） | 0.47 | **0.09** | **0.02** | 0 | 6.7 | **0.59** | **8.1** | 1 |
| 切口断裂关（前） | 7.6 | 28.9 | 4.3 | 0 | 4.8 | 41.2 | 46.2 | |
| 切口断裂关（后） | 15.8 | **0.08** | **0.02** | 0 | 7.3 | **15.9** | 21.9 | 1 |
| 直到第一次 split（前） | 7.4 | 27.8 | 4.5 | 28.5 | 33.4（commit 29） | 68.8 | 73.1 | |
| 直到第一次 split（后） | 12.6 | **0.09** | **0.02** | **0.04** | 6.6（commit **0.6**） | **12.7** | 17.9 | 1 |
| split 后碎块（后） | 9.5 | 0.08 | 0.01 | 0 | 5.2 | 9.5 | 14.4 | 1 |

原先 split 当帧 ~29 ms 主要是 `applyHardThreshold` 里又一次 O(B²) 导出，不是 occupancy GPU。提交改走候选后 commit 中位 0.6 ms。

## 两种状态（实测，不是 FPS 承诺）

- **完整已收敛：** CPU 物理路径约 A 0.5 + 导出消费 0.1 + D ~5–7 ≈ **6–8 ms / tick**。不含 DDA，不能当成 144 FPS。
- **切口未收敛：** 探针修好后，求解 8–27 ms 成为主项，要另做收敛/迭代调度。
- **分裂瞬间：** 旧 29 ms 已随第三次导出消失；若以后 occupancy pack 再冒出来，单独测。

未改 2S 未收敛仍断裂的判据。多 actor 后提交会按候选归属应用到各个 actor（与「显示的候选 = 实际断键」一致）；首次分裂前仍是单 actor。
