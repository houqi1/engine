# E1 体素薄壳圆柱：自重与切口应力转移

> 日期：2026-09-13。断裂关闭。不进入 E2 刚体下落。
> 前置：仓库内已有原型四项修复与 E0（`ve_structure_core`、`BlastRuntime`、空 `StructureWorld`、固定步通知）。本轮开始前再跑过 E0/P4。

## 几何（占用与结构同一份 fine）

| 量 | 值 |
|---|---|
| 轴 | +Y |
| R, t, H | 2.0, 0.2, 6.0 m |
| fine | 0.1 m（壁 2 格，高 60 格） |
| 栅格 | 64³ fine，轴 (32,32)，物体 coarse=4×1.6 m |
| 聚合 | 2（对照 1） |
| ρ | 1000 kg/m³ |
| 运动类型 | Static，与地面分对象 |
| 锚固 | 显式：fy∈[0,2) 的壁体素 → world bond，不是接触 |

270° 切口（固定按钮，不是刷子）：保留 fy∈[0,2) 锚固层，删 fy∈[2,10)（约 0.8 m，一层 P1 层高）且方位角不在 [0, π/2) 的壁体素。90° 壁带连到顶。不全高挖。

## 实测（Release，2026-09-13）

占用：14640 fine，质量 14640 kg（离散圆柱，不等于解析壳）。

| 项 | agg=2 | 说明 |
|---|---|---|
| 节点 / 键 / world | 3000 / 6660 / 100 | 高度 30 档、周向 16 档，不是单节点 |
| 空腔 | 0 个体素落在内半径内 | 六邻域壳连通 |
| 完整 Σmg vs Ry | W=143618 N，Ry=143625 N，rel=4.4×10⁻⁵ | 收敛，好于 0.1% 目标 |
| 完整壁带 max | 1.21×10⁵ Pa | |
| 切口后壁带 | 1.21×10⁵ → 3.29×10⁶ Pa（约 27×） | 头测预算内 conv=0，lin≈111 |
| 双倍密度 | 应力比 2.002 | 线性 |
| 原点平移 | relR=1×10⁻⁷，relS=6×10⁻⁷ | |
| agg=1 | 14640 节点 | 仍有 30 层 × 16 周向档 |

强度窗口（定性，切口未进容差前不当硬阈值）：完整 ~0.12 MPa，切口高一个数量级。1×10³ Pa 会在完整时就过；5×10⁷ Pa 完整和切口都远低于。E2 前要用收敛后的切口应力再锁 S。

## 操作

```
cmake --build build --config Release --target blast_e1_tests vulkan_engine_voxel
build\Release\blast_e1_tests.exe
```

主程序 ImGui「Structure (E1 cylinder, fracture off)」：Spawn / Cut 270 / Double density / Solver iters / Stress colors（0–2 MPa 固定色标）。打开 Simulate 才走 `onPhysicsTick` 自重。不生成断裂命令。

## 未做

- 硬阈值断键、actor→体素碎块（E2）
- 接触冲量（E3）
- 任意连续挖洞重建（E4）
