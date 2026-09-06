# 可破坏体素物理（下一套引擎路径）

活文档。目标：砸开以后每一块都能自己掉、互相撞、能堆住。数据模型对齐 Tuxedo **下一套引擎**（8³ 空不存、物理和画面同一套占用、子步进），碰撞格子对齐你们现在看见的形状（**fine = 0.1 m**）。

Status: **方案，未实现。** 碰撞从第一版就用角/棱分类，不做「先测所有表面格」的过渡。

Related shipped code:

- `src/scene/VoxelScene.h` / `.cpp` — `VoxelObject`、coarse 页表、全局 brick pool、`getFine` / `setFineCpu`、`flushDirtyPages`
- `shaders/voxel_dda.comp` — 物体循环 + coarse DDA + 8³ + 2³；`usampler3D grids[2]`
- `src/render/VoxelRenderer.cpp` — hull + compute DDA
- `docs/nested-brick-voxels.md` — 空 brick 不租页（已上）
- `docs/object-order-ess.md` — occupancy hull（已上，多物体可见性要接到这里）

不覆盖：结构塌方（2026 原型，不是「能掉」的前提）、Teardown 的 OBB 片元光栅、硬件 RT、火/烟/水。

---

## 1. 要做成什么样

做完应同时成立：

1. 挖穿一块，掉下来的每一坨都是独立物体，自己掉。
2. 砸在地上、砸在别的渣上，能堆住、能停住。
3. 再打还能再碎。
4. 空 coarse 不租 brick；锯子从一盒积木中间过，两边的砖都在。
5. 物理碰到的实心格 = 画面 DDA 打到的 fine 格。
6. 第 3 个及以后的物体能画出来。

不做：楼没锯断自己塌；不改成「一物体一张稠密 3D 图」。

干活顺序不能倒：**先能掉 → 再能画很多块 → 再能砸开 → 再能互撞能堆。**

---

## 2. 尺度（已对齐，物理跟 fine）

看见的形状是 fine。物理必须认同一层，否则锯齿会变成更方的盒子。

| 层 | 边长 | 角色 |
|---|---|---|
| **fine** | **0.1 m** | 看见的形状、碰撞、质量、断裂连通 |
| micro | 0.2 m | 8³ 占用位图，空 brick 不存；跳空、页分配 |
| coarse | 1.6 m | 页表（`material` + `brickPage`） |

`VoxelScene::kGameplayVoxelMeters = 0.1f`，`kDefaultVoxelSize = 1.6f`。  
`gameplayVoxelSize()` 返回 fine。

世界 64³ ≈ 102 m。地面 2 格 coarse，顶面 3.2 m。

下一套引擎公开的是「8³ 当物理和渲染占用」。你们多了一层 2³ 颜色。本方案：**占用账以 fine 为准**；micro 只是「这 8 个 fine 有没有东西」的打包，不能单独当碰撞格。

---

## 3. 积木怎么存

实心的才占地方，空气不存。物理问「这里有没有东西」、画面问「这里画不画」，看 **同一本 fine 占用账**。颜色字（page 里 4096 个 `0xAARRGGBB`）不进碰撞。

```text
Shape  = 现有 VoxelObject
         局部页表 cells[] + 刚体变换 + 名下的 brick page
Chunk  = brick page 前 16 word（512 bit micro）+ fine 表
         空 page 不分配
Body   = 质量 / 速度 / 休眠，挂 1 个或多个 Shape
```

查询只走已有 `getFine(o, coarse, micro, fine)`。  
fine 变了但 micro 没变：物理仍脏（形状变了）。  
只改颜色 alpha/RGB、占用没变：物理不脏。

运行时禁止 `rebuildVoxels()`（会 `waitIdle` 拆整个池）。只允许关卡重载。局部编辑走 `flushDirtyPages`。

---

## 4. 角 / 棱分类（第一版就要有）

Teardown 公开技术说明：两个多面体碰到，一定经过角或棱；盒子着地只应留下四个底角接触，不是整面一千个点。内部格永不测。面不对面。

分类在 **fine 格、6 邻接** 上做。对每个占用 fine，看 X/Y/Z 三轴是否「两侧都有邻居」：

| 被夹住的轴数 | 类别 | 碰撞 |
|---|---|---|
| 3 | Inside | 永不测 |
| 2 | Face | 不对 Face |
| 1 | Edge | 测：edge–edge，以及对方的 corner |
| 0 | Corner | 测：corner–corner、corner–edge、以及对方占用（先对 corner/edge 集合） |

每格 1 字节，CPU 上、不进 GPU。加载和每次占用变脏后重建。可顺手建 `corners[]` / `edges[]` 下标列表，窄相只扫列表。

**第一版窄相就必须：**

1. 双方世界 AABB → 对方局部，求交集盒（先 coarse，再收到 fine）。
2. 只遍历盒内的 **Corner 和 Edge**。
3. 每个候选当球，半径 `0.5 * 0.1 = 0.05 m`。球心变到对方格子，查中心格占用，**再查 6 邻**（最多偏半格）。无占用 = 没撞上。没有 closest-feature，没有 clip。
4. 接触按占用物体上命中格的空邻面分组（6 个离散方向）。每组按 Box3D `b3ReduceManifoldPoints` / Gregorius 2015 留最多 4 个点：最深、接触平面上最远、三角形面积最大、四边形外侧面积最大。完好的盒子砸地，期望大约 **四个底角**。不要按穿透深度全局截断。

不存成对 contact manifold（体素没有凸接触斑；状态可以挂在 Body 上）。

3D 下「谁测谁」Gustafsson 说过可能记混 2D/3D。本方案锁死为上表；若堆叠不稳，只许改配对规则并写进本节，不许退回「所有表面格」。

---

## 5. 砸开以后怎么办

连通只认 **六面**。角碰、棱碰不算连；那种结构一损坏或变成动态就会散。

每个 Shape 在编辑前视为单连通。挖完从被删格的邻居出发，看还能不能走回主体。没断：**O(1) 离开**，不 flood 整块。

断了：

1. flood 出各分量。
2. **最大块留在原来的 `objects_[i]`**（下标稳定）。
3. 其余：新 `VoxelObject`（紧包围盒，`gridSize` 8 或 16，不是 64）+ 新 Body。速度 `v + w × (com' - com)`。
4. 占用 fine **少于 8** 的渣直接删。
5. 裁掉全空的边（包围盒收紧）。

积木怎么分（两种都要写，不能只搬家）：

- 锯子走在 **coarse 边上**：整页 `brickPage` **改挂** 到新物体，不拷贝。
- 锯子从 **一页 8×8×8 中间** 过：新开一页，按 bit 把占用和颜色劈成两份。只搬家会丢砖或粘错。

---

## 6. 怎么掉、怎么撞、怎么堆

每块是刚体。断开的带着原来的速度飞。碰撞在 CPU 上，fine 对 fine，不丢给显卡。

时钟（子步进，不是「一大步里反复推 8 次」）：

```text
一大步 h = 1/60 秒
每大步 nSub = 6 个小步     // 公开「新引擎用 6」；不稳再加
hSub = h / nSub
每帧最多 2 个大步
重力 (0, -9.81, 0)
密度先用木头 600 kg/m³
质量 = 占用 fine 数 × 0.1³ × 密度
静的 invMass = 0
```

每个小步：重力 → 宽相 → 角/棱窄相 → 对接触做 **1 次** 法向+摩擦冲量 → 积分位置。  
睡着：`|v|`、`|w|` 低于阈值持续约 0.5 s（阈值未公开，先用 `0.05 m/s`，用验收调）。

宽相：Shape AABB。物体少时可 N²，接口按 BVH 写。静的一棵树，页表脏了局部更新。

并行（后做，结构先留好）：接触生成按 pair 可并行；求解按接触图分岛。小岛并行，一大堆粘着的渣先单线程。不要第一版就上图着色。

楼没锯断就还站着。不要做结构力学。

---

## 7. 画面能画第 3 块（砸开的硬前置）

`grids[2]` 只能画地 + 一个动态物。物理可以有 100 个 Body，看不见也对不了。

```text
coarsePool SSBO：所有 Shape 的页表紧排
GpuVoxelObject.voxelOffset = 该 Shape 在 pool 的起点
gridSize 可变（地 64，碎块 8/16）
brick slabs 不动，全局 page 索引
```

Shader `readCell`：

- 世界可暂时留 `grids[0]`：`是世界` → `texelFetch`
- 其它：`coarsePool[offset + x + yN + zN²]`

不要 `grids[非均匀下标]`（MoltenVK）。最终态：世界也进 `coarsePool`，去掉特权 3D 图。

物体数组先按 **256** 开，满了拒绝 new、删最小渣。

像素不要 `for objectCount`。hull 带物体编号，compute 只 traverse 命中的 id（接 `object-order-ess.md`）。碎块经常 >200 再做深度 bin。

---

## 8. 预算

没有这一层，「任意破坏」会自己把机器吃满。

| 项 | 第一版 |
|---|---|
| Shape 上限 | 256（以后 1024） |
| 残渣 | 占用 fine < 8 删除 |
| 睡着 | 不进窄相 |
| 页用光 | 删最小渣，不许崩 |
| 合并 | 后做：贴着、同材质、都睡着 |

---

## 9. 接到现有这一帧

`main_voxel.cpp` 已是：相机 → `update` → 刷子 → 画。改成：

```text
刷子 / 爆炸
  → 标记哪些 fine 变了
  → 该拆就拆（可能 new Shape）
  → 只对脏 Shape 重建分类 / 质量
  → 子步进物理
  → 写回每个 VoxelObject 的 position / rotation
  → uploadObjectTransforms + flushDirtyPages
  → 画
```

同一帧不要对同一 page 又刷又拆无栅栏。ImGui 勾「Simulate」；关掉 = 现在的 spinner 空转。

---

## 10. 执行步骤

### 提交 A — 时钟 + 一箱落地（角/棱从一开始就有）

新建：

- `src/physics/PhysicsWorld.h` / `.cpp` — 累加器、`step`
- `src/physics/RigidBody.h` — Body
- `src/physics/Classify.cpp` — fine 角/棱/面/内，输出 `corners[]` / `edges[]`
- `src/physics/VoxelCollide.cpp` — 交集盒 + 只测角棱 + 球 + 6 邻占用
- `src/physics/Solver.cpp` — 每小步 1 次冲量

改：

- `CMakeLists.txt` — `VE_VOXEL_SOURCES` 加上上述 cpp
- `VoxelScene::update` — 累加器；Simulate 开时关掉 `time * spinSpeed`
- `VoxelRenderer.cpp` — Simulate 勾选

数据：地 = `objects_[0]` 静；`objects_[1]` 动。质量用 `getFine` 计数。不改 `grids[2]`。

验收：

- 勾 Simulate，箱子从空中落到 3.2 m 地面上，弹一两下睡着。
- 侧放：接触点大约四个底角，不是整面。
- 关掉 Simulate，回到现在的转法。
- 分类：实心盒 8 个 corner；挖掉一个角，corner 列表变。

### 提交 B — 能画很多块（可与 A 并行）

改：

- `VoxelScene` — `coarsePool` CPU + GPU；`packObjectPool` 按物体拼接 `cells[]`
- `GpuVoxelObject.voxelOffset` — 页表起点，不是 `grids[]` 下标
- `voxel_dda.comp` / `voxel_dda_coarse.comp` — `readCell` 两路
- `VoxelRenderer.cpp` — descriptor、物体数 ≠ 2
- hull 带 `objectId`；DDA 去掉全物体循环

验收：不跑物理，CPU spawn 30 个小 `VoxelObject`（grid 8），各自转，格子不错位；关掉能回收。

### 提交 C — 砸开（依赖 B）

新建 `src/physics/Fracture.cpp`：预连通、flood、搬走整页、劈页、trim、< 8 fine 删除。

改 `handleEditInput`：`setFineCpu` 成功后 `maybeFracture`，禁止 `rebuildVoxels`。

验收：

- 一刀沿 coarse 边：页数几乎不增。
- 一刀穿过一页：页数 +1，两边占用互补、不丢 bit。
- 挖穿动态物，每坨都是列表里的新物体，能看见、能掉（接 A 的求解器）。
- 最大块仍是原来的下标。

### 提交 D — 互撞、堆、上限（依赖 C）

宽相 BVH；窄相仍是第 4 节角/棱。岛、休眠、Shape 上限 256。

验收：8～32 个 0.1 m 盒子能堆成柱、停住不往下沉；打穿一面，渣能堆能睡，再挖会醒；超上限丢最小渣，不崩。

---

## 11. 和现有文件

| 事 | 动哪里 |
|---|---|
| 占用 | 已有 `getFine` / `setFineCpu`，物理只用它们 |
| 脏页 | 已有 `flushDirtyPages` |
| 变换 | 已有 `uploadObjectTransforms` |
| 两物体上限 | `kGridTexCount`、`grids[2]`、`readCell` |
| 每像素扫全体 | `voxel_dda.comp` 的 `for objectCount` |
| spinner 空转 | `VoxelScene::update` 里 `angleAxis(time * spinSpeed)` |
| 刷子 | `handleEditInput` → 改格后接 `maybeFracture` |
| brick 布局 | **不重写** |

积木仓库已经是「空不存」。要补的是：角/棱碰撞、多物体能画、劈开、子步进堆叠。

---

## 12. 待完善（下一轮讨论）

写进文档是为了钉死，不是已经拍板：

1. **世界 coarse 最终是否也进 `coarsePool`，去掉 `grids[0]`。** 第一版可混合；最终更干净。
2. **Body 原点：** 网格中心 vs 质心。第一版建议网格中心，惯量绕它算，少一套偏移。
3. **静→动：** 完全脱离地面/世界才 dynamic（Teardown 玩法），还是一断开就掉。建议前者。
4. **`nSub = 6`、休眠 0.05 m/s / 0.5 s、残渣 8 fine、每法线组最多 4 个接触（Box3D 缩减）** — 都是经验值，用验收调，改了要写回本节。
5. **3D 角/棱配对** 若堆不稳，只改第 4 节表格。
6. **一个 Body 多个 Shape**（铰链前）第一版不做，每个 Body 一个 Shape。
7. **深度 bin / 硬件 RT** 在碎块经常 >200 或开始打阴影再做。

继续完善时改的是本文，不要另起一份。
