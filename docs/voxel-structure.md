# 体素结构坍塌（约束冲量 / 截面内力）

活文档。在已有连通断裂和质心刚体之上，给还连着的楼补上「自重会把键拉断」。

Status: **P0–P2 已接上。** 鬼影准静态（每子步从静止加重力，λ 暖启动）、整岛睡、死键锯邻面再 `maybeFracture`。地面接触不叫醒楼。P3 加密不做。

Related:

- `docs/voxel-physics.md` — 占用、角/棱接触、连通断裂、`maybeFracture`、质心 Body
- `src/physics/Fracture.cpp` — 六面剥岛
- `src/physics/PhysicsWorld.cpp` — 6 子步 SI、流形 ≤4 点
- `src/physics/Solver.cpp` — 接触 Sequential Impulse、`lambdaN` / `lambdaT`

对齐：市售 Teardown 只有连通断裂；2026 原型才是应力。求解器受 Erin Catto **Box2D / Box3D** 启发（Soft Step、软约束、图着色、整岛睡），键按体素裁，不是把 Box3D 当通用引擎嵌进来。

---

## 0. 目标

做完应同时成立：

1. 楼还连着时，自重、偏心、挖承重墙能把键拉断（结构破坏）。
2. 键断之后走现有 `maybeFracture`：剥岛 → 新 Shape + Body，质心刚体掉、能堆。
3. 完好盒子砸地仍是大约四个底角，结构键不参与接触。
4. 睡着的完好楼结构 CPU ≈ 0；挖 / 撞才醒。
5. 一栋占用几百～几千 coarse 的测试楼，结构步能进 60 fps 预算（M2 上先单线程按颜色扫）。

**不做**

- 连续体 FEM（`Ku = F` 分解）、相场裂纹、近场动力学
- 每个 0.1 m fine 一根键
- 每个 coarse 一个独立 Box3D Body 再焊
- 把鬼影位移画成弯曲（那是软体，不是 Teardown 楼）
- 结构求解器自己积分塌落轨迹（掉落必须交给现有刚体）
- N=64 存储、hull、GPU flood、MergeShape

市售 Teardown 做不到「细柱撑不住楼」。本方案加上 \(\Phi\) 才能出现「还连着但撑不住」。

塌的观感来自 **断键顺序 + 刚体掉落**，不是来自格子在晃。

---

## 1. 和现有系统的分工

| 层 | 谁负责 | 现状 |
|---|---|---|
| 占用 / 渲染 | `VoxelObject`、coarsePool、occupied coarses vis、DDA | 已有 |
| 连通断裂 | `maybeFracture`：六面 flood，剥岛，`<8` fine 删 | 已有 |
| 刚体掉落 | Body 质心、角/棱接触、6 子步 SI、流形 ≤4 点 | 已有 |
| **结构** | coarse 邻接键上的 \(N,V,M\)，\(\Phi \ge 1\) 破键 | **本文要做** |

结构是断裂的**前置**：还连着但撑不住 → 删键（等价于锯开那些邻面）→ 再剥岛。不要第二套掉落。

```text
编辑 / 爆炸 / 砸
    ↓
占用变了 → 局部重键
    ↓
醒着的岛：鬼影 TGS 解键（重力 + 锚）
    ↓
Φ ≥ 1 → 删键
    ↓
maybeFracture(虚拟切口) → 新 Body
    ↓
接触 SI（只对动态块 / 地面）
    ↓
vis / coarsePool 刷新（已有 flushObject）
```

连锁：掉下来的块砸到还站着的结构上，接触冲量再砸一批键。

---

## 2. 数据模型

### 2.1 节点 `BondNode`

每个**占用 coarse** 一个节点，挂在某个静 Shape 上（第一版只给 `invM == 0` 且连到地面的楼建键）。

```text
BondNode
  shapeIndex, coarse (ivec3)
  mass          // ρ × 实心体积（fine 计数 × 0.1³）
  Idiag         // 该格砖块惯量，轴对齐近似即可
  u, theta      // 鬼影位移/转角，不写进 VoxelObject
  v, w          // 鬼影速度，重阻尼
  anchored      // 贴 objects_[0] 地面 → 6 自由度锁死
```

画面仍用 `VoxelObject.position`（格子中心）和刚体 `Body.x`（质心）。**不画 \(u\)。**

### 2.2 键 `Bond`

正交 6 面邻接一条键 = Box3D weld 的体素版。

```text
Bond
  a, b          // BondNode 下标
  axis          // 0/1/2 = x/y/z
  A, I, J, L    // 截面：默认 a = voxelSize，按两格实心 fine 比例缩小
  kn, kv, kt, km
  lambda[6]     // N, V1, V2, T, M1, M2 暖启动
  strength      // σt, σc, τu, Mu 来自材质
  alive
```

短柱必须按 **Timoshenko**（带剪切），不要 Euler（剪切锁死）。

\[
k_N = \frac{EA}{L},\quad
k_V = \frac{GA}{\alpha L},\quad
k_T = \frac{GJ}{L},\quad
k_M = \frac{EI}{L}
\]

\(L = a = 1.6\,\mathrm{m}\)（coarse）。\(A = a^2\) 再乘两格实心比例的较小值，挖空的 coarse 键更弱。

纯轴力桁架在正交 6 面网格上是**运动学机构**，侧向一碰就数值炸掉。键必须带剪和弯。

### 2.3 岛 `Island`（抄 Box3D）

- 节点 + 键 + 与该楼相关的接触
- 碰到 / 键还在 → **并岛**（快）
- 键断了、接触没了 → 标「该拆」，**每步最多拆一个岛**（拆岛贵）
- **整岛睡**，不是单个节点 / Body 睡
- `objects_[0]` 永远是锚，不进岛的动态集合

完好、没人挖：整栋一个睡着的岛，结构 TGS 直接 skip。挖一刀 / 砸一下：只唤醒那一岛。塌开以后：落下的块自己成小岛。

### 2.4 约束图（颜色桶，抄 Box3D）

键、接触都是边。同色边不共享动态节点 → 同色可无锁并行；换色才同步。

- 颜色数先 8～12 + 1 个 overflow
- **静–动键 / 接触颜色在前**（先解和地面的锚，减少把楼推穿地面）
- 邻居特别多的节点进 overflow，串行

第一版即使单线程，也**按颜色分桶再扫**，不要按网格乱序扫。

### 2.5 材质（先两档）

| | 木头（刷子 1） | 石头（刷子 2） |
|---|---|---|
| \(E\) | 低 | 高 |
| \(\sigma_t\) | 低 | **很低** |
| \(\sigma_c\) | 中 | 高 |
| \(\tau_u\) | 低 | 中 |
| 密度 | 已有 600 kg/m³ | 可 2200 |

石头纯压能撑，一弯 / 一拉就裂。木头拉、剪都差。

---

## 3. 应力怎么算

### 3.1 鬼影 TGS（Box3D Soft Step）

时钟与刚体同一套，不要另开隐式 FEM：

```text
h = 1/60
nSub = 6
hSub = h / 6
每帧最多 2 个大步
接触：每子步 8 次 SI      // 已有 kContactIters
键：每子步 1～2 次 TGS    // 不要 20
```

键约束（每条 6 个标量），软约束：

\[
\Delta\lambda = -\frac{\dot C + \beta C / h_\mathrm{sub}}{K_\mathrm{eff} + \mathrm{CFM}}
\]

- \(\beta\)：位置误差推进度（可低于接触的 `kBaumgarte = 0.2`，键更准静态）
- **CFM > 0**：键比接触软，细柱顶楼不抖、λ 才能当应力
- 暖启动：\(\lambda \leftarrow\) 上一子步
- 拉力 \(\lambda_N\) 可夹紧到 \(A\sigma_t \cdot h_\mathrm{sub}\)；压力上限大得多

重力只加在鬼影节点上，**不推动静 Shape**。锚节点 \(v = w = u = \theta = 0\)（DOF elimination，不要惩罚弹簧除非必要）。

积分鬼影后加重阻尼（准静态）。\(u\) 只用于出内力，不同步到渲染。无限刚焊（CFM = 0、β 很大）在体素短柱上会锁死、抖、细柱数值爆炸。

### 3.2 冲量 → 截面力 → 应力

\[
N = \frac{\lambda_N}{h_\mathrm{sub}},\quad
V = \frac{\sqrt{\lambda_{V1}^2 + \lambda_{V2}^2}}{h_\mathrm{sub}},\quad
M = \frac{\sqrt{\lambda_{M1}^2 + \lambda_{M2}^2}}{h_\mathrm{sub}}
\]

\[
\sigma_N = \frac{N}{A},\quad
\tau = \frac{V}{A},\quad
\sigma_M = \frac{M \cdot (a/2)}{I}
\]

调试热图画 \(\Phi\)（或 \(\sigma_N,\tau,\sigma_M\)），对齐 Dennis 2026 先画应力再塌。

### 3.3 破坏准则 \(\Phi\)

用**梁截面相关**，比单点 von Mises 更适合这种短柱：

\[
\Phi_{NM} =
\begin{cases}
N / (A\sigma_t) + |M| / M_u & N > 0 \text{（拉）} \\
|N| / (A\sigma_c) + |M| / M_u & N < 0 \text{（压）}
\end{cases}
\qquad
\Phi = \Phi_{NM} + |V| / (A\tau_u)
\]

\(\Phi \ge 1\) → `alive = false`。

只比轴力：细柱顶楼不会断。必须带 \(V\) 和 \(M\)。

接触破坏（锤 / 子弹）仍用冲量阈值，**不用** \(F = J / \Delta t\)（帧率一变破坏就变）：

\[
J_\mathrm{contact} > J_\mathrm{mat}
\]

两条路都变成「切口」，喂给 `maybeFracture`。

### 3.4 破键后必须再解

断一条不要停。同子步或下一子步再 TGS 1～2 轮：邻键 λ 还在（暖启动），荷载改道，高 \(\Phi\) 带扩展。这才是塌，而不是切了一刀。

每子步破键上限（先 8～32 条），避免一帧删光整栋导致接触爆炸。

欧拉压杆特征值屈曲第一版不做；可用「压力过大也判坏」近似。连续体裂纹尖端、相场分叉不做。

---

## 4. 接到现有断裂

键 \((A,B)\) 死了 ≡ 这两个 coarse 的邻面被锯开。

```text
收集本步死去的键 → 两侧 coarse 上贴面的 fine 列表
当成 deletedAbsFines
VoxelScene::maybeFracture(shapeIndex, deletedAbsFines)
```

之后完全复用现有逻辑：

- 多种子 BFS + 并查集，没断 O(1) 离开
- 小岛剥走；最大块留在 `objects_[i]`（小屋不连地面时也不能把整栋都剥走，否则 vis/DDA 只剩地面）
- `objects_[0]` 上连着地面（coarse y<2）的块留下，禁止把地面 64×2×64 整板卷进 flood
- 新块 `gridSize` 8/16，质心、\(v + \omega \times r\)
- fine `< 8` 删
- Shape 到 256 上限时不再剥、也不删占用（删了会整栋消失）
- `flushObject` + vis coarses（已有；Simulate / 重建必须等 GPU idle；脏 brick 页一次提交）

剥出去的 Shape：`invM > 0`，**拆掉它内部的键图**（刚体不再做结构）。还连在锚上的剩余部分继续解键。

地面永远静。从地面剥下来的块已经是动态（现有 `onSplit`）。

缺 \(\Phi\) 重分配或剥岛，就只是热图，不会塌。

---

## 5. Box3D 优化（性能主路径）

从 Box3D 拿的不是整套引擎，是**约束怎么解才快、才稳**。这些不是可选项。

| 机制 | 做什么 | 不做会怎样 |
|---|---|---|
| Soft Step | 6 子步，键每步 1～2 次 | 大步 PGS 抖、细柱飞 |
| 软焊 CFM | 键略软 | 无限刚锁死、λ 爆炸 |
| 图着色 | 同色无共享节点 | 不能并行；乱序扫缓存差 |
| 静–动优先 | 锚键先解 | 楼被推穿地面 |
| 岛：并快拆懒 | 每步最多拆 1 岛 | 塌开瞬间卡死 |
| 整岛睡 | 完好楼 skip 结构 | 每帧扫全关 |
| 暖启动 λ | 准静态几乎不迭代 | 每步从零解，连锁塌接不上 |
| 流形 ≤4 | 砸地仍角棱 | 塌下来接触数爆炸 |
| Weld = 6 λ | 键就是约束 | 不要另写力求解器 |

**不要搬：** GJK、凸包、每个格子一个 `b3Body`、全局 \(K\) 分解。

已对齐、不算新做：`kSubsteps = 6`、`kContactIters = 8`、流形缩到 4 点、接触 `lambdaN` / `lambdaT`。

一帧结构 + 刚体骨架（对齐 Box3D `b3Solve`）：

```text
若岛睡着 → skip 结构
并岛；若有断开且本步尚未拆岛 → 拆至多 1 个
键+接触放入颜色桶（静–动在前）
for 子步:
    动态 Body 重力（现有）
    鬼影节点重力
    for 颜色:
        键 TGS 1～2（暖启动）
        接触 SI 8
    Φ ≥ 1 删键（上限 N）；邻键 λ 保留
    若有删键 → maybeFracture
积分动态 Body；同步质心 → 格子中心（现有）
速度小 → 整岛睡
```

---

## 6. 接到现有文件

新建：

- `src/physics/Structure.h` / `.cpp` — `BondNode`、`Bond`、\(\Phi\)、鬼影 TGS
- `src/physics/ConstraintGraph.h` / `.cpp` — 着色、overflow
- `src/physics/Island.h` / `.cpp` — 并 / 拆 / 睡

改：

- `PhysicsWorld::substep` — 在接触前插入结构步；睡改为整岛
- `PhysicsWorld::markDirty` / `onSplit` — 占用变了局部重键；剥出去的 Shape 拆键
- `VoxelScene::handleEditInput` — 现有 `maybeFracture` 之后 `structure.markDirty(obj)`
- `VoxelRenderer` — 调试：Voxel DDA 面板里的 **Bond stress heatmap**（默认关）。勾上后按 coarse 的 \(\Phi\) 给命中体素上色（绿→红），不改 DDA 遍历、不改 coarsePool 布局。

**不改：** DDA 主路径、coarsePool 布局、断裂连通算法、角棱分类。

重键范围：只重建被挖 coarse 的 6 邻（`markDirtyCells` / `rebuildNeighborhood`）。剥岛或脏格太多才整栋 `rebuildShape`。鬼影 \(u,\theta\) 留下做伸长，不画成弯曲。\(\lambda_N\) 在 Φ 判定之后夹到 \(A\sigma h\)。接触 \(|J| > j_\mathrm{break}\) 打断命中 coarse 的键。\(\Phi\) 含扭转 \(T\) 和短柱 \(P\)-\(\Delta\)（\(N/N_\mathrm{cr}\)）。

GPU：结构纯 CPU。`flushObject` / `setSimulate` 已有 waitIdle 延后，塌出新物体时走同一条。

---

## 7. 干活顺序

**P0 支撑 flood（还不是应力）**

从地面 6 面走，走不到的岛直接 `maybeFracture`。验收：挖底层，上半截掉。细柱仍能撑楼——用来验证唤醒、剥岛、GPU。

**P1 键 + 鬼影 TGS + \(\Phi\) + 热图**

P0 的边上加键。验收：

1. 细柱顶重物：柱断，楼塌。
2. 挖承重墙：墙上先红再裂再塌，不是瞬移。
3. 完好测试盒砸地：结构键不参与，四角着地。
4. 热图：左侧 **Voxel DDA** 面板 **Bond stress heatmap**。默认关。勾上后应力画在体素上：绿低 Φ、红 Φ≥1。需要 Simulate 才有非零 Φ。

**P2 着色 + 整岛睡 + 每步最多拆 1 岛**

完好楼结构 CPU ≈ 0。碎片堆接触进同一张图（可第二步）。

**P3 才考虑加密**

只对正在裂的窄带收到 micro。整栋 fine 键不做。

---

## 8. 验收数字（先用，再调）

- 结构子步键迭代 1～2，接触 8
- 每子步最多破键 8～32
- 睡：与现有 `kSleepLin = 0.05`、`kSleepTime = 0.5` 同一量级，但 **整岛** 判定
- 残渣仍 `< 8` fine 删
- Shape 上限仍 256
- 调试：`DebugSolve` 加 `bondsAlive`、`bondsBrokenThisStep`、`maxPhi`

---

## 9. 能塌什么、不能塌什么

**能（游戏结构坍塌要的就是这些）**

- 挖底、挖墙之后，上面因拉 / 剪 / 弯超限而断
- 偏心、悬挑：弯矩项 \(M / M_u\) 会先爆
- 石头怕拉不怕压：纯压柱能撑，一侧一挖就弯裂
- 连锁：一块砸下去，接触 \(J > J_\mathrm{mat}\) 再碎一圈
- 和现有碎片堆、角棱接触、质心刚体兼容

**不能（第一版也不做）**

- 肉眼可见的弯曲、徐变
- 欧拉压杆那种特征值屈曲（除非再加 \(P\)-\(\Delta\)）
- 连续体裂纹尖端、相场光滑分叉
- 每颗 0.1 m fine 自己的应力场

---

## 10. 一句话

coarse 上的软焊键（6 个 λ → \(N,V,M\) → \(\Phi\)）+ Box3D 的 TGS / 着色 / 整岛睡 + 现有断裂剥岛和质心刚体。

实现的是「承重不够就裂，裂完当刚体掉」，不是实验室级连续体破坏。P0 验证管道，P1 才是「看起来像塌」，P2 才是「整关能玩」。
