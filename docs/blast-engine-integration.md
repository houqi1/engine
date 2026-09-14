# Blast 接入本引擎：完整方案

> 编制：2026-09-10。对应执行方案第 4、7、8、10、13 节，以及用户要求「引擎接入先查资料再给完整方案」。
> 状态：方案。未改 `vulkan_engine_voxel` 链接，未实现本页模块。
> 固定 SDK：NVIDIA-Omniverse/PhysX Blast 5.0.6，SHA `7ef568f5b557a6dad9023ecebc43cb809270e035`。

本文完成的是「把已验收的 LL + ExtStress 接到本仓库 Vulkan 体素引擎」这一条路径，不是重选应力求解器，也不是把刚体换成 PhysX。

---

## 0. 检索结论（相对公开资料）

官方把 Blast 分成三层，职责写得很清楚：

| 层 | 公开职责 | 对本引擎 |
|---|---|---|
| NvBlast Low Level | 无物理、无图形、无线程框架；`NvBlastActor` 抽象对应刚体，由应用自己建碰撞与渲染 | **采用**。P0–P4 已走这条 |
| NvBlastTk | 事件、分组、关节、内存回调 | **不采用**。本引擎已有 `VoxelObjectId` / family 生命周期 |
| ExtPhysX / ExtPxStressSolver / ExtImpactDamageManager | 在用户提供的 `PxScene` 里同步 `PxActor`，接触回调做冲击损伤 | **不采用**。刚体权威是 `PhysicsWorld` |
| ExtStress | 只依赖 LL；`setNodeInfo` / `addForce` / `update` / `generateFractureCommands`；**不依赖外部物理库** | **采用**，并保留 P1–P3 补丁 |
| ExtShaders / ExtAuthoring | 径向伤害着色器、Voronoi 预破碎 | **不采用**。破坏来自体素占用与应力硬阈值，不是预破碎网格 |

官方原文要点（必须遵守，不是风格建议）：

1. LL「Notably absent: physics or collision representation. It is entirely up to the user to create such representations when blast objects are created. Updates … are passed to the user as the output of a split function.」
2. ExtStress 标准环：加力 → `update` → 若 `getOverstressedBondCount()>0` 生成断裂命令并应用 → 若 split，先 `notifyActorDestroyed` 再 `notifyActorCreated`。
3. 重力要收敛，图上必须有 **质量为 0 的节点**（world / dummy），否则整栋被当成自由落体，准静态自重内力为 0。
4. 公开 `addForce(actor, position, impulse)` 把冲量加到**最近图节点**。偏心力矩会丢。本仓库 P3 已加 `addLoad(node, F, τ)`，引擎必须走这条，不得退回最近节点。
5. `graphReductionLevel` 官方标 experimental。保持 0。
6. `NvBlastAsset` 创建后图不可原地改节点。体素编辑必须重建 asset/family（P4 `compactReplace`）。
7. ExtPx 是「reference implementation」，不是自定义引擎的必经层。把 ExtPx 整段搬进来等于再引入 PhysX。
8. UE/GameWorks 插件把 Impact Damage（命中点扣 health）和 Stress（弱处断）混在同一 Actor 上。本项目冲击只进结构载荷，不走 ExtShaders 径向伤害。
9. Teardown / Voxel Play / 本仓库 `VoxelSceneFracture` 是 **占用连通掉块**，不是站立建筑的应力路径。两者要并存，不能互相替代。

GameWorks 1.1 的 hardness、线性/角冲量经验因子、旧 ExtPxStressSolver 的 `addAngularVelocity` 命名，与 5.0.6 的 Pa 强度、`addCentrifugalAcceleration` **禁止混用**。

交叉核对（深度检索报告，状态 Partial，与上文一致处不重复；下列是方案正文原先写得不够硬的点）：

- `NvBlastActorSplit` 在 `newActors` 里**可以复用父 actor 指针**（该岛仍有 chunk 时），同时 `deletedActor` 仍指向它。ExtPx 一律销毁旧 `PxActor` 再新建。本引擎策略：按占用集合 `replaceShape` 最大块、新槽给其余块；不要看见同一 `NvBlastActor*` 就跳过刚体更新。
- 仅含 world 节点、visible chunk 为 0 的 actor：split 会丢掉，**不要**给它建 `VoxelObject`。
- `ExtForceMode`：GameWorks 默认 `IMPULSE`；钉死的 5.0.6 是 `FORCE` / `ACCELERATION`。接触 `J` 必须先变成 `F=J/kDt` 再 `FORCE`，禁止把 λ 当 IMPULSE 传进 5.0.6。
- `addGravity` 在 5.0.6 源码里对动态节点不是「整栋自重」通道；官方/ExtPx 只给 kinematic/world-supported 加重力应力。站立楼：world 节点质量 0 + 质量节点 `m g`（`addLoad` 或 `FORCE`）。最后 world bond 断开后**不要**再插假固定点（T05/T07）。
- 0 质量节点是**重力准静态**条件，不是接触载荷条件。冲击仍可打在有质量节点上。
- ExtPx `postSplitUpdate` **不是**刚体重映射，只是按 PhysX actor 数量上限 `release()`。禁止当引擎 split 钩子抄。
- `generateFractureCommands` 指向内部缓冲，下次 generate 或 release 后失效。本仓库用 `copyBondProbes` + 应用侧硬阈值，不要改回直接吃 SDK 面积损伤量。
- `NvBlastFamilySetAsset` 在 asset ID 不同时失败。拓扑变了不能「换 asset 保 family」。
- P4 `VoxelGraph` **目前不写 world bond**（只有节点-节点键）。圆柱/站立楼的 OccupancySampler 必须像 `CylinderGraph` 那样加 `chunkIndices[1]=UINT32_MAX`，不能指望现成 `createVoxelBlast`。
- UE「Kinematics Max Contact Impulse」只对 kinematic/world-supported：砸碎后让打击物穿过去。本引擎 Static 是 `invM=0`，无限质量会把打击物弹开。E3 记录该差异；需要时在结构 Static 接触上加冲量上限，不默抄 PhysX 字段。
- 碰撞几何跟的是 **占用 / visible chunk 下标**，不是 `userData`。`userData` 只存稳定 ID。
- NVIDIA 没有「体素编辑重建」API。整 family `compactReplace` 是本项目策略。`notifyActorDestroyed` 不能当删节点。

---

## 1. 本仓库现状（Fit）

已经具备、不要重做：

| 能力 | 位置 | 引擎接入时的角色 |
|---|---|---|
| 占用权威 | `VoxelScene` / `setFineCpu` / `flushDirtyPages`，fine = 0.1 m | 几何唯一权威 |
| 刚体 + 接触 | `PhysicsWorld`，`kDt=1/60`，6 子步，SI `lambdaN`/`lambdaT` | 动量通道唯一权威 |
| 连通掉块 | `VoxelSceneFracture` + `VoxelConnectivity` | 挖穿后的岛屿分离（Teardown 路径） |
| Blast LL + ExtStress | `nvblast` 静态库，仅 `blast_p0`–`p4_tests` 链接 | 结构通道 |
| 手工圆柱力学 | P1 T01–T06 / T21 / T22 / T24 | 接入后的数值对照 |
| actor→刚体 | P2 `ActorBodies`，速度 `v'=v+ω×(c'-c)` | 映射到 `VoxelObject` + `RigidBody` |
| 接触→结构 | P3 `ContactLoads` + `addLoad` | 从 `Contact.lambda*` 取 J |
| 体素粗图 + family 替换 | P4 `VoxelGraph` / `compactReplace` | 从 fine 占用采样，不再用独立 `VoxelGrid` 演示网格 |

明确缺口：

- `CMakeLists.txt` 里 `vulkan_engine_voxel` **不链接** `nvblast`，不编译 `src/blast/*`。
- `PhysicsWorld` 只保留**最后一次子步**的 `debug_.lastContacts`，没有「一个物理 tick 的净冲量」。
- 连通断裂只对 `MotionType::Dynamic` 入队；站立建筑若是 `Static`，挖洞不会掉岛。
- 密度：物理 `kDensityWood=600`，Blast 测试 `1000`。未在引擎边界统一。
- 没有 `StructureInstance`：`VoxelObject` 与 `NvBlastActor` 之间无表。

---

## 2. 路径比较（Spectrum）

| 路径 | 保证 / 启发式 | 对本仓库 |
|---|---|---|
| **A. LL + ExtStress + 自写 PhysicsBridge**（选定） | 官方保证：物理无关、split 由应用建刚体。应力是实时近似反力，不是 FEM 保证 | 与已锁定路线、P0–P4、现有 Box3D 接触一致 |
| B. ExtPx + PhysX 5 | 保证：官方刚体同步参考实现 | 替换 `PhysicsWorld`，违反「不因 Blast 换物理」 |
| C. Tk + 自写监听 | 保证：split 事件；仍无碰撞 | 多一层框架，本引擎已有 ID/槽位 |
| D. 只用连通掉块（Teardown / 现有 Fracture） | 保证：挖穿后岛屿下落。不保证：剩余壁带因偏心弯矩断裂 | 已有；不能充当 T02 |
| E. Voxel Play 式「到地面/锚的连通预算」 | 启发式塌方，有搜索预算上限 | 与「剩余支撑不强制断」冲突 |
| F. 逐 fine 软体 / 重写 CGNR | 无实时保证 | 明确非目标 |
| G. 旧 GameWorks Blast 1.1 调参 | 启发式 hardness | 禁止与 5.0.6 Pa 混用 |

选定 A。B–G 不是「以后再加一层」，是错误接入。

---

## 3. 失败条件（Failures）

这条路径在下列输入下**不保证**产品观感或数值：

| 情况 | 发生什么 | 处置 |
|---|---|---|
| 无 0 质量 world 节点却加重力 | ExtStress 当自由落体，自重内力≈0 | 底座 world bond，`setNodeInfo` 质量 0 |
| 用 `addForce(position)` 映射偏心撞击 | 力矩丢失，壁根不受弯 | 只用 `addLoad` |
| 把 `lambdaN` 当力，或除以 `kSubDt` | 力放大 6 倍（6 子步）或单位错 | `J_tick = Σ_sub (λn n + λt t)`，`F=J/kDt` |
| 只读 `lastContacts`（最后子步） | 丢掉前 5 个子步冲量 | tick 累加器 |
| 把接触冲量再 `applyImpulse` 一次 | 双重动量 | 结构通道只读 J，不写回刚体 |
| 挖洞后不重建 family | 幽灵键、隔空承重 | `compactReplace` |
| 薄壳 `agg=4/8` | 填腔 | 圆柱只允许 1 或 2 |
| 连通掉块替代应力 | T02 变成「切开就掉」 | 两种系统分责，见 §5 |
| `equalizeMasses=true` | 不同质量节点支反力偏 | 保持 P1 关闭 |
| 不收敛仍 `generateFractureCommands` | 假断裂 | `converged()` 门控 |
| 把 ExtPx `FilterShader` / UE `OnComponentHit` 当冲击损伤 | 命中点扣 health，不是弱处断 | 冲击只进应力 |
| `graphReductionLevel>0` | 实验简化，薄壳键被并 | 保持 0 |
| 运行时焊接/新增体素当结构修复 | 首版非目标 | 放置体素只改占用，不自动建 bond |
| 屈曲 / Donnell | 远低于 bond 强度也可能失稳 | 非目标；T01–T04 只比连接应力 |
| 密度 600 与 1000 混用 | 同一几何两套质量 | 每个 `StructureInstance` 锁定一个 ρ |
| 看见同一 `NvBlastActor*` 就跳过刚体 | 最大块占用已变，碰撞/质量仍是旧的 | 按占用集合 `replaceShape`，不按指针相等 |
| 给 visible=0 的 world actor 建物体 | 空刚体或幽灵锚 | 不生成 `VoxelObject` |
| 把 5.0.6 `addForce` 当 GameWorks IMPULSE | 力/冲量差一个 `dt` | 只传 `FORCE`/`ACCELERATION` |
| 对已脱离的动态岛继续 `addGravity` 当自重应力 | 无对侧，或被当成仍锚固 | 无 world bond 则只做刚体重力，结构步不再加准静态自重 |
| 抄 ExtPx `postSplitUpdate` | 只是数量上限释放，不是建刚体 | 用本引擎 `addBody`/`replaceShape` |
| `VoxelGraph` 直接当站立楼 | 无 world bond，T01 变自由落体 | OccupancySampler 显式写 UINT32_MAX 键 |

相对误差分母用预定义特征尺度。Blast 截面应力不要求等于连续体 FEM。

---

## 4. 目标架构

```text
输入：挖洞 / 重力 / 接触冲量 / 工具
                │
                v
     VoxelScene 占用 + 断开面集合     ← 几何权威
                │
                v
     StructureGraph（fine 聚合 1 或 2）
                │
                v
     NvBlastAsset + Family + ExtStress  ← 结构权威
                │
                v
     硬阈值断键 → NvBlastActorSplit
                │
                v
     1 actor → 1 VoxelObject → 1 RigidBody
                │
                v
     PhysicsWorld 积分与接触            ← 动量权威
                │
                v
     本 tick 净 J → 下一结构步载荷
```

三通道互不越权：

| 通道 | 写什么 | 不写什么 |
|---|---|---|
| 占用 | fine 实心/空、材料、颜色、断开面 | 应力、速度 |
| 结构 | bond health、actor 拓扑、family | 刚体速度、接触 λ |
| 动量 | `v,w,x,q`、接触 λ | bond health |

模块（新建，不要把逻辑堆进 `VoxelScene.cpp`）：

| 模块 | 文件（建议） | 职责 |
|---|---|---|
| StructureInstance | `src/blast/StructureWorld.h` | 一个 family + solver + 稳定 ID 映射 + 所属 `VoxelObjectId` 列表 |
| OccupancySampler | `src/blast/OccupancySampler.cpp` | 从 `VoxelScene` 读 fine，填 P4 `VoxelGrid`（含 brokenFaces） |
| PhysicsBridge | `src/blast/PhysicsBridge.cpp` | tick 冲量累加、世界→asset 变换、`addLoad`、Static↔Dynamic |
| StructureWorld | 同上 | 固定步调度、重建事务、split 提交 |
| 现有 HardFracture / VoxelGraph / ContactLoads | 保持 | 数值行为与 P1–P4 测试一致 |

`VoxelObject` 不塞 Blast 指针。并行表：`VoxelObjectId → StructureInstance*`，实例持有 `actorIndex → VoxelObjectId`。

---

## 5. 对象映射与两种破坏并存

### 5.1 一对一

```text
NvBlastActor  1—1  VoxelObject  1—1  RigidBody
NvBlastFamily 1—N  Actor（分裂后）
StructureInstance  持有 1 个 family；compactReplace 后变成 N 个实例
```

split 后：删除的 actor 对应的 `VoxelObject` 按 P4/连通路径切占用；每个新 actor 得到新槽位（或最大块留原槽，与现有 `VoxelSceneFracture` 一致）。

速度继承与现有碎块相同：

```text
v' = v + ω × (c' − c)
ω' = ω
```

质量、COM、惯量仍由 `computeMassProperties` 从 fine 算，**不用** ExtStress 标量惯量。

### 5.2 站立建筑的运动类型

| 状态 | `MotionType` | `PhysicsWorld` | Blast |
|---|---|---|---|
| 仍有 world bond 的结构 | `Static` | `invM=0`，不受重力积分 | 有 0 质量 world 节点 |
| 最后锚固路径断开（T07） | 改为 `Dynamic` | 正常刚体 | 无 world bond 的 actor |
| 连通挖出的碎块 | `Dynamic` | 已有 | 单节点 actor：`notifyActorCreated` 返回 false，**退出应力** |
| 场景 spinner | `Kinematic` | 场景写 pose，物理只跟 | **不**进 StructureWorld |

禁止用 `Kinematic` 表示站立楼：`syncTransformsToScene` 会把场景 pose 写回刚体，和结构静止语义冲突。

### 5.3 连通 vs 应力（必须同时存在）

| | 连通掉块（已有） | Blast 应力（要接） |
|---|---|---|
| 触发 | 占用删除后六邻域走不到主体 | 连接应力 > S |
| 作用对象 | 已断开的岛屿 | **仍连着的**结构 |
| 当前限制 | 只对 Dynamic 入队 | 未接引擎 |
| 站立楼挖洞 | 必须也对 Static 结构跑连通 | 剩余主体 `compactReplace` |

一帧挖洞后的顺序：

1. `setFineCpu`（占用权威）。
2. 连通搜索：断开的岛 → 新 `VoxelObject`（Dynamic）；主体留下。Static 结构也要跑这一步（改 `wantFracture` 条件）。
3. 对仍挂 StructureInstance 的物体标 `topologyDirty`。
4. 物理安全点：`compactReplace` 或对每个活 actor 建紧凑 family。
5. 后续物理 tick 才应力求解。禁止在同一占用事务里用旧图求应力。

单节点碎块：不建 solver，避免无意义 `update`。

首版不把放置体素当成焊接。`F` 键只改占用；结构实例标脏后重建，新接触面若应用侧 `brokenFaces` 未记断开，会按几何重新成键——若产品要「锯缝永不粘」，挖与放都必须写 `brokenFaces`。默认：只删除时写断开面；放置是新材料，允许新键。这一点要在 UI 上可关。

---

## 6. 接触冲量（本引擎特有坑）

`Contact` 里 `lambdaN` / `lambdaT` 是 **顺序冲量迭代的累积冲量**（单位 N·s），不是力。每个物理 tick：

```text
kDt     = 1/60 s
kSubsteps = 6
kSubDt  = kDt/6
```

`PhysicsWorld::step` 每个 tick 调 6 次 `substep()`，每次重建接触并从头累加 λ。`debug_.lastContacts` 只有最后一次子步。

正确提取：

```text
每个 substep 求解结束后：
  对每个接触点（含静态侧）：
    J_n = lambdaN * n
    J_t = lambdaT * t     // t 需在求解时记下，或保存切向冲量向量
    acc[a] +=  J   at p     // 动态侧
    acc[b] += -J   at p     // 对侧，含 Static 地面

一个物理 tick 结束后：
  J_tick = acc
  F_avg  = J_tick / kDt     // 除 tick，不是 kSubDt
  τ      = (p - c_node) × F_avg   在 asset-local
  addLoad(node, F, τ)
```

禁止：

- 把 8 次 SI 迭代的中间 λ 相加（λ 已是累积量）。
- 用 `kSubDt` 当 `delta_t_contact`。
- 用未求解前的穿透深度当力。
- 只映射动态侧（楼承重时地面反力必须进结构）。

持续接触：每 tick 重新采样，不用 event-id 消费。  
单次撞击：`(tick, pair, contactIndex)` 生成 event-id，`ImpulseEvents::consume` 一次。  
断裂后：`LoadSnapshot.valid=false`，等下一物理步。

世界点 → asset-local：点用逆变换；力/力矩只旋转。

---

## 7. 固定步耦合（改帧循环）

现循环（`main_voxel` + `VoxelScene::update`）：

```text
handleEditInput
advanceFractureWork
commitReadyFractures
scene.update → physics_.step(dt)
renderer.draw
```

目标（一阶分区，延迟约 1 个物理步）：

```text
handleEditInput                         // 占用事务入队
advanceFractureWork                     // 连通（含 Static 结构）
commitReadyFractures                    // 岛屿提交；标 Structure dirty
scene.update:
    physics_.step(dt)                   // 内部每个 kDt：
        提交已准备的 geometry/family 事务
        6 × substep，累加 J_tick
        StructureWorld::onPhysicsTick(J_tick, kDt):
            装载重力 + 映射接触
            solver.update()
            不收敛 → 不断
            否则 HardFracture + Split
            新 actor → VoxelObject/RigidBody（速度继承）
            失去 world bond → Static 改 Dynamic
            若本 tick 已 split：作废本 tick 接触快照，不再级联
        结构平衡轮上限 2
upload / draw
```

`StructureWorld::onPhysicsTick` 必须在 `physics_.step` 的 **tick 边界**调用，不能在渲染帧里按 `dt` 调。T13：渲染 30/60/120 时物理 tick 序列不变。

同一 family 串行。不同 family 以后可并行（P5），首版单线程。

---

## 8. OccupancySampler（P4 网格 ← 引擎占用）

P4 `VoxelGrid` 是稠密 XYZ。引擎占用是 coarse/brick/fine。采样规则：

1. 只扫该 `VoxelObject` 的占用 AABB。
2. `agg`：薄壳圆柱 1 或 2；厚实体可 2（4 仅对照，禁止默认）。
3. `solid[i]=1` iff `occupancyFine(...)`。
4. 底座锚固：fine 格邻接场景地面（或显式锚标记）→ 该粗节点 world bond，health = `kUnbreakableLimit`。
5. `brokenFaces`：应力断键后写入对应 fine 面；连通切开的界面也要写，重建才不愈合。
6. 密度：实例创建时写入，圆柱演示默认 1000，与 P1 对照；普通木块可 600，但同一实例内不得混。

薄壳圆柱体素版（执行方案 1.1）：

| 量 | 值 |
|---|---|
| R, t, H | 2.0, 0.2, 6.0 m |
| fine | 0.1 m → 壁厚 2 格，高 60 格 |
| 周向/高度节点 | 聚合后仍要多节点，禁止整筒 1 节点 |
| 底座 | j=0 全周 world，不可断 |
| 地面 | 现有 Static 地面；圆柱不要做成 Dynamic 盒子 |

场景生成：独立 `spawnStressCylinder()`，不要塞进 pirate hut 导入。

---

## 9. Split 提交（复用现有碎块路径）

`NvBlastActorSplit` 之后：

1. `notifyActorDestroyed(old)`（指针仍有效时，与 `HardFracture::splitIfNeeded` 一致）。若 `newActors` 里再次出现同一指针，仍要按新占用做 `replaceShape`，不能当成「没 split」。
2. `deletedActor == nullptr` 且返回 0：刚体不变。
3. 收集每个活 actor 的 support/visible chunks → fine 集合（`userData` → `stableNodeId`，碰撞仍走占用，不走 chunk 网格）。
4. 最大块留原 `VoxelObject` 槽；其余走 `extractFragmentFromMasks` 同类逻辑（新槽、`addBody`、速度继承）。
5. `HasExternalBonds` 为真保持 `Static`；否则改 `Dynamic` 并 `replaceShape`。仅 world 节点、无可见 chunk：不建物体。
6. `notifyActorCreated` 每个新 actor；单节点返回 false 则从应力表删除。
7. `flushDirtyPages` + `markDirty` + 邻域唤醒。
8. 禁止附加爆开速度；出生穿透沿用 P2 slop。禁止调用 ExtPx `postSplitUpdate`。

连通提交与应力提交不要各写一套切占用。抽 `commitOccupancySplit(parent, fineSets, inherit)`，两条路径共用。

---

## 10. CMake 与依赖边界

```text
nvblast  (已有)  ← 只 LL + ExtStress + 本仓库补丁
src/blast/*.cpp  ← 现测 + 新 StructureWorld
vulkan_engine_voxel  ← 增加上述源文件并 PRIVATE 链接 nvblast
```

不链接：PhysX、ExtPx、Tk、ExtAuthoring、ExtShaders。

`NOMINMAX` 已在测试目标上定义，引擎目标同样需要。

MSVC Debug 的 NvBlast `bool` 返回 `nullptr` 仍在；引擎验收以 Release 为准，与 P0 相同。

---

## 11. 分步执行（引擎接入阶段，不是重做 P0–P4）

阶段名 E*，避免和已通过的 P0–P4 混淆。每步有退出测试。未要求实现前不改生产代码。

### E0 链接与空转 — 已完成（2026-09-13）

前置：超过 512 键不再漏判断裂、稳定 ID / 损伤迁移、`OwnerId=uint32_t`、`compactReplace` 可回滚。见 `docs/blast-graph-regression.md`。E0 开始前 P0–P4 与图回归均已再跑通过。

落地：

- 共享库 `ve_structure_core`（BlastMemory / VoxelGraph / ContactLoads / StructureWorld）链接 `nvblast`。测试与 `vulkan_engine_voxel` 都链它；圆柱图和 ActorBodies 仍只在对应测试里。
- `BlastRuntime` 在 `main` 里于任何场景对象之前 `init`，退出时先 `scene.cleanup` 再 `runtime.shutdown`。场景重置只 `StructureWorld::clear`，不拆运行时。
- 空 `StructureWorld`：0 个实例；`onPhysicsTick` 只计数。
- `PhysicsWorld::step` 用 `FixedStepClock`（`kDt`、最多 2 个 tick/帧）。完成一个固定 tick 后通过函数指针通知场景，物理模块不包含 Blast。
- ImGui「Structure (E0 idle)」只读：已初始化、实例数 0、tick 数、最后 dt、Blast live bytes / 错误数。无强度或应力开关。

未做：应力圆柱、重力求解、断裂、接触载荷。

退出记录见 `docs/blast-e0.md`。E1–E4 未开始。

### E1 体素圆柱进场景（静力）— 已完成（2026-09-13）

断裂关闭。`spawnStressCylinder()` 从占用生成薄壳；`OccupancySampler` 建图并写显式 world bond；`onPhysicsTick` 每 tick 加一次自重。固定 270° 底部切口按钮。ImGui 数值 + 可选 0–2 MPa 体素着色。

头测见 `docs/blast-e1.md`。完整圆柱支反力 rel=4.4×10⁻⁵ 且收敛。切口应力上升、密度加倍成比例；3000 节点切口在头测迭代预算内尚未达到 SDK 残差容差，E2 锁强度前要先看收敛后的切口场。

未做：硬阈值断裂与下落刚体（E2）。

### E2 Split → VoxelObject — 已完成（2026-09-13）

S_fail=5×10⁵ Pa，S_hold=5×10⁷ Pa。收敛后收集超限键；提交时写 `brokenFaces`、全额断键、一次 split；最大块留原槽；有 world bond 保持 Static。见 `docs/blast-e2.md`。

未做：接触冲量二次断裂（E3）。

### E3 接触载荷

- tick 冲量累加器进 `PhysicsWorld` 或 Bridge。
- `addLoad`；event-id；静态侧反力。
- 墙板砸障碍二次断裂。

退出：引擎内 T10–T13。用固定输入回放，不能只看「好像断了」。

### E4 任意挖洞重建

- Static 结构也走连通。
- 挖/断键写 `brokenFaces`。
- dirty family → `compactReplace`。
- 未编辑 actor 状态保留。

退出：引擎内 T14–T17；连续挖不泄漏。

之后才是原方案 P5：休眠、跨 family 并行、碎屑退出应力、预算。没有 E1–E4 的 P5 是空优化。

---

## 12. 引擎内验收（在 P1–P4 头测之外）

头测必须继续绿。引擎测是同一判据的占用/刚体版。

| ID | 内容 | 相对头测 |
|---|---|---|
| E-T01 | 场景圆柱自重不破 | T01 |
| E-T02 | 底座 270° 挖空，弱/强材 | T02；占用刷，不是直接 `applySdkBondDamage` |
| E-T07 | 锚固路径断后下落 | T07 |
| E-T10 | 偏心砸，壁根弯矩 | T10 |
| E-T12 | 二次断裂，冲量消费一次 | T12 |
| E-T13 | 渲染 30/60/120 断裂序列 | T13 |
| E-T15 | 锯缝重建不愈合 | T15 |
| E-REG | 关闭 StructureWorld 时现有连通/堆叠不变 | 新 |

参数与 `docs/blast-p1-params.md` 等锁定文件一致：`S_hold=5e7 Pa`，`S_fail=1e3 Pa`，圆柱尺寸不随画面改。

---

## 13. 明确不在本接入做的事

- 把 `PhysicsWorld` 换成 PhysX / Box3D 官方刚体以外的后端。
- 引入 Tk 关节当「未断键的弹性」。
- 运行时焊接、两栋楼粘回一栋。
- Donnell 屈曲通过条件。
- 可见弹性振动。
- GPU 应力。
- 多人同步。
- 把 pirate hut 默认改成可塌结构（另开场景开关）。

---

## 14. 资料（实施时用 SHA 链，不用 floating latest）

1. ExtStress 用法：https://nvidia-omniverse.github.io/PhysX/blast/docs/api/extensions/ext_stress.html
2. LL 用户指南（asset / family / generate-apply-split）：https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/api_ll_users_guide.html
3. 扩展总览（ExtStress 不依赖物理库）：https://nvidia-omniverse.github.io/PhysX/blast/docs/api/extensions/index.html
4. GameWorks README（LL 物理无关；ExtPx 仅为参考）：https://github.com/NVIDIAGameWorks/Blast
5. ExtPx 冲击管理器与 ExtPxStressSolver（对照，勿照搬）：GameWorks `pageextphysx.html`
6. UE Blast 插件 Impact vs Stress：https://docs.nvidia.com/gameworks/content/gameworkslibrary/blast/1.1/authoring_docs/BlastUe4_BlastSettings.html
7. ExtAssetUtils world/external bonds：https://nvidia-omniverse.github.io/PhysX/blast/docs/api/extensions/ext_assetutils.html
8. Teardown 占用体积 + 断开后新物体（连通，非应力）：Gustafsson 访谈 / Acko frame teardown
9. 本仓库：`docs/体素应力破坏系统-完整执行方案.md`，`docs/blast-p0`–`p4-params.md`，`docs/voxel-physics.md`

---

## 15. 完成定义（仅引擎接入）

同时成立才算接入完成，而不是「链上库能编译」：

1. `vulkan_engine_voxel` Release 能生成薄壳圆柱，T01/T02 行为与头测同类。
2. 占用仍是 `VoxelScene`；刚体仍是 `PhysicsWorld`；Blast 不写速度。
3. 接触 J 每个 tick 只进结构一次，不二次 `applyImpulse`。
4. 挖洞走连通掉岛 + 主体 family 重建；锯缝不愈合。
5. 渲染帧率不改变固定 tick 断裂序列。
6. P0–P4 头测仍通过。
7. 无结构物体时旧 demo 行为不变。
