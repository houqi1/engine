# E5：任意体素物体接入结构应力

> 编制：2026-09-24。接在 E4（任意挖洞重建，已验收）之后、原方案 P5（性能生产化）之前。
> 状态：方案。未改代码。
> 固定 SDK 不变：Blast 5.0.6，SHA `7ef568f5b557a6dad9023ecebc43cb809270e035`。
> 上位文档：`docs/体素应力破坏系统-完整执行方案.md`、`docs/blast-engine-integration.md`。本文不改它们的判据，只补「挂载任意物体」这一段空档。

E4 解决的是「已挂载结构上任意挖」。E5 解决「任意 `VoxelObject` 都能挂成结构」：导入网格、手工搭建、Static 建筑、Dynamic 物体。
没有 E5，P5 基准矩阵（小房间 ~1k、单栋 5k–10k、多栋 ~50k 节点）没有可测场景。

---

## 0. 资料结论

标注：**[P]** 一手（SDK 源码/头文件/官方文档/公开源码），**[S]** 二手（wiki/媒体），**[I]** 推断。

### 0.1 Blast 自己怎么处理「任意资产 + 锚固」

| 结论 | 来源 |
|---|---|
| world bond = `chunkIndices[1] = UINT32_MAX`；含该键的 actor `NvBlastActorHasExternalBonds` 为真 | [P] `NvBlastTypes.h` |
| `NvBlastExtAssetUtilsAddExternalBonds` 给指定 support chunk 加外部键；`AddWorldBonds` 已标 DEPRICATED 别名 | [P] `NvBlastExtAssetUtils.h` |
| omni.blast `createExternalAttachment(path, …, relativePadding, externalBondsUnbreakable)`：「support chunks which touch static geometry are bound to the world」；`relativePadding` 放大 chunk 包围再测接触 | [P] omni.blast C++ API |
| omni.blast 0.9.0：「Creation/removal of external attachments is no longer allowed during simulation」 | [P] omni.blast CHANGELOG |
| 多个独立资产要共同承重：`NvBlastExtAuthoringFindAssetConnectingBonds(maxSeparation)` + `MergeAssets`，在**创作期**合成一张 support 图 | [P] `NvBlastExtAuthoring.h` |
| ExtStress：图中必须有质量 0 节点，否则「add a supporting force to all nodes at the base」 | [P] ext_stress 用户指南 |
| 动态 actor：离心力 + 碰撞冲量到节点；**不加重力** | [P] 用户指南；GameWorks `ExtPxStressSolverImpl`：kinematic → `addGravityForce`，dynamic → 只 `addAngularVelocity` |
| `addGravity` 头文件说「只对 static actor 有意义」，实现里**不检查** | [P] 5.x 头文件 vs `.cpp`，调用方负责 |
| `notifyActorCreated` 单节点返回 false | [P] |
| 耗时与 `maxSolverIterationsPerFrame` 线性；**官方无节点数上限建议** | [P] |
| SDK 不替你跳过求解；休眠/事件驱动由调用方做 | [P] `update()` 源码 |

### 0.2 已发行游戏的做法

| 游戏 | 锚固 | 重算策略 | 来源 |
|---|---|---|---|
| Teardown | 连到 static body（含经关节间接连接）才算固定；不动的 body 移出模拟；2026 原型有应力求解，无公开细节 | 连通性、命令流 | [P] 官方 modding API、2026-03 博客；[S] 80.lv |
| Space Engineers / Medieval Engineers | `BlocksConnectedToWorld`：与体素地形接触的块是静态 | 块数变化才后台全量重算，双缓冲，断裂节流 `DestructionDelay=10` | [P] 公开 C# 源码 `MyAdvancedStaticSimulator.cs` |
| 7 Days to Die | 基岩/地面恒为竖向支撑 | 只在放/拆块时局部重算 | [S] 官方 wiki |
| Valheim | 接地/岩石 = 满支撑，按距离衰减取最大 | 事件驱动 | [S] |
| Unreal Chaos | Anchor Field 在构建期锁定；连接图按邻近（Precise + ProjectedBoundsOverlap 过滤角/小接触）；AutoCluster 只把物理连通的骨骼并簇 | 构建期 | [P] Epic 文档 |
| Red Faction Guerrilla / Instruments of Destruction | 无公开技术资料（后者由 Radiangames 开发，非 Grab Games） | — | [S] |

### 0.3 对 E5 的直接推论

1. **锚固只在挂载时自动判定，一次定死**：与 static 地形接触（带 padding）的 fine → world bond。运行中不新增锚固（omni.blast 0.9、SE、Chaos 一致；也符合原方案「不焊接」）。
2. **静止在地上的 Dynamic 物体不是锚固**：它是自由体，只进离心力和**持续接触载荷**。[I] 自由体上「接触反力 − 均匀惯性」与静态「重力 + 支反力」产生相同内力，所以不需要、也不能造假锚。
3. **多个物体共同承重必须在创作期合并为一个结构**（Blast MergeAssets 的思路），不能运行时互当锚固。
4. **图粗化在应用侧做**：粗格内按连通分量拆节点（Chaos AutoCluster 的规则）；`graphReductionLevel` 保持 0。
5. **调度自己做**：SE / 7DTD / Teardown 都是事件驱动，Blast 不提供休眠。

---

## 1. 现状差距（代码核查，2026-09-24）

### 1.1 单实例假设（阻塞项）

`StructureWorld` 用 `std::vector<StructureInstance>` 存实例，但大量接口只作用于 `front()`：

| 位置 | 问题 |
|---|---|
| `instance()` `StructureWorld.cpp:1609`、`debug()` `:1338`、`pendingFracture()`、`takePendingFracture` `:611` | 只看第一个实例 |
| `markCut` `:306`、`warmupGravity` `:313`、`setKeepColumnBox` `:340`、`setSolverIters` `:449`、`recacheOccupied` `:489` | 同上 |
| `setDensityScale` `:427` | 基准写死 `kE1Density` |
| `mountSample` `:377-381` | 从 `instance()` 继承强度/断裂开关，**覆盖传入的 `strengthPa`** |
| `mountSample` 后 `VoxelScene.cpp:1879, 2181` | 用 `instance()` 写 `topologyRevision`，多实例时写错对象 |
| `applyOccupancyRemoval` `:232, 300` | erase 后 push_back，**实例顺序变化**；`vector` 扩容使指针失效 |
| `freeObjectSlot` `VoxelScene.cpp:187` | 只对 `stressCylinderId_` unmount；`unmount` 只按 `objectId`，不查子 binding |
| UI `VoxelRenderer.cpp:2134-2270` | 只显示第一个实例 |

### 1.2 场景专用字段

`StructureInstance` 里 `axisX/axisZ/keepAz0/keepAz1/keepBox/keepX0..Z1/cutApplied` 是圆柱/四柱诊断用；`rebuildBondMeta` `:461-486` 用它们算 strip 与 neck 统计。只影响诊断，不影响断裂，但 `mountSample` 对任何物体都写 `kE1KeepAz0/1`。

### 1.3 挂载入口

- 只有 `CylinderObjectView`（`VoxelScene.cpp:1808`）与 `FrameObjectView`（`:1832`，固定 96³）。锚固是谓词：`isBaseAnchorFine` / `isFrameAnchorFine`（柱底 2 fine）。**全仓库没有贴地检测。**
- `validateStructureGraph`：无 world bond → `NotAnchored`（`OccupancySampler.cpp:158`）；有节点到不了锚 → `DisconnectedFromAnchor`（`:161`）。纯动态物体、带浮空块的物体都挂不上。
- `nodeReachesAnchor` 每个节点重建一次邻接表，**O(N²)**；3000 节点尚可，1 万节点以上会成为挂载瓶颈。
- 对比：`compactReplace` 在重建后**允许**无锚 family（`VoxelGraph.cpp:895-914`）。挂载与重建语义不一致。

### 1.4 载荷通道

- **E3 的 `addLoad` 路径实际未接通。** `buildLoadSnapshots`（`StructureWorld.cpp:1525`）每 tick 构建 `loadSnapshots`，随后 `:1266` 只把它们标记为 `evaluated`，**从不调用 `addLoad`**。
- 实际进 Blast 的只有 `applyViewerImpact`（`:1288`）：
  - `stressImpactImpulses_` 开：`addForce` 到最近节点（丢偏心力矩）；
  - 关（默认）：Shear/ImpactSpread 伤害着色器，不进应力。
- 后果：
  - Dynamic 物体压在 Static 结构上，结构感受不到这份持续重量；
  - 自由体结构（木板搭在两块石头上）只有离心载荷，自重下不会断。
- `skipCollidePair`（`PhysicsWorld.cpp:60-62`）跳过两侧都非 Dynamic 的配对：**Static–Static 之间没有接触，也就没有载荷**。

### 1.5 物体与数据

- 导入的 pirate hut 被 `stampMeshIntoWorld`（`VoxelScene.cpp:2412`）烤进地面物体（slot 0）；slot 0 被分裂逻辑显式排除（`VoxelSceneFracture.cpp:910, 959`）。**它不是独立物体，挂不了。**
- hut 包围盒 8.81 × 8.07 × 8.57 m ≈ 89 × 81 × 86 fine。按公式网格底 y≈4.0 m，而地面顶 3.2 m（未实机核对），可能悬空 0.8 m，贴地检测会失败。
- 密度：刚体用 `o.density`（缺省 600），结构图用 `view.density()`（1000）。连通碎块不复制父密度（`VoxelSceneFracture.cpp:301-312`）。
- VoxelGrid 稠密：约 6 B/格；另有每个实心体素一项 `unordered_map voxelNode`；`compactReplace` 每个 owner 拷贝一份整网格。hut 裁剪后约 62 万格 ≈ 4 MB/份，可接受；整张地面 1024³ 不可行。
- 结构**从不休眠**：`onPhysicsTick` 每 tick 对每个实例求解。E2 数据：3000 节点完整收敛约 0.3–0.5 ms，切口未收敛 8–27 ms。

---

## 2. 范围

### 2.1 E5 做

- 多实例正确性（§1.1）。
- 通用 occupancy view、挂载描述、挂载/卸载生命周期。
- 挂载时自动锚固（贴 static 地形），显式锚固谓词保留给回归场景。
- 无锚 Dynamic 结构、挂载时浮空块直接分裂。
- 持续接触载荷真正进入 `addLoad`（修 §1.4）。
- 每物体材料：密度、强度、聚合尺寸、迭代数；两条通道共用一个 ρ。
- 导入网格作为独立 Static 物体（不再烤进地面）。
- 最小休眠（不做完整预算调度）。
- UI：选中物体「设为结构」，按实例显示调试。

### 2.2 E5 不做

- 运行时新增锚固、焊接、两个物体运行时粘合（原方案非目标）。
- 在已挂载结构上放置体素后自动成键：E5 忽略并提示，留给后续。
- 多个 Static 物体之间传力：必须创作期合为一个 `VoxelObject`（§3.3）。
- 稀疏 VoxelGrid、跨 family 并行、帧预算调度、碎屑降级：P5。
- 混合材料 / 逐 bond 强度：原方案 P5。

---

## 3. 设计

### 3.1 数据与接口

```cpp
struct StructureMaterial {      // 每实例，不再继承 front()
  float densityKgM3;           // 与 VoxelObject::density 一致（唯一 ρ）
  float strengthPa;
  uint32_t solverIters;        // 默认 200
  bool fractureEnabled;
};

enum class AnchorPolicy {
  Auto,        // Static → GroundContact；Dynamic → None
  None,        // 自由体
  GroundContact,
  Explicit,    // 回归场景：圆柱、四柱沿用原谓词
};

struct StructureMountDesc {
  VoxelObjectId object;
  StructureMaterial material;
  AnchorPolicy anchors = AnchorPolicy::Auto;
  uint32_t agg = 0;            // 0 = 自动（§3.5）
  uint32_t maxNodes = 12000;   // 超出拒绝挂载并报原因
  float anchorPaddingFines = 0.5f;
  const DiagnosticProfile* diag = nullptr; // 圆柱/四柱的 strip、neck 统计，可空
};

StructureHandle StructureWorld::mount(const StructureMountDesc&, const OccupancyView&);
```

- 实例存 slot map（`StructureHandle{index, generation}`），不再用会重排的 vector 下标或裸指针。
- 所有接口改为按 handle 或 `VoxelObjectId` 定位；删掉 `instance()` 的隐式 front 语义。保留一个「UI 当前选中实例」，只供显示。
- `axis/keep*/cut` 移入 `DiagnosticProfile`；`rebuildBondMeta` 只在有 profile 时算 strip/neck。
- `setDensityScale` 的基准改为 `inst.material.densityKgM3`。
- `compactReplace` 产出的新实例继承 material 与 profile（它现在就复制这些字段，只是换成结构体）。

### 3.2 通用 view

`VoxelObjectView : OccupancyView`（已实现为 `blast::ObjectOccupancyView`，见 §4.1）：

- 范围：E5.1 覆盖物体整个 fine 网格（`gridSize × 16`，原点 0），与原圆柱/四柱专用 view 相同，回归逐位一致。裁剪推迟到 E5.5：节点与键坐标经 `grid.ox` 平移后浮点运算顺序改变，可能差几个 ulp，回归场景无法再逐位对比。裁剪时起点须对齐 agg，`origin` = 裁剪偏移，写入 `structureFineOrigin` 与挂载时 binding 的 `fineOrigin`。
- `solid` = `occupancyFine`；`density` = 物体 ρ。
- `anchor` = 按 `AnchorPolicy` 查预先算好的锚固位图（§3.4），不在采样循环里做世界查询。
- 性能：采样前把 fine 占用按 brick 批量解码到一块连续位图，避免每格一次虚调用加 `splitFineIndex`。E5 验收记录挂载耗时。

### 3.3 什么物体可以挂

| 物体 | 默认 | 说明 |
|---|---|---|
| 导入网格（hut 等） | 可挂，Static，Auto 锚 | 必须先作为独立物体生成（§3.7） |
| 场景 Static 建筑 | 可挂 | 同上 |
| Dynamic 物体 | 可挂，None 锚 | 只受离心力 + 接触载荷 |
| 结构分裂出的碎块 | 已在结构内 | 现有 binding 路径，不重复挂 |
| 连通掉块（未挂载父物体） | 不挂 | 保持旧行为 |
| 地面 slot 0、spinner（Kinematic） | 禁止 | 地面作锚源；Kinematic 按集成文档 §5.2 不进结构 |
| scatter 箱 | 默认不挂，可手动 | 压测用 |

**共同承重规则**：两个 Static 物体面接触、需要互相传力时，创作期合并成一个 `VoxelObject` 再挂（对应 Blast `MergeAssets`）。挂载时若检测到 fine 面贴着**另一个已挂载结构**，默认拒绝并提示合并。原因：Static–Static 没有接触载荷（§1.4），拿对方当锚会在对方断裂后留下幽灵支撑。

### 3.4 自动锚固（GroundContact）

仅对 Static 物体、仅在挂载时执行，结果写入 `VoxelGrid::anchor` 后不再变化：

1. 取物体的边界 fine（至少一个面邻格为空，或位于网格边缘）。
2. 对每个边界面，把「面中心 + 法向 × padding」变换到世界，再变换到候选锚源的局部坐标。
3. 候选锚源 = 与物体世界 AABB（外扩 padding）相交的 Static 物体中，**未挂载**的（地面、不可破坏地形）。用现有宽相位树查询。
4. 候选点落在锚源实心 fine 内 → 本 fine 为锚固，所在节点得到 world bond（不可断，与 `createVoxelBlast` 一致）。
5. `anchorPaddingFines = 0.5` 对应 omni.blast `relativePadding`，吸收对齐误差；旋转物体同样适用。

锚固结果进调试叠加层，可视化锚固 fine。

**无锚 Static 物体**：自动检测后没有锚点，默认挂载失败并提示「悬空」。可选择转成 Dynamic 后按 None 挂载，不静默造锚。

### 3.5 聚合尺寸与节点预算

- 允许的 `agg` 为 {1, 2, 4}，默认 2（fine 0.1 m → 节点 0.2 m）。
- 粗格内已按连通分量拆节点（`extractGraph` 在 agg³ 块内做 flood fill），不会把隔空部分并成一个节点。
- 原方案 §1.1「聚合大于壁厚会吞壳/填腔」：在分量拆分下不会真填腔，但节点质心会离开壳面、连接面积被粗化。所以 agg=4 只在节点超预算时自动尝试，并报告 T19 式对比。
- `agg = 0`（自动）：先用 2；节点数 > `maxNodes` 时试 4；仍超出则拒绝挂载，报告节点数。不静默丢细节。
- `validateStructureGraph` 的锚固可达性改为从所有 world bond 节点做一次多源 BFS，O(N+B)。

### 3.6 浮空块与无锚图

- `NotAnchored`：只对 `GroundContact` 策略报错；`None` 策略合法。
- `DisconnectedFromAnchor` 不再拒绝：挂载后立即 `splitAllRequired`，浮空岛按现有 split 路径成为 Dynamic 碎块（与 Teardown 的「static 上不连通就变动态」一致）。挂载报告浮空岛数量。
- 纯无锚图直接 `createVoxelBlast`（`compactReplace` 已支持同类情况）；`bindings.anchored = false`，只走离心力分支。

### 3.7 导入网格成为独立物体

- 新增 `importMeshAsObject(path, pose, density, motion)`：体素化进新 `VoxelObject`，不再调用 `stampMeshIntoWorld`。旧入口保留给纯装饰场景。
- 落地对齐：体素化后把最低实心 fine 的底面对齐地面顶（或导入时给定 pose），修正 §1.5 的 0.8 m 悬空疑点。实机核对后写进验收记录。
- `gridSize` 是立方 coarse：hut 需 6 coarse（96³ fine）。稠密结构网格按 occupancy 裁剪（§3.2），不用整个立方。

### 3.8 载荷（修复 E3 通道）

每个 tick、每个 `stressSolve` actor，分三类：

| 载荷 | 条件 | API |
|---|---|---|
| 自重 | actor 有 world bond | `addGravity`（现有） |
| 离心 | 无 world bond | `addCentrifugalAcceleration`（现有） |
| **持续接触** | `persistent` 接触，含 Static 侧反力 | `addLoad(node, F, τ)`，`F = J_tick / kDt` |
| 单次撞击 | `eventId` 接触 | 现有 Viewer 路由：着色器伤害或 `stressImpactImpulses` |

- 持续接触用已经构建好的 `loadSnapshots`，补上 `addLoad` 调用。力矩按集成文档 §6 在 asset-local 下计算，不退回最近节点 `addForce`。
- **防重复**：同一接触点在一个 tick 内只进一个通道。persistent 进 `addLoad`；event 走 Viewer 路由；`stressImpactImpulses` 开时 event 不再进 `addLoad`。
- 有锚 actor 自身落在锚源上的接触不存在（Static–Static 被跳过），不会和 world bond 反力重复。
- Dynamic 物体放在 Static 结构上：结构侧得到对侧 `-J`，这正是集成文档 §6「只映射动态侧是错的」。
- 自由体结构：只有接触载荷加离心力。自由落体无接触时新增内力应近零（原方案 T05）。

需要先确认的一点：Viewer 对齐提交把默认冲击改成伤害着色器，与原方案「冲击只进应力」冲突。本节只恢复**持续**载荷，不改 Viewer 冲击默认值；两者分工写进 UI 提示。

### 3.9 最小休眠

E5 的挂载数量会超过 2 个，每 tick 全量求解不可接受。只做正确性最低限度：

实例（按 actor）可跳过 `update()` 的条件，**全部**满足才跳过：

- 上次已收敛，且无超限候选；
- 拓扑 epoch、材料 epoch 未变；
- 本 tick 无新的持续接触变化（按合力、合力矩与上次相对差 < 1%）且无 event 接触；
- Dynamic actor 的刚体处于睡眠，或角速度 < 阈值。

唤醒条件：挖洞/重建、断裂、强度或密度调整、有接触事件、刚体醒来、邻近结构断裂（`activateBodiesInBounds` 已有的邻域唤醒）。

原方案 §10.2 约束照旧：「有超限但预算用完」的结构不得休眠。完整预算调度留给 P5。

### 3.10 生命周期

- `freeObjectSlot`：任何被 `ownsObject` 覆盖的物体都要 unmount 或从 binding 中移除；不能只处理 `stressCylinderId_`。
- `unmount(handle)`：释放 family/asset/solver；把所有子 binding 对应的物体恢复为普通刚体。
- 在已挂载物体上放置体素（F 键）：E5 拒绝并提示。删除体素走 E4。
- 场景重置：`StructureWorld::clear` 不变。

---

## 4. 实施顺序

每一步都要让 P0–P4、E0–E4、Viewer 对齐测试保持绿。

| 步 | 内容 | 退出 |
|---|---|---|
| E5.0 | 多实例重构：slot map、handle 接口、`StructureMaterial`、`DiagnosticProfile`、去 `front()`、`freeObjectSlot`/`unmount`、多源 BFS | 圆柱与四柱同时挂载，互不串参；现有测试全绿 |
| E5.1 | `VoxelObjectView` + `mount(desc)`；圆柱/四柱改走通用入口 + Explicit 锚 | 与旧专用 view 产出的节点/键/world bond/支反力逐项一致 |
| E5.2 | GroundContact 自动锚固 + 调试叠加层 | 圆柱改 Auto 锚后与 Explicit 结果一致；旋转与悬空用例 |
| E5.3 | 放开无锚挂载与浮空岛分裂 | 自由体、浮空块用例 |
| E5.4 | 持续接触 `addLoad`，并与 Viewer 冲击分通道 | 压重与自由梁用例；冲量只消费一次 |
| E5.5 | `importMeshAsObject`、落地对齐、统一密度 | hut 独立挂载站立 |
| E5.6 | 最小休眠 | 多实例静止开销近零 |
| E5.7 | UI：设为结构、实例列表、锚固显示；验收文档 `docs/blast-e5.md` | §5 全部通过 |

E5.0 和 E5.1 是纯重构，不改行为，适合先单独提交。

### 4.1 进度

**E5.0 完成（2026-09-24）**

- `StructureHandle`（单调递增 id，永不复用）；实例改存 `unique_ptr`，地址在挂载/重建中稳定。`find(handle)`、`find(VoxelObjectId)`。
- `StructureMaterial`（baseDensity / strengthPa / solverIters / fractureEnabled）取代散字段；`DiagnosticProfile`（None / CylinderStrip / ColumnBox）取代 `axis*/keep*/cutApplied`，无 profile 时不算 strip/neck。
- `mount(StructureMountDesc)`：只用 desc 的材料；`keepMaterialOfReplaced` 时仅从**同一物体**的旧实例继承强度、断裂开关与 strengthEpoch，不再从第一个实例继承。旧 `mountSample` 保留为测试用包装。
- 挖洞重建后，仍属于原根物体的实例沿用旧句柄；新分出的实例拿新句柄。
- `setDensityScale / setSolverIters / markCut / setKeepColumnBox / warmupGravity / recacheOccupied / debug` 都接受目标实例；省略时仍指第一个实例，只供单结构测试。场景、`main_voxel`、UI 全部显式传实例（`VoxelScene::stressStructure()`、`debug(VoxelObjectId)`）。
- `unmount(id)` 同时解除其他实例 binding 中对该物体的引用；`freeObjectSlot` 对任何物体都执行，不再只认圆柱。
- 全局 `setStrengthPa / setFractureEnabled` 仍作用于所有实例（UI 每帧调用），逐实例 UI 留到 E5.7。

**E5.1 完成（2026-09-24）**

- `src/blast/ObjectOccupancyView.h`：整网格、物体密度、fine 尺寸 = `voxelSize / 16`、锚点为显式谓词。
- `VoxelScene::mountObjectStructure(id, agg, anchorFine, desc)`；圆柱与四柱改走此入口，专用 view 已删除。

**验收**

- `blast_e5_tests`：通用 view 与复刻的旧专用 view 在圆柱完整/切口、四柱完整/切口上，采样结果（掩码、节点、键、面、坐标、质量）逐位相同；`mount(desc)` 与 `mountSample` 的预热求解（支反力、最大应力、残差、候选）逐位相同；四柱失效强度 4 tick 后仍逐位相同。多实例隔离、句柄失效、定向设置、重建句柄、卸载解绑全部通过。
- 引擎 `--frame-fail --frame-height 4 --frame-impact shear --frame-render-hz 60`：改动前后 730 行输出（屏蔽毫秒值后）完全一致，最终 `single-node=20 multi-node=6 largestNodes=66`。
- 引擎 `--e2-perf`：圆柱生成、切 270° 重挂载、失效分裂流程通过（`split=yes`）。改动前未录圆柱引擎基线，圆柱等价性以无头测试为准。
- 其余 18 个测试（P0–P4、图回归、E0–E3、四柱、Viewer 对齐、探针导出、物理）全部通过。
- 补充：圆柱（保持强度、断裂关）与切口四柱（失效强度、断裂开）同时挂载，各运行 6 tick，与各自单独挂载时的求解结果逐位相同。

**E5.2 完成（2026-09-26）**

决定（用户确认方案 A）：自动锚点只标记**实际接触的那一层**，锚固面积 = 真实接触面积。验收标准由「与手写规则逐位一致」改为「锚固节点集合相同，支反力相对误差 ≤ `kE1ReactionRelTol`（1e-3）」。回归场景继续用手写规则。

- `src/blast/GroundAnchors.{h,cpp}`：不依赖场景的纯函数 `findGroundAnchors`。实心体素的每个外露面，取「面中心外 0.5 格」（即相邻格中心）换到各静态来源的局部坐标；落在来源实心格内即为接触。容差为半格：间隙 < 0.05 m 算接触。
- 来源：未挂载、启用中的 Static 物体（地面等），不含自身。已挂载结构的面记为 `blockedFaces`，不产生锚点。
- `VoxelScene::findGroundContactAnchors / mountObjectOnGround`：只接受 Static 物体；有 blocked 面则拒绝并提示合并；无接触则拒绝（悬空）。挂载状态写入 `structureMountStatus()`。
- UI：「Ground-contact anchors (E5.2)」开关（下次生成/切割生效）、「Show anchors (green)」锚点高亮、面板「Anchors:」状态行。
- 引擎：`--frame-anchors ground|explicit`（默认 explicit）。

**验收**

- `blast_e5_tests` 边界：静置箱体只锚 8×8 底面；间隙 0.04 m 仍接触、0.06 m / 0.1 m 不接触；绕 Y 旋转 30° 不变；绕 Z 倾倒 90° 改为锚 x=4 侧面；压在已挂载结构上全部 blocked；一半压在结构上时一半 blocked、一半锚固；远离地面时无锚点。
- 回归几何（场景地面与生成位姿）：锚点恰为底层接触体素；锚固节点集合与手写规则相同；锚固面积为手写规则的一半。

  | 场景 | 锚点体素 | 锚固节点 | 重量 W (N) | Ry 手写 | Ry 自动 | 相对差 |
  |---|---:|---:|---:|---:|---:|---:|
  | 圆柱 | 244 | 100 | 143618 | 143625 | 143626 | 8.9e-6 |
  | 四柱 | 144 | 16 | 86641.9 | 86642 | 86642 | 0 |
  | 四柱切口 | 144 | 16 | 46381.7 | 46404.4 | 46384.3 | 4.3e-4 |

- 引擎 `--frame-fail`（手写锚点）：与 E5 前基线 730 行差异 0。
- 引擎 `--frame-fail --frame-anchors ground`：`anchors=ground fines=144 faces=144 blocked=0`，站立与切柱后均收敛，塌落结果 `single-node=20 multi-node=6 largestNodes=66`，`OK frame-fail`。
- 19 个测试全部通过。

**E5.3 完成（2026-09-26，用户手动验收通过；按用户要求未新增自动测试）**

- `OccupancySampleOpts::allowFloating`（默认关）：接受无 world bond 的图和到不了锚点的节点。`validateStructureGraph(..., requireAnchored)` 的可达性检查改为从所有锚点出发的一次多源 BFS（原为每节点一次搜索，O(N²)）。
- `mountObjectStructure(..., allowFloating)`：图有多个连通分量时，挂载后立即 `splitAllRequired(force)` 并标记 `occupancyDirty`，下一次结构提交按现有分裂流程把每块变成物体，无锚的块为 Dynamic。完整连通的物体不分裂，E5.2 行为不变。
- `mountObjectOnGround` 允许浮空块；Static 物体完全不接触地面仍拒绝。
- `mountObjectFree`：只接受 Dynamic 物体，无 world bond、无假锚点，只受离心力与撞击载荷。`mountObjectAuto`：Static 走贴地锚点，Dynamic 走自由体，Kinematic 拒绝。
- 演示：「Spawn free plank (E5.3)」（4 m 木板，ρ=600，倾斜，从约 4 m 高落下；Lift & drop again 可重复）、「Spawn pillar + floating block (E5.3)」（同一物体内的立柱与浮空方块）。
- 回归：19 个已有测试通过；`--frame-fail` 手写锚点与 E5 前基线、自动锚点与 E5.2 结果均逐行一致。

---

## 5. 验收

新增 `tests/blast_e5_tests.cpp`，另在引擎内加 `--e5-*` 回放场景。阈值在实现前固定。

| ID | 内容 | 通过条件 |
|---|---|---|
| E5-T01 | 通用入口等价 | 圆柱、四柱经 `VoxelObjectView` 挂载，节点/键/world bond 数与旧 view 相同；完整圆柱支反力相对误差仍 ≤ 1e-4 |
| E5-T02 | 自动锚固等价 | 圆柱 Auto 锚固集合 = Explicit 底座集合 |
| E5-T03 | 多实例隔离 | 两个结构不同强度/密度同时运行；一边挖洞重建，另一边状态、强度、诊断不变；实例顺序变化后 handle 仍正确 |
| E5-T04 | 锚固边界 | 物体离地 1 fine → 无锚（Static 报错，转 Dynamic 后下落）；贴地 → 有锚；旋转 30° 物体仍正确；贴另一个已挂载结构 → 拒绝并提示 |
| E5-T05 | 浮空块 | 挂载时的浮空岛立即成为 Dynamic 碎块，主体保留锚固；质量守恒 |
| E5-T06 | 自由梁 | Dynamic 长梁两端搁在地面块上：中段应力由接触载荷产生；与同几何两端锚固的静力解同量级（记录比值）；抬高后自由落体时新增内力 < 参考的 0.1% |
| E5-T07 | 压重 | Dynamic 重物放到 Static 结构顶上：危险键应力随重物质量单调上升；移走后回落；弱材断、高强度不断 |
| E5-T08 | 通道唯一 | 同一接触在一个 tick 内只进 `addLoad` 或 Viewer 路由之一；event 只消费一次 |
| E5-T09 | 质量一致 | 刚体质量（`computeMassProperties`）与结构图总质量相对误差 ≤ 1e-4；碎块继承父 ρ |
| E5-T10 | 导入建筑 | hut 独立挂载：自重下不破；挖掉承重部分后按应力坍塌；落地可二次断裂；30/60 Hz 渲染断裂序列一致（ImpactSpread 除外，按 Viewer 对齐文档记录） |
| E5-T11 | 节点预算 | 超 `maxNodes` 时自动试 agg 4，仍超则拒绝并给出节点数；报告 hut 在 agg 2/4 下的节点、键、临界载荷、挂载耗时（T19 格式） |
| E5-T12 | 休眠 | N 个静止已收敛结构每 tick 结构开销 ≈ 0（记录 p95）；接触、挖洞、改强度都能唤醒；有超限候选时不休眠 |
| E5-T13 | 生命周期 | 反复挂载/卸载/删除物体 N 次后 Blast live bytes 回到基线；无悬空 binding |
| E5-REG | 回归 | 不挂任何结构时，现有 demo、连通掉块、堆叠行为不变；P0–P4、E0–E4 全绿 |

---

## 6. 风险

| 风险 | 症状 | 处置 |
|---|---|---|
| 大图不收敛 | 真实建筑在迭代预算内达不到容差 → 收敛门控下永不断裂（E2 切口已见 8–27 ms 未收敛） | 载荷不变时跨 tick 热启动继续收敛（原方案 §9.3 允许）；记录收敛所需 tick 数；不放开「未收敛也断」 |
| 稠密网格内存 | 大物体 + `compactReplace` 每 owner 一份拷贝 | E5 按 occupancy 裁剪 + 格数上限；稀疏网格留 P5 |
| 挂载耗时 | 采样虚调用、`voxelNode` 哈希表、asset 创建 | 只在加载/显式操作时挂；E5-T11 记录；不在模拟中途批量挂 |
| 锚固误判 | padding 过大把邻近物体当地面；过小使贴地物体悬空 | 可视化锚固；固定 0.5 fine，由测试覆盖 |
| Static–Static 无载荷 | 分开建的屋顶与柱子之间不传力 | 规则：创作期合并；检测到相贴已挂载结构时拒绝 |
| 冲击路线未定 | Viewer 着色器伤害与原方案硬阈值 + 冲击载荷判据冲突 | E5 只修持续载荷；冲击默认值由用户决定后再定 T07/T10 阈值 |
| 自动 agg 改变断点 | agg 4 使临界载荷偏移 | 只在超预算时启用并报告；薄壳场景仍按原方案 §1.1 锁 1 或 2 |

---

## 7. 资料

1. `NvBlastTypes.h`（world bond 定义）：https://raw.githubusercontent.com/NVIDIA-Omniverse/PhysX/main/blast/include/lowlevel/NvBlastTypes.h
2. `NvBlastExtAssetUtils.h`（AddExternalBonds）：https://raw.githubusercontent.com/NVIDIA-Omniverse/PhysX/main/blast/include/extensions/assetutils/NvBlastExtAssetUtils.h
3. `NvBlastExtAuthoring.h`（FindAssetConnectingBonds / MergeAssets）：https://raw.githubusercontent.com/NVIDIA-Omniverse/PhysX/main/blast/include/extensions/authoring/NvBlastExtAuthoring.h
4. `NvBlastExtStressSolver.h` / `.cpp`：https://raw.githubusercontent.com/NVIDIA-Omniverse/PhysX/main/blast/include/extensions/stress/NvBlastExtStressSolver.h
5. ExtStress 用户指南：https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/extensions/ext_stress.html
6. omni.blast C++ API（createExternalAttachment）：https://docs.omniverse.nvidia.com/kit/docs/omni.blast/latest/_build/docs/omni.blast/latest/structcarb_1_1blast_1_1_blast.html
7. omni.blast CHANGELOG：https://docs.omniverse.nvidia.com/kit/docs/omni_blast/latest/source/extensions/omni.blast/docs/CHANGELOG.html
8. Blast CHANGELOG（1.1.0 world bonds）：https://nvidia-omniverse.github.io/PhysX/blast/docs/CHANGELOG.html
9. Teardown modding API：https://teardowngame.com/modding/api.html ；多人博客：https://blog.voxagon.se/2026/03/13/teardown-multiplayer.html
10. Space Engineers `MyAdvancedStaticSimulator.cs`：https://raw.githubusercontent.com/KeenSoftwareHouse/SpaceEngineers/master/Sources/Sandbox.Game/Game/GameSystems/StructuralIntegrity/MyAdvancedStaticSimulator.cs
11. 7 Days to Die Structural Integrity：https://7daystodie.wiki.gg/wiki/Structural_Integrity
12. Valheim Building Stability：https://valheim.gamecore.wiki/en/buildings/building-stability/
13. Chaos Destruction 概览：https://dev.epicgames.com/documentation/en-us/unreal-engine/chaos-destruction-overview?application_version=4.27 ；Dataflow Proximity / AutoCluster / SetAnchorState 节点参考
14. GMTK「How games do destruction」：https://gmtk.substack.com/p/how-games-do-destruction

未找到：Blast 官方节点规模建议、omni.blast 用户手册正文、Red Faction Guerrilla GDC 技术细节、Teardown 应力求解公开说明。
