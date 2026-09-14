# Blast 原型基础限制修正（连接遍历 / 稳定 ID / owner / 重建事务）

> 日期：2026-09-13。范围：`src/blast`、对应测试、本页与 `docs/blast-p4-params.md`。
> 不进入：圆柱渲染、碰撞接入、`vulkan_engine_voxel` 链接、稀疏局部重建。
> 完成后下一步才是 E0（空 `StructureWorld`）。

本页记录的是 P4 头测已经通过之后、引擎接入之前的正确性修复。P0–P4 原阶段测试仍要绿。新增回归集中在 `blast_graph_regression_tests`。

## 1. 完整连接遍历及 actor 过滤

`copyBondProbes` 是 family 级接口，按 asset bond 下标 0..N-1 填写。旧实现申请 512 个 probe，求解器只返回能装下的连接，下标 ≥ 512 的可破坏键被静默跳过。

现规则：

- 从 actor 的 family → asset 读取 `NvBlastAssetGetBondCount`，按该数量准备 probe。
- 返回数量必须覆盖 asset 全部连接（回归里断言 `nProbe == bondCount`）。
- 仅处理：属于当前 actor、`canTakeDamage`、health > 0、应力 **严格大于** 阈值。
- 求解器未收敛则不执行断裂。
- 归属：两端 support 节点都在当前 actor 的 graph node 集合内，或一端在当前 actor、另一端是 world 节点。另一个 actor 的键不得提交给当前 actor。

缓冲复用未做。

| 测试 | 必须证明 |
|---|---|
| R1 520 格链 | asset 键数 > 512；sdk 下标 512 的 health 在超阈值后为 0 |
| R1 未超限对照 | 同一键在阈值远高于应力时 health 不变；下标 0 也不无故断裂 |
| R1 双 actor | 只对 actor A 执行判断；actor B 的指定键 health 不变；A 的指定键归零 |
| R1 边界 | 等于阈值不破；略超阈值该键完全断；不收敛不破 |

测试检查的是指定 sdk 下标的 health，不是“有连接断了”。

## 2. 稳定 ID 与损伤迁移

节点 ID：先抽出全部新连通块并统计与旧节点的体素重叠；每个旧 ID 最多给一个新节点；重叠多的优先；数量相同则按体素坐标 `(x,y,z)` 最小者，不依赖哈希遍历顺序。其余节点从实例持有的单调计数器分配。计数器耗尽（含回绕到 0）返回 `IdExhausted`，图保持重建前状态。

连接 ID：不再用截断拼接。节点对只作查询键；身份与损伤迁移还要看实际面集合。

- 节点对与来源都未变的连接保留 ID 和损伤。
- `brokenFaces` 仍是断面权威，重建不得把已断面重新连上。
- 节点拆分后按旧面归属迁移 `d`；不能因为节点换了 ID 就恢复满强度。
- 一条新连接的面来自多于一条旧连接 → `AmbiguousDamage`，拒绝该次重建。

| 测试 | 必须证明 |
|---|---|
| R2 二分 / 三分 | 新 ID 全部唯一；旧 ID 只留给一块 |
| R2 连续切割 | 新节点不复用已退役 ID |
| R2 重复建图 | 相同输入得到相同 voxel→id 映射 |
| R2 节点编号 > 65535 | 连接 ID 不碰撞，且不是 `(nodeA<<16)\|nodeB` |
| R2 拆分迁移 | 4 面 `d=0.4` 拆成两个 2 面连接后 `d` 与 `A_eff` 仍正确 |
| R2 断面 | 任意多次重建都不把已断面连回 |
| R2 歧义 / 耗尽 | 图与计数器保持失败前状态 |

## 3. owner 类型与归属

`OwnerId = uint32_t`。0 的两种用法按调用点区分，值相同、语义不同：

- `VoxelGrid::owner` / `GraphNode::owner` / `CompactFamily::owner`：`kUnowned`，尚未划给有效 actor。
- `extractGraph(..., ownerFilter)`：`kOwnerAll`，查询全部；过滤某一个 owner 时 **不** 收入未归属体素，也不跨不同非零 owner 走连通。

写回：

- 每个有效体素至多一个有效 actor；有固体却无人认领视为 `OwnerConflict`。
- 分配到 `uint32_t` 上限则 `OwnerCapacity`，不回绕到 0。
- 无可见 chunk 的 actor 不分配几何 owner。

| 测试 | 必须证明 |
|---|---|
| R3 16×17 单格 family | >255 个实际 actor；owner 不重复、不为 0 |
| 同上 | 体素无遗漏、无重复；分裂前后占用体素数一致 |
| 同上 | 每个 owner 对应带可见 chunk 的紧凑实例 |
| R3 过滤 | 按 owner 抽图不会吃掉另一个 actor 的体素 |

## 4. 重建事务

`compactReplace` 流程：

```
读旧实例（不改调用方）
    → 在网格副本上算 actor 归属
    → 逐个构建替代图 / asset / family / solver
    → 检查 ID、体素归属、质量、断面、SDK 映射
    → 全部成功：交换网格、释放旧实例、交出替代集合
    → 任一失败：临时 VoxelBlast 析构释放 solver 与对齐块，旧实例仍可求解
```

提交阶段不再创建 asset。全部体素被删除是合法空结果（`out` 清空、旧资源释放），不是构建失败。

`VoxelBlast` 不可拷贝；移动后源指针置空。`createVoxelBlast` 任一阶段失败都会 `destroyVoxelBlast`。

故障注入经 `CompactReplaceOpts`（仅测试）：

| 故障 | 预期 |
|---|---|
| `failAfterCreates = 0` | 旧实例不变，live bytes / live allocs 回到调用前基线 |
| `failAfterCreates = 1` | 第一个临时实例也释放，旧 family 仍可 `update` |
| `failMappingCheck` | 不提交，返回 `MappingInvalid` 及原因字符串 |
| 全部成功 | 新实例可用，旧 `family`/`solver` 为空 |
| 全部体素删除 | 成功提交空结构，旧资源释放，live bytes 低于创建后 |

内存比较的是 **每次失败前后的分配基线**，不是进程退出时是否归零。

整网格复制成本：每个 owner 一份 `VoxelGrid` 拷贝。本轮不把它改成稀疏局部重建。

## 5. 尚未接入引擎的边界

- `vulkan_engine_voxel` 仍不链接 `nvblast`，仍不编译 `src/blast/*`。
- 没有 `StructureWorld` / OccupancySampler / PhysicsBridge。
- 体素场景、圆柱渲染、刚体碰撞几何都不在本轮验证。
- P4 `VoxelGraph` 仍不写 world bond；站立楼底座锚固要等 OccupancySampler。
- E0 已在 2026-09-13 落地（`docs/blast-e0.md`）。应力圆柱仍是 E1。

## 6. 运行

```bat
cmake --build build --config Release --target blast_p0_tests blast_p1_tests blast_p2_tests blast_p3_tests blast_p4_tests blast_graph_regression_tests
build\Release\blast_p0_tests.exe
build\Release\blast_p1_tests.exe
build\Release\blast_p2_tests.exe
build\Release\blast_p3_tests.exe
build\Release\blast_p4_tests.exe
build\Release\blast_graph_regression_tests.exe
```

本机 Release 实测（2026-09-13）：

```
blast_graph_regression_tests NvBlast 5.0.6 sha 7ef568f5b557a6dad9023ecebc43cb809270e035
R1 large graph fractures sdk bond >= 512
  asset bonds=519 graph bonds=519
R3 family with >255 actors stamps unique nonzero owners
  actors=272 solids=272
OK graph regression R1-R4
```

同日 P0–P4 Release 全通过（`resets=50 liveBytes=0`；P1 T01–T06/T21/T22/T24；P2 T07–T09；P3 T10–T13；P4 T14–T17）。
