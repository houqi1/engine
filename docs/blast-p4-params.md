# P4 体素粗图与 family 重建（T14–T17）

对应执行方案第 6 节与第 13 节 P4。
断面以断开的面集合为权威；几何挖除只减 `A_geo`，不把 `d` 再扣一遍。`A_eff = A_geo * (1 - d)`。首版 family 退役是全部活跃 actor 各做一份紧凑 asset/family 后原子替换。

| 项 | 值 |
|---|---|
| 聚合 | 测试用 2 或 4（厚实体） |
| 连接 | 六邻域面，边/角不连 |
| 粗格内 | 未断面连通分量各成节点 |

运行：

```bat
cmake --build build --config Release --target blast_p4_tests
build\Release\blast_p4_tests.exe
```

本机 Release 实测（2026-09-10）：

| 测试 | 结果 |
|---|---|
| T14 | 4³ 粗格中间掏空后 2 节点 0 键，无跨空隙连接 |
| T15 | 断开面重建后不愈合；另一键 `d=0.4` 保留，health = A_eff |
| T16 | 界面 4 面删到 2 面，`d` 不变，质量下降 |
| T17 | split 成 2 actor 后 compact replace 两份 family；未编辑侧体积保留，旧 family 释放 |

本阶段未接到 `VoxelScene` 交互挖洞。

## 本轮基础限制修正（P4 之后、E0 之前）

P4 头测仍覆盖 T14–T17。下列正确性漏洞在接入引擎前单独修，测试目标是 `blast_graph_regression_tests`，不扩大 P0–P4 职责。

| 限制 | 修复 | 仍未做 |
|---|---|---|
| `applyHardThreshold` 固定 512 个 probe | 按 family/asset 实际 bond 数取 probe；只把当前 actor 的连接交给 `NvBlastActorApplyFracture` | 缓冲复用 |
| 多个新连通块可继承同一节点 ID；bond ID 用 `nodeA<<16\|nodeB` | 旧 ID 全局最多分给一个新节点；bond 用独立计数器；损伤按旧面集合迁移；歧义则拒绝重建 | 运行时焊接/节点合并 |
| `owner` 为 `uint8_t`，超过 255 个碎块回绕 | 统一 `OwnerId = uint32_t`；0 只表示未归属/查询全部；写回检查一对一；无可见 chunk 不分配 | 与 `VoxelObjectId` 对表 |
| `compactReplace` 失败仍可能改旧状态或漏释放 | 准备阶段写临时状态；solver 随 `VoxelBlast` RAII 释放；提交只交换完整状态；全删是合法空提交 | 稀疏局部重建 |

当前 `compactReplace` 仍对每个 owner **复制整份稠密网格**（`solid` + `owner`）。K 个 owner、N 个体素时准备阶段额外峰值约 K 倍网格。这是正确性优先的成本，不是性能验收。稀疏局部重建留到实测之后。

运行：

```bat
cmake --build build --config Release --target blast_graph_regression_tests
build\Release\blast_graph_regression_tests.exe
```

本轮完成后才能进入 E0：主程序链接 Blast、创建空 `StructureWorld`，验证旧场景行为不受影响。圆柱渲染和碰撞接入仍不在本轮。

验收条目与实测见 `docs/blast-graph-regression.md`。本机 2026-09-13 Release：`blast_graph_regression_tests` 与 P0–P4 均 exit 0。
