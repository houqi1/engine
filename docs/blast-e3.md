# E3 接触冲量入结构

> 日期：2026-09-14。刚体仍由 PhysicsWorld 积分；结构通道不 `applyImpulse`。
> 探针每 tick 只导出一次。E2「未收敛但 >2S 仍断」已从正式路径去掉。

## 顺序

1. 六个子步求解接触，**积分前**记录冲量（法向不含 Baumgarte；切向累加 `dLamT * t`）。
2. `J_tick = Σ J_sub`，`F = J_tick / kDt`（不是 `kSubDt`）。
3. 命中体素 → 该 actor 的结构节点；否则在该 actor 节点中取最近并补偿力矩。
4. 有锚：自重 + 接触。无锚动态块：只加载接触，不造假 world 锚。
5. `solver.update()` → 一次 `copyBondProbes` → 仅收敛后筛选候选 → 复用 E2 提交。
6. 分裂后 `bindVisibleActors`：每个可见 actor ↔ VoxelObjectId ↔ 刚体。

## 头测

```
cmake --build build --config Release --target blast_e3_tests blast_probe_export_tests blast_e2_tests
build\Release\blast_e3_tests.exe
```

## 实机

Simulate 开。撞击已锚固结构或让多节点碎块落地。Fail strength + Stress fracture。高强度对照应只发生刚体碰撞。
