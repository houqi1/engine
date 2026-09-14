# E0 主程序链接与空 StructureWorld

> 日期：2026-09-13。依赖上一轮图回归修复已通过。
> 不进入：应力圆柱、重力求解、应力断裂、接触载荷、OccupancySampler。

## 基线（改代码前）

| 项 | 结果 |
|---|---|
| 上一轮四项修复 | 已在 `HardFracture.h` / `VoxelGraph` / `OwnerId` / `compactReplace` |
| P0–P4 + `blast_graph_regression_tests` Release | 全通过 |
| `vulkan_engine_voxel` Release | 已能构建（当时尚未链 Blast） |

## 构建

```
nvblast
    ↑
ve_structure_core   ← BlastMemory, VoxelGraph, ContactLoads, StructureWorld
    ↑                 ↑
Blast 测试       vulkan_engine_voxel
```

圆柱 `CylinderGraph` / `ActorBodies` 仍只链进 P1/P2 测试。不引入 PhysX / Tk。SDK SHA 不变。

## 生命周期

```
BlastRuntime::init          // 安装分配器与错误回调
    → VoxelScene::init      // StructureWorld::init + 绑定 tick 回调
    → 帧循环 PhysicsWorld::step → StructureWorld::onPhysicsTick
    → 重置：clear 实例 + resetTickSession；运行时保留
    → VoxelScene::cleanup   // 先停 tick，再 shutdown StructureWorld
    → BlastRuntime::shutdown
```

`VoxelObject` 不持有 Blast 指针。tick 回调是函数指针 + userdata，不因 `vector` 扩容失效。

## 固定步

`physics::FixedStepClock` 就是 `PhysicsWorld::step` 用的累加器：

- 通知次数 = 实际完成的物理 tick（最多 `kMaxStepsPerFrame=2`）。
- 传入 `kDt=1/60`，不是 `frameDt`。
- `1/120` 两帧才 1 个 tick；`1/30` 一帧 2 个 tick；超长帧截断，不补被丢掉的时间。
- Simulate 关闭时 `step` 不跑，结构侧不计数。

空实例时 `onPhysicsTick` 只写计数器，不建图、不跑 solver、不每 tick 分配。

## 测试

```bat
cmake --build build --config Release --target blast_e0_tests vulkan_engine_voxel
build\Release\blast_e0_tests.exe
```

| 项 | 通过条件 |
|---|---|
| 运行时 | init 前拒绝 StructureWorld；shutdown 可重复 |
| 空世界 | 实例数 0；init 不增加 live bytes |
| 固定步 | 与 `kDt` / cap 一致；编号单调；reset 后从 1 再计 |
| 零步帧 | `1/120` 第一帧不调用 |
| 多步帧 | `1/30` 两次调用 |
| 空 tick | live bytes 不变 |
| 连续 clear | 50 次后 live bytes / live allocs 回到基线 |

## 本机 Release 实测（2026-09-13）

头测：`blast_e0_tests`、P0–P4、`blast_graph_regression_tests` 全部 exit 0。

主程序：

| 步骤 | 结果 |
|---|---|
| `--help` | 原 CLI 不变，exit 0 |
| `--benchmark --frames 3 --warmup 1 --width 640 --height 480 --stage 5` | GPU 启动、导入 pirate hut、渲染、cleanup、exit 0。走完整 `BlastRuntime::init` → `scene.init` → `cleanup` → `shutdown` |
| 交互 demo 启动约 8 s 后结束进程 | 进程未自行退出（未崩溃）；无新增 Blast 错误输出 |

默认 Simulate 关闭，所以启动后结构 tick 保持 0，这是接上了真实 `PhysicsWorld::step`，不是漏通知。打开 Simulate 后才会按 `kDt` 计数。

未能在本环境里用鼠标完成「挖断动态盒看碎块落地」的完整点击序列。连通掉块 / 刚体代码路径本轮未改；若日后发现旧的物理观感问题，不记作 E0 已修复。

本轮不声称修好任何既有物理观感问题。
