# P0 接入信息与可构建基线

> 对应 `docs/体素应力破坏系统-完整执行方案.md` 第 13 节 P0。
> 本页是接入记录，不是 P1 薄壳圆柱验收。

## 目标项目

| 项 | 值 |
|---|---|
| 路径 | `C:\Users\win11\engine` |
| 渲染 | Vulkan 体素引擎，目标 `vulkan_engine_voxel` |
| 刚体后端 | 现有 `src/physics/PhysicsWorld`（Box3D 风格接触）。P0 不替换、不链接 PhysX 刚体。 |
| 结构求解 | 新版 NvBlast Low Level + ExtStress，静态库 `nvblast` |
| 固定物理步（已有） | `physics::kDt = 1/60 s`，`kGravity = (0, -9.81, 0) m/s^2` |
| P0 测试密度 | `1000 kg/m^3`（SI 演示值，尚未接到体素质心） |

P0 测试程序独立于 `VoxelScene`，不猜测现有场景类名做耦合。

## 工具链与硬件（本机登记）

| 项 | 值 |
|---|---|
| OS | Windows 11 家庭中文版 |
| CPU | 13th Gen Intel Core i7-13650HX |
| 逻辑处理器 | 20 |
| 内存 | 23.7 GB |
| CMake | 4.4.2 |
| Git | 2.50.1.windows.1 |
| 生成器 | Visual Studio 17 2022 |
| 编译器 | MSVC 14.34.31933（VS 2022 Community） |
| Windows SDK | 10.0.22621.0 |
| 目标 | x64 |
| 构建类型 | 以 `Release` 作为 P0 可运行基线；Debug 另编 |
| ExtStress SIMD | MSVC `/arch:AVX2`（Linux 路径 `-mavx2 -mfma`） |

MSVC 14.3 相对官方 packman 的 14.16 缺 `typeinfo.h`，且 ExtStress 使用 `std::copysignf`。适配放在 `cmake/nvblast_msvc_compat/`，不改 SDK 求解器源码。

未在本页签署第 12 节性能达标。上面只是 P0 要求的硬件登记。

## Blast 版本

| 项 | 值 |
|---|---|
| 仓库 | https://github.com/NVIDIA-Omniverse/PhysX |
| 稀疏目录 | `blast/` |
| commit SHA | `7ef568f5b557a6dad9023ecebc43cb809270e035` |
| 提交说明 | Fix Blast build issues（2025-04-07） |
| `blast/VERSION.md` | 5.0.6 |
| 文档入口 | https://nvidia-omniverse.github.io/PhysX/blast/docs/api/extensions/ext_stress.html |
| 永久源码 | https://github.com/NVIDIA-Omniverse/PhysX/tree/7ef568f5b557a6dad9023ecebc43cb809270e035/blast |
| 许可证 | `third_party/nvblast/LICENSE.md`（NVIDIA BSD 风格） |

不使用旧 GameWorks Blast，不混用 hardness / 冲量经验因子。文档 5.0.6 与 `main` 不当成同一二进制；本仓库只认上面的 SHA。

源码在配置时稀疏克隆到 `build/_deps/nvblast-src`，不进 git。CMake 见 `cmake/NvBlast.cmake`。

## 用户已验证演示

未在本轮提供。P0 不把 SampleAssetViewer 或 Omniverse 演示的调参拷进测试。

## 单位与 ID

- 适配边界使用 m、kg、s、N、N·m、Pa。P0 测试几何直接按米写。
- 应用稳定 ID：`stableNode` 100/101，`stableBond` 200/201。
- SDK `chunkIndex`、`graphNodeIndex`、bond 下标分开存，见 `src/blast/BlastIds.h`。
- 可破坏 bond 的 `initialBondHealths` 填有效面积 `A_eff`（P0 为 `1 m^2`）。
- 世界锚固 health 为 `2 * Nv::Blast::kUnbreakableLimit`，与面积分开，不参与面积换算。

## 构建与运行

```bat
cmake -S . -B build
cmake --build build --config Release --target blast_p0_tests
build\Release\blast_p0_tests.exe
```

首次配置需要网络，以便按 SHA 稀疏拉取 `blast/`。

## P0 交付与退出条件

交付：

1. 本页依赖版本记录。
2. 上面的构建说明。
3. 可执行测试 `blast_p0_tests`：两个 support 节点、一条可断连接、一条不可断 world bond。

退出条件（测试内断言）：

- 重力：`addGravity` + `update`，线/角残差为有限值。
- 固定连接：`NvBlastActorHasExternalBonds` 为真；world bond 不可断。
- 手动断键：把内部 bond 的当前 health 一次扣完，health 归零并 `split`。
- actor 生命周期：split 后至少两个 actor；一个仍带 world bond，一个没有。
- 重复重置：连续 50 次创建/求解/断键/释放，跟踪分配器 live bytes 回到基线。
- 无 NvBlast error-callback 错误。

不算 P0 通过：薄壳空心圆柱自重/底座 270° 切口、关 `equalizeMasses`、接触冲量、体素挖除。那些是 P1 及之后。

本机 `Release` 实测（2026-09-10）：

```
blast_p0_tests NvBlast 5.0.6 sha 7ef568f5b557a6dad9023ecebc43cb809270e035
  stableNode 100 -> chunkIndex 0 graphNode 0
  stableNode 101 -> chunkIndex 1 graphNode 1
  world graphNode 2
  stableBond 200 -> sdkBond 0
  stableBond 201 -> sdkBond 1
stress.update 0.0261 ms  lin=0 ang=0 converged=1 overstressed=0
OK  resets=50 liveBytes=0 totalAllocs=816
```

`lin`/`ang` 是求解残差，不是内力大小。残差为 0 表示该微型图已收敛，不是 P1 支反力验收。

复跑（同日）：Release 再编再跑仍为 exit 0，`resets=50 liveBytes=0`。Debug 配置在 `NV_DEBUG=1` 时编不过：`NvBlastActor.cpp:844` 的 `NVBLASTLL_CHECK(..., return nullptr)` 写在返回 `bool` 的函数里，MSVC 14.34 / C++17 报 C2440。这是上游 param-check 宏问题，Release 因 `NVBLASTLL_CHECK` 被空掉所以能过。P0 基线仍以 Release 为准。

## 本轮明确未做

- 未关闭 ExtStress 内部默认 `equalizeMasses = true`（P1）。
- 未实现无 split 断键的应力图同步（P1 / T22）。
- 未接到 `PhysicsWorld` / `VoxelScene`。
- 未记录用户侧 Blast 演示操作步骤。
