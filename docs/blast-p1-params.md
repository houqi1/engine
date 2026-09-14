# P1 薄壳圆柱参数（实现前锁定）

对应 `docs/体素应力破坏系统-完整执行方案.md` 第 1.1 节与第 13 节 P1。
尺寸与强度不随单次失败改几何。

| 量 | 值 |
|---|---|
| R | 2.0 m |
| t | 0.2 m |
| H | 6.0 m |
| 周向 × 高度 | 16 × 8 |
| ρ | 1000 kg/m³ |
| g | (0, -9.81, 0) m/s² |
| S_hold | 5×10⁷ Pa（T01 / T02 高强） |
| S_fail | 1×10³ Pa（T02 弱材） |
| graphReductionLevel | 0 |
| equalizeMasses | false（P1 补丁） |
| 迭代扫描 | 25 / 50 / 100 / 200 |

竖向 bond 面积 `(2πR/16)·t`，周向 `(H/8)·t`。底座 world bond 不可断。

T02 切口：j=0 竖向与该层周向，k ∈ [4, 16)，留下 k=0..3。

运行：

```bat
cmake --build build --config Release --target blast_p1_tests
build\Release\blast_p1_tests.exe
```

本机 Release 实测（2026-09-10，SHA `7ef568f5b557a6dad9023ecebc43cb809270e035`）：

| 项 | 结果 |
|---|---|
| T24 | ΣR_y = 147931 N，相对 Σmg 误差 4.2×10⁻⁷ |
| T05 | max\|F\|/W = 0 |
| T02 | 剩余壁带应力 5.15×10⁴ → 1.59×10⁶ Pa；弱材断、高强不断 |
| T03 | 密度加倍，应力 1.59×10⁶ → 3.18×10⁶ Pa |
| T04 | 单侧 90° 拉力 9.04×10⁵ Pa，对称双壁拉力 0 |
| 迭代 | 25/50/100/200 均收敛，残差同量级 |

P1 不接到 `PhysicsWorld`。T02 弱材下 actor 数会很多，这是硬阈值全额断键，不是 P2 刚体下落验收。
