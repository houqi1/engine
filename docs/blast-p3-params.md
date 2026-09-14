# P3 接触载荷与二次断裂（T10–T13）

对应执行方案第 8.3–8.4 节与第 13 节 P3。
刚体积分仍不把同一冲量再喂给结构求解器。`F_avg = J / dt`，`τ = (p − c) × F`。

| 项 | 决定 |
|---|---|
| 偏心力矩 | ExtStress `addLoad(node, F, τ)`（P3 补丁），不是虚构的 addTorque-only API |
| 持续接触 | 每个固定物理步重新采样 |
| 单次撞击 | `ImpulseEvents` 按 event ID 消费一次 |
| 断裂后 | `LoadSnapshot.valid = false`，等下一物理步 |

运行：

```bat
cmake --build build --config Release --target blast_p3_tests
build\Release\blast_p3_tests.exe
```

本机 Release 实测（2026-09-10）：

| 测试 | 结果 |
|---|---|
| T10 | 三点偏心映射 relF=0、relM=0 |
| T11 | 自由梁两端拉伸内力 1000 Pa；力偶内应力 135 Pa；无假 world 锚 |
| T12 | 命中端点，中间弱键断开；同一 event ID 不能再消费 |
| T13 | 相同物理 tick 下 30/60/120 渲染细分断裂序列均为 `101 101 101 101`；`dt×2` 本例序列相同，仅作敏感性记录 |

P1/P2 在 P3 补丁后仍通过。下一步是 P4：体素挖除与 family 重建。
