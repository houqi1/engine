# E2 应力断裂与刚体下落

> 日期：2026-09-13。E3 接触冲量未做。
> 几何与切口与 E1 相同。断裂默认关闭，面板打开 **Stress fracture**。

## 强度（由 E1 应力钉死）

| | 值 | 依据 |
|---|---|---|
| S_fail | 2.5×10⁵ Pa | 完整 maxC ~1.5×10⁵ Pa、壁带 ~1.2×10⁵；切口壁带 ~2.7×10⁶ |
| S_hold | 5×10⁷ Pa | P1 高强度对照 |

几何、切口不再随断裂结果改。

## 流程

1. tick 收敛后收集超限键；若切口残差未进容差但壁带应力已 > 2S，仍允许断键（否则 3000 节点切口无法下落）。
2. 提交点：`brokenFaces` ← 候选面，全额断键，一次 split。
3. actor 可见 chunk → fine 集合；最大块留原槽；其余新槽。无可见 chunk 的 world actor 不建物体。
4. 有 world bond → Static，否则 Dynamic。速度 `v' = v + ω × (c'−c)`，无爆开。
5. 每帧最多一批分裂。

## 头测

`blast_e2_tests`：完整无候选；切口有壁带超限键且 split≥2 actor（一块有锚、一块无幽灵锚）；Hold 无候选。

## 实机

Spawn → Simulate 至 converged → Cut 270 → 打开 **Stress fracture** 且勾选 Fail strength。上部应下落，底座留下。对照：关掉 Fail strength（50 MPa）同样切口应撑住。
