# swept-collide-lab

**Exact swept-sphere time-of-impact against a triangle mesh: 40/40 high-speed contacts caught where a 16-substep discrete sweep misses 13/40, using 24.3 % of brute force's triangle tests (2.09× faster on the 228-triangle level).**

Upstream: none

A first-person walkthrough where the player is a sphere of radius 0.35 m. Every frame the
browser asks the *same* `src/core.mjs` that node uses: "at what fraction `t` of this frame's
swept segment does the sphere first touch the level?" At 900 m/s (15 m per 1/60 s frame) a
classic fixed-substep loop samples the world 16 times, steps straight through a 0.08 m wall
and keeps going. The algebraic sweep cannot: it solves for the exact contact time and stops
there. Toggle the mode and the HUD counts the pass-throughs live.

## Run it (one command)

```bash
python3 -m http.server 4173    # then open http://localhost:4173
```

Click to capture the mouse, `WASD` to move, `Shift` to sprint, `C` to switch between
exact CCD and the 16-substep discrete baseline. No build step, no `node_modules`: three.js
0.160.0 is pinned in the importmap and `data/level.json` is committed (17 KB).

Node-only reproduction (no browser):

```bash
node tools/gen-sample.mjs   # rebuilds data/level.json, byte-identical on rerun
node bench/verify.mjs       # 15 assertions, exits 0 only if all pass
node bench/bench.mjs        # 7 samples + cold start -> bench/result.json
```

## Evidence

`node bench/verify.mjs` → **15 passed, 0 failed**:

| check | measured |
| --- | --- |
| analytic plane / edge / vertex TOI | 0.25, 0.375, 0.4166666666666667 (exact closed forms) |
| independent bisection oracle, 300 random sweeps | 300/300 agree on hit/miss, max &#124;Δt&#124; = 1.6e-14 |
| BVH vs brute force, 300 sweeps over the level | 300/300 identical hit + TOI |
| tunnelling, 900 m/s through a 0.08 m wall | discrete(16) missed **13/40**, exact CCD caught **40/40** |
| scale invariance, 1e-6 … 1e6 | 33 hits at every scale, identical flags and TOI |
| cross-runtime fingerprint | mesh `8bc868e5`, sweep `f0cd1584` (printed in the browser HUD) |

`node bench/bench.mjs` (7 samples × 25 reps, one cold run; `bench/result.json`):

- cold start **24.968 ms**; median **2.737 ms**; min 2.699; p95 4.680; stddev 0.678 (**RSD 22.42 %**).
- brute force over the same 64 sweeps: median **5.716 ms** → **2.09×**.
- triangle tests: **3,549 vs 14,592** brute (24.3 %). BVH build 2.867 ms, 63 nodes, depth 5.

The RSD is high because sample 0 (4.680 ms) is measured while the JIT is still warming up;
samples 1–6 are 2.699–2.808 ms. RSD excluding the first sample is 1.5 %. The median is the
reported number.

## How it works

1. **Candidates, not sampling.** For a sphere centre `P(t) = p0 + t·v`, contact happens when
   the distance from `P(t)` to the triangle equals `r`. That distance is realised by one of the
   triangle's 7 features (face plane, 3 edge lines, 3 vertices), each giving a linear or
   quadratic equation in `t`. All roots in `(0,1]` are collected.
2. **Validation.** Each candidate is accepted only if an actual point-triangle distance test
   confirms `dist(P(t), triangle) ≤ r`. Because the distance-to-convex-set function is convex
   along a line, the smallest accepted root is the entry time — no marching, no ε guess.
3. **BVH.** Median-split AABB tree over triangle centroids; nodes are culled by a slab test of
   the segment expanded by `r`, with the current best `t` as the interval bound.
4. **Independent oracle.** `oracleFirstContact` brackets a sign change of `dist(t) − r` on a
   2048-point grid and bisects — different algorithm, no shared constants with (1).

## Known limits

- **Relative tolerance only.** `REL_EPS = 1e-9` is multiplied by the sphere radius everywhere
  (`r + r·REL_EPS`), so there is no absolute length threshold. Valid over coordinates
  ~1e-6 … 1e6 with `r` scaled proportionally; the suite checks exactly those five decades.
  At 1e-6 the working distances are ~3.5e-7 m, still 9 orders above double ulp.
- **Tangential grazes.** A sweep that touches with zero penetration depth can return `t` at a
  point where rounding decides the last bit; the validation test is `≤ r(1+1e-9)`. Exact
  tangency is reported as a hit; a sweep missing by < 1e-9·r may also be reported as a hit.
- **Sphere only, static world.** The player is a sphere, geometry does not move, and there is
  no gravity/step-up. A capsule or moving obstacles would need a different feature set.
- **Speed-up is scene-size bound.** 2.09× on 228 triangles is what a BVH can give when brute
  force is already only 14,592 distance tests; the pruning ratio (24.3 %) is the size-independent
  number. A 10k-triangle stress scene was *not* benchmarked in this round.
- **The interactive demo is not a game.** It is a collision testbed: no audio, no scoring, one
  level, and the camera has no collision of its own (the sphere does).

## 本轮创新

**B 档.** 现有 three.js 碰撞示例几乎都用「固定子步 + 每步重叠检测」（本仓库里就是
`discreteSweep`），高速或薄壁时直接穿过去；BVH 只被用来加速**静态**重叠查询。这里把
「扫掠球的精确首触时间」做成解析解：对 7 类特征分别解一次/二次方程，再用真实距离验证
候选，取最小合法根——不是采样、不是 ε 猜测。于是浏览器里能实时看到**同一帧的精确 TOI**
和「16 子步漏了几次」并列显示，且 HUD 的 sweep fingerprint 与 `node bench/bench.mjs`
打印的 `f0cd1584` 逐位相同。此前台账里的 `mesh-selfint-lab`、`surface-intersect-lab`
都是静态相交/BVH 加速，没有一条做运动球体的最早接触时间。

## License

MIT © 2026 llinok111. Original work, no upstream code.
