// Timing harness: >=5 samples, median + p95 + cold start, BVH vs brute force.
// Usage: node bench/bench.mjs   -> writes bench/result.json
import { readFileSync, writeFileSync } from "node:fs";
import { performance } from "node:perf_hooks";
import { buildBVH, sweepSphereMesh, sweepSphereTriangle, sweepFingerprint } from "../src/core.mjs";

const doc = JSON.parse(readFileSync(new URL("../data/level.json", import.meta.url)));
const mesh = { positions: doc.positions, indices: doc.indices };
const R = doc.meta.radius;
const moves = doc.moves;
const REPS = 25;

const t0 = performance.now();
const bvh = buildBVH(mesh);
const buildMs = performance.now() - t0;

function bvhScan() {
  let tests = 0, hits = 0;
  for (const [p0, p1] of moves) {
    const r = sweepSphereMesh(bvh, p0, p1, R);
    tests += r.tests; if (r.hit) hits++;
  }
  return { tests, hits };
}
function bruteScan() {
  let tests = 0, hits = 0;
  for (const [p0, p1] of moves) {
    let hit = false;
    for (let i = 0; i < mesh.indices.length; i++) {
      const [a, b, c] = mesh.indices[i];
      tests++;
      if (sweepSphereTriangle(p0, p1, R, mesh.positions[a], mesh.positions[b], mesh.positions[c])) { hit = true; break; }
    }
    if (hit) hits++;
  }
  return { tests, hits };
}

// Cold start: very first full scan (JIT + cache cold).
const c0 = performance.now();
const cold = bvhScan();
const coldMs = performance.now() - c0;

const samples = [];
let last = null;
for (let s = 0; s < 7; s++) {
  const a = performance.now();
  for (let k = 0; k < REPS; k++) last = bvhScan();
  samples.push((performance.now() - a) / REPS);
}
const bruteSamples = [];
for (let s = 0; s < 5; s++) {
  const a = performance.now();
  for (let k = 0; k < 3; k++) bruteScan();
  bruteSamples.push((performance.now() - a) / 3);
}
const sorted = [...samples].sort((x, y) => x - y);
const median = sorted[Math.floor(sorted.length / 2)];
const min = sorted[0];
const p95 = sorted[Math.min(sorted.length - 1, Math.ceil(0.95 * sorted.length) - 1)];
const mean = samples.reduce((a, b) => a + b, 0) / samples.length;
const stddev = Math.sqrt(samples.reduce((a, b) => a + (b - mean) ** 2, 0) / samples.length);
const bruteMedian = [...bruteSamples].sort((x, y) => x - y)[2];
const bruteTests = mesh.indices.length * moves.length;

const result = {
  date: new Date().toISOString().slice(0, 10),
  node: process.version,
  scene: { triangles: mesh.indices.length, vertices: mesh.positions.length, moves: moves.length, radius: R },
  bvh: { buildMs: +buildMs.toFixed(3), nodes: bvh.stats.nodes, leaves: bvh.stats.leaves, maxDepth: bvh.stats.maxDepth },
  cold: { ms: +coldMs.toFixed(3), hits: cold.hits },
  samples: { reps: REPS, count: samples.length, ms: samples.map((x) => +x.toFixed(3)) },
  median: +median.toFixed(3),
  min: +min.toFixed(3),
  p95: +p95.toFixed(3),
  stddev: +stddev.toFixed(3),
  rsdPct: +((100 * stddev) / mean).toFixed(2),
  brute: { median: +bruteMedian.toFixed(3), samples: bruteSamples.map((x) => +x.toFixed(3)) },
  speedupVsBrute: +(bruteMedian / median).toFixed(2),
  triTests: { bvh: last.tests, brute: bruteTests, ratioPct: +((100 * last.tests) / bruteTests).toFixed(1) },
  hits: last.hits,
  sweepFingerprint: sweepFingerprint(bvh, moves, R),
};
writeFileSync(new URL("./result.json", import.meta.url), JSON.stringify(result, null, 2) + "\n");
console.log(JSON.stringify(result, null, 2));