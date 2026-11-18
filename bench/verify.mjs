// Assertions on the pure core. Every number printed here is computed live.
import { readFileSync } from "node:fs";
import {
  sweepSphereTriangle, sweepSphereMesh, discreteSweep, oracleFirstContact,
  buildBVH, distPointTriangle, boxMesh, mergeMeshes, meshFingerprint,
  sweepFingerprint, REL_EPS,
} from "../src/core.mjs";

let pass = 0, fail = 0;
const check = (name, cond, detail) => {
  if (cond) { pass++; console.log(`PASS ${name}  ${detail}`); }
  else { fail++; console.log(`FAIL ${name}  ${detail}`); }
};
const near = (a, b, tol) => Math.abs(a - b) <= tol;

const doc = JSON.parse(readFileSync(new URL("../data/level.json", import.meta.url)));
const mesh = { positions: doc.positions, indices: doc.indices };
const R = doc.meta.radius;
const bvh = buildBVH(mesh);

// --- 1. Analytic plane: centre must travel to distance r from the plane -----
{
  const tri = [[-100, -100, 0], [100, -100, 0], [0, 100, 0]];
  const res = sweepSphereTriangle([0, 0, 1], [0, 0, -1], 0.5, ...tri);
  check("plane-toi", res && near(res.t, 0.25, 1e-12), `t=${res && res.t} expected 0.25`);
}
// --- 2. Analytic edge: triangle lies in x<=0, closest feature is the edge ---
{
  const tri = [[0, 0, -10], [0, 0, 10], [-5, 0.5, 0]];
  const res = sweepSphereTriangle([2, 0, 0], [-2, 0, 0], 0.5, ...tri);
  check("edge-toi", res && near(res.t, 0.375, 1e-9), `t=${res && res.t} expected 0.375`);
  const d = distPointTriangle([0.5, 0, 0], ...tri);
  check("edge-distance", near(d, 0.5, 1e-12), `d=${d}`);
}
// --- 3. Analytic vertex -----------------------------------------------------
{
  const tri = [[0, 0, 0], [-2, 2, 0], [-2, -2, 0]];
  const res = sweepSphereTriangle([3, 0, 0], [-3, 0, 0], 0.5, ...tri);
  check("vertex-toi", res && near(res.t, 2.5 / 6, 1e-9), `t=${res && res.t} expected ${2.5 / 6}`);
}
// --- 4. Start already overlapping -> t = 0; 5. clean miss -> null -----------
{
  const tri = [[-1, -1, 0], [1, -1, 0], [0, 1, 0]];
  const inside = sweepSphereTriangle([0, 0, 0.1], [0, 0, 2], 0.5, ...tri);
  check("start-inside", inside && inside.t === 0 && inside.inside0 === true, `t=${inside && inside.t}`);
  const miss = sweepSphereTriangle([5, 5, 1], [5, 5, 3], 0.5, ...tri);
  check("clean-miss", miss === null, `res=${miss && miss.t}`);
}
// --- 6. Independent search/bisection oracle on 240 forced hits + 60 misses --
{
  const rnd = (() => { let a = 0xdecafbad; return () => { a = (a * 1664525 + 1013904223) >>> 0; return a / 4294967296; }; })();
  let hits = 0, mismatches = 0, worst = 0;
  for (let i = 0; i < 300; i++) {
    const A = [rnd() * 4 - 2, rnd() * 4 - 2, rnd() * 4 - 2];
    const B = [A[0] + rnd() * 2 - 1, A[1] + rnd() * 2 - 1, A[2] + rnd() * 2 - 1];
    const C = [A[0] + rnd() * 2 - 1, A[1] + rnd() * 2 - 1, A[2] + rnd() * 2 - 1];
    const target = [A[0] * 0.3 + B[0] * 0.3 + C[0] * 0.4, A[1] * 0.3 + B[1] * 0.3 + C[1] * 0.4, A[2] * 0.3 + B[2] * 0.3 + C[2] * 0.4];
    const dir = [target[0] - 0.5 + rnd(), target[1] - 0.5 + rnd(), target[2] - 0.5 + rnd()];
    const p0 = [target[0] + dir[0] * 3, target[1] + dir[1] * 3, target[2] + dir[2] * 3];
    const p1 = [target[0] - dir[0] * 3, target[1] - dir[1] * 3, target[2] - dir[2] * 3];
    const rr = 0.05 + rnd() * 0.4;
    const got = sweepSphereTriangle(p0, p1, rr, A, B, C);
    const want = oracleFirstContact(p0, p1, rr, A, B, C, 1024);
    if (want !== null) hits++;
    if ((got === null) !== (want === null)) { mismatches++; continue; }
    if (got && want !== null) worst = Math.max(worst, Math.abs(got.t - want));
  }
  check("oracle-agreement", mismatches === 0, `${300 - mismatches}/300 agree, ${hits} hits, max |dt|=${worst.toExponential(2)}`);
  check("oracle-tolerance", worst < 1e-6, `max |dt|=${worst.toExponential(2)} < 1e-6`);
}
// --- 7. BVH vs brute force on the real level, 300 random moves --------------
{
  let mismatch = 0, hits = 0, bvhTests = 0, bruteTests = 0;
  for (let i = 0; i < 300; i++) {
    const p0 = [((i * 7.13) % 20) - 10, 0.5 + ((i * 3.7) % 3.0), ((i * 11.7) % 28) - 14];
    const p1 = [p0[0] + Math.sin(i) * 1.7, p0[1] + Math.cos(i * 1.3) * 0.8, p0[2] + Math.cos(i * 0.7) * 1.7];
    const a = sweepSphereMesh(bvh, p0, p1, R);
    let bestT = Infinity, bh = false;
    for (let t = 0; t < mesh.indices.length; t++) {
      const [ia, ib, ic] = mesh.indices[t];
      const r = sweepSphereTriangle(p0, p1, R, mesh.positions[ia], mesh.positions[ib], mesh.positions[ic]);
      bruteTests++;
      if (r) { bh = true; if (r.t < bestT) bestT = r.t; }
    }
    bvhTests += a.tests;
    if (a.hit !== bh) mismatch++;
    else if (a.hit && Math.abs(a.t - bestT) > 1e-9) mismatch++;
    if (a.hit) hits++;
  }
  check("bvh-vs-brute", mismatch === 0, `${300 - mismatch}/300 identical hit+TOI, ${hits} hits`);
  check("bvh-prunes", bvhTests < bruteTests * 0.35, `${bvhTests} tri tests vs ${bruteTests} brute (${(100 * bvhTests / bruteTests).toFixed(1)}%)`);
}
// --- 8. Tunnelling: 0.08 m wall, 900 m/s, 1/60 s frames ---------------------
{
  const wall = boxMesh([-4, 0, -0.04], [4, 5, 0.04]);
  const wm = mergeMeshes([wall]);
  const wb = buildBVH(wm);
  let miss = 0, ccd = 0, n = 0;
  for (let i = 0; i < 40; i++) {
    const y = 0.6 + (i % 8) * 0.5, z0 = 5 + (i % 3);
    const p0 = [0.0, y, z0], p1 = [0.0, y, z0 - 15];
    n++;
    if (!discreteSweep(wm, p0, p1, R, 16).hit) miss++;
    if (sweepSphereMesh(wb, p0, p1, R).hit) ccd++;
  }
  check("tunnelling-baseline", miss > 0, `discrete(16) missed ${miss}/${n} shots`);
  check("tunnelling-ccd", ccd === n, `exact CCD caught ${ccd}/${n} shots`);
}
// --- 9. Scale invariance 1e-6 .. 1e6 (all tolerances relative to r) ----------
{
  const ref = doc.moves.map(([p0, p1]) => sweepSphereMesh(bvh, p0, p1, R));
  const rows = [];
  let ok = true;
  for (const s of [1e-6, 1e-3, 1, 1e3, 1e6]) {
    const sm = { positions: mesh.positions.map((p) => p.map((x) => x * s)), indices: mesh.indices };
    const sb = buildBVH(sm);
    const got = doc.moves.map(([p0, p1]) => sweepSphereMesh(sb, p0.map((x) => x * s), p1.map((x) => x * s), R * s));
    let h = 0;
    for (let i = 0; i < got.length; i++) {
      if (got[i].hit !== ref[i].hit) ok = false;
      else if (got[i].hit && Math.abs(got[i].t - ref[i].t) > 1e-6) ok = false;
      if (got[i].hit) h++;
    }
    rows.push(`${s}:${h}`);
  }
  check("scale-invariance", ok, `hits ${rows.join(" ")} (identical flags & TOI at 5 scales)`);
}
// --- 10. Runtime fingerprint -----------------------------------------------
{
  const fp = sweepFingerprint(bvh, doc.moves, R);
  const fp2 = sweepFingerprint(bvh, doc.moves, R);
  check("mesh-fingerprint", meshFingerprint(mesh) === doc.meta.meshFingerprint, `${meshFingerprint(mesh)} == meta`);
  check("sweep-fingerprint", fp === fp2 && /^[0-9a-f]{8}$/.test(fp), `${fp} (same in browser HUD)`);
  console.log(`META triangles=${mesh.indices.length} vertices=${mesh.positions.length} radius=${R} relEps=${REL_EPS} sweepFingerprint=${fp}`);
}
console.log(`\n${pass} passed, ${fail} failed`);
process.exit(fail === 0 ? 0 : 1);