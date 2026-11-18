// Deterministic sample generator: a 22 m "shooting gallery" (floor, outer
// walls, five thin interior walls, eight pillars) plus a fixed set of 64
// swept-sphere moves used for the cross-runtime fingerprint.
//
//   node tools/gen-sample.mjs   ->  data/level.json   (byte-identical on rerun)
import { writeFileSync, mkdirSync } from "node:fs";
import { boxMesh, mergeMeshes, meshFingerprint } from "../src/core.mjs";

function mulberry32(a) {
  return function () {
    a |= 0; a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
const round = (x) => Math.round(x * 1000) / 1000;

export function buildLevel() {
  const parts = [];
  const box = (x0, y0, z0, x1, y1, z1) => parts.push(boxMesh([x0, y0, z0], [x1, y1, z1]));
  // floor + ceiling slab (thin, small enough to stay under 5 MB)
  box(-11, -0.5, -15, 11, 0, 15);
  box(-11, 4.5, -15, 11, 5, 15);
  // outer walls
  box(-11.5, 0, -15.5, -11, 5, 15.5);
  box(11, 0, -15.5, 11.5, 5, 15.5);
  box(-11, 0, -15.5, 11, 5, -15);
  box(-11, 0, 15, 11, 5, 15.5);
  // thin interior walls (0.08 m): the geometry the discrete baseline tunnels
  box(-6, 0, -8, -0.2, 5, -7.92);
  box(0.2, 0, -2, 6, 5, -1.92);
  box(-9, 0, 5, -3, 5, 5.08);
  box(-0.04, 0, 2, 0.04, 5, 12);
  box(4, 0, 6, 9, 5, 6.08);
  // pillars
  for (let i = 0; i < 8; i++) {
    const x = -8 + (i % 4) * 5.2, z = -11 + Math.floor(i / 4) * 8;
    box(x, 0, z, x + 0.9, 5, z + 0.9);
  }
  return mergeMeshes(parts);
}

export function buildMoves(n = 64) {
  const rnd = mulberry32(0x5eed1234);
  const moves = [];
  for (let i = 0; i < n; i++) {
    const p0 = [round(-10 + rnd() * 20), round(0.4 + rnd() * 3.4), round(-14 + rnd() * 28)];
    const th = rnd() * Math.PI * 2, ph = (rnd() - 0.5) * Math.PI;
    const dir = [Math.cos(th) * Math.cos(ph), Math.sin(ph), Math.sin(th) * Math.cos(ph)];
    // speed spans human (5 m/s) to projectile (900 m/s); dt = 1/60 s
    const speed = 5 + rnd() * rnd() * 895;
    const p1 = [
      round(p0[0] + dir[0] * speed / 60),
      round(p0[1] + dir[1] * speed / 60),
      round(p0[2] + dir[2] * speed / 60),
    ];
    moves.push([p0, p1]);
  }
  return moves;
}

const level = buildLevel();
const moves = buildMoves(64);
const doc = {
  meta: {
    generator: "tools/gen-sample.mjs",
    seed: "0x5eed1234",
    radius: 0.35,
    triangleCount: level.indices.length,
    moveCount: moves.length,
    meshFingerprint: meshFingerprint(level),
    note: "All coordinates in metres; moves are 1/60 s apart.",
  },
  positions: level.positions.map((p) => p.map(round)),
  indices: level.indices,
  moves,
};
mkdirSync(new URL("../data/", import.meta.url), { recursive: true });
writeFileSync(new URL("../data/level.json", import.meta.url), JSON.stringify(doc, null, 1) + "\n");
console.log(`level: ${level.indices.length} triangles, ${level.positions.length} vertices`);
console.log(`moves: ${moves.length}, fingerprint ${doc.meta.meshFingerprint}`);