// Pure geometry core: swept-sphere continuous collision (exact earliest TOI)
// against triangle meshes with a BVH. No three.js, no DOM, no I/O, no globals.
//
// Tolerance policy (see README "Known limits"): every tolerance is RELATIVE to
// the sphere radius r, so results are invariant under uniform scaling.

export const REL_EPS = 1e-9;

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const mul = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const len2 = (a) => dot(a, a);
const lerp3 = (a, b, t) => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];

// Closest point on triangle (Ericson, Real-Time Collision Detection, 5.1.5).
// Returns the squared distance. Pure, allocation-light.
export function closestPointTriangle2(p, a, b, c, out) {
  const ab = sub(b, a), ac = sub(c, a), ap = sub(p, a);
  const d1 = dot(ab, ap), d2 = dot(ac, ap);
  if (d1 <= 0 && d2 <= 0) { if (out) { out[0] = a[0]; out[1] = a[1]; out[2] = a[2]; } return len2(ap); }
  const bp = sub(p, b);
  const d3 = dot(ab, bp), d4 = dot(ac, bp);
  if (d3 >= 0 && d4 <= d3) { if (out) { out[0] = b[0]; out[1] = b[1]; out[2] = b[2]; } return len2(bp); }
  const vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) {
    const v = d1 / (d1 - d3);
    const q = [a[0] + ab[0] * v, a[1] + ab[1] * v, a[2] + ab[2] * v];
    if (out) { out[0] = q[0]; out[1] = q[1]; out[2] = q[2]; }
    return len2(sub(p, q));
  }
  const cp = sub(p, c);
  const d5 = dot(ab, cp), d6 = dot(ac, cp);
  if (d6 >= 0 && d5 <= d6) { if (out) { out[0] = c[0]; out[1] = c[1]; out[2] = c[2]; } return len2(cp); }
  const vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) {
    const w = d2 / (d2 - d6);
    const q = [a[0] + ac[0] * w, a[1] + ac[1] * w, a[2] + ac[2] * w];
    if (out) { out[0] = q[0]; out[1] = q[1]; out[2] = q[2]; }
    return len2(sub(p, q));
  }
  const va = d3 * d6 - d5 * d4;
  if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
    const w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    const q = [
      b[0] + (c[0] - b[0]) * w, b[1] + (c[1] - b[1]) * w, b[2] + (c[2] - b[2]) * w,
    ];
    if (out) { out[0] = q[0]; out[1] = q[1]; out[2] = q[2]; }
    return len2(sub(p, q));
  }
  const denom = 1 / (va + vb + vc);
  const v = vb * denom, w = vc * denom;
  const q = [a[0] + ab[0] * v + ac[0] * w, a[1] + ab[1] * v + ac[1] * w, a[2] + ab[2] * v + ac[2] * w];
  if (out) { out[0] = q[0]; out[1] = q[1]; out[2] = q[2]; }
  return len2(sub(p, q));
}

export function distPointTriangle(p, a, b, c) {
  return Math.sqrt(closestPointTriangle2(p, a, b, c));
}

function triOf(mesh, i) {
  const [ia, ib, ic] = mesh.indices[i];
  return [mesh.positions[ia], mesh.positions[ib], mesh.positions[ic]];
}

// Real roots of a t^2 + b t + c = 0 (a may be ~0). Returns up to 2 roots.
function quadRoots(a, b, c, relScale) {
  const eps = (relScale || 1) * 1e-14;
  if (Math.abs(a) <= eps) {
    if (Math.abs(b) <= eps) return [];
    return [-c / b];
  }
  const disc = b * b - 4 * a * c;
  if (disc < 0) return [];
  const s = Math.sqrt(disc);
  const q = -0.5 * (b + (b >= 0 ? s : -s));
  const r1 = q / a;
  const r2 = q === 0 ? -b / (2 * a) : c / q;
  return [r1, r2];
}

// Earliest t in [0,1] at which a sphere of radius r centred at p0+t*v touches
// triangle (a,b,c). Exact up to double rounding: candidate times are the
// analytic roots of "distance to face plane / edge line / vertex == r", then
// each candidate is validated by an actual distance test.
// Returns {t, inside0} or null. Tests are counted on `stats`.
export function sweepSphereTriangle(p0, p1, r, a, b, c, stats) {
  const v = sub(p1, p0);
  if (stats) stats.triTests++;
  const d0 = distPointTriangle(p0, a, b, c);
  const reach = r + r * REL_EPS;
  if (d0 <= reach) return { t: 0, inside0: true };
  if (len2(v) === 0) return null;

  const rr = r * r;
  const ts = [];
  const n = cross(sub(b, a), sub(c, a));
  const nlen = Math.sqrt(len2(n));
  if (nlen > 0) {
    // distance to the face plane == r, both sides
    for (const sgn of [1, -1]) {
      const num = dot(n, a) + sgn * r * nlen - dot(n, p0);
      const den = dot(n, v);
      if (Math.abs(den) > 0) ts.push(num / den);
    }
  }
  // vertices
  for (const V of [a, b, c]) {
    const w = sub(p0, V);
    for (const t of quadRoots(dot(v, v), 2 * dot(v, w), dot(w, w) - rr, rr)) ts.push(t);
  }
  // edge lines
  for (const [A, B] of [[a, b], [b, c], [c, a]]) {
    const e = sub(B, A);
    const c0 = cross(sub(p0, A), e);
    const c1 = cross(v, e);
    const A2 = dot(c1, c1);
    const B2 = 2 * dot(c0, c1);
    const C2 = dot(c0, c0) - rr * dot(e, e);
    for (const t of quadRoots(A2, B2, C2, rr)) ts.push(t);
  }
  let best = Infinity;
  for (const t of ts) {
    if (!(t > 0) || t > 1) continue;
    if (t >= best) continue;
    const p = lerp3(p0, p1, t);
    if (distPointTriangle(p, a, b, c) <= reach) best = t;
  }
  return best === Infinity ? null : { t: best, inside0: false };
}

// --- BVH -------------------------------------------------------------------
export function buildBVH(mesh, opts = {}) {
  const leafSize = opts.leafSize || 8;
  const n = mesh.indices.length;
  const cent = new Float64Array(n * 3);
  const bmin = new Float64Array(n * 3);
  const bmax = new Float64Array(n * 3);
  for (let i = 0; i < n; i++) {
    const [a, b, c] = triOf(mesh, i);
    for (let k = 0; k < 3; k++) {
      const lo = Math.min(a[k], b[k], c[k]), hi = Math.max(a[k], b[k], c[k]);
      bmin[i * 3 + k] = lo; bmax[i * 3 + k] = hi;
      cent[i * 3 + k] = (a[k] + b[k] + c[k]) / 3;
    }
  }
  const order = new Uint32Array(n);
  for (let i = 0; i < n; i++) order[i] = i;
  const nodes = [];
  const stats = { nodes: 0, maxDepth: 0, leaves: 0 };

  function build(start, end, depth) {
    const id = nodes.length;
    const node = { min: [Infinity, Infinity, Infinity], max: [-Infinity, -Infinity, -Infinity], start, count: end - start, left: -1, right: -1 };
    nodes.push(node);
    stats.nodes++;
    if (depth > stats.maxDepth) stats.maxDepth = depth;
    for (let i = start; i < end; i++) {
      const t = order[i];
      for (let k = 0; k < 3; k++) {
        if (bmin[t * 3 + k] < node.min[k]) node.min[k] = bmin[t * 3 + k];
        if (bmax[t * 3 + k] > node.max[k]) node.max[k] = bmax[t * 3 + k];
      }
    }
    if (end - start <= leafSize) { stats.leaves++; return id; }
    let ext = [node.max[0] - node.min[0], node.max[1] - node.min[1], node.max[2] - node.min[2]];
    let axis = ext[0] > ext[1] ? (ext[0] > ext[2] ? 0 : 2) : (ext[1] > ext[2] ? 1 : 2);
    // median split on centroid along the longest axis
    const slice = Array.from(order.subarray(start, end));
    slice.sort((i, j) => cent[i * 3 + axis] - cent[j * 3 + axis]);
    const mid = start + (slice.length >> 1);
    for (let i = 0; i < slice.length; i++) order[start + i] = slice[i];
    node.left = build(start, mid, depth + 1);
    node.right = build(mid, end, depth + 1);
    node.start = -1; node.count = 0;
    return id;
  }
  build(0, n, 0);
  return { nodes, order, mesh, stats };
}

// Slab test: does segment p0->p1 (expanded by r) enter the AABB within [0,tMax]?
function slabHit(min, max, p0, v, r, tMax) {
  let t0 = 0, t1 = tMax;
  for (let k = 0; k < 3; k++) {
    const lo = min[k] - r, hi = max[k] + r;
    if (Math.abs(v[k]) < 1e-300) {
      if (p0[k] < lo || p0[k] > hi) return false;
      continue;
    }
    let ta = (lo - p0[k]) / v[k], tb = (hi - p0[k]) / v[k];
    if (ta > tb) { const tmp = ta; ta = tb; tb = tmp; }
    if (ta > t0) t0 = ta;
    if (tb < t1) t1 = tb;
    if (t0 > t1) return false;
  }
  return t1 >= 0;
}

// Earliest contact of a swept sphere against the whole mesh.
// Returns {hit, t, tri, tests} — `tests` is the number of triangles actually
// tested (the metric shown in the browser HUD and in bench/result.json).
export function sweepSphereMesh(bvh, p0, p1, r) {
  const mesh = bvh.mesh;
  const v = sub(p1, p0);
  const stats = { triTests: 0 };
  let best = 1, bestTri = -1, hit = false;
  const stack = [0];
  while (stack.length) {
    const ni = stack.pop();
    const node = bvh.nodes[ni];
    const tMax = hit ? best : 1;
    if (!slabHit(node.min, node.max, p0, v, r, tMax)) continue;
    if (node.left < 0) {
      for (let k = node.start; k < node.start + node.count; k++) {
        const ti = bvh.order[k];
        const [a, b, c] = triOf(mesh, ti);
        const res = sweepSphereTriangle(p0, p1, r, a, b, c, stats);
        if (res && (res.t < best || (!hit && res.t <= 1))) {
          best = res.t; bestTri = ti; hit = true;
          if (best === 0) { stack.length = 0; break; }
        }
      }
    } else {
      stack.push(node.left, node.right);
    }
  }
  return { hit, t: hit ? best : 1, tri: bestTri, tests: stats.triTests };
}

// --- Baseline: fixed-substep discrete sweep ---------------------------------
// Classic game-loop CCD replacement: move in N equal steps and test overlap at
// the sample points only. Tunnels when the sphere passes through thin geometry
// between two samples.
export function discreteSweep(mesh, p0, p1, r, steps = 16) {
  const reach = r + r * REL_EPS;
  let tests = 0;
  for (let s = 0; s <= steps; s++) {
    const p = lerp3(p0, p1, s / steps);
    for (let i = 0; i < mesh.indices.length; i++) {
      const [a, b, c] = triOf(mesh, i);
      tests++;
      if (closestPointTriangle2(p, a, b, c) <= reach * reach) return { hit: true, step: s, tests };
    }
  }
  return { hit: false, step: -1, tests };
}

// --- Independent search-based oracle (no shared algebra, no shared eps) -----
// Brackets a sign change of d(t)-r on a dense grid, then bisects. Independent
// of sweepSphereTriangle's analytic roots.
export function oracleFirstContact(p0, p1, r, a, b, c, grid = 2048) {
  const f = (t) => distPointTriangle(lerp3(p0, p1, t), a, b, c) - r;
  let prevT = 0, prevF = f(0);
  if (prevF <= 0) return 0;
  for (let i = 1; i <= grid; i++) {
    const t = i / grid, ft = f(t);
    if (ft <= 0) {
      let lo = prevT, hi = t, flo = prevF;
      for (let it = 0; it < 80; it++) {
        const mid = 0.5 * (lo + hi), fm = f(mid);
        if (fm > 0) { lo = mid; flo = fm; } else { hi = mid; }
      }
      return hi;
    }
    prevT = t; prevF = ft;
  }
  return null;
}

// --- Deterministic sample geometry -----------------------------------------
export function boxMesh(min, max) {
  const [x0, y0, z0] = min, [x1, y1, z1] = max;
  const p = [
    [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
    [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1],
  ];
  const f = [
    [0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
    [0, 1, 5], [0, 5, 4], [3, 7, 6], [3, 6, 2],
    [0, 4, 7], [0, 7, 3], [1, 2, 6], [1, 6, 5],
  ];
  return { positions: p, indices: f };
}

export function mergeMeshes(meshes) {
  const positions = [];
  const indices = [];
  for (const m of meshes) {
    const base = positions.length;
    for (const p of m.positions) positions.push(p);
    for (const [a, b, c] of m.indices) indices.push([a + base, b + base, c + base]);
  }
  return { positions, indices };
}

// Deterministic FNV-1a over the mesh, quantised so it is stable across engines.
export function meshFingerprint(mesh) {
  let h = 0x811c9dc5 >>> 0;
  const feed = (x) => {
    const s = Math.round(x * 1e6) | 0;
    for (let b = 0; b < 4; b++) {
      h ^= (s >>> (b * 8)) & 255;
      h = Math.imul(h, 0x01000193) >>> 0;
    }
  };
  for (const p of mesh.positions) { feed(p[0]); feed(p[1]); feed(p[2]); }
  for (const [a, b, c] of mesh.indices) { feed(a); feed(b); feed(c); }
  return h.toString(16).padStart(8, "0");
}

// Scan a fixed set of moves and hash the resulting TOIs: the browser and node
// print the same string, which is the "所见即所验" check.
export function sweepFingerprint(bvh, moves, r) {
  let h = 0x811c9dc5 >>> 0;
  const feed = (x) => {
    for (let b = 0; b < 4; b++) {
      h ^= (x >>> (b * 8)) & 255;
      h = Math.imul(h, 0x01000193) >>> 0;
    }
  };
  for (const [p0, p1] of moves) {
    const res = sweepSphereMesh(bvh, p0, p1, r);
    feed(res.hit ? Math.round(res.t * 1e9) | 0 : -1);
  }
  return h.toString(16).padStart(8, "0");
}