// Render + interaction only. All collision maths comes from ./src/core.mjs,
// the same module bench/verify.mjs imports in node.
import * as THREE from "three";
import {
  buildBVH, sweepSphereMesh, discreteSweep, sweepFingerprint, meshFingerprint,
} from "./src/core.mjs";

const RADIUS = 0.35;
const DT = 1 / 60;
const SUBSTEPS = 16;
const START = new THREE.Vector3(0, 0.9, 12);

const doc = await (await fetch("./data/level.json")).json();
const mesh = { positions: doc.positions, indices: doc.indices };
const bvh = buildBVH(mesh);
const meshFp = meshFingerprint(mesh);
const sweepFp = sweepFingerprint(bvh, doc.moves, RADIUS);

// ---- scene ---------------------------------------------------------------
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
renderer.shadowMap.enabled = false;
document.getElementById("view").appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0e14);
scene.fog = new THREE.Fog(0x0b0e14, 18, 46);
const camera = new THREE.PerspectiveCamera(72, innerWidth / innerHeight, 0.05, 200);

const positions = new Float32Array(mesh.positions.flat());
const geo = new THREE.BufferGeometry();
geo.setAttribute("position", new THREE.BufferAttribute(positions, 3));
geo.setIndex(mesh.indices.flat());
geo.computeVertexNormals();
const mat = new THREE.MeshStandardMaterial({ color: 0x9fb3cc, metalness: 0.1, roughness: 0.75, side: THREE.DoubleSide });
const level = new THREE.Mesh(geo, mat);
scene.add(level);
scene.add(new THREE.LineSegments(new THREE.EdgesGeometry(geo, 25), new THREE.LineBasicMaterial({ color: 0x2b3a52 })));
scene.add(new THREE.HemisphereLight(0x9fc7ff, 0x101820, 1.1));
const key = new THREE.DirectionalLight(0xffffff, 1.4);
key.position.set(6, 12, 8);
scene.add(key);

const player = new THREE.Mesh(
  new THREE.SphereGeometry(RADIUS, 20, 14),
  new THREE.MeshBasicMaterial({ color: 0x7ee0c0, wireframe: true }),
);
scene.add(player);
const contact = new THREE.Mesh(
  new THREE.SphereGeometry(0.06, 8, 6),
  new THREE.MeshBasicMaterial({ color: 0xff8a7a }),
);
contact.visible = false;
scene.add(contact);
const trailGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
const trail = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: 0xffc86b }));
scene.add(trail);

// ---- input ---------------------------------------------------------------
const pos = START.clone();
const keys = new Set();
let yaw = Math.PI, pitch = -0.05;
let speed = 6, exact = true, tunnelEvents = 0, frames = 0, fps = 0, last = performance.now();
let lastInfo = { exactT: null, discreteHit: false, tests: 0 };

addEventListener("keydown", (e) => keys.add(e.code));
addEventListener("keyup", (e) => keys.delete(e.code));
addEventListener("resize", () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});
addEventListener("mousemove", (e) => {
  if (document.pointerLockElement !== renderer.domElement) return;
  yaw -= e.movementX * 0.0022;
  pitch = Math.max(-1.3, Math.min(1.3, pitch - e.movementY * 0.0022));
});
renderer.domElement.addEventListener("click", () => renderer.domElement.requestPointerLock());
document.getElementById("speed").addEventListener("input", (e) => {
  speed = +e.target.value;
  document.getElementById("speedVal").textContent = `${speed} m/s`;
});
document.getElementById("mode").addEventListener("click", () => {
  exact = !exact;
  document.getElementById("mode").textContent = `mode: ${exact ? "exact CCD" : "16-substep discrete"}`;
});
document.getElementById("reset").addEventListener("click", () => {
  pos.copy(START); yaw = Math.PI; pitch = -0.05; tunnelEvents = 0; contact.visible = false;
});

// ---- loop ----------------------------------------------------------------
const q = new THREE.Quaternion().setFromEuler(new THREE.Euler(pitch, yaw, 0, "YXZ"));
const fwd = new THREE.Vector3(), right = new THREE.Vector3(), dir = new THREE.Vector3();

function tick(now) {
  requestAnimationFrame(tick);
  const dtMs = Math.max(1, now - last);
  last = now;
  frames++;
  if (frames % 15 === 0) fps = Math.round(1000 / dtMs);

  camera.position.copy(pos).add(new THREE.Vector3(0, 0.55, 0));
  camera.quaternion.copy(q.setFromEuler(new THREE.Euler(pitch, yaw, 0, "YXZ")));

  fwd.set(Math.sin(yaw) * -1, 0, Math.cos(yaw) * -1).normalize();
  right.crossVectors(fwd, new THREE.Vector3(0, 1, 0)).normalize();
  dir.set(0, 0, 0);
  if (keys.has("KeyW")) dir.add(fwd);
  if (keys.has("KeyS")) dir.sub(fwd);
  if (keys.has("KeyD")) dir.add(right);
  if (keys.has("KeyA")) dir.sub(right);
  if (dir.lengthSq() > 0) dir.normalize().multiplyScalar(speed * DT);

  if (dir.lengthSq() > 0) {
    const p0 = [pos.x, pos.y, pos.z];
    const p1 = [pos.x + dir.x, pos.y + dir.y, pos.z + dir.z];
    const ex = sweepSphereMesh(bvh, p0, p1, RADIUS);
    const di = discreteSweep(mesh, p0, p1, RADIUS, SUBSTEPS);
    lastInfo = { exactT: ex.hit ? ex.t : null, discreteHit: di.hit, tests: ex.tests };
    // A pass-through event is exactly the disagreeing case: the continuous
    // sweep hits inside this frame while the substepped sampler reports none.
    if (ex.hit && !di.hit) tunnelEvents++;

    const chosen = exact ? ex : di;
    if (chosen.hit) {
      pos.add(dir.multiplyScalar(Math.max(0, chosen.t - 0.002)));
      contact.position.set(pos.x + dir.x, pos.y + dir.y, pos.z + dir.z);
      contact.visible = true;
    } else {
      pos.add(dir);
      contact.visible = false;
    }
    contact.visible = chosen.hit;
  }

  player.position.copy(pos);
  trailGeo.setFromPoints([new THREE.Vector3(...[pos.x, pos.y, pos.z]), new THREE.Vector3(pos.x - dir.x, pos.y - dir.y, pos.z - dir.z)]);
  trailGeo.attributes.position.needsUpdate = true;

  const el = document.getElementById("stats");
  el.innerHTML =
    `<span class="k">mode        </span> <span class="${exact ? "ok" : "warn"}">${exact ? "exact CCD" : "16-substep discrete"}</span>\n` +
    `<span class="k">speed       </span> ${speed} m/s (${(speed * DT).toFixed(2)} m/frame)\n` +
    `<span class="k">position    </span> ${pos.x.toFixed(2)}, ${pos.y.toFixed(2)}, ${pos.z.toFixed(2)}\n` +
    `<span class="k">exact TOI   </span> ${lastInfo.exactT === null ? "—" : lastInfo.exactT.toFixed(6)}\n` +
    `<span class="k">discrete    </span> ${lastInfo.discreteHit ? "<span class=hit>hit</span>" : "miss"}\n` +
    `<span class="k">tri tests   </span> ${lastInfo.tests} (bvh)\n` +
    `<span class="k">tunnel evts </span> <span class="${tunnelEvents ? "hit" : "ok"}">${tunnelEvents}</span>  <span class="k">fps</span> ${fps}\n` +
    `<span class="k">mesh fp     </span> ${meshFp}\n` +
    `<span class="k">sweep fp    </span> ${sweepFp} <span class="k">(node bench prints this)</span>`;
  renderer.render(scene, camera);
}
requestAnimationFrame(tick);
window.__lab = { meshFp, sweepFp, doc }; // read-only probe, used by smoke checks