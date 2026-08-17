import { contains, aabbFromCenterRadius } from './hyperrect.js';

function binom(n, k) {
  if (k < 0 || k > n) return 0;
  let v = 1;
  for (let i = 1; i <= k; i++) v *= (n + 1 - i) / i;
  return v;
}

function bernstein(n, i, s) {
  if (i < 0 || i > n) return 0;
  return binom(n, i) * s ** i * (1 - s) ** (n - i);
}

function bernsteinDeriv(n, i, s, k) {
  if (k === 0) return bernstein(n, i, s);
  if (k > n) return 0;
  return n * (bernsteinDeriv(n - 1, i - 1, s, k - 1) - bernsteinDeriv(n - 1, i, s, k - 1));
}

function evalBezier(ctrl, s, deriv = 0) {
  const n = ctrl.length - 1;
  const out = [0, 0, 0];
  for (let i = 0; i < ctrl.length; i++) {
    const b = deriv === 0 ? bernstein(n, i, s) : bernsteinDeriv(n, i, s, deriv);
    out[0] += ctrl[i][0] * b;
    out[1] += ctrl[i][1] * b;
    out[2] += ctrl[i][2] * b;
  }
  return out;
}

function boxAabb(box) {
  return box.aabb || aabbFromCenterRadius(box.p, box.r);
}

function clampToBox(p, box) {
  const aabb = boxAabb(box);
  return [
    Math.min(aabb.max[0], Math.max(aabb.min[0], p[0])),
    Math.min(aabb.max[1], Math.max(aabb.min[1], p[1])),
    Math.min(aabb.max[2], Math.max(aabb.min[2], p[2])),
  ];
}

function lerp(a, b, t) {
  return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
}

function add(a, b) {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function scale(a, s) {
  return [a[0] * s, a[1] * s, a[2] * s];
}

/**
 * 在安全管内生成分段 Bézier。第一段满足静止起步；其余段把控制点夹在对应盒子内。
 * 这是论文 Algorithm 2 的启发式替代（完整版需迭代线性规划）。
 */
export function bezierInTube(boxes, options = {}) {
  const np = options.np ?? 5;
  if (!boxes || boxes.length < 2) return { ok: false, segments: [] };

  const segments = [];
  for (let i = 0; i < boxes.length - 1; i++) {
    const a = boxes[i];
    const b = boxes[i + 1];
    const delta = Math.max(0.12, b.t - a.t);
    const ctrl = [];
    if (i === 0) {
      for (let k = 0; k <= np; k++) {
        const u = k <= 2 ? 0 : (k - 2) / (np - 2);
        ctrl.push(clampToBox(lerp(a.p, b.p, u), a));
      }
      ctrl[0] = a.p.slice();
      ctrl[1] = a.p.slice();
      ctrl[2] = a.p.slice();
      ctrl[np] = clampToBox(b.p, a);
    } else {
      const prev = segments[i - 1];
      const vEnd = scale(
        [
          prev.ctrl[np][0] - prev.ctrl[np - 1][0],
          prev.ctrl[np][1] - prev.ctrl[np - 1][1],
          prev.ctrl[np][2] - prev.ctrl[np - 1][2],
        ],
        np / prev.delta
      );
      for (let k = 0; k <= np; k++) ctrl.push(clampToBox(lerp(a.p, b.p, k / np), a));
      ctrl[0] = a.p.slice();
      ctrl[1] = clampToBox(add(a.p, scale(vEnd, delta / np)), a);
      ctrl[np] = clampToBox(b.p, a);
    }
    segments.push({ ctrl, t0: a.t, t1: b.t, delta, box: a });
  }

  const duration = segments[segments.length - 1].t1;
  return { ok: true, segments, duration, np };
}

export function evalTrajectory(traj, t) {
  if (!traj?.segments?.length) return null;
  const last = traj.segments[traj.segments.length - 1];
  const tt = Math.min(Math.max(0, t), last.t1);
  let seg = last;
  for (const s of traj.segments) {
    if (tt <= s.t1 + 1e-9) {
      seg = s;
      break;
    }
  }
  const u = Math.min(1, Math.max(0, (tt - seg.t0) / seg.delta));
  const p = evalBezier(seg.ctrl, u, 0);
  const d1 = evalBezier(seg.ctrl, u, 1);
  const d2 = evalBezier(seg.ctrl, u, 2);
  const d3 = evalBezier(seg.ctrl, u, 3);
  const inv = 1 / seg.delta;
  const v = scale(d1, inv);
  const a = scale(d2, inv * inv);
  const jerk = scale(d3, inv * inv * inv);
  const speed = Math.hypot(v[0], v[1], v[2]);
  return {
    p,
    v,
    a,
    jerk,
    yawDir: speed > 1e-3 ? [v[0] / speed, v[1] / speed, v[2] / speed] : [0, 0, 1],
    t: tt,
    done: t >= last.t1 - 1e-3,
    box: seg.box,
  };
}

export function sampleTrajectory(traj, dt = 0.08) {
  const samples = [];
  const tangents = [];
  const sTable = [];
  let acc = 0;
  let prev = null;
  for (let t = 0; t <= traj.duration + 1e-6; t += dt) {
    const e = evalTrajectory(traj, t);
    if (!e) continue;
    if (prev) acc += Math.hypot(e.p[0] - prev[0], e.p[1] - prev[1], e.p[2] - prev[2]);
    samples.push(e.p);
    tangents.push(e.v);
    sTable.push(acc);
    prev = e.p;
  }
  const end = evalTrajectory(traj, traj.duration);
  if (end && (!prev || Math.hypot(end.p[0] - prev[0], end.p[1] - prev[1], end.p[2] - prev[2]) > 1e-6)) {
    acc += prev ? Math.hypot(end.p[0] - prev[0], end.p[1] - prev[1], end.p[2] - prev[2]) : 0;
    samples.push(end.p);
    tangents.push(end.v);
    sTable.push(acc);
  }
  return { samples, tangents, s: sTable, length: acc };
}

export function pointInTube(p, boxes, t) {
  if (!boxes?.length) return false;
  let box = boxes[0];
  for (let i = 0; i < boxes.length - 1; i++) {
    if (t >= boxes[i].t && t <= boxes[i + 1].t) {
      box = boxes[i];
      break;
    }
    if (t > boxes[boxes.length - 1].t) box = boxes[boxes.length - 1];
  }
  const aabb = boxAabb(box);
  const pad = (box.lp ?? 0) + 0.05;
  return contains(
    {
      min: [aabb.min[0] - pad, aabb.min[1] - pad, aabb.min[2] - pad],
      max: [aabb.max[0] + pad, aabb.max[1] + pad, aabb.max[2] + pad],
    },
    p
  );
}
