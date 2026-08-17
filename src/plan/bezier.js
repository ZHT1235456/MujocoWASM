import { DRONE_RADIUS } from '../world.js';
import { segmentCollides, dist, lerp } from './collide.js';

function bezierPoint(p0, p1, p2, p3, t) {
  const u = 1 - t;
  const a = u * u * u;
  const b = 3 * u * u * t;
  const c = 3 * u * t * t;
  const d = t * t * t;
  return [
    a * p0[0] + b * p1[0] + c * p2[0] + d * p3[0],
    a * p0[1] + b * p1[1] + c * p2[1] + d * p3[1],
    a * p0[2] + b * p1[2] + c * p2[2] + d * p3[2],
  ];
}

function bezierDerivative(p0, p1, p2, p3, t) {
  const u = 1 - t;
  return [
    3 * u * u * (p1[0] - p0[0]) + 6 * u * t * (p2[0] - p1[0]) + 3 * t * t * (p3[0] - p2[0]),
    3 * u * u * (p1[1] - p0[1]) + 6 * u * t * (p2[1] - p1[1]) + 3 * t * t * (p3[1] - p2[1]),
    3 * u * u * (p1[2] - p0[2]) + 6 * u * t * (p2[2] - p1[2]) + 3 * t * t * (p3[2] - p2[2]),
  ];
}

function sampleCubic(p0, p1, p2, p3, ds = 0.12) {
  const pts = [];
  const tans = [];
  const n = Math.max(8, Math.ceil((dist(p0, p3) + dist(p0, p1) + dist(p2, p3)) / ds));
  for (let i = 0; i <= n; i++) {
    const t = i / n;
    pts.push(bezierPoint(p0, p1, p2, p3, t));
    tans.push(bezierDerivative(p0, p1, p2, p3, t));
  }
  return { pts, tans };
}

function curveCollides(samples, radius) {
  for (let i = 0; i < samples.length - 1; i++) {
    if (segmentCollides(samples[i], samples[i + 1], radius, 0.1)) return true;
  }
  return false;
}

/**
 * 对折线路标做 G1 三次贝塞尔平滑；碰撞则缩小切向手柄或回退折线。
 */
export function smoothBezier(path, options = {}) {
  const radius = options.robotRadius ?? DRONE_RADIUS + 0.06;
  if (path.length < 2) return { samples: path, tangents: path.map(() => [0, 0, 1]), ok: false };

  if (path.length === 2) {
    const samples = [];
    const tangents = [];
    const n = Math.max(8, Math.ceil(dist(path[0], path[1]) / 0.12));
    for (let i = 0; i <= n; i++) {
      samples.push(lerp(path[0], path[1], i / n));
      tangents.push([path[1][0] - path[0][0], path[1][1] - path[0][1], path[1][2] - path[0][2]]);
    }
    return { samples, tangents, ok: true };
  }

  const samples = [];
  const tangents = [];
  const alpha0 = options.alpha ?? 0.32;

  for (let i = 0; i < path.length - 1; i++) {
    const p0 = path[i];
    const p3 = path[i + 1];
    const prev = path[Math.max(0, i - 1)];
    const next = path[Math.min(path.length - 1, i + 2)];
    const dIn = [p0[0] - prev[0], p0[1] - prev[1], p0[2] - prev[2]];
    const dOut = [next[0] - p3[0], next[1] - p3[1], next[2] - p3[2]];
    const seg = dist(p0, p3);

    let ok = false;
    let chosen = null;
    for (const alpha of [alpha0, 0.22, 0.12, 0.05, 0]) {
      const p1 = [p0[0] + dIn[0] * alpha, p0[1] + dIn[1] * alpha, p0[2] + dIn[2] * alpha];
      const p2 = [p3[0] - dOut[0] * alpha, p3[1] - dOut[1] * alpha, p3[2] - dOut[2] * alpha];
      const { pts, tans } = sampleCubic(p0, p1, p2, p3, Math.max(0.1, seg / 10));
      if (!curveCollides(pts, radius)) {
        ok = true;
        chosen = { pts, tans };
        break;
      }
    }
    if (!ok) {
      const { pts, tans } = sampleCubic(p0, lerp(p0, p3, 0.33), lerp(p0, p3, 0.66), p3, 0.12);
      chosen = { pts, tans };
    }

    const start = samples.length ? 1 : 0;
    for (let k = start; k < chosen.pts.length; k++) {
      samples.push(chosen.pts[k]);
      tangents.push(chosen.tans[k]);
    }
  }

  return { samples, tangents, ok: true };
}

export function resampleArc(samples, tangents, ds = 0.08) {
  if (samples.length < 2) return { samples, tangents, s: [0], length: 0 };
  const outP = [samples[0]];
  const outT = [tangents[0]];
  const s = [0];
  let acc = 0;
  let carry = 0;
  for (let i = 1; i < samples.length; i++) {
    let remaining = dist(samples[i - 1], samples[i]);
    let a = samples[i - 1];
    const b = samples[i];
    while (carry + remaining >= ds) {
      const need = ds - carry;
      const t = need / remaining;
      a = lerp(a, b, t);
      acc += need;
      outP.push(a);
      outT.push(tangents[i]);
      s.push(acc);
      remaining -= need;
      carry = 0;
    }
    carry += remaining;
  }
  if (dist(outP[outP.length - 1], samples[samples.length - 1]) > 1e-4) {
    acc += dist(outP[outP.length - 1], samples[samples.length - 1]);
    outP.push(samples[samples.length - 1]);
    outT.push(tangents[tangents.length - 1]);
    s.push(acc);
  }
  return { samples: outP, tangents: outT, s, length: acc };
}
