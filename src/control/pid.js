import { MASS, GRAVITY, HOVER_THRUST, motorSitesMj } from '../world.js';
import { threePosToMj } from '../coords.js';

const KP_POS = [6.5, 6.5, 9.5];
const KD_POS = [3.8, 3.8, 5.2];
const KP_ATT = 18;
const KD_ATT = 2.4;
const MAX_TILT = 0.45;
const MAX_THRUST = 7.2;

function clamp(v, a, b) {
  return Math.min(b, Math.max(a, v));
}

function quatRotate(q, v) {
  const [w, x, y, z] = q;
  const vx = v[0];
  const vy = v[1];
  const vz = v[2];
  const tx = 2 * (y * vz - z * vy);
  const ty = 2 * (z * vx - x * vz);
  const tz = 2 * (x * vy - y * vx);
  return [vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx)];
}

function quatConj(q) {
  return [q[0], -q[1], -q[2], -q[3]];
}

function quatMul(a, b) {
  return [
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  ];
}

function normalize(v) {
  const n = Math.hypot(v[0], v[1], v[2]) || 1;
  return [v[0] / n, v[1] / n, v[2] / n];
}

function cross(a, b) {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

function matFromAxes(x, y, z) {
  return [
    [x[0], y[0], z[0]],
    [x[1], y[1], z[1]],
    [x[2], y[2], z[2]],
  ];
}

function rotToQuat(R) {
  const t = R[0][0] + R[1][1] + R[2][2];
  let w, x, y, z;
  if (t > 0) {
    const s = Math.sqrt(t + 1) * 2;
    w = 0.25 * s;
    x = (R[2][1] - R[1][2]) / s;
    y = (R[0][2] - R[2][0]) / s;
    z = (R[1][0] - R[0][1]) / s;
  } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
    const s = Math.sqrt(1 + R[0][0] - R[1][1] - R[2][2]) * 2;
    w = (R[2][1] - R[1][2]) / s;
    x = 0.25 * s;
    y = (R[0][1] + R[1][0]) / s;
    z = (R[0][2] + R[2][0]) / s;
  } else if (R[1][1] > R[2][2]) {
    const s = Math.sqrt(1 + R[1][1] - R[0][0] - R[2][2]) * 2;
    w = (R[0][2] - R[2][0]) / s;
    x = (R[0][1] + R[1][0]) / s;
    y = 0.25 * s;
    z = (R[1][2] + R[2][1]) / s;
  } else {
    const s = Math.sqrt(1 + R[2][2] - R[0][0] - R[1][1]) * 2;
    w = (R[1][0] - R[0][1]) / s;
    x = (R[0][2] + R[2][0]) / s;
    y = (R[1][2] + R[2][1]) / s;
    z = 0.25 * s;
  }
  const n = Math.hypot(w, x, y, z) || 1;
  return [w / n, x / n, y / n, z / n];
}

function desiredRotation(b3, yawDir) {
  const z = normalize(b3);
  let nose = normalize([yawDir[0], yawDir[1], 0]);
  if (Math.hypot(nose[0], nose[1]) < 1e-4) nose = [0, -1, 0];
  let y = normalize([-nose[0], -nose[1], -nose[2]]);
  let x = cross(y, z);
  if (Math.hypot(x[0], x[1], x[2]) < 1e-6) {
    x = [1, 0, 0];
  } else {
    x = normalize(x);
  }
  y = cross(z, x);
  return rotToQuat(matFromAxes(x, y, z));
}

const SITES = motorSitesMj();

export function mix(thrust, tau) {
  const A = SITES.map((s) => [1, s.pos[1], -s.pos[0], -s.spin * 0.018]);
  const u = [HOVER_THRUST, HOVER_THRUST, HOVER_THRUST, HOVER_THRUST];
  const wrench = [thrust, tau[0], tau[1], tau[2]];
  for (let iter = 0; iter < 8; iter++) {
    const pred = [0, 0, 0, 0];
    for (let i = 0; i < 4; i++) {
      for (let k = 0; k < 4; k++) pred[k] += A[i][k] * u[i];
    }
    const err = wrench.map((w, k) => w - pred[k]);
    for (let i = 0; i < 4; i++) {
      let g = 0;
      for (let k = 0; k < 4; k++) g += A[i][k] * err[k];
      u[i] = clamp(u[i] + 0.18 * g, 0, MAX_THRUST);
    }
  }
  return u;
}

/**
 * 级联位置/姿态控制，输出 4 个电机推力。
 * 轨迹点在 Three.js Y-up，内部转到 MuJoCo Z-up。
 */
export function computeControl(data, ref) {
  const qpos = data.qpos;
  const qvel = data.qvel;
  const p = [qpos[0], qpos[1], qpos[2]];
  const q = [qpos[3], qpos[4], qpos[5], qpos[6]];
  const v = [qvel[0], qvel[1], qvel[2]];
  const w = [qvel[3], qvel[4], qvel[5]];

  const pd = threePosToMj(ref.p[0], ref.p[1], ref.p[2]);
  const vd = threePosToMj(ref.v[0], ref.v[1], ref.v[2]);
  const yawDir = threePosToMj(ref.yawDir[0], 0, ref.yawDir[2]);

  const a = [
    KP_POS[0] * (pd[0] - p[0]) + KD_POS[0] * (vd[0] - v[0]),
    KP_POS[1] * (pd[1] - p[1]) + KD_POS[1] * (vd[1] - v[1]),
    KP_POS[2] * (pd[2] - p[2]) + KD_POS[2] * (vd[2] - v[2]) + GRAVITY,
  ];

  const horiz = Math.hypot(a[0], a[1]);
  const maxH = Math.tan(MAX_TILT) * Math.max(0.4, a[2]);
  if (horiz > maxH) {
    const s = maxH / horiz;
    a[0] *= s;
    a[1] *= s;
  }

  const b3 = normalize(a);
  const qd = desiredRotation(b3, yawDir);
  const qe = quatMul(quatConj(q), qd);
  if (qe[0] < 0) {
    qe[0] *= -1;
    qe[1] *= -1;
    qe[2] *= -1;
    qe[3] *= -1;
  }
  const eR = [2 * qe[1], 2 * qe[2], 2 * qe[3]];
  const tau = [KP_ATT * eR[0] - KD_ATT * w[0], KP_ATT * eR[1] - KD_ATT * w[1], KP_ATT * eR[2] - KD_ATT * w[2]];

  const bodyZ = quatRotate(q, [0, 0, 1]);
  let thrust = MASS * (a[0] * bodyZ[0] + a[1] * bodyZ[1] + a[2] * bodyZ[2]);
  thrust = clamp(thrust, 0, MAX_THRUST * 4);

  return mix(thrust, tau);
}

export function hoverThrusts() {
  return [HOVER_THRUST, HOVER_THRUST, HOVER_THRUST, HOVER_THRUST];
}
