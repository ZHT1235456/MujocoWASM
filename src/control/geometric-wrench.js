import { MASS, GRAVITY } from '../world-scene.js';

export const MAX_TILT = 0.42;
const KP = 6.8;
const KV = 3.9;
const KR = 8.5;
const KW = 2.2;
const J = [0.014, 0.014, 0.022];

let prevRd = null;
let prevT = null;

export function resetGeometric() {
  prevRd = null;
  prevT = null;
}

function quatToR(q) {
  const [w, x, y, z] = q;
  return [
    [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
    [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
  ];
}

function mulMatVec(R, v) {
  return [
    R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
    R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
    R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
  ];
}

function transpose(R) {
  return [
    [R[0][0], R[1][0], R[2][0]],
    [R[0][1], R[1][1], R[2][1]],
    [R[0][2], R[1][2], R[2][2]],
  ];
}

function mulMat(A, B) {
  const C = [
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
  ];
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      C[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j];
    }
  }
  return C;
}

function subMat(A, B) {
  return [
    [A[0][0] - B[0][0], A[0][1] - B[0][1], A[0][2] - B[0][2]],
    [A[1][0] - B[1][0], A[1][1] - B[1][1], A[1][2] - B[1][2]],
    [A[2][0] - B[2][0], A[2][1] - B[2][1], A[2][2] - B[2][2]],
  ];
}

/** vee(hat(w)) = w */
function vee(S) {
  return [S[2][1], S[0][2], S[1][0]];
}

function clamp(v, a, b) {
  return Math.min(b, Math.max(a, v));
}

function cross(a, b) {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

function norm(v) {
  return Math.hypot(v[0], v[1], v[2]);
}

function normalize(v) {
  const n = norm(v) || 1;
  return [v[0] / n, v[1] / n, v[2] / n];
}

function so3LogOmega(Rd, prev) {
  const Rerr = mulMat(transpose(prev), Rd);
  const tr = Rerr[0][0] + Rerr[1][1] + Rerr[2][2];
  const theta = Math.acos(Math.min(1, Math.max(-1, (tr - 1) / 2)));
  if (theta < 1e-8) return [0, 0, 0];
  const w = [
    (Rerr[2][1] - Rerr[1][2]) / (2 * Math.sin(theta)),
    (Rerr[0][2] - Rerr[2][0]) / (2 * Math.sin(theta)),
    (Rerr[1][0] - Rerr[0][1]) / (2 * Math.sin(theta)),
  ];
  return [w[0] * theta, w[1] * theta, w[2] * theta];
}

/** 与已验证的级联 PID 同一套机体轴：+Z 推力，-Y 机头。 */
export function desiredRotation(b3, yawDirMj) {
  const z = normalize(b3);
  let nose = normalize([yawDirMj[0], yawDirMj[1], 0]);
  if (Math.hypot(nose[0], nose[1]) < 1e-4) nose = [0, -1, 0];
  let y = normalize([-nose[0], -nose[1], -nose[2]]);
  let x = cross(y, z);
  if (norm(x) < 1e-6) x = [1, 0, 0];
  else x = normalize(x);
  y = cross(z, x);
  return [
    [x[0], y[0], z[0]],
    [x[1], y[1], z[1]],
    [x[2], y[2], z[2]],
  ];
}

function limitTilt(Fd) {
  const maxH = Math.tan(MAX_TILT) * Math.max(0.4, Fd[2]);
  const horiz = Math.hypot(Fd[0], Fd[1]);
  if (horiz > maxH) {
    const s = maxH / horiz;
    return [Fd[0] * s, Fd[1] * s, Fd[2]];
  }
  return Fd;
}

/**
 * SE(3) 几何跟踪：输出总推力与体轴力矩（MuJoCo Z-up）。
 */
export function geometricWrench(state, ref, t, dt) {
  const R = quatToR(state.quat);
  const omega = state.omega;
  const ep = [
    clamp(state.pos[0] - ref.p[0], -0.8, 0.8),
    clamp(state.pos[1] - ref.p[1], -0.8, 0.8),
    clamp(state.pos[2] - ref.p[2], -0.8, 0.8),
  ];
  const ev = [
    clamp(state.vel[0] - (ref.v?.[0] ?? 0), -3, 3),
    clamp(state.vel[1] - (ref.v?.[1] ?? 0), -3, 3),
    clamp(state.vel[2] - (ref.v?.[2] ?? 0), -3, 3),
  ];
  const ad = [
    clamp(ref.a?.[0] ?? 0, -6, 6),
    clamp(ref.a?.[1] ?? 0, -6, 6),
    clamp(ref.a?.[2] ?? 0, -6, 6),
  ];

  let Fd = [
    -KP * ep[0] - KV * ev[0] + MASS * ad[0],
    -KP * ep[1] - KV * ev[1] + MASS * ad[1],
    -KP * ep[2] - KV * ev[2] + MASS * GRAVITY + MASS * ad[2],
  ];
  Fd[2] = Math.max(MASS * GRAVITY * 0.35, Fd[2]);
  Fd = limitTilt(Fd);

  const e3 = [R[0][2], R[1][2], R[2][2]];
  const f = Math.max(0, Fd[0] * e3[0] + Fd[1] * e3[1] + Fd[2] * e3[2]);

  const yaw = ref.yawDirMj || [0, -1, 0];
  const Rd = desiredRotation(Fd, yaw);
  const eR = vee(subMat(mulMat(transpose(Rd), R), mulMat(transpose(R), Rd))).map((v) => 0.5 * v);

  let omegaD = [0, 0, 0];
  if (prevRd && prevT != null && dt > 1e-4) {
    omegaD = so3LogOmega(Rd, prevRd).map((w) => clamp(w / dt, -6, 6));
  }
  prevRd = Rd;
  prevT = t;

  const omegaDBody = mulMatVec(transpose(R), mulMatVec(Rd, omegaD));
  const eOmega = [omega[0] - omegaDBody[0], omega[1] - omegaDBody[1], omega[2] - omegaDBody[2]];
  const Jw = [J[0] * omega[0], J[1] * omega[1], J[2] * omega[2]];
  const tau = [
    clamp(-KR * eR[0] - KW * eOmega[0] + (omega[1] * Jw[2] - omega[2] * Jw[1]), -0.35, 0.35),
    clamp(-KR * eR[1] - KW * eOmega[1] + (omega[2] * Jw[0] - omega[0] * Jw[2]), -0.35, 0.35),
    clamp(-KR * eR[2] - KW * eOmega[2] + (omega[0] * Jw[1] - omega[1] * Jw[0]), -0.25, 0.25),
  ];

  const tilt = Math.acos(clamp(normalize(Fd)[2], -1, 1));
  return { f, tau, Fd, eR, Rd, tilt };
}

export function hoverState(pos = [0, 0, 1.4]) {
  return {
    pos,
    quat: [1, 0, 0, 0],
    vel: [0, 0, 0],
    omega: [0, 0, 0],
  };
}
