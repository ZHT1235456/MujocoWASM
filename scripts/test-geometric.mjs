import { MASS, GRAVITY } from '../src/world-scene.js';
import { geometricWrench, resetGeometric, MAX_TILT, hoverState } from '../src/control/geometric-wrench.js';
import { mix, motorWrench } from '../src/control/pid.js';

function assert(cond, msg) {
  if (!cond) throw new Error(msg);
}

function rotX(a) {
  const c = Math.cos(a);
  const s = Math.sin(a);
  const q = [Math.cos(a / 2), Math.sin(a / 2), 0, 0];
  return q;
}

const hoverRef = {
  p: [0, 0, 1.4],
  v: [0, 0, 0],
  a: [0, 0, 0],
  yawDirMj: [0, -1, 0],
};

resetGeometric();
const h = geometricWrench(hoverState(), hoverRef, 0, 0.002);
assert(Math.abs(h.f - MASS * GRAVITY) < 0.15, `hover thrust ${h.f} != mg`);
assert(h.tau.every((t) => Math.abs(t) < 1e-4), `hover tau ${h.tau}`);
assert(h.tilt < 0.05, `hover tilt ${h.tilt}`);
console.log('ok hover', { f: +h.f.toFixed(3), tau: h.tau, tilt: +h.tilt.toFixed(4) });

const hoverMotors = mix(MASS * GRAVITY, [0, 0, 0]);
const hoverWrench = motorWrench(hoverMotors);
assert(hoverWrench.every((value, i) => Math.abs(value - [MASS * GRAVITY, 0, 0, 0][i]) < 1e-9), `hover allocation ${hoverWrench}`);
assert(Math.abs(hoverMotors[0] - hoverMotors[1]) > 0.5, 'asymmetric frame requires asymmetric hover thrusts');
console.log('ok hover allocation', { motors: hoverMotors.map((v) => +v.toFixed(3)), wrench: hoverWrench });

const desiredTau = [0.1, -0.08, 0.01];
const allocatedWrench = motorWrench(mix(MASS * GRAVITY, desiredTau));
assert(
  allocatedWrench.every((value, i) => Math.abs(value - [MASS * GRAVITY, ...desiredTau][i]) < 1e-9),
  `wrench allocation ${allocatedWrench}`
);
console.log('ok wrench allocation', allocatedWrench.map((v) => +v.toFixed(3)));

const saturatedYawWrench = motorWrench(mix(MASS * GRAVITY, [0, 0, 0.25]));
assert(Math.abs(saturatedYawWrench[0] - MASS * GRAVITY) < 1e-9, `yaw saturation changed thrust ${saturatedYawWrench}`);
assert(Math.abs(saturatedYawWrench[1]) < 1e-9, `yaw saturation caused roll ${saturatedYawWrench}`);
assert(Math.abs(saturatedYawWrench[2]) < 1e-9, `yaw saturation caused pitch ${saturatedYawWrench}`);
assert(saturatedYawWrench[3] > 0 && saturatedYawWrench[3] < 0.25, `yaw saturation was not bounded ${saturatedYawWrench}`);
console.log('ok priority saturation', saturatedYawWrench.map((v) => +v.toFixed(3)));

resetGeometric();
const rolled = hoverState();
rolled.quat = rotX(0.2);
const r = geometricWrench(rolled, hoverRef, 0.002, 0.002);
assert(r.tau[0] < 0, `roll error should produce negative roll torque, got ${r.tau[0]}`);
console.log('ok roll sign', { eR: r.eR.map((v) => +v.toFixed(4)), tau: r.tau.map((v) => +v.toFixed(4)) });

resetGeometric();
const far = hoverState();
const farRef = { ...hoverRef, p: [8, 0, 1.4] };
const g = geometricWrench(far, farRef, 0, 0.002);
assert(g.tilt <= MAX_TILT + 0.02, `tilt ${g.tilt} exceeds MAX_TILT ${MAX_TILT}`);
assert(g.Fd[2] > 0, 'Fz must stay positive');
console.log('ok tilt limit', { tilt: +g.tilt.toFixed(3), Fd: g.Fd.map((v) => +v.toFixed(2)) });

resetGeometric();
let quat = rotX(0.25);
let omega = [0, 0, 0];
const Jx = 0.014;
let angle = 0.25;
for (let i = 0; i < 200; i++) {
  const st = hoverState();
  st.quat = quat;
  st.omega = omega;
  const w = geometricWrench(st, hoverRef, i * 0.002, 0.002);
  const acc = w.tau[0] / Jx;
  omega[0] += acc * 0.002;
  angle += omega[0] * 0.002;
  quat = rotX(angle);
}
assert(Math.abs(angle) < 0.08, `roll did not recover, angle=${angle}`);
console.log('ok roll recover', { angle: +angle.toFixed(4) });

console.log('all geometric tests passed');
