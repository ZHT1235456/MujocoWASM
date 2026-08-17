import { threePosToMj } from '../coords.js';
import { mix } from './pid.js';
import { geometricWrench, resetGeometric } from './geometric-wrench.js';

export { resetGeometric };

const MAX_THRUST = 7.2;

export function computeGeometricControl(data, ref, t, dt) {
  const qpos = data.qpos;
  const qvel = data.qvel;
  const state = {
    pos: [qpos[0], qpos[1], qpos[2]],
    quat: [qpos[3], qpos[4], qpos[5], qpos[6]],
    vel: [qvel[0], qvel[1], qvel[2]],
    omega: [qvel[3], qvel[4], qvel[5]],
  };
  const mjRef = {
    p: threePosToMj(ref.p[0], ref.p[1], ref.p[2]),
    v: threePosToMj(ref.v[0], ref.v[1], ref.v[2]),
    a: threePosToMj(ref.a?.[0] ?? 0, ref.a?.[1] ?? 0, ref.a?.[2] ?? 0),
  };
  const { f, tau } = geometricWrench(state, mjRef, t, dt);
  return mix(Math.min(MAX_THRUST * 4, Math.max(0, f)), tau);
}
