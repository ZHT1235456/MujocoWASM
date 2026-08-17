import fs from 'node:fs';
import loadMujoco from '@mujoco/mujoco';
import { planSafeTube } from '../src/plan/rrt-tube.js';
import { bezierLpInTube } from '../src/plan/bezier-lp.js';
import { evalTrajectory } from '../src/plan/bezier-tube.js';
import { computeGeometricControl, resetGeometric } from '../src/control/geometric.js';

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

let seed = 0x5eed1234;
Math.random = () => {
  seed = (1664525 * seed + 1013904223) >>> 0;
  return seed / 0x100000000;
};

const start = [-8, 1.4, -8];
const goal = [8, 1.8, 8];
const planned = await planSafeTube(start, goal, { nv: 1500, margin0: 0.18, alphaV: 1.6 });
assert(planned.ok, planned.message || 'safe tube planning failed');
const traj = await bezierLpInTube(planned.boxes, { np: 9 });
assert(traj.ok, traj.message || 'trajectory LP failed');

const mujoco = await loadMujoco();
const xml = fs.readFileSync(new URL('../src/sim/quadrotor.xml', import.meta.url), 'utf8');
const model = mujoco.MjModel.from_xml_string(xml);
const data = new mujoco.MjData(model);

try {
  mujoco.mj_forward(model, data);
  resetGeometric();
  const dt = model.opt?.timestep ?? model.option?.timestep ?? 0.002;
  let trajectoryTime = 0;
  let completedAt = null;
  let minAltitude = Infinity;
  let maxTilt = 0;
  let maxPositionError = 0;
  let maxMotor = 0;

  for (let step = 0; step < 40000; step++) {
    const evaluated = evalTrajectory(traj, trajectoryTime);
    const ref = evaluated.done
      ? { p: evaluated.p, v: [0, 0, 0], a: [0, 0, 0], done: true }
      : evaluated;
    const motors = computeGeometricControl(data, ref, data.time, dt);
    assert(
      motors.every(Number.isFinite),
      `non-finite motors at step ${step}, sim=${data.time}, trajectory=${trajectoryTime}, dt=${dt}, ref=${JSON.stringify(ref)}`
    );
    for (let i = 0; i < 4; i++) {
      data.ctrl[i] = motors[i];
      maxMotor = Math.max(maxMotor, motors[i]);
    }
    mujoco.mj_step(model, data);

    const pThree = [data.qpos[0], data.qpos[2], -data.qpos[1]];
    minAltitude = Math.min(minAltitude, pThree[1]);
    const positionError = Math.hypot(
      pThree[0] - ref.p[0],
      pThree[1] - ref.p[1],
      pThree[2] - ref.p[2]
    );
    maxPositionError = Math.max(maxPositionError, positionError);
    const bodyZDotWorldZ = 1 - 2 * (data.qpos[4] ** 2 + data.qpos[5] ** 2);
    maxTilt = Math.max(maxTilt, Math.acos(Math.min(1, Math.max(-1, bodyZDotWorldZ))));

    if (data.time >= 0.4 && !evaluated.done) {
      let speedScale = 1;
      if (positionError > 0.55) speedScale = 0.08;
      else if (positionError > 0.28) speedScale = 0.35;
      trajectoryTime += dt * speedScale;
    }
    if (evaluated.done && completedAt == null) completedAt = data.time;
    if (completedAt != null && data.time - completedAt >= 2) break;
  }

  const finalPosition = [data.qpos[0], data.qpos[2], -data.qpos[1]];
  const finalError = Math.hypot(
    finalPosition[0] - traj.segments.at(-1).ctrl.at(-1)[0],
    finalPosition[1] - traj.segments.at(-1).ctrl.at(-1)[1],
    finalPosition[2] - traj.segments.at(-1).ctrl.at(-1)[2]
  );
  const metrics = {
    tubeBoxes: planned.boxes.length,
    duration: +traj.duration.toFixed(2),
    timeScale: +traj.timeScale.toFixed(3),
    completedAt: completedAt == null ? null : +completedAt.toFixed(2),
    minAltitude: +minAltitude.toFixed(3),
    maxTilt: +maxTilt.toFixed(3),
    maxPositionError: +maxPositionError.toFixed(3),
    finalError: +finalError.toFixed(4),
    maxMotor: +maxMotor.toFixed(3),
  };
  console.log('full LP trajectory MuJoCo metrics', metrics);
  assert(completedAt != null, `trajectory did not complete (trajectory time ${trajectoryTime})`);
  assert(minAltitude > 1.2, `startup/flight altitude dropped to ${minAltitude}`);
  assert(maxTilt < 0.7, `vehicle tilt reached ${maxTilt} rad`);
  assert(maxPositionError < 0.65, `position error reached ${maxPositionError}`);
  assert(finalError < 0.12, `terminal hover error ${finalError}`);
  console.log('ok full LP trajectory MuJoCo flight');
} finally {
  data.delete();
  model.delete();
}
