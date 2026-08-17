import fs from 'node:fs';
import loadMujoco from '@mujoco/mujoco';
import { computeGeometricControl, resetGeometric } from '../src/control/geometric.js';

function assert(cond, msg) {
  if (!cond) throw new Error(msg);
}

const mujoco = await loadMujoco();
const xml = fs.readFileSync(new URL('../src/sim/quadrotor.xml', import.meta.url), 'utf8');
const model = mujoco.MjModel.from_xml_string(xml);
const data = new mujoco.MjData(model);

try {
  mujoco.mj_forward(model, data);
  resetGeometric();

  const ref = {
    p: [-8, 1.4, -8],
    v: [0, 0, 0],
    a: [0, 0, 0],
    yawDir: [0, 0, -1],
  };

  for (let step = 0; step < 500; step++) {
    const motors = computeGeometricControl(data, ref, data.time, 0.002);
    for (let i = 0; i < 4; i++) data.ctrl[i] = motors[i];
    mujoco.mj_step(model, data);
  }

  const positionError = Math.hypot(data.qpos[0] + 8, data.qpos[1] - 8, data.qpos[2] - 1.4);
  const attitudeVector = Math.hypot(data.qpos[4], data.qpos[5], data.qpos[6]);
  const speed = Math.hypot(...Array.from(data.qvel).slice(0, 6));

  assert(positionError < 1e-3, `one-second hover position error ${positionError}`);
  assert(attitudeVector < 1e-3, `one-second hover attitude error ${attitudeVector}`);
  assert(speed < 1e-3, `one-second hover speed ${speed}`);
  console.log('ok MuJoCo one-second hover', {
    positionError,
    attitudeVector,
    speed,
    motors: Array.from(data.ctrl).map((v) => +v.toFixed(3)),
  });
} finally {
  data.delete();
  model.delete();
}
