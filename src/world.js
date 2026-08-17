import { SPEC, ROTORS, rotorAnchors } from '@drone/spec.js';
import { threePosToMj } from './coords.js';

/** 机体碰撞球半径：桨尖包络 + 余量 */
export const DRONE_RADIUS = 0.28;

export const MASS = 0.85;
export const GRAVITY = 9.81;
export const HOVER_THRUST = (MASS * GRAVITY) / 4;
export const YAW_GEAR = 0.018;

export const WORLD = {
  bounds: {
    min: [-10, 0.35, -10],
    max: [10, 6.0, 10],
  },
  start: [-8.0, 1.4, -8.0],
  goal: [8.0, 1.8, 8.0],
  obstacles: [
    { name: 'tower_c', pos: [0.0, 1.7, 0.2], size: [2.2, 3.4, 2.0] },
    { name: 'block_a', pos: [-3.6, 1.4, -2.2], size: [2.4, 2.8, 1.8] },
    { name: 'block_b', pos: [3.8, 1.8, -3.0], size: [2.0, 3.6, 2.2] },
    { name: 'wall_n', pos: [-1.2, 1.1, 4.4], size: [5.5, 2.2, 1.1] },
    { name: 'pillar_1', pos: [5.4, 1.6, 2.2], size: [1.3, 3.2, 1.3] },
    { name: 'pillar_2', pos: [-6.2, 1.3, 1.6], size: [1.4, 2.6, 1.6] },
    { name: 'low_bar', pos: [2.2, 0.7, 6.2], size: [4.0, 1.4, 1.2] },
    { name: 'gate_l', pos: [-5.0, 1.5, -5.4], size: [1.0, 3.0, 2.6] },
    { name: 'gate_r', pos: [-2.4, 1.5, -5.4], size: [1.0, 3.0, 2.6] },
  ],
};

export function motorSitesMj() {
  return ROTORS.map((rotor) => {
    const { motor } = rotorAnchors(rotor);
    const [x, y, z] = threePosToMj(motor[0], motor[1], motor[2]);
    return { ...rotor, pos: [x, y, z] };
  });
}

function boxToMj(obs) {
  const [px, py, pz] = threePosToMj(obs.pos[0], obs.pos[1], obs.pos[2]);
  const [sx, sy, sz] = obs.size;
  return {
    name: obs.name,
    pos: [px, py, pz],
    size: [sx / 2, sz / 2, sy / 2],
  };
}

export function buildMjcf(start = WORLD.start) {
  const [sx, sy, sz] = threePosToMj(start[0], start[1], start[2]);
  const sites = motorSitesMj();
  const boxes = WORLD.obstacles.map(boxToMj);

  const siteXml = sites
    .map(
      (s) =>
        `      <site name="thrust${s.id}" pos="${s.pos[0].toFixed(4)} ${s.pos[1].toFixed(4)} ${s.pos[2].toFixed(4)}" size="0.01"/>`
    )
    .join('\n');

  const actuatorXml = sites
    .map((s) => {
      const yaw = (-s.spin * YAW_GEAR).toFixed(4);
      return `    <motor name="m${s.id}" site="thrust${s.id}" gear="0 0 1 0 0 ${yaw}" ctrllimited="true" ctrlrange="0 8"/>`;
    })
    .join('\n');

  const boxXml = boxes
    .map(
      (b) =>
        `    <geom name="${b.name}" type="box" pos="${b.pos.map((v) => v.toFixed(3)).join(' ')}" size="${b.size
          .map((v) => v.toFixed(3))
          .join(' ')}" rgba="0.35 0.4 0.48 1" contype="1" conaffinity="1"/>`
    )
    .join('\n');

  return `
<mujoco model="quadrotor_corridor">
  <compiler angle="radian" inertiafromgeom="true"/>
  <option timestep="0.002" gravity="0 0 -${GRAVITY}" integrator="RK4"/>
  <default>
    <geom condim="3" friction="0.6 0.1 0.1"/>
  </default>
  <worldbody>
    <light directional="true" pos="8 4 12" dir="-0.3 -0.2 -1" diffuse="0.8 0.8 0.8"/>
    <geom name="floor" type="plane" size="20 20 0.1" rgba="0.12 0.14 0.18 1" contype="1" conaffinity="1"/>
${boxXml}
    <body name="drone" pos="${sx.toFixed(4)} ${sy.toFixed(4)} ${sz.toFixed(4)}">
      <freejoint name="root"/>
      <inertial pos="0 0 0" mass="${MASS}" diaginertia="0.014 0.014 0.022"/>
      <geom name="drone_col" type="sphere" size="${DRONE_RADIUS}" rgba="1 1 1 0" contype="1" conaffinity="1" mass="0"/>
${siteXml}
    </body>
  </worldbody>
  <actuator>
${actuatorXml}
  </actuator>
</mujoco>`.trim();
}

export { SPEC };
