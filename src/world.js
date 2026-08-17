import { SPEC } from '@drone/spec.js';
import { threePosToMj } from './coords.js';
import { DRONE_RADIUS, MASS, GRAVITY, HOVER_THRUST, YAW_GEAR, MOTOR_SITES_MJ, WORLD } from './world-scene.js';

export { DRONE_RADIUS, MASS, GRAVITY, HOVER_THRUST, YAW_GEAR, WORLD };

export function motorSitesMj() {
  return MOTOR_SITES_MJ.map((site) => ({ ...site, pos: site.pos.slice() }));
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
  <compiler angle="radian" inertiafromgeom="false"/>
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
