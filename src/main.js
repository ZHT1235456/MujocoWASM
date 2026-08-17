import * as THREE from 'three';
import { WORLD, DRONE_RADIUS } from './world.js';
import { initMujoco, createSim, stepSim } from './sim/mujoco.js';
import { planRrtStar, shortcutPath } from './plan/rrtstar.js';
import { smoothBezier, resampleArc } from './plan/bezier.js';
import { inflateCorridor, pointOnPath, corridorRadiusAt } from './plan/corridor.js';
import { dist } from './plan/collide.js';
import { computeControl, hoverThrusts } from './control/pid.js';
import {
  createRenderer,
  createScene,
  createCamera,
  createWorldMeshes,
  createDroneVisual,
  syncDrone,
  spinProps,
} from './vis/scene.js';
import { createCorridorView, drawCorridor, drawTree, setCorridorViolated } from './vis/corridor.js';
import { bindPanel, setStatus, setMetrics, setBusy, waitFrame } from './ui/panel.js';

const canvas = document.getElementById('viewport');
const renderer = createRenderer(canvas);
const scene = createScene(renderer);
const { camera, controls } = createCamera(canvas);
const worldMeshes = createWorldMeshes(scene);
const { drone, body, axes } = createDroneVisual(scene);
const corridorGroup = createCorridorView(scene);

const clock = new THREE.Clock();
const app = {
  follow: false,
  paused: true,
  flying: false,
  holding: false,
  planning: false,
  sim: null,
  planResult: null,
  show: { corridor: true, centerline: true, tree: false, wire: false, axes: false },
  s: 0,
  speed: 1.6,
  violations: 0,
  thrusts: hoverThrusts(),
};

function resize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight, false);
}
window.addEventListener('resize', resize);
resize();

function currentStartGoal(start, goal) {
  worldMeshes.startMark.position.set(...start);
  worldMeshes.goalMark.position.set(...goal);
}

function redrawPath() {
  const r = app.planResult;
  if (!r) return;
  drawCorridor(corridorGroup, r.samples, r.radii, {
    corridor: app.show.corridor,
    centerline: app.show.centerline,
    violated: r.violated,
  });
  drawTree(corridorGroup, r.nodes, app.show.tree);
}

app.setShow = (partial) => {
  Object.assign(app.show, partial);
  axes.visible = !!app.show.axes;
  worldMeshes.buildings.traverse((obj) => {
    if (obj.userData.wire) obj.visible = !!app.show.wire;
  });
  redrawPath();
};

app.plan = async (start, goal, opts) => {
  if (app.planning) return;
  app.planning = true;
  app.paused = true;
  app.flying = false;
  app.holding = false;
  setBusy(true, '正在规划路径…');
  setStatus('正在运行 RRT*，请稍候…');
  currentStartGoal(start, goal);
  await waitFrame();
  try {
    const planned = planRrtStar(start, goal, {
      iters: opts.iters,
      bounds: WORLD.bounds,
    });
    if (!planned.ok || planned.path.length < 2) {
      setStatus('规划失败：请增加迭代次数或调整起终点');
      app.planResult = { nodes: planned.nodes, samples: [], radii: [], violated: false };
      redrawPath();
      return;
    }
    const short = shortcutPath(planned.path);
    const bezier = smoothBezier(short);
    const resampled = resampleArc(bezier.samples, bezier.tangents, 0.1);
    const corridor = inflateCorridor(resampled.samples, { rMax: opts.rMax, rMin: DRONE_RADIUS + 0.04 });
    app.planResult = {
      ...planned,
      path: short,
      samples: resampled.samples,
      tangents: resampled.tangents,
      sTable: resampled.s,
      length: resampled.length,
      radii: corridor.radii,
      minClearance: corridor.minClearance,
      violated: false,
    };
    app.s = 0;
    app.violations = 0;
    redrawPath();
    setMetrics({
      length: resampled.length,
      clearance: corridor.minClearance,
      time: 0,
      inside: '已规划',
    });
    setStatus(`规划完成：${short.length} 个路标，长度 ${resampled.length.toFixed(1)} m`);
  } finally {
    app.planning = false;
    setBusy(false);
  }
};

app.moveStart = (start) => {
  if (!app.sim || app.planning) return;
  if (!start.every(Number.isFinite)) return;
  currentStartGoal(start, [
    Number(document.getElementById('c-gx').value),
    Number(document.getElementById('c-gy').value),
    Number(document.getElementById('c-gz').value),
  ]);
  app.reset(start);
};

app.moveGoal = (goal) => {
  if (!goal.every(Number.isFinite)) return;
  worldMeshes.goalMark.position.set(...goal);
};

app.fly = (speed) => {
  if (app.planning) return;
  if (!app.planResult?.samples?.length) {
    setStatus('请先规划路径');
    return;
  }
  const start = [
    Number(document.getElementById('c-sx').value),
    Number(document.getElementById('c-sy').value),
    Number(document.getElementById('c-sz').value),
  ];
  try {
    app.sim.model.delete?.();
    app.sim.data.delete?.();
  } catch {
    /* ignore */
  }
  app.sim = createSim(start);
  app.speed = speed;
  app.paused = false;
  app.flying = true;
  app.holding = false;
  app.s = 0;
  app.violations = 0;
  if (app.planResult) app.planResult.violated = false;
  document.getElementById('c-pause').textContent = '暂停';
  setStatus('沿安全走廊跟踪飞行');
  redrawPath();
};

app.togglePause = () => {
  if (app.planning) return;
  app.paused = !app.paused;
  document.getElementById('c-pause').textContent = app.paused ? '继续' : '暂停';
  setStatus(app.paused ? '已暂停' : '仿真运行中');
};

app.reset = (start) => {
  if (!app.sim || app.planning) return;
  try {
    app.sim.model.delete?.();
    app.sim.data.delete?.();
  } catch {
    /* ignore */
  }
  const next = createSim(start);
  app.sim = next;
  app.paused = true;
  app.flying = false;
  app.holding = false;
  app.s = 0;
  app.violations = 0;
  app.thrusts = hoverThrusts();
  if (app.planResult) app.planResult.violated = false;
  document.getElementById('c-pause').textContent = '暂停';
  currentStartGoal(start, [
    Number(document.getElementById('c-gx').value),
    Number(document.getElementById('c-gy').value),
    Number(document.getElementById('c-gz').value),
  ]);
  redrawPath();
  setMetrics({ time: 0, inside: '已重置' });
  setStatus('已重置到起点');
};

function trajectoryRef(dt) {
  const r = app.planResult;
  if (!r?.samples?.length) return null;
  const vmax = app.speed;
  const acc = 1.2;
  const length = r.length;
  const pNow = [body.position.x, body.position.y, body.position.z];
  const here = pointOnPath(app.s, r.sTable, r.samples, r.tangents);
  const trackingErr = dist(pNow, here.p);
  const tAcc = vmax / acc;
  const dAcc = 0.5 * acc * tAcc * tAcc;
  let v = vmax;
  if (app.s < dAcc) v = Math.sqrt(Math.max(0.05, 2 * acc * app.s));
  if (length - app.s < dAcc) v = Math.sqrt(Math.max(0.05, 2 * acc * Math.max(0, length - app.s)));
  if (trackingErr > 0.55) v *= 0.15;
  else if (trackingErr > 0.28) v *= 0.45;
  app.s = Math.min(length, app.s + v * dt);
  const { p, t } = pointOnPath(app.s, r.sTable, r.samples, r.tangents);
  const n = Math.hypot(t[0], t[1], t[2]) || 1;
  const yawDir = [t[0] / n, t[1] / n, t[2] / n];
  const done = app.s >= length - 1e-3;
  return {
    p: done ? r.samples[r.samples.length - 1] : p,
    v: done ? [0, 0, 0] : [yawDir[0] * v, yawDir[1] * v, yawDir[2] * v],
    yawDir,
    done,
  };
}

function checkCorridor() {
  const r = app.planResult;
  if (!r?.samples?.length) return true;
  const p = [body.position.x, body.position.y, body.position.z];
  const { p: q } = pointOnPath(app.s, r.sTable, r.samples, r.tangents);
  const rad = corridorRadiusAt(app.s, r.sTable, r.radii);
  const d = dist(p, q);
  const inside = d <= rad + 0.02;
  if (!inside && !r.violated) {
    r.violated = true;
    app.violations += 1;
    setCorridorViolated(corridorGroup, true);
  }
  setMetrics({
    time: app.sim.data.time,
    inside: inside ? '走廊内' : `越界 ×${app.violations}`,
  });
  return inside;
}

function animate() {
  try {
    const dt = Math.min(clock.getDelta(), 0.05);
    if (app.sim && !app.paused) {
      let thrusts = hoverThrusts();
      if (app.flying || app.holding) {
        const ref = trajectoryRef(dt);
        if (ref) {
          thrusts = computeControl(app.sim.data, ref);
          if (ref.done && app.flying) {
            app.flying = false;
            app.holding = true;
            setStatus('到达终点，定点悬停');
          }
        }
      }
      app.thrusts = thrusts;
      stepSim(app.sim, thrusts, dt);
      checkCorridor();
    }
    if (app.sim) syncDrone(body, app.sim.model, app.sim.data);
    spinProps(drone, app.thrusts, dt);

    if (app.follow) {
      const offset = new THREE.Vector3(-3.2, 2.2, 3.6);
      const desired = body.position.clone().add(offset);
      if (Number.isFinite(desired.x)) {
        camera.position.lerp(desired, 1 - Math.pow(0.04, dt));
        controls.target.lerp(body.position, 1 - Math.pow(0.08, dt));
      }
    }
    controls.update();
    renderer.render(scene, camera);
  } catch (err) {
    console.error(err);
    setStatus(`运行错误：${err.message || err}`);
    return;
  }
  requestAnimationFrame(animate);
}

async function main() {
  try {
    setStatus('正在加载 MuJoCo WASM…');
    await initMujoco();
    const start = WORLD.start.slice();
    app.sim = createSim(start);
    syncDrone(body, app.sim.model, app.sim.data);
    bindPanel(app);
    window.__app = app;
    window.__body = body;
    setStatus('就绪：点击「规划路径」');
    animate();
  } catch (err) {
    console.error(err);
    setStatus(`初始化失败：${err.message || err}`);
  }
}

main();
