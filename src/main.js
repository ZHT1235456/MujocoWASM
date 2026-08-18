import * as THREE from 'three';
import { WORLD } from './world.js';
import { initMujoco, createSim, stepSim } from './sim/mujoco.js';
import { planSafeTube } from './plan/rrt-tube.js';
import { evalTrajectory, sampleTrajectory, pointInTube, tubeBoxIndex } from './plan/bezier-tube.js';
import { bezierLpInTube } from './plan/bezier-lp.js';
import { hoverThrusts } from './control/pid.js';
import { computeGeometricControl, resetGeometric } from './control/geometric.js';
import {
  createRenderer,
  createScene,
  createCamera,
  setStartCameraView,
  dumpCameraView,
  createWorldMeshes,
  createDroneVisual,
  syncDrone,
  spinProps,
} from './vis/scene.js';
import { createCorridorView, drawCorridor, drawTree, setCorridorViolated } from './vis/corridor.js';
import { bindPanel, setStatus, setMetrics, clearPlanMetrics, setBusy, setConfigLocked, waitFrame } from './ui/panel.js';

const canvas = document.getElementById('viewport');
const renderer = createRenderer(canvas);
const scene = createScene(renderer);
const { camera, controls } = createCamera(canvas);
const worldMeshes = createWorldMeshes(scene);
const { drone, body, axes } = createDroneVisual(scene);
const corridorGroup = createCorridorView(scene);

const clock = new THREE.Clock();
const followTailOffset = new THREE.Vector3(0, 0.72, -1.9);
const followLookOffset = new THREE.Vector3(0, 0.08, 0.72);
const followPosition = new THREE.Vector3();
const followTarget = new THREE.Vector3();
const followDirection = new THREE.Vector3();
const followRay = new THREE.Ray();
const followObstacleBox = new THREE.Box3();
const followObstacleCenter = new THREE.Vector3();
const followObstacleSize = new THREE.Vector3();
const followHit = new THREE.Vector3();
let cameraViewBeforeFollow = null;
const corridorConfirmFrames = 3;
const app = {
  follow: false,
  paused: true,
  flying: false,
  holding: false,
  planning: false,
  sim: null,
  planResult: null,
  show: { corridor: true, centerline: true, tree: false, wire: false, axes: false },
  t: 0,
  speed: 1.6,
  violations: 0,
  corridorOutsideFrames: 0,
  corridorInsideFrames: 0,
  violatedBoxIndex: -1,
  thrusts: hoverThrusts(),
};

const stage = document.getElementById('stage');

function resize() {
  const width = Math.max(1, stage.clientWidth);
  const height = Math.max(1, stage.clientHeight);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height, false);
}
window.addEventListener('resize', resize);
new ResizeObserver(resize).observe(stage);
resize();

function dumpCamera() {
  controls.update();
  const pose = dumpCameraView(camera, controls);
  const text = JSON.stringify(pose, null, 2);
  console.log('[camera view]\n' + text);
  navigator.clipboard?.writeText(text).catch(() => {});
  setStatus('当前视角已复制到剪贴板，请粘贴发给我');
  return pose;
}

window.dumpCamera = dumpCamera;
window.addEventListener('keydown', (event) => {
  if (event.key !== 'v' && event.key !== 'V') return;
  if (event.ctrlKey || event.metaKey || event.altKey) return;
  const tag = event.target?.tagName;
  if (tag === 'INPUT' || tag === 'TEXTAREA' || event.target?.isContentEditable) return;
  event.preventDefault();
  dumpCamera();
});

function currentStartGoal(start, goal) {
  worldMeshes.startMark.position.set(...start);
  worldMeshes.goalMark.position.set(...goal);
}

function redrawPath() {
  const r = app.planResult;
  if (!r) {
    drawCorridor(corridorGroup, [], [], {});
    return;
  }
  drawCorridor(corridorGroup, r.samples, r.radii, {
    corridor: app.show.corridor,
    centerline: app.show.centerline,
    violated: r.violated,
    boxes: r.boxes,
  });
  drawTree(corridorGroup, r.nodes, app.show.tree);
}

function resetCorridorViolation() {
  app.corridorOutsideFrames = 0;
  app.corridorInsideFrames = 0;
  app.violatedBoxIndex = -1;
  if (app.planResult) app.planResult.violated = false;
  setCorridorViolated(corridorGroup, false);
}

function invalidatePlan(message) {
  app.planResult = null;
  app.paused = true;
  app.flying = false;
  app.holding = false;
  app.t = 0;
  resetCorridorViolation();
  redrawPath();
  clearPlanMetrics();
  setConfigLocked(false);
  document.getElementById('c-pause').textContent = '暂停';
  setStatus(message);
}

function updateFollowCamera(dt, immediate = false) {
  followPosition.copy(followTailOffset).applyQuaternion(body.quaternion).add(body.position);
  followTarget.copy(followLookOffset).applyQuaternion(body.quaternion).add(body.position);

  followDirection.subVectors(followPosition, body.position);
  const desiredDistance = followDirection.length();
  if (desiredDistance > 1e-6) {
    followDirection.multiplyScalar(1 / desiredDistance);
    followRay.set(body.position, followDirection);
    let allowedDistance = desiredDistance;
    for (const obstacle of WORLD.obstacles) {
      followObstacleCenter.set(...obstacle.pos);
      followObstacleSize.set(...obstacle.size);
      followObstacleBox.setFromCenterAndSize(followObstacleCenter, followObstacleSize).expandByScalar(0.12);
      const hit = followRay.intersectBox(followObstacleBox, followHit);
      if (!hit) continue;
      const distance = hit.distanceTo(body.position);
      if (distance < allowedDistance) allowedDistance = Math.max(0.5, distance - 0.14);
    }
    if (allowedDistance < desiredDistance) {
      followPosition.copy(body.position).addScaledVector(followDirection, allowedDistance);
    }
  }

  if (immediate) {
    camera.position.copy(followPosition);
    controls.target.copy(followTarget);
  } else {
    camera.position.lerp(followPosition, 1 - Math.pow(0.025, dt));
    controls.target.lerp(followTarget, 1 - Math.pow(0.05, dt));
  }
}

app.setShow = (partial) => {
  Object.assign(app.show, partial);
  axes.visible = !!app.show.axes;
  worldMeshes.buildings.traverse((obj) => {
    if (obj.userData.wire) obj.visible = !!app.show.wire;
  });
  redrawPath();
};

app.planningConfigChanged = () => {
  if (app.planResult?.traj) invalidatePlan('规划参数已修改，请重新规划路径');
};

app.setFollow = (enabled) => {
  if (enabled === app.follow) return;
  if (enabled) {
    cameraViewBeforeFollow = {
      position: camera.position.clone(),
      target: controls.target.clone(),
    };
    app.follow = true;
    updateFollowCamera(0, true);
    controls.update();
    return;
  }
  app.follow = false;
  if (cameraViewBeforeFollow) {
    camera.position.copy(cameraViewBeforeFollow.position);
    controls.target.copy(cameraViewBeforeFollow.target);
    controls.update();
    cameraViewBeforeFollow = null;
  }
};

app.plan = async (start, goal, opts) => {
  if (app.planning) return;
  app.planning = true;
  app.paused = true;
  app.flying = false;
  app.holding = false;
  setBusy(true, '正在规划安全超矩形管…');
  setStatus('正在运行 Algorithm 1（安全管 RRT），请稍候…');
  currentStartGoal(start, goal);
  await waitFrame();
  try {
    const planned = await planSafeTube(start, goal, {
      nv: opts.iters,
      margin0: opts.rMax,
      alphaV: opts.speed ?? app.speed,
      yieldFn: waitFrame,
    });
    if (!planned.ok || planned.boxes.length < 2) {
      setStatus(planned.message || '规划失败：请增加采样次数或减小跟踪裕度');
      app.planResult = { nodes: planned.nodes, samples: [], radii: [], boxes: [], violated: false };
      redrawPath();
      return;
    }
    setStatus('安全管已生成，正在求解 Algorithm 2 轨迹 LP…');
    await waitFrame();
    const traj = await bezierLpInTube(planned.boxes, { np: 9 });
    if (!traj.ok) {
      setStatus(traj.message || 'Algorithm 2 轨迹 LP 求解失败');
      app.planResult = { ...planned, samples: [], radii: [], violated: false };
      redrawPath();
      return;
    }
    planned.boxes = planned.boxes.map((box, i) => ({ ...box, t: traj.times[i] }));
    planned.times = traj.times;
    planned.duration = traj.duration;
    const resampled = sampleTrajectory(traj, 0.08);
    const radii = planned.boxes.map((b) => Math.min(...b.r));
    app.planResult = {
      ...planned,
      traj,
      samples: resampled.samples,
      tangents: resampled.tangents,
      sTable: resampled.s,
      length: resampled.length,
      radii,
      minClearance: planned.minClearance,
      violated: false,
    };
    app.t = 0;
    app.violations = 0;
    resetCorridorViolation();
    setConfigLocked(true);
    redrawPath();
    setMetrics({
      length: resampled.length,
      clearance: planned.minClearance,
      time: 0,
      inside: '已规划',
    });
    setStatus(`规划完成：${planned.boxes.length} 个安全盒，时长 ${planned.duration.toFixed(1)} s`);
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
  invalidatePlan('起点已修改，请重新规划路径');
  app.reset(start);
  if (!app.follow) {
    const goal = [
      Number(document.getElementById('c-gx').value),
      Number(document.getElementById('c-gy').value),
      Number(document.getElementById('c-gz').value),
    ];
    setStartCameraView(camera, controls);
  }
  setStatus('起点已修改，请重新规划路径');
};

app.moveGoal = (goal) => {
  if (!goal.every(Number.isFinite)) return;
  worldMeshes.goalMark.position.set(...goal);
  invalidatePlan('终点已修改，请重新规划路径');
  if (!app.follow) {
    const start = [
      Number(document.getElementById('c-sx').value),
      Number(document.getElementById('c-sy').value),
      Number(document.getElementById('c-sz').value),
    ];
    setStartCameraView(camera, controls);
  }
};

app.fly = (speed) => {
  if (app.planning) return;
  if (!app.planResult?.traj) {
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
  setConfigLocked(true);
  app.paused = false;
  app.flying = true;
  app.holding = false;
  app.t = 0;
  app.violations = 0;
  resetGeometric();
  resetCorridorViolation();
  clock.getDelta();
  document.getElementById('c-pause').textContent = '暂停';
  setStatus('几何控制跟踪安全管');
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
  app.t = 0;
  app.violations = 0;
  app.thrusts = hoverThrusts();
  setConfigLocked(false);
  resetGeometric();
  resetCorridorViolation();
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

function trajectoryRef() {
  const r = app.planResult;
  if (!r?.traj) return null;
  const e = evalTrajectory(r.traj, app.t);
  if (!e) return null;
  if (e.done || app.holding) {
    const goal = r.samples[r.samples.length - 1] || r.boxes[r.boxes.length - 1].p;
    return { p: goal, v: [0, 0, 0], a: [0, 0, 0], yawDir: e.yawDir, done: true };
  }
  return e;
}

function advanceTrajectory(simDt, ref) {
  if (!app.flying || app.holding || !ref || ref.done) return;
  if (app.sim.data.time < 0.4) return;
  const r = app.planResult;
  const pNow = [body.position.x, body.position.y, body.position.z];
  const err = Math.hypot(pNow[0] - ref.p[0], pNow[1] - ref.p[1], pNow[2] - ref.p[2]);
  let scale = app.speed / Math.max(0.2, r.alphaV || 1.6);
  if (err > 0.55) scale *= 0.08;
  else if (err > 0.28) scale *= 0.35;
  app.t += Math.min(0.04, simDt) * scale;
}

function checkCorridor() {
  const r = app.planResult;
  if (!r?.boxes?.length) return true;
  const p = [body.position.x, body.position.y, body.position.z];
  const inside = pointInTube(p, r.boxes, app.t);
  const activeBoxIndex = tubeBoxIndex(r.boxes, app.t);
  if (inside) {
    app.corridorOutsideFrames = 0;
    app.corridorInsideFrames += 1;
    if (r.violated && app.corridorInsideFrames >= corridorConfirmFrames) {
      r.violated = false;
      app.violatedBoxIndex = -1;
      setCorridorViolated(corridorGroup, false);
    }
  } else {
    app.corridorInsideFrames = 0;
    app.corridorOutsideFrames += 1;
    if (app.corridorOutsideFrames >= corridorConfirmFrames && (!r.violated || app.violatedBoxIndex !== activeBoxIndex)) {
      if (!r.violated) app.violations += 1;
      r.violated = true;
      app.violatedBoxIndex = activeBoxIndex;
      setCorridorViolated(corridorGroup, true, activeBoxIndex);
    }
  }
  setMetrics({
    time: app.sim.data.time,
    inside: r.violated ? `越界 ×${app.violations}` : inside ? '管内' : '边界确认中',
  });
  return inside;
}

function animate() {
  try {
    const dt = Math.min(clock.getDelta(), 1 / 30);
    let simAdvanced = false;
    if (app.sim && !app.paused) {
      let thrusts = hoverThrusts();
      const t0 = app.sim.data.time;
      if (app.flying || app.holding) {
        const ref = trajectoryRef();
        if (ref) {
          stepSim(app.sim, (h) => computeGeometricControl(app.sim.data, ref, app.sim.data.time, h), dt);
          const ctrl = app.sim.data.ctrl;
          thrusts = [ctrl[0], ctrl[1], ctrl[2], ctrl[3]];
          if (ref.done && app.flying) {
            app.flying = false;
            app.holding = true;
            setStatus('到达终点，定点悬停');
          } else {
            advanceTrajectory(app.sim.data.time - t0, ref);
          }
        } else {
          stepSim(app.sim, hoverThrusts(), dt);
        }
      } else {
        stepSim(app.sim, thrusts, dt);
      }
      app.thrusts = thrusts;
      simAdvanced = true;
    }
    if (app.sim) syncDrone(body, app.sim.model, app.sim.data);
    if (simAdvanced) checkCorridor();
    spinProps(drone, app.thrusts, dt);

    if (app.follow) {
      updateFollowCamera(dt);
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
    setStartCameraView(camera, controls);
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
