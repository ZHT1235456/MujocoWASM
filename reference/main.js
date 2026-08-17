import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { CSS2DRenderer, CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';

import { createDrone, createBodyAxes, createSpinIndicators } from './drone/index.js';
import { setWireframe } from './drone/materials.js';
import { SPEC } from './drone/spec.js';

const canvas = document.getElementById('viewport');
const labelHost = document.getElementById('labels');

/* ------------------------------------------------------------------ *
 * 渲染器与场景
 * ------------------------------------------------------------------ */

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 0.95;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;

const labelRenderer = new CSS2DRenderer({ element: labelHost });

const scene = new THREE.Scene();
scene.background = makeBackdrop();
scene.fog = new THREE.Fog(0x11141b, 1.1, 3.0);

// 参考照片是长焦近正交的效果，这里用小视场角 + 远机位来还原
const camera = new THREE.PerspectiveCamera(20, 1, 0.02, 40);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.07;
controls.minDistance = 0.2;
controls.maxDistance = 5;
controls.target.set(0, 0.078, 0);

/** 背景：中心偏亮的径向渐变，让银灰机身在暗底上有轮廓。 */
function makeBackdrop() {
  const size = 512;
  const c = document.createElement('canvas');
  c.width = c.height = size;
  const ctx = c.getContext('2d');
  const grad = ctx.createRadialGradient(size / 2, size * 0.42, 0, size / 2, size * 0.42, size * 0.78);
  grad.addColorStop(0, '#2c333f');
  grad.addColorStop(0.55, '#191d25');
  grad.addColorStop(1, '#0b0d12');
  ctx.fillStyle = grad;
  ctx.fillRect(0, 0, size, size);
  const texture = new THREE.CanvasTexture(c);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

/* ------------------------------------------------------------------ *
 * 光照
 * ------------------------------------------------------------------ */

const pmrem = new THREE.PMREMGenerator(renderer);
scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
scene.environmentIntensity = 0.62;

const keyLight = new THREE.DirectionalLight(0xffffff, 1.55);
keyLight.position.set(0.55, 0.85, 0.6);
keyLight.castShadow = true;
keyLight.shadow.mapSize.set(2048, 2048);
keyLight.shadow.camera.near = 0.1;
keyLight.shadow.camera.far = 3.0;
keyLight.shadow.camera.left = -0.32;
keyLight.shadow.camera.right = 0.32;
keyLight.shadow.camera.top = 0.32;
keyLight.shadow.camera.bottom = -0.32;
keyLight.shadow.bias = -0.0006;
keyLight.shadow.normalBias = 0.004;
scene.add(keyLight);

const rimLight = new THREE.DirectionalLight(0x9fc4ff, 0.62);
rimLight.position.set(-0.7, 0.42, -0.7);
scene.add(rimLight);

const fillLight = new THREE.DirectionalLight(0xffd9b0, 0.22);
fillLight.position.set(-0.35, -0.4, 0.5);
scene.add(fillLight);

scene.add(new THREE.HemisphereLight(0x8fa5c8, 0x1a1d24, 0.28));

/* ------------------------------------------------------------------ *
 * 地面
 * ------------------------------------------------------------------ */

const ground = new THREE.Group();

const floor = new THREE.Mesh(
  new THREE.CircleGeometry(1.6, 96),
  new THREE.MeshStandardMaterial({ color: 0x171b22, roughness: 0.82, metalness: 0.15 })
);
floor.rotation.x = -Math.PI / 2;
floor.receiveShadow = true;
ground.add(floor);

const grid = new THREE.GridHelper(1.6, 32, 0x3d4756, 0x232a35);
grid.material.transparent = true;
grid.material.opacity = 0.5;
grid.position.y = 0.0004;
ground.add(grid);

scene.add(ground);

/* ------------------------------------------------------------------ *
 * 无人机
 * ------------------------------------------------------------------ */

const drone = createDrone();
const REST_HEIGHT = -SPEC.gear.footY; // 起落架触地时机体原点的高度
drone.root.position.y = REST_HEIGHT;
scene.add(drone.root);

const bodyAxes = createBodyAxes();
bodyAxes.visible = false;
drone.root.add(bodyAxes);

const spinIndicators = createSpinIndicators();
spinIndicators.visible = false;
drone.root.add(spinIndicators);

/* ------------------------------------------------------------------ *
 * 部件标注
 * ------------------------------------------------------------------ */

const ANNOTATIONS = [
  { text: '机身外壳', at: [-0.034, 0.034, -0.03] },
  { text: '电池仓', at: [0.0, 0.006, -0.094] },
  { text: '前视觉双目', at: [0.033, -0.011, 0.089] },
  { text: '三轴云台相机', at: [0.0, -0.056, 0.078] },
  { text: '折叠机臂', at: [0.088, -0.0085, -0.073] },
  { text: '无刷电机', at: [0.1314, 0.016, -0.1125] },
  { text: '折叠螺旋桨', at: [0.208, 0.018, 0.056] },
  { text: '起落架', at: [0.126, -0.055, 0.076] },
];

const annotationGroup = new THREE.Group();
annotationGroup.visible = false;
for (const { text, at } of ANNOTATIONS) {
  const el = document.createElement('div');
  el.className = 'annot';
  el.textContent = text;
  const label = new CSS2DObject(el);
  label.position.set(...at);
  annotationGroup.add(label);
}
drone.root.add(annotationGroup);

/* ------------------------------------------------------------------ *
 * 交互状态
 * ------------------------------------------------------------------ */

const state = {
  rpm: 0,
  gimbalPitch: 0,
  gimbalYaw: 0,
  hover: false,
  explode: 0,
  autoOrbit: false,
};

const VIEWS = {
  iso: { pos: [0.82, 0.62, 1.02], target: [0, 0.05, 0] },
  front: { pos: [0, 0.09, 1.5], target: [0, 0.062, 0] },
  side: { pos: [1.45, 0.09, 0.004], target: [0, 0.062, 0] },
  // 正上方会让相机朝向退化（视线与 up 平行），因此略微后移一点机位
  top: { pos: [0, 1.5, 0.18], target: [0, 0.03, 0] },
  back: { pos: [0, 0.09, -1.5], target: [0, 0.062, 0] },
  detail: { pos: [0.3, 0.03, 0.62], target: [0.004, 0.03, 0.06] },
};

let cameraTween = null;

function goToView(nameOrView, instant = false) {
  const name = typeof nameOrView === 'string' ? nameOrView : null;
  const view = name ? VIEWS[name] : nameOrView;
  if (!view) return;
  if (instant) {
    cameraTween = null;
    camera.position.set(...view.pos);
    controls.target.set(...view.target);
  } else {
    cameraTween = {
      fromPos: camera.position.clone(),
      toPos: new THREE.Vector3(...view.pos),
      fromTarget: controls.target.clone(),
      toTarget: new THREE.Vector3(...view.target),
      t: 0,
    };
  }
  for (const button of document.querySelectorAll('.views button')) {
    button.classList.toggle('active', button.dataset.view === name);
  }
}

// 供 tools/shot.mjs 在无头浏览器里直接定位机位
window.__setView = goToView;

goToView('iso');
camera.position.set(...VIEWS.iso.pos);

/* ------------------------------------------------------------------ *
 * 面板绑定
 * ------------------------------------------------------------------ */

const $ = (id) => document.getElementById(id);

function bindSlider(id, readoutId, format, apply) {
  const input = $(id);
  const readout = $(readoutId);
  const update = () => {
    const value = Number(input.value);
    readout.textContent = format(value);
    apply(value);
  };
  input.addEventListener('input', update);
  update();
}

bindSlider('c-rpm', 'v-rpm', (v) => `${Math.round(v * 82)} rpm`, (v) => {
  state.rpm = v / 100;
});
bindSlider('c-pitch', 'v-pitch', (v) => `${v}°`, (v) => {
  state.gimbalPitch = THREE.MathUtils.degToRad(v);
});
bindSlider('c-yaw', 'v-yaw', (v) => `${v}°`, (v) => {
  state.gimbalYaw = THREE.MathUtils.degToRad(v);
});
bindSlider('c-explode', 'v-explode', (v) => `${v}%`, (v) => {
  state.explode = v / 100;
});

$('c-hover').addEventListener('change', (e) => {
  state.hover = e.target.checked;
});
$('c-orbit').addEventListener('change', (e) => {
  state.autoOrbit = e.target.checked;
  controls.autoRotate = e.target.checked;
  controls.autoRotateSpeed = 0.9;
});
$('c-ground').addEventListener('change', (e) => {
  ground.visible = e.target.checked;
});
$('c-axes').addEventListener('change', (e) => {
  bodyAxes.visible = e.target.checked;
});
$('c-spin').addEventListener('change', (e) => {
  spinIndicators.visible = e.target.checked;
});
$('c-annot').addEventListener('change', (e) => {
  annotationGroup.visible = e.target.checked;
});
$('c-wire').addEventListener('change', (e) => {
  setWireframe(e.target.checked);
});

for (const button of document.querySelectorAll('.views button')) {
  button.addEventListener('click', () => goToView(button.dataset.view));
}

$('c-shot').addEventListener('click', () => {
  renderer.render(scene, camera);
  const link = document.createElement('a');
  link.download = `quadrotor-${Date.now()}.png`;
  link.href = renderer.domElement.toDataURL('image/png');
  link.click();
});

/* ------------------------------------------------------------------ *
 * 尺寸自适应
 * ------------------------------------------------------------------ */

function resize() {
  const width = window.innerWidth;
  const height = window.innerHeight;
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height, false);
  labelRenderer.setSize(width, height);
}
window.addEventListener('resize', resize);
resize();

/* ------------------------------------------------------------------ *
 * 动画循环
 * ------------------------------------------------------------------ */

const clock = new THREE.Clock();
let bladeAngle = 0;

function animate() {
  const dt = Math.min(clock.getDelta(), 0.05);
  const time = clock.elapsedTime;

  // 旋翼：低速时看得清桨叶，高速时切换成半透明桨盘
  const omega = state.rpm * 260;
  bladeAngle += omega * dt;
  const blur = THREE.MathUtils.clamp((state.rpm - 0.45) / 0.4, 0, 1);
  for (const rotor of drone.rotors) {
    rotor.propeller.rotation.y = rotor.phase + bladeAngle * rotor.spin;
    const disc = rotor.propeller.userData.disc;
    disc.visible = blur > 0.01;
    disc.material.opacity = blur * 0.5;
    for (const child of rotor.propeller.children) {
      if (child !== disc) child.visible = blur < 0.99;
    }
  }

  // 云台
  drone.gimbalAxes.pitch.rotation.x = state.gimbalPitch;
  drone.gimbalAxes.yaw.rotation.y = state.gimbalYaw;

  // 悬停：机体轻微起伏并伴随小幅姿态修正
  const lift = state.hover ? 0.075 : 0;
  const bob = state.hover ? Math.sin(time * 1.6) * 0.005 : 0;
  drone.root.position.y += (REST_HEIGHT + lift + bob - drone.root.position.y) * Math.min(1, dt * 4);
  const targetRoll = state.hover ? Math.sin(time * 1.05) * 0.028 : 0;
  const targetPitch = state.hover ? Math.sin(time * 0.78 + 1.2) * 0.024 : 0;
  drone.root.rotation.z += (targetRoll - drone.root.rotation.z) * Math.min(1, dt * 3);
  drone.root.rotation.x += (targetPitch - drone.root.rotation.x) * Math.min(1, dt * 3);
  // 云台反向补偿机体姿态，模拟增稳效果
  drone.gimbalAxes.roll.rotation.z = -drone.root.rotation.z;

  // 爆炸视图
  for (const child of drone.root.children) {
    const rest = child.userData.restPosition;
    if (!rest || !child.userData.explode) continue;
    child.position.copy(rest).addScaledVector(child.userData.explode, state.explode * 0.11);
  }

  // 视角过渡
  if (cameraTween) {
    cameraTween.t = Math.min(1, cameraTween.t + dt * 1.8);
    const k = 1 - Math.pow(1 - cameraTween.t, 3);
    camera.position.lerpVectors(cameraTween.fromPos, cameraTween.toPos, k);
    controls.target.lerpVectors(cameraTween.fromTarget, cameraTween.toTarget, k);
    if (cameraTween.t >= 1) cameraTween = null;
  }

  controls.update();
  renderer.render(scene, camera);
  labelRenderer.render(scene, camera);
  requestAnimationFrame(animate);
}

animate();
