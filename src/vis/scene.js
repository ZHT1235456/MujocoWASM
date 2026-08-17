import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { createDrone, createBodyAxes } from '@drone/index.js';
import { SPEC } from '@drone/spec.js';
import { WORLD } from '../world.js';
import { readMjPos, readMjQuat } from '../coords.js';

export function createRenderer(canvas) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 0.92;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  return renderer;
}

function makeBackdrop() {
  const size = 512;
  const c = document.createElement('canvas');
  c.width = c.height = size;
  const ctx = c.getContext('2d');
  const grad = ctx.createRadialGradient(size / 2, size * 0.38, 0, size / 2, size * 0.42, size * 0.82);
  grad.addColorStop(0, '#2a3140');
  grad.addColorStop(0.55, '#161a22');
  grad.addColorStop(1, '#0b0d12');
  ctx.fillStyle = grad;
  ctx.fillRect(0, 0, size, size);
  const texture = new THREE.CanvasTexture(c);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

export function createScene(renderer) {
  const scene = new THREE.Scene();
  scene.background = makeBackdrop();
  scene.fog = new THREE.Fog(0x10131a, 28, 55);

  const pmrem = new THREE.PMREMGenerator(renderer);
  scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
  scene.environmentIntensity = 0.55;

  const key = new THREE.DirectionalLight(0xffffff, 1.7);
  key.position.set(6, 12, 5);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  key.shadow.camera.near = 0.5;
  key.shadow.camera.far = 40;
  key.shadow.camera.left = -16;
  key.shadow.camera.right = 16;
  key.shadow.camera.top = 16;
  key.shadow.camera.bottom = -16;
  key.shadow.bias = -0.0004;
  scene.add(key);
  const fill = new THREE.DirectionalLight(0x9fc4ff, 0.45);
  fill.position.set(-8, 6, -6);
  scene.add(fill);
  scene.add(new THREE.HemisphereLight(0x8fa5c8, 0x1a1d24, 0.4));
  return scene;
}

export function createCamera(canvas) {
  const camera = new THREE.PerspectiveCamera(42, 1, 0.05, 80);
  camera.position.set(WORLD.start[0] - 1.7, WORLD.start[1] + 1.1, WORLD.start[2] + 1.9);
  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.minDistance = 0.55;
  controls.maxDistance = 28;
  controls.maxPolarAngle = Math.PI * 0.48;
  controls.target.set(...WORLD.start);
  controls.update();
  return { camera, controls };
}

export function createWorldMeshes(scene) {
  const group = new THREE.Group();
  group.name = 'world';

  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(40, 40),
    new THREE.MeshStandardMaterial({ color: 0x171b22, roughness: 0.9, metalness: 0.08 })
  );
  floor.rotation.x = -Math.PI / 2;
  floor.receiveShadow = true;
  group.add(floor);

  const grid = new THREE.GridHelper(20, 20, 0x3d4756, 0x232a35);
  grid.material.transparent = true;
  grid.material.opacity = 0.45;
  grid.position.y = 0.002;
  group.add(grid);

  const buildings = new THREE.Group();
  buildings.name = 'obstacles';
  const mat = new THREE.MeshStandardMaterial({
    color: 0x8b96a8,
    roughness: 0.55,
    metalness: 0.18,
    emissive: 0x1a222c,
    emissiveIntensity: 0.25,
  });
  const wireMat = new THREE.MeshBasicMaterial({
    color: 0x9aa6b8,
    wireframe: true,
    transparent: true,
    opacity: 0.35,
  });

  for (const obs of WORLD.obstacles) {
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(obs.size[0], obs.size[1], obs.size[2]), mat.clone());
    mesh.position.set(...obs.pos);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    buildings.add(mesh);
    const wire = new THREE.Mesh(mesh.geometry, wireMat);
    wire.position.copy(mesh.position);
    wire.visible = false;
    wire.userData.wire = true;
    buildings.add(wire);
  }
  group.add(buildings);

  const startMark = makeMarker(0x3dce7a);
  startMark.position.set(...WORLD.start);
  const goalMark = makeMarker(0xf7941e);
  goalMark.position.set(...WORLD.goal);
  group.add(startMark, goalMark);

  scene.add(group);
  return { group, buildings, startMark, goalMark };
}

function makeMarker(color) {
  const g = new THREE.Group();
  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(0.11, 0.012, 8, 28),
    new THREE.MeshBasicMaterial({ color })
  );
  ring.rotation.x = Math.PI / 2;
  ring.position.y = -0.13;
  g.add(ring);
  const pole = new THREE.Mesh(
    new THREE.CylinderGeometry(0.008, 0.008, 0.1, 8),
    new THREE.MeshBasicMaterial({ color })
  );
  pole.position.y = -0.07;
  g.add(pole);
  return g;
}

export function createDroneVisual(scene) {
  const drone = createDrone();
  const body = new THREE.Group();
  body.name = 'droneBody';
  body.add(drone.root);
  const axes = createBodyAxes(0.18);
  axes.visible = false;
  drone.root.add(axes);
  scene.add(body);
  return { drone, body, axes, restHeight: -SPEC.gear.footY };
}

export function syncDrone(body, model, data) {
  try {
    const droneBody = data.body('drone');
    const p = droneBody.xpos;
    const q = droneBody.xquat;
    body.position.set(p[0], p[2], -p[1]);
    body.quaternion.set(-q[1], q[3], -q[2], q[0]);
    return;
  } catch {
    readMjPos(data.xpos, 1, body.position);
    readMjQuat(data.xquat, 1, body.quaternion);
  }
}

export function spinProps(drone, thrusts, dt) {
  const maxT = 8;
  for (let i = 0; i < drone.rotors.length; i++) {
    const rotor = drone.rotors[i];
    const t = thrusts[i] ?? 0;
    const rpm = (t / maxT) * 280;
    rotor.propeller.rotation.y += rpm * dt * rotor.spin;
    const blur = Math.min(1, Math.max(0, (t / maxT - 0.12) / 0.45));
    const disc = rotor.propeller.userData.disc;
    if (disc) {
      disc.visible = blur > 0.02;
      disc.material.opacity = blur * 0.28;
      for (const child of rotor.propeller.children) {
        // Keep the detailed reference blades visible; the disc is only a
        // subtle speed cue and should not replace the propeller model.
        if (child !== disc) child.visible = true;
      }
    }
  }
}
