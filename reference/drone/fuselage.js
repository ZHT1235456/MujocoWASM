import * as THREE from 'three';
import { SPEC } from './spec.js';
import { materials } from './materials.js';
import { compileProfile, loft, roundedPlate, steppedShellRing } from './geometry.js';

/**
 * 机身外壳。
 *
 * 主体是一段沿 Z 轴放样的水滴形壳体：尾部截面高而饱满（电池仓），
 * 向机头方向逐渐收窄下沉；机头分叉成左右两个鼓包容纳前视双目，
 * 两鼓包之间留出下凹的中脊。
 */

// 主壳体沿轴向的关键站位。t=0 为机尾，t=1 为主壳体前端；
// yTop / yBot 是该站位处外壳的上下缘高度，exp 控制截面的"方圆度"。
// 主壳体在机头前留出一段收窄的中脊，真正的机头尖端由左右两个鼓包构成。
// 机尾按俯视梯形：后臂根部附近仍接近全宽，向后直线收窄到一段平切端面
const HULL_KEYS = [
  { t: 0.0, w: 0.0192, yTop: 0.0188, yBot: -0.0224, expTop: 3.6, expBot: 3.2, inset: 0.0042, step: 0.0014, topFillet: 0.009, botFillet: 0.011 },
  { t: 0.04, w: 0.0256, yTop: 0.0226, yBot: -0.0262, expTop: 3.7, expBot: 3.35, inset: 0.0056, step: 0.0018, topFillet: 0.0085, botFillet: 0.0105 },
  { t: 0.085, w: 0.0328, yTop: 0.0264, yBot: -0.0296, expTop: 3.9, expBot: 3.55, inset: 0.0074, step: 0.0024, topFillet: 0.008, botFillet: 0.0095 },
  { t: 0.14, w: 0.0386, yTop: 0.0288, yBot: -0.0316, expTop: 4.1, expBot: 3.7, inset: 0.0092, step: 0.0028, topFillet: 0.0074, botFillet: 0.0085 },
  { t: 0.2, w: 0.0416, yTop: 0.0298, yBot: -0.0326, expTop: 4.25, expBot: 3.85, inset: 0.0108, step: 0.0034, topFillet: 0.0068, botFillet: 0.0078 },
  { t: 0.26, w: 0.0426, yTop: 0.0304, yBot: -0.033, expTop: 4.4, expBot: 3.95, inset: 0.0116, step: 0.0036, topFillet: 0.0065, botFillet: 0.0072 },
  { t: 0.311, w: 0.0431, yTop: 0.0306, yBot: -0.0332, expTop: 4.55, expBot: 4.05, inset: 0.0124, step: 0.0038, topFillet: 0.0064, botFillet: 0.007 },
  { t: 0.447, w: 0.0428, yTop: 0.03, yBot: -0.0326, expTop: 4.5, expBot: 4.0, inset: 0.0122, step: 0.0036, topFillet: 0.007, botFillet: 0.0068 },
  { t: 0.558, w: 0.0412, yTop: 0.0284, yBot: -0.0314, expTop: 4.35, expBot: 3.8, inset: 0.0112, step: 0.0032, topFillet: 0.0082, botFillet: 0.0064 },
  { t: 0.661, w: 0.038, yTop: 0.0243, yBot: -0.0288, expTop: 3.9, expBot: 3.4, inset: 0.0088, step: 0.0024, topFillet: 0.01, botFillet: 0.0055 },
  { t: 0.753, w: 0.0333, yTop: 0.018, yBot: -0.0249, expTop: 3.6, expBot: 3.2, inset: 0.004, step: 0.001, topFillet: 0.011, botFillet: 0.0042 },
  { t: 0.831, w: 0.0295, yTop: 0.0124, yBot: -0.0221, expTop: 3.2, expBot: 3.0, inset: 0.0048, step: 0.0007, topFillet: 0.01, botFillet: 0.003 },
  { t: 0.895, w: 0.0253, yTop: 0.0066, yBot: -0.0199, expTop: 3.0, expBot: 2.9, inset: 0.0055, step: 0.0005, topFillet: 0.0082, botFillet: 0.0022 },
  { t: 0.938, w: 0.0225, yTop: 0.0024, yBot: -0.0188, expTop: 2.95, expBot: 2.85, inset: 0.0056, step: 0.0005, topFillet: 0.0065, botFillet: 0.0017 },
  { t: 0.966, w: 0.0211, yTop: 0.0002, yBot: -0.0182, expTop: 2.9, expBot: 2.8, inset: 0.0057, step: 0.0005, topFillet: 0.0055, botFillet: 0.0014 },
  { t: 0.986, w: 0.0204, yTop: -0.0011, yBot: -0.0179, expTop: 2.9, expBot: 2.8, inset: 0.0057, step: 0.0005, topFillet: 0.005, botFillet: 0.0013 },
  { t: 1.0, w: 0.0201, yTop: -0.0021, yBot: -0.0177, expTop: 2.95, expBot: 2.85, inset: 0.0058, step: 0.0005, topFillet: 0.0046, botFillet: 0.0012 },
];

const hullProfile = compileProfile(HULL_KEYS);

/** 把 yTop/yBot 形式的关键帧换算成 loft() 需要的中心+半高形式。 */
function toSection(raw) {
  const yOff = (raw.yTop + raw.yBot) / 2;
  return {
    w: raw.w,
    hTop: raw.yTop - yOff,
    hBot: yOff - raw.yBot,
    expTop: raw.expTop,
    expBot: raw.expBot,
    yOff,
    inset: raw.inset ?? 0,
    step: raw.step ?? 0,
    topFillet: raw.topFillet ?? 0.004,
    botFillet: raw.botFillet ?? 0.0075,
  };
}

const Z_TAIL = SPEC.body.zTail;
// 主壳体前端是一块钝头的中脊面板，比鼓包尖端略微内收
const Z_HULL_NOSE = 0.0865;

function tFromZ(z) {
  return THREE.MathUtils.clamp((z - Z_TAIL) / (Z_HULL_NOSE - Z_TAIL), 0, 1);
}

/**
 * 求主壳体在给定 (x, z) 处上表面或下表面的高度。
 * 由超椭圆方程 (x/w)^n + (y/h)^n = 1 反解得到，用来把各种面板、
 * 散热栅、传感器窗口精确地贴合到曲面上。
 */
export function hullSurfaceY(x, z, up = true) {
  const sec = toSection(hullProfile(tFromZ(z)));
  const h = up ? sec.hTop : sec.hBot;
  const exp = up ? sec.expTop : sec.expBot;
  const nx = Math.min(Math.abs(x) / sec.w, 1);
  const k = Math.pow(Math.max(0, 1 - Math.pow(nx, exp)), 1 / exp);
  return sec.yOff + (up ? h * k : -h * k);
}

/**
 * 求主壳体在给定 (y, z) 处侧面的 |x|，是 hullSurfaceY 的对偶：
 * 同样由超椭圆方程反解，只是这次解出的是横向半宽。
 */
function hullSurfaceX(y, z) {
  const sec = toSection(hullProfile(tFromZ(z)));
  const dy = y - sec.yOff;
  const h = dy >= 0 ? sec.hTop : sec.hBot;
  const exp = dy >= 0 ? sec.expTop : sec.expBot;
  const ny = Math.min(Math.abs(dy) / h, 1);
  return sec.w * Math.pow(Math.max(0, 1 - Math.pow(ny, exp)), 1 / exp);
}

/**
 * 把一块贴片摆到壳体侧面上。侧面的斜率沿轴向变化很快，
 * 直接按中心站位的半宽摆会让贴片两端翘出壳体，所以要按真实法线定向。
 */
function placeOnHullSide(mesh, side, y, z, { proud = 0 } = {}) {
  const d = 0.0006;
  const dxdy = (hullSurfaceX(y + d, z) - hullSurfaceX(y - d, z)) / (2 * d);
  const dxdz = (hullSurfaceX(y, z + d) - hullSurfaceX(y, z - d)) / (2 * d);
  const normal = new THREE.Vector3(side, -dxdy, -dxdz).normalize();
  mesh.position.set(side * hullSurfaceX(y, z), y, z).addScaledVector(normal, proud);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), normal);
  return mesh;
}

/** 曲面法向（数值差分），用于让贴片沿壳体外法线方向摆正。 */
function hullNormal(x, z, up = true) {
  const d = 0.0006;
  const dx = (hullSurfaceY(x + d, z, up) - hullSurfaceY(x - d, z, up)) / (2 * d);
  const dz = (hullSurfaceY(x, z + d, up) - hullSurfaceY(x, z - d, up)) / (2 * d);
  const n = new THREE.Vector3(-dx, 1, -dz);
  if (!up) n.set(dx, -1, dz);
  return n.normalize();
}

/** 把一块贴片摆到壳体表面上：+Z 轴对齐外法线，proud 为凸出高度。 */
function placeOnHull(mesh, x, z, { up = true, proud = 0.0004, spin = 0 } = {}) {
  const y = hullSurfaceY(x, z, up);
  const normal = hullNormal(x, z, up);
  mesh.position.set(x, y, z).addScaledVector(normal, proud);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), normal);
  if (spin) mesh.rotateZ(spin);
  return mesh;
}

/**
 * 通用镜头组件：黑井压圈 + 多层光阑 + 镀膜玻璃。
 * 组件以 +Z 为光轴方向，原点位于压圈根部，方便直接贴到曲面上。
 */
export function createLens(radius, { depth = 0.006 } = {}) {
  const group = new THREE.Group();
  const segs = 48;

  const bezel = new THREE.Mesh(
    new THREE.CylinderGeometry(radius, radius * 0.98, depth, segs, 1, true),
    materials.lensBarrel
  );
  bezel.rotation.x = Math.PI / 2;
  bezel.position.z = depth / 2;
  group.add(bezel);

  const rim = new THREE.Mesh(new THREE.RingGeometry(radius * 0.78, radius, segs), materials.lensBarrel);
  rim.position.z = depth + 0.00015;
  group.add(rim);

  const chrome = new THREE.Mesh(
    new THREE.RingGeometry(radius * 0.72, radius * 0.78, segs),
    materials.lensRing
  );
  chrome.position.z = depth;
  group.add(chrome);

  const well = new THREE.Mesh(
    new THREE.CylinderGeometry(radius * 0.72, radius * 0.42, depth * 0.92, segs, 1, true),
    materials.lensInner
  );
  well.rotation.x = Math.PI / 2;
  well.position.z = depth * 0.52;
  group.add(well);

  const baffles = [
    { inner: 0.58, outer: 0.71, z: 0.82 },
    { inner: 0.44, outer: 0.58, z: 0.58 },
    { inner: 0.3, outer: 0.44, z: 0.36 },
  ];
  for (const b of baffles) {
    const ring = new THREE.Mesh(
      new THREE.RingGeometry(radius * b.inner, radius * b.outer, segs),
      materials.lensInner
    );
    ring.position.z = depth * b.z;
    group.add(ring);
  }

  const floor = new THREE.Mesh(new THREE.CircleGeometry(radius * 0.3, 32), materials.lensInner);
  floor.position.z = depth * 0.16;
  group.add(floor);

  const pupil = new THREE.Mesh(new THREE.CircleGeometry(radius * 0.16, 28), materials.lensAperture);
  pupil.position.z = depth * 0.18;
  group.add(pupil);

  const innerGlass = new THREE.Mesh(
    new THREE.SphereGeometry(radius * 0.46, 36, 16, 0, Math.PI * 2, 0, Math.PI / 2),
    materials.lensGlassInner
  );
  innerGlass.rotation.x = Math.PI / 2;
  innerGlass.scale.set(1, 0.22, 1);
  innerGlass.position.z = depth * 0.34;
  group.add(innerGlass);

  const glass = new THREE.Mesh(
    new THREE.SphereGeometry(radius * 0.72, 48, 20, 0, Math.PI * 2, 0, Math.PI / 2),
    materials.lensGlass
  );
  glass.rotation.x = Math.PI / 2;
  glass.scale.set(1, 0.2, 1);
  glass.position.z = depth * 0.78;
  group.add(glass);

  return group;
}

/** 主壳体。 */
function createHull() {
  const geometry = loft({
    path: (t) => new THREE.Vector3(0, 0, THREE.MathUtils.lerp(Z_TAIL, Z_HULL_NOSE, t)),
    section: (t) => toSection(hullProfile(t)),
    steps: 132,
    segments: 80,
    capStart: true,
    capEnd: true,
    softCapStart: true,
    ring: steppedShellRing,
  });
  const mesh = new THREE.Mesh(geometry, materials.shellUpper);
  mesh.name = '机身外壳';
  return mesh;
}

/** 机头鼓包：从机身肩部内部长出，末端收成圆孔把镜头嵌进去。 */
function createNosePod(side) {
  const { xOffset, zStart, zTip, lensY } = SPEC.nosePod;
  const podKeys = compileProfile([
    { t: 0.0, w: 0.0158, yTop: 0.022, yBot: -0.024, expTop: 3.8, expBot: 3.5 },
    { t: 0.18, w: 0.0167, yTop: 0.0204, yBot: -0.0224, expTop: 3.4, expBot: 3.2 },
    { t: 0.38, w: 0.0171, yTop: 0.0168, yBot: -0.0196, expTop: 2.9, expBot: 2.8 },
    { t: 0.62, w: 0.0156, yTop: 0.0136, yBot: -0.0164, expTop: 2.5, expBot: 2.4 },
    { t: 0.82, w: 0.0126, yTop: 0.0114, yBot: -0.013, expTop: 2.2, expBot: 2.2 },
    { t: 0.93, w: 0.0108, yTop: 0.0106, yBot: -0.0112, expTop: 2.12, expBot: 2.12 },
    { t: 1.0, w: 0.0101, yTop: 0.0105, yBot: -0.0107, expTop: 2.05, expBot: 2.05 },
  ]);

  // smoothstep：两端切向沿 +Z，开口正对前方；根部埋在壳体内慢慢鼓出
  const ease = (t) => t * t * (3 - 2 * t);
  const geometry = loft({
    path: (t) => {
      const e = ease(t);
      return new THREE.Vector3(
        side * THREE.MathUtils.lerp(0.0204, xOffset + 0.001, e),
        THREE.MathUtils.lerp(0.001, lensY + 0.001, e),
        THREE.MathUtils.lerp(zStart, zTip, t)
      );
    },
    section: (t) => {
      const raw = podKeys(t);
      const yOff = (raw.yTop + raw.yBot) / 2;
      return {
        w: raw.w,
        hTop: raw.yTop - yOff,
        hBot: yOff - raw.yBot,
        expTop: raw.expTop,
        expBot: raw.expBot,
        yOff,
      };
    },
    steps: 84,
    segments: 56,
    capStart: false,
    capEnd: false,
  });

  const mesh = new THREE.Mesh(geometry, materials.shellUpper);
  mesh.name = side > 0 ? '右前视觉鼓包' : '左前视觉鼓包';
  return mesh;
}

/** 前视觉镜头：嵌在鼓包末端的圆孔里，灰壳一直收到镜头压圈。 */
function createFrontVision(side) {
  const { xOffset, lensY, lensRadius, zTip, yaw, pitch } = SPEC.nosePod;
  const depth = 0.0078;
  const group = new THREE.Group();
  group.name = side > 0 ? '前视觉相机(右)' : '前视觉相机(左)';

  const lens = createLens(lensRadius, { depth });
  lens.position.z = -depth + 0.0002;
  group.add(lens);

  group.position.set(side * (xOffset + 0.001), lensY + 0.001, zTip);
  group.rotation.y = side * yaw;
  group.rotation.x = pitch;
  return group;
}

/** 机身两侧的接口盖与分模线细节。 */
function createSideDetails() {
  const group = new THREE.Group();
  group.name = '侧面接口';

  for (const side of [-1, 1]) {
    const port = new THREE.Mesh(roundedPlate(0.0105, 0.006, 0.0022, 0.0025), materials.darkPlasticSoft);
    placeOnHullSide(port, side, -0.012, -0.072, { proud: -0.0004 });
    group.add(port);
  }

  // 机头正面那块指示窗：贴在两鼓包之间的桥面上，因此朝向偏前而不是朝上
  const badge = new THREE.Mesh(roundedPlate(0.022, 0.0068, 0.0014, 0.003), materials.shellDeck);
  placeOnHull(badge, 0, 0.0705, { proud: -0.0012 });
  group.add(badge);

  return group;
}

/** 机腹：下视觉双目、红外测距窗与补光灯。 */
function createBelly() {
  const group = new THREE.Group();
  group.name = '下视觉系统';

  const panel = new THREE.Mesh(roundedPlate(0.038, 0.031, 0.0025, 0.008), materials.darkPlasticSoft);
  placeOnHull(panel, 0, 0.012, { up: false, proud: 0.0006 });
  group.add(panel);

  for (const side of [-1, 1]) {
    const cam = createLens(0.0045, { depth: 0.003 });
    const y = hullSurfaceY(side * 0.013, 0.006, false);
    cam.position.set(side * 0.013, y + 0.0012, 0.006);
    cam.rotation.x = Math.PI / 2;
    group.add(cam);

    const ir = new THREE.Mesh(
      new THREE.CylinderGeometry(0.0028, 0.0028, 0.002, 20),
      materials.lensBarrel
    );
    ir.position.set(side * 0.015, hullSurfaceY(side * 0.015, 0.021, false) + 0.0006, 0.021);
    group.add(ir);
  }

  const light = new THREE.Mesh(
    new THREE.CylinderGeometry(0.0035, 0.0035, 0.0018, 24),
    materials.ledGreen
  );
  light.position.set(0, hullSurfaceY(0, -0.004, false) + 0.0006, -0.004);
  group.add(light);

  // 机腹后部的脚垫与散热口
  const rearPad = new THREE.Mesh(roundedPlate(0.032, 0.016, 0.002, 0.0045), materials.darkPlasticSoft);
  placeOnHull(rearPad, 0, -0.062, { up: false, proud: 0.0005 });
  group.add(rearPad);

  return group;
}

/** 机臂折叠时嵌入机身的四个凹槽。 */
function createArmSockets() {
  const group = new THREE.Group();
  group.name = '机臂铰链槽';
  const geometry = roundedPlate(0.0182, 0.0146, 0.0038, 0.0034);

  for (const side of [-1, 1]) {
    for (const z of [SPEC.arm.rootFront[2], SPEC.arm.rootRear[2]]) {
      const mesh = new THREE.Mesh(geometry, materials.darkPlastic);
      placeOnHullSide(mesh, side, -0.006, z, { proud: -0.0012 });
      group.add(mesh);
    }
  }
  return group;
}

export function createFuselage() {
  const group = new THREE.Group();
  group.name = '机身';
  group.userData.explode = new THREE.Vector3(0, 1, 0);

  group.add(createHull());
  for (const side of [-1, 1]) {
    group.add(createNosePod(side));
    group.add(createFrontVision(side));
  }
  group.add(createSideDetails());
  group.add(createBelly());
  group.add(createArmSockets());

  return group;
}
