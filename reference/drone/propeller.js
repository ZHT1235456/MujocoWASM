import * as THREE from 'three';
import { SPEC } from './spec.js';
import { materials } from './materials.js';
import { compileProfile, roundedBox } from './geometry.js';

/**
 * 折叠螺旋桨。
 *
 * 桨叶不是简单的扁片，而是按真实桨叶的方式生成：
 * 沿展向给出弦长、后掠量、几何扭转角和相对厚度，
 * 每个展向站位放一个 NACA 四位翼型截面，再缝合成曲面。
 * 桨尖 15% 用第二个材质组染成橙色，对应实物的警示涂装。
 */

// 展向站位表：s=0 在桨根，s=1 在桨尖
const BLADE_KEYS = compileProfile([
  { s: 0.0, chord: 0.0078, sweep: 0.0, twist: 26.0, thick: 0.3, rise: 0.0 },
  { s: 0.08, chord: 0.0112, sweep: 0.0007, twist: 24.0, thick: 0.21, rise: 0.0002 },
  { s: 0.2, chord: 0.0158, sweep: 0.0017, twist: 21.0, thick: 0.14, rise: 0.0006 },
  { s: 0.35, chord: 0.0186, sweep: 0.003, twist: 17.5, thick: 0.105, rise: 0.0011 },
  { s: 0.5, chord: 0.0196, sweep: 0.0044, twist: 14.5, thick: 0.085, rise: 0.0017 },
  { s: 0.65, chord: 0.0192, sweep: 0.006, twist: 12.0, thick: 0.078, rise: 0.0022 },
  { s: 0.78, chord: 0.0175, sweep: 0.0076, twist: 10.0, thick: 0.072, rise: 0.0027 },
  { s: 0.88, chord: 0.0148, sweep: 0.009, twist: 8.8, thick: 0.07, rise: 0.0031 },
  { s: 0.95, chord: 0.0104, sweep: 0.0104, twist: 8.0, thick: 0.075, rise: 0.0034 },
  { s: 1.0, chord: 0.0032, sweep: 0.0118, twist: 7.5, thick: 0.1, rise: 0.0036 },
].map((k) => ({ t: k.s, ...k })));

/** NACA 四位翼型：4.5% 弯度、最大弯度位于 40% 弦长处。 */
const CAMBER = 0.045;
const CAMBER_POS = 0.4;

function thicknessAt(u, thick) {
  return (
    5 *
    thick *
    (0.2969 * Math.sqrt(u) - 0.126 * u - 0.3516 * u * u + 0.2843 * u ** 3 - 0.1036 * u ** 4)
  );
}

function camberAt(u) {
  const m = CAMBER;
  const p = CAMBER_POS;
  if (u < p) return (m / (p * p)) * (2 * p * u - u * u);
  return (m / ((1 - p) ** 2)) * (1 - 2 * p + 2 * p * u - u * u);
}

/**
 * 生成一片桨叶（沿 +X 展开）。
 * @param {number} hand +1 为逆时针桨，-1 为顺时针桨（镜像翼型与扭转方向）
 */
function createBladeGeometry(hand) {
  const { radius, rootRadius, tipFraction } = SPEC.propeller;
  const chordPoints = 26; // 上/下表面各采样点数

  // 展向站位：加密并强制在橙色分界处放一个站位，保证配色边界笔直
  const stations = [];
  const count = 64;
  for (let i = 0; i <= count; i++) stations.push(i / count);
  stations.push(tipFraction);
  stations.sort((a, b) => a - b);

  const perRing = chordPoints * 2;
  const positions = [];
  const normals = [];
  const uvs = [];
  const blackIndices = [];
  const orangeIndices = [];

  for (const s of stations) {
    const k = BLADE_KEYS(s);
    const r = THREE.MathUtils.lerp(rootRadius, radius, s);
    const twist = THREE.MathUtils.degToRad(k.twist);
    const cosT = Math.cos(twist);
    const sinT = Math.sin(twist);

    for (let j = 0; j < perRing; j++) {
      // 上表面 j∈[0, chordPoints]，下表面折回；余弦分布让前后缘采样更密
      const upper = j <= chordPoints;
      const idx = upper ? j : perRing - j;
      const u = 0.5 * (1 - Math.cos((Math.PI * idx) / chordPoints));
      const yt = thicknessAt(u, k.thick);
      const yc = camberAt(u);
      const yLocal = (yc + (upper ? yt : -yt)) * k.chord;
      const zLocal = (0.3 - u) * k.chord - k.sweep;

      const y = yLocal * cosT + zLocal * sinT + k.rise;
      const z = (-yLocal * sinT + zLocal * cosT) * hand;

      positions.push(r, y, z);
      uvs.push(u, s);
    }
  }

  const ringCount = stations.length;
  for (let i = 0; i < ringCount - 1; i++) {
    const midS = (stations[i] + stations[i + 1]) / 2;
    const target = midS < tipFraction ? blackIndices : orangeIndices;
    for (let j = 0; j < perRing; j++) {
      const a = i * perRing + j;
      const b = i * perRing + ((j + 1) % perRing);
      const c = (i + 1) * perRing + j;
      const d = (i + 1) * perRing + ((j + 1) % perRing);
      if (hand > 0) {
        target.push(a, c, b, b, c, d);
      } else {
        target.push(a, b, c, b, d, c);
      }
    }
  }

  // 桨尖与桨根封盖
  const capRing = (ringOffset, indices, flip) => {
    const centerIndex = positions.length / 3;
    let cx = 0;
    let cy = 0;
    let cz = 0;
    for (let j = 0; j < perRing; j++) {
      cx += positions[(ringOffset + j) * 3];
      cy += positions[(ringOffset + j) * 3 + 1];
      cz += positions[(ringOffset + j) * 3 + 2];
    }
    positions.push(cx / perRing, cy / perRing, cz / perRing);
    uvs.push(0.5, 0.5);
    for (let j = 0; j < perRing; j++) {
      const a = ringOffset + j;
      const b = ringOffset + ((j + 1) % perRing);
      if (flip) indices.push(centerIndex, b, a);
      else indices.push(centerIndex, a, b);
    }
  };
  capRing(0, blackIndices, hand > 0);
  capRing((ringCount - 1) * perRing, orangeIndices, hand < 0);

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geometry.setAttribute('uv', new THREE.Float32BufferAttribute(uvs, 2));
  geometry.setIndex([...blackIndices, ...orangeIndices]);
  geometry.addGroup(0, blackIndices.length, 0);
  geometry.addGroup(blackIndices.length, orangeIndices.length, 1);
  geometry.computeVertexNormals();
  return geometry;
}

/** 桨毂：黑色夹片 + 两颗不锈钢紧固螺钉。 */
function createHub() {
  const group = new THREE.Group();
  const { hubLength, hubWidth, hubHeight } = SPEC.propeller;

  const body = new THREE.Mesh(
    roundedBox(hubLength, hubHeight, hubWidth, 0.0026, 4),
    materials.bladeHub
  );
  group.add(body);

  for (const sign of [-1, 1]) {
    const boss = new THREE.Mesh(
      new THREE.CylinderGeometry(0.0044, 0.0044, hubHeight + 0.0016, 24),
      materials.bladeHub
    );
    boss.position.set(sign * 0.0068, 0, 0);
    group.add(boss);

    const screw = new THREE.Mesh(
      new THREE.CylinderGeometry(0.0019, 0.0019, 0.0012, 18),
      materials.steel
    );
    screw.position.set(sign * 0.0068, hubHeight / 2 + 0.0009, 0);
    group.add(screw);

    const washer = new THREE.Mesh(new THREE.RingGeometry(0.0019, 0.0028, 20), materials.aluminium);
    washer.rotation.x = -Math.PI / 2;
    washer.position.set(sign * 0.0068, hubHeight / 2 + 0.0006, 0);
    group.add(washer);
  }

  return group;
}

/**
 * 一副螺旋桨。
 * @param {number} spin +1 逆时针 / -1 顺时针（俯视）
 */
export function createPropeller(spin) {
  const group = new THREE.Group();
  group.name = spin > 0 ? '螺旋桨(逆时针)' : '螺旋桨(顺时针)';

  const geometry = createBladeGeometry(spin);
  const bladeMaterials = [materials.bladeBody, materials.bladeTip];

  // 折叠桨的两片桨叶分别夹在桨毂上下，因此有一点高度差
  const bladeA = new THREE.Mesh(geometry, bladeMaterials);
  bladeA.position.y = 0.0014;
  group.add(bladeA);

  const bladeB = new THREE.Mesh(geometry, bladeMaterials);
  bladeB.position.y = -0.0014;
  bladeB.rotation.y = Math.PI;
  group.add(bladeB);

  group.add(createHub());

  // 高转速时出现的桨盘虚影
  const disc = new THREE.Mesh(
    new THREE.RingGeometry(SPEC.propeller.rootRadius, SPEC.propeller.radius, 96, 1),
    new THREE.MeshBasicMaterial({
      color: 0x2f3237,
      transparent: true,
      opacity: 0,
      side: THREE.DoubleSide,
      depthWrite: false,
    })
  );
  disc.rotation.x = -Math.PI / 2;
  disc.visible = false;
  disc.name = 'rotorDisc';
  group.add(disc);
  group.userData.disc = disc;

  return group;
}
