import * as THREE from 'three';
import { SPEC, rotorAnchors } from './spec.js';
import { materials } from './materials.js';
import { compileProfile, loft, splinePath, roundedPlate, roundedBox } from './geometry.js';

/**
 * 机臂总成：折叠机臂 + 无刷外转子电机 + 起落架。
 *
 * 每条机臂按其实际的根部/电机坐标在机体坐标系中直接放样，
 * 因此前臂前掠、后臂后掠的差异是几何本身带来的，而不是靠旋转复制。
 */

const ARM_KEYS = compileProfile([
  { t: 0.0, w: 0.0096, hTop: 0.0082, hBot: 0.0082, exp: 4.0 },
  { t: 0.12, w: 0.0088, hTop: 0.0074, hBot: 0.0074, exp: 4.1 },
  { t: 0.38, w: 0.0074, hTop: 0.0062, hBot: 0.0064, exp: 4.1 },
  { t: 0.68, w: 0.0065, hTop: 0.0055, hBot: 0.0057, exp: 3.8 },
  { t: 0.9, w: 0.0060, hTop: 0.0052, hBot: 0.0054, exp: 3.4 },
  { t: 1.0, w: 0.0064, hTop: 0.0053, hBot: 0.0055, exp: 3.1 },
]);

/** 无刷外转子电机：铝制钟罩 + 定子铁芯 + 外露的漆包线。 */
function createMotor() {
  const group = new THREE.Group();
  group.name = '无刷电机';
  const { baseRadius, bellRadius, statorRadius, shaftRadius } = SPEC.motor;

  // 机臂端面上的塑料托盘
  const seat = new THREE.Mesh(
    new THREE.CylinderGeometry(baseRadius, baseRadius * 0.94, 0.0042, 48),
    materials.shellAccent
  );
  seat.position.y = 0.0012;
  group.add(seat);

  // 定子底盘：钟罩下沿在这里收进去，留出一圈深色缝隙
  const base = new THREE.Mesh(
    new THREE.CylinderGeometry(bellRadius * 0.9, baseRadius * 0.97, 0.0024, 48),
    materials.darkPlastic
  );
  base.position.y = 0.0045;
  group.add(base);

  // 钟罩：下沿一圈大倒角 → 直筒 → 顶面收口，半径单调不回缩，避免出现台阶
  const skirt = new THREE.Mesh(
    new THREE.CylinderGeometry(bellRadius, bellRadius * 0.87, 0.0028, 56),
    materials.aluminium
  );
  skirt.position.y = 0.0071;
  group.add(skirt);

  const bell = new THREE.Mesh(
    new THREE.CylinderGeometry(bellRadius, bellRadius, 0.0084, 56, 1, true),
    materials.aluminium
  );
  bell.position.y = 0.0127;
  group.add(bell);

  // 钟罩腰部那道车削凹线
  const groove = new THREE.Mesh(
    new THREE.TorusGeometry(bellRadius * 1.001, 0.00026, 6, 56),
    materials.aluminiumDark
  );
  groove.rotation.x = Math.PI / 2;
  groove.position.y = 0.0112;
  group.add(groove);

  const bellTop = new THREE.Mesh(
    new THREE.CylinderGeometry(bellRadius * 0.93, bellRadius, 0.0024, 56),
    materials.aluminium
  );
  bellTop.position.y = 0.0181;
  group.add(bellTop);

  // 顶面：转子压板压住绕组，齿间露出漆包线的橙色
  const winding = new THREE.Mesh(
    new THREE.CylinderGeometry(statorRadius, statorRadius, 0.0028, 44),
    materials.copper
  );
  winding.position.y = 0.0188;
  group.add(winding);

  const toothGeometry = new THREE.BoxGeometry(0.0026, 0.0028, 0.0066);
  for (let i = 0; i < 12; i++) {
    const tooth = new THREE.Mesh(toothGeometry, materials.darkPlastic);
    const a = (i / 12) * Math.PI * 2;
    const r = statorRadius * 0.76;
    tooth.position.set(Math.cos(a) * r, 0.0196, Math.sin(a) * r);
    tooth.rotation.y = -a;
    group.add(tooth);
  }

  const statorRim = new THREE.Mesh(
    new THREE.CylinderGeometry(statorRadius * 1.05, statorRadius * 1.05, 0.003, 48, 1, true),
    materials.darkPlastic
  );
  statorRim.position.y = 0.019;
  group.add(statorRim);

  const hubCap = new THREE.Mesh(
    new THREE.CylinderGeometry(0.0052, 0.0058, 0.0046, 32),
    materials.darkPlastic
  );
  hubCap.position.y = 0.0204;
  group.add(hubCap);

  const shaft = new THREE.Mesh(
    new THREE.CylinderGeometry(shaftRadius, shaftRadius, 0.005, 16),
    materials.steel
  );
  shaft.position.y = 0.0234;
  group.add(shaft);

  return group;
}

/** 支腿截面：顶部几乎铺满安装台，随后平滑收细；+y 指向电机端。 */
function strutSection(t) {
  const u = t < 0.1 ? 0 : (t - 0.1) / 0.9;
  const k = u * u * (3 - 2 * u);
  const w = THREE.MathUtils.lerp(0.0128, 0.0055, k);
  const yInner = THREE.MathUtils.lerp(-0.0108, -0.0048, k);
  const yOuter = THREE.MathUtils.lerp(0.0116, 0.0054, k);
  const yOff = (yOuter + yInner) / 2;
  return {
    w,
    hTop: yOuter - yOff,
    hBot: yOff - yInner,
    expTop: 5.8,
    expBot: 5.8,
    yOff,
    yOuter,
  };
}

/** 起落架：电机座到支腿一根连续方管，末端套橡胶脚垫。 */
function createLandingLeg(motorPos, armDir, isFront) {
  const group = new THREE.Group();
  group.name = '起落架';

  const footY = SPEC.gear.footY;
  const strutTop = motorPos.y + 0.0014;
  const strutBot = footY + 0.0012;
  const atY = (t) => THREE.MathUtils.lerp(strutTop, strutBot, t);

  const strut = new THREE.Mesh(
    loft({
      path: (u) => new THREE.Vector3(motorPos.x, atY(u * 0.96), motorPos.z),
      section: (u) => strutSection(u * 0.96),
      steps: 48,
      segments: 36,
      up: armDir.clone(),
    }),
    materials.shellUpper
  );
  group.add(strut);

  const FOOT_FROM = 0.9;
  const foot = new THREE.Mesh(
    loft({
      path: (u) => new THREE.Vector3(motorPos.x, atY(FOOT_FROM + (1 - FOOT_FROM) * u), motorPos.z),
      section: (u) => {
        const sec = strutSection(FOOT_FROM + (1 - FOOT_FROM) * u);
        const pad = 0.0009;
        return {
          w: sec.w + pad,
          hTop: sec.hTop + pad,
          hBot: sec.hBot + pad,
          expTop: 4.6,
          expBot: 4.6,
          yOff: sec.yOff,
        };
      },
      steps: 12,
      segments: 36,
      up: armDir.clone(),
    }),
    materials.rubber
  );
  group.add(foot);

  const lampT = 0.16;
  const lampSec = strutSection(lampT);
  const lamp = new THREE.Mesh(
    roundedPlate(0.0042, 0.0048, 0.0011, 0.0007),
    isFront ? materials.ledGreen : materials.ledRed
  );
  const lampAlong = lampSec.yOuter + 0.00035;
  lamp.position.set(
    motorPos.x + armDir.x * lampAlong,
    atY(lampT),
    motorPos.z + armDir.z * lampAlong
  );
  lamp.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), armDir);
  group.add(lamp);

  return group;
}

/**
 * 构建一条完整机臂。
 * @returns {{ group: THREE.Group, hub: THREE.Vector3 }} hub 为桨毂安装点
 */
export function createArm(rotor) {
  const group = new THREE.Group();
  group.name = `${rotor.label} 机臂总成`;

  const { motor, root } = rotorAnchors(rotor);
  const motorPos = new THREE.Vector3(...motor);
  const rootPos = new THREE.Vector3(...root);

  // 根部沿机臂轴线反向伸进机身内部，让机臂与外壳自然相贯而不产生折弯
  const axis = new THREE.Vector3().subVectors(motorPos, rootPos).normalize();
  const inner = rootPos.clone().addScaledVector(axis, -0.016);
  const mid = rootPos.clone().lerp(motorPos, 0.5);

  const armPath = splinePath([inner, mid, motorPos.clone().lerp(mid, 0.05)]);

  const armGeometry = loft({
    path: armPath,
    section: (t) => {
      const k = ARM_KEYS(t);
      return { w: k.w, hTop: k.hTop, hBot: k.hBot, expTop: k.exp, expBot: k.exp * 0.9 };
    },
    steps: 64,
    segments: 44,
  });
  const spar = new THREE.Mesh(armGeometry, materials.shellUpper);
  spar.name = '机臂';
  group.add(spar);

  // 机臂根部的折叠铰链
  const hingeDir = new THREE.Vector3().subVectors(motorPos, rootPos).setY(0).normalize();
  // 铰链完全落在机身侧面的凹槽里，只在机臂根部露出很窄的一圈深色端面
  const hinge = new THREE.Mesh(roundedBox(0.0062, 0.0112, 0.013, 0.002, 4), materials.darkPlastic);
  hinge.position.copy(rootPos).addScaledVector(hingeDir, -0.007);
  hinge.rotation.y = Math.atan2(hingeDir.x, hingeDir.z);
  group.add(hinge);

  const pivot = new THREE.Mesh(
    new THREE.CylinderGeometry(0.0019, 0.0019, 0.0124, 20),
    materials.aluminiumDark
  );
  pivot.position.copy(rootPos).addScaledVector(hingeDir, -0.0032);
  group.add(pivot);

  const motorGroup = createMotor();
  motorGroup.position.copy(motorPos).add(new THREE.Vector3(0, 0.0028, 0));
  group.add(motorGroup);

  group.add(createLandingLeg(motorPos, hingeDir, rotor.z > 0));

  const hub = motorPos.clone().add(new THREE.Vector3(0, SPEC.motor.hubY + 0.0028, 0));

  group.userData.explode = new THREE.Vector3(motorPos.x, 0.15, motorPos.z).normalize();
  return { group, hub };
}
