import * as THREE from 'three';
import { SPEC } from './spec.js';
import { materials } from './materials.js';
import { roundedBox, roundedPlate } from './geometry.js';
import { createLens } from './fuselage.js';

/**
 * 三轴机械增稳云台。
 *
 * 层级严格按真实轴序组织：减震板 → 偏航(Yaw) → 横滚(Roll) → 俯仰(Pitch) → 相机，
 * 因此外部只要旋转对应的 Group 就能得到物理上正确的姿态。
 */

/** 顶部减震板：金手指触点 + 四颗减震球。 */
function createDamperPlate() {
  const group = new THREE.Group();
  group.name = '减震板';

  const plate = new THREE.Mesh(roundedBox(0.0335, 0.0024, 0.0126, 0.0014, 3), materials.darkPlastic);
  group.add(plate);

  // 六针供电/数据触点
  for (let i = 0; i < 6; i++) {
    const pin = new THREE.Mesh(new THREE.CylinderGeometry(0.00055, 0.00055, 0.001, 10), materials.copper);
    pin.position.set(-0.005 + i * 0.002, 0.0017, 0);
    group.add(pin);
  }

  // 四颗橡胶减震球
  for (const sx of [-1, 1]) {
    for (const sz of [-1, 1]) {
      const damper = new THREE.Mesh(new THREE.SphereGeometry(0.0026, 18, 12), materials.rubber);
      damper.scale.y = 0.78;
      damper.position.set(sx * 0.013, -0.0029, sz * 0.0042);
      group.add(damper);
    }
  }

  const lower = new THREE.Mesh(roundedBox(0.0286, 0.0024, 0.0108, 0.0013, 3), materials.darkPlastic);
  lower.position.y = -0.0054;
  group.add(lower);

  return group;
}

/** 相机本体：方形镜头压圈 + 主镜头 + 两侧俯仰电机。 */
function createCameraHead() {
  const group = new THREE.Group();
  group.name = '相机';
  const { cameraWidth, cameraHeight, cameraDepth, rollRadius, rollOffset, lensRadius } = SPEC.gimbal;

  const body = new THREE.Mesh(
    roundedBox(cameraWidth, cameraHeight, cameraDepth, 0.0026, 5),
    materials.darkPlastic
  );
  group.add(body);

  // 前脸是逐层内收的方形压圈：亮边框 → 消光遮光框 → 镜头
  const bezel = new THREE.Mesh(
    roundedPlate(cameraWidth * 0.88, cameraHeight * 0.9, 0.0036, 0.0042),
    materials.aluminiumDark
  );
  bezel.position.z = cameraDepth / 2 + 0.0009;
  group.add(bezel);

  const innerBezel = new THREE.Mesh(
    roundedPlate(cameraWidth * 0.72, cameraHeight * 0.74, 0.0028, 0.0034),
    materials.darkPlasticSoft
  );
  innerBezel.position.z = cameraDepth / 2 + 0.0026;
  group.add(innerBezel);

  const shade = new THREE.Mesh(
    roundedPlate(cameraWidth * 0.6, cameraHeight * 0.62, 0.002, 0.0026),
    materials.lensBarrel
  );
  shade.position.z = cameraDepth / 2 + 0.0038;
  group.add(shade);

  const lens = createLens(lensRadius, { depth: 0.005 });
  lens.position.z = cameraDepth / 2 + 0.0042;
  group.add(lens);

  // 两侧的俯仰轴电机
  for (const side of [-1, 1]) {
    const motor = new THREE.Mesh(
      new THREE.CylinderGeometry(rollRadius, rollRadius, 0.0132, 30),
      materials.darkPlastic
    );
    motor.rotation.z = Math.PI / 2;
    motor.position.set(side * rollOffset, -0.0012, -0.0022);
    group.add(motor);

    const cap = new THREE.Mesh(
      new THREE.CylinderGeometry(rollRadius * 0.6, rollRadius * 0.6, 0.0142, 22),
      materials.darkPlasticSoft
    );
    cap.rotation.z = Math.PI / 2;
    cap.position.set(side * rollOffset, -0.0012, -0.0022);
    group.add(cap);
  }

  return group;
}

export function createGimbal() {
  const root = new THREE.Group();
  root.name = '三轴云台';
  root.userData.explode = new THREE.Vector3(0, -1, 0.35).normalize();

  const { mountY, mountZ, cameraY, cameraZ } = SPEC.gimbal;
  // 减震板与偏航电机基本藏在机腹的凹槽里，机外只看得到横滚叉臂和相机
  const YAW_DROP = 0.0062;
  const ROLL_DROP = 0.006;

  const plate = createDamperPlate();
  plate.position.set(0, mountY, mountZ);
  root.add(plate);

  // —— 偏航轴 ——
  const yaw = new THREE.Group();
  yaw.name = 'Yaw 偏航轴';
  yaw.position.set(0, mountY - YAW_DROP, mountZ);
  root.add(yaw);

  const yawMotor = new THREE.Mesh(
    new THREE.CylinderGeometry(0.0062, 0.0068, 0.0058, 26),
    materials.darkPlastic
  );
  yawMotor.position.y = -0.0025;
  yaw.add(yawMotor);

  // —— 横滚轴 ——
  const roll = new THREE.Group();
  roll.name = 'Roll 横滚轴';
  roll.position.set(0, -ROLL_DROP, 0);
  yaw.add(roll);

  const rollHub = new THREE.Mesh(
    new THREE.CylinderGeometry(0.0052, 0.0052, 0.0068, 24),
    materials.darkPlasticSoft
  );
  rollHub.rotation.x = Math.PI / 2;
  rollHub.position.z = 0.0014;
  roll.add(rollHub);

  // 相机俯仰轴相对横滚轴的位置
  const pitchOffsetY = cameraY - (mountY - YAW_DROP - ROLL_DROP);
  const pitchOffsetZ = cameraZ - mountZ;

  // 横滚臂：从横滚轴分叉、斜撑下来抱住相机两侧的俯仰轴
  const armOut = SPEC.gimbal.rollOffset + 0.0034;
  const armIn = SPEC.gimbal.rollOffset - 0.0032;
  for (const side of [-1, 1]) {
    const shape = new THREE.Shape();
    shape.moveTo(0, 0.004);
    shape.lineTo(side * 0.008, 0.0028);
    shape.lineTo(side * armOut, pitchOffsetY + 0.0042);
    shape.lineTo(side * armOut, pitchOffsetY - 0.0042);
    shape.lineTo(side * armIn, pitchOffsetY - 0.0042);
    shape.lineTo(side * armIn, pitchOffsetY + 0.0016);
    shape.lineTo(side * 0.0055, -0.0028);
    shape.lineTo(0, -0.0035);
    const geometry = new THREE.ExtrudeGeometry(shape, {
      depth: 0.005,
      bevelEnabled: true,
      bevelThickness: 0.0007,
      bevelSize: 0.0007,
      bevelSegments: 2,
      curveSegments: 8,
    });
    const fork = new THREE.Mesh(geometry, materials.darkPlastic);
    fork.position.set(0, 0.0014, pitchOffsetZ - 0.003);
    roll.add(fork);
  }

  // —— 俯仰轴 ——
  const pitch = new THREE.Group();
  pitch.name = 'Pitch 俯仰轴';
  pitch.position.set(0, pitchOffsetY, pitchOffsetZ);
  roll.add(pitch);
  pitch.add(createCameraHead());

  root.userData.axes = { yaw, roll, pitch };
  return root;
}
