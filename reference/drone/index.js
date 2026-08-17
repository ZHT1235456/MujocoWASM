import * as THREE from 'three';
import { SPEC, ROTORS, rotorAnchors } from './spec.js';
import { createFuselage } from './fuselage.js';
import { createArm } from './arm.js';
import { createPropeller } from './propeller.js';
import { createGimbal } from './gimbal.js';

/**
 * 装配整机。
 *
 * 返回的对象里保留了各运动部件的引用（旋翼、云台三轴），
 * 供动画循环与控制面板直接驱动。
 */
export function createDrone() {
  const root = new THREE.Group();
  root.name = '四旋翼无人机';

  const fuselage = createFuselage();
  root.add(fuselage);

  const gimbal = createGimbal();
  root.add(gimbal);

  const rotors = [];
  for (const rotor of ROTORS) {
    const { group: armGroup, hub } = createArm(rotor);
    root.add(armGroup);

    const propeller = createPropeller(rotor.spin);
    propeller.position.copy(hub);
    propeller.userData.explode = new THREE.Vector3(0, 1, 0);
    root.add(propeller);

    // 停机时桨叶横在机臂上（俯视图里四副桨都与各自机臂垂直）
    const { motor, root: armRoot } = rotorAnchors(rotor);
    const phase = Math.atan2(motor[0] - armRoot[0], motor[2] - armRoot[2]);
    propeller.rotation.y = phase;

    rotors.push({ ...rotor, arm: armGroup, propeller, phase, hub: hub.clone() });
  }

  root.traverse((object) => {
    if (object.isMesh) {
      object.castShadow = true;
      object.receiveShadow = true;
    }
  });

  // 记录爆炸视图的位移方向：没有显式指定的部件沿其自身质心方向散开
  root.traverse((object) => {
    if (object.parent === root) {
      object.userData.restPosition = object.position.clone();
      if (!object.userData.explode) {
        const dir = object.position.clone();
        object.userData.explode = dir.lengthSq() > 1e-8 ? dir.normalize() : new THREE.Vector3(0, 1, 0);
      }
    }
  });

  return {
    root,
    rotors,
    gimbal,
    gimbalAxes: gimbal.userData.axes,
    /** 桨盘半径等参数，供相机取景与地面尺寸使用 */
    spec: SPEC,
  };
}

/** 机体坐标系指示（+X 右 / +Y 上 / +Z 机头）。 */
export function createBodyAxes(length = 0.16) {
  const group = new THREE.Group();
  group.name = '机体坐标系';

  const axes = [
    { dir: new THREE.Vector3(1, 0, 0), color: 0xff4d5a },
    { dir: new THREE.Vector3(0, 1, 0), color: 0x63d471 },
    { dir: new THREE.Vector3(0, 0, 1), color: 0x4d9bff },
  ];

  for (const { dir, color } of axes) {
    const arrow = new THREE.ArrowHelper(dir, new THREE.Vector3(0, 0, 0), length, color, 0.022, 0.012);
    arrow.line.material.linewidth = 2;
    group.add(arrow);
  }
  return group;
}

/** 旋翼转向示意环：按各旋翼实际转向绘制带箭头的圆弧。 */
export function createSpinIndicators() {
  const group = new THREE.Group();
  group.name = '旋翼转向';

  for (const rotor of ROTORS) {
    const { motor } = rotorAnchors(rotor);
    const radius = SPEC.propeller.radius * 0.62;
    const curve = new THREE.EllipseCurve(0, 0, radius, radius, 0, Math.PI * 1.55, rotor.spin < 0);
    const points = curve.getPoints(64).map((p) => new THREE.Vector3(p.x, 0, p.y));
    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(points),
      new THREE.LineBasicMaterial({ color: rotor.spin > 0 ? 0x4d9bff : 0xffa53d })
    );
    line.position.set(motor[0], motor[1] + 0.045, motor[2]);
    group.add(line);

    const tip = points[points.length - 1];
    const prev = points[points.length - 6];
    const dir = new THREE.Vector3().subVectors(tip, prev).normalize();
    const head = new THREE.Mesh(
      new THREE.ConeGeometry(0.006, 0.016, 12),
      new THREE.MeshBasicMaterial({ color: rotor.spin > 0 ? 0x4d9bff : 0xffa53d })
    );
    head.position.copy(tip).add(new THREE.Vector3(motor[0], motor[1] + 0.045, motor[2]));
    head.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
    group.add(head);
  }

  return group;
}
