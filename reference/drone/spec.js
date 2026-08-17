/**
 * 整机尺寸基准表（单位：米）。
 *
 * 数值来自对 figures/ 中三视图的像素量测：以机身全长 183 mm 作为标定基准，
 * 再按俯视图中机身长度所占像素数换算出电机间距、桨盘直径等其余尺寸。
 *
 * 机体坐标系：+X 指向右侧，+Y 指向上方，+Z 指向机头。
 * 原点位于机身包络的几何中心（机身中截面处）。
 */
export const SPEC = {
  body: {
    zTail: -0.096,
    zNose: 0.09,
    halfWidth: 0.0431,
    topY: 0.0306,
    bottomY: -0.0332,
  },

  // 前视觉双目所在的两个鼓包
  nosePod: {
    xOffset: 0.0295,
    zStart: 0.012,
    zTip: 0.0885,
    lensY: -0.0105,
    lensRadius: 0.0092,
    /** 镜头轴线相对机头方向的外偏角与下俯角 */
    yaw: 0.08,
    pitch: -0.05,
  },

  // 四条机臂都铰接在机身中段的同一段凹槽里（折叠时正好贴合机身侧面），
  // 前臂前掠、后臂后掠，与俯视图量得的 ~42° 后掠角一致
  arm: {
    /** 机臂根部（铰链中心）在机身上的位置 */
    rootFront: [0.0388, -0.0086, 0.0065],
    rootRear: [0.0388, -0.0086, -0.057],
    /** 电机轴心位置 */
    motorFront: [0.1378, -0.008, 0.0932],
    motorRear: [0.1378, -0.008, -0.1413],
  },

  // 正视图/背视图里电机钟罩的直径约为机身宽度的 0.28，据此定标
  motor: {
    baseRadius: 0.0146,
    bellRadius: 0.0136,
    bellHeight: 0.0112,
    statorRadius: 0.0116,
    shaftRadius: 0.0016,
    /** 桨毂安装面高度（相对电机安装座底面） */
    hubY: 0.0212,
  },

  propeller: {
    /** 桨尖半径 */
    radius: 0.0955,
    rootRadius: 0.0125,
    hubLength: 0.031,
    hubWidth: 0.0112,
    hubHeight: 0.0064,
    /** 桨尖橙色段所占展向比例 */
    tipFraction: 0.79,
  },

  // 支腿外侧与电机安装台齐平，内侧略收，截面为圆角方管
  gear: {
    footY: -0.0705,
  },

  gimbal: {
    /** 减震板中心（嵌进机腹凹槽里，整机装配后基本不外露） */
    mountY: -0.0128,
    mountZ: 0.0615,
    /** 相机主体中心 */
    cameraY: -0.0418,
    cameraZ: 0.0668,
    cameraWidth: 0.0368,
    cameraHeight: 0.0334,
    cameraDepth: 0.029,
    rollRadius: 0.0086,
    rollOffset: 0.0256,
    lensRadius: 0.0112,
  },
};

/** 四个旋翼的编号、位置与转向。按 X 型布局：对角同向，相邻反向。 */
export const ROTORS = [
  { id: 1, label: 'M1 右前', x: 1, z: 1, spin: -1 },
  { id: 2, label: 'M2 左后', x: -1, z: -1, spin: -1 },
  { id: 3, label: 'M3 左前', x: -1, z: 1, spin: 1 },
  { id: 4, label: 'M4 右后', x: 1, z: -1, spin: 1 },
];

/** 取某个旋翼的电机轴心与机臂根部坐标。 */
export function rotorAnchors(rotor) {
  const front = rotor.z > 0;
  const motor = front ? SPEC.arm.motorFront : SPEC.arm.motorRear;
  const root = front ? SPEC.arm.rootFront : SPEC.arm.rootRear;
  return {
    motor: [motor[0] * rotor.x, motor[1], motor[2]],
    root: [root[0] * rotor.x, root[1], root[2]],
  };
}
