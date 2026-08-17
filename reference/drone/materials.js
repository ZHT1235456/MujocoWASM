import * as THREE from 'three';

/**
 * 材质库。取色直接对照 figures/ 中的实机照片：
 * 机身是带金属漆感的银灰注塑件，机臂铰链与云台是哑光黑塑，
 * 电机钟罩为阳极氧化铝，桨叶黑身橙尖，镜头为深色镀膜玻璃。
 */

const registry = [];

function register(material) {
  registry.push(material);
  return material;
}

/** 银灰机身漆：微弱清漆层 + 低粗糙度，形成照片里那种柔和的高光带。 */
function shell(color, { roughness = 0.42, metalness = 0.18, clearcoat = 0.5 } = {}) {
  return register(
    new THREE.MeshPhysicalMaterial({
      color: new THREE.Color(color),
      roughness,
      metalness,
      clearcoat,
      clearcoatRoughness: 0.35,
    })
  );
}

function plastic(color, roughness = 0.55, metalness = 0.05) {
  return register(new THREE.MeshStandardMaterial({ color: new THREE.Color(color), roughness, metalness }));
}

function metal(color, roughness = 0.28) {
  return register(new THREE.MeshStandardMaterial({ color: new THREE.Color(color), roughness, metalness: 1 }));
}

export const materials = {
  // —— 机身外壳 ——
  shellUpper: shell(0x9ba1a8),
  shellLower: shell(0x878d94, { roughness: 0.5, clearcoat: 0.3 }),
  shellDeck: shell(0xa4aab1, { roughness: 0.38 }),
  shellAccent: shell(0x9298a0, { roughness: 0.46 }),

  // —— 深色结构件 ——
  darkPlastic: plastic(0x2a2d32, 0.5),
  darkPlasticSoft: plastic(0x35383d, 0.62),
  vent: plastic(0x141618, 0.75),
  rubber: plastic(0x151618, 0.88),

  // —— 金属件 ——
  aluminium: metal(0xd2d6da, 0.24),
  aluminiumDark: metal(0x8d9298, 0.34),
  steel: metal(0xb9bec4, 0.2),
  copper: register(
    new THREE.MeshStandardMaterial({ color: 0xc2701c, roughness: 0.42, metalness: 0.85 })
  ),

  // —— 螺旋桨 ——
  bladeBody: plastic(0x2f3237, 0.58),
  bladeTip: register(
    new THREE.MeshStandardMaterial({ color: 0xf7941e, roughness: 0.5, metalness: 0.05 })
  ),
  bladeHub: plastic(0x202225, 0.48),

  // —— 光学件 ——
  lensBarrel: plastic(0x16181b, 0.38),
  lensGlass: register(
    new THREE.MeshPhysicalMaterial({
      color: 0x05080e,
      roughness: 0.06,
      metalness: 0.12,
      clearcoat: 1,
      clearcoatRoughness: 0.04,
      envMapIntensity: 0.42,
      iridescence: 0.95,
      iridescenceIOR: 1.6,
      iridescenceThicknessRange: [180, 480],
    })
  ),
  lensGlassInner: register(
    new THREE.MeshPhysicalMaterial({
      color: 0x03060a,
      roughness: 0.1,
      metalness: 0.08,
      clearcoat: 0.7,
      clearcoatRoughness: 0.08,
      envMapIntensity: 0.28,
      iridescence: 0.7,
      iridescenceIOR: 1.9,
      iridescenceThicknessRange: [280, 720],
    })
  ),
  // 镜筒内壁做消光处理，否则会把环境光反射成一片灰白
  lensInner: register(
    new THREE.MeshStandardMaterial({
      color: 0x050607,
      roughness: 0.82,
      metalness: 0,
      envMapIntensity: 0.12,
    })
  ),
  lensAperture: plastic(0x020203, 0.9),
  lensRing: metal(0x6a7076, 0.32),

  // —— 指示灯 ——
  // 航行灯是嵌在壳体里的深色灯窗，只透出一点点光，不做成发光贴片
  ledGreen: register(
    new THREE.MeshStandardMaterial({
      color: 0x0d1710,
      emissive: 0x2fbe57,
      emissiveIntensity: 0.5,
      roughness: 0.3,
    })
  ),
  ledRed: register(
    new THREE.MeshStandardMaterial({
      color: 0x160d0e,
      emissive: 0xd8323f,
      emissiveIntensity: 0.5,
      roughness: 0.3,
    })
  ),
};

/** 线框模式开关（用于展示网格拓扑）。 */
export function setWireframe(enabled) {
  for (const material of registry) {
    material.wireframe = enabled;
  }
}
