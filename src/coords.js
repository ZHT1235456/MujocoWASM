import * as THREE from 'three';

/** MuJoCo Z-up (x,y,z) → Three.js Y-up (x, z, -y) */

export function mjPosToThree(x, y, z, target = new THREE.Vector3()) {
  return target.set(x, z, -y);
}

export function threePosToMj(x, y, z) {
  return [x, -z, y];
}

export function mjQuatToThree(w, x, y, z, target = new THREE.Quaternion()) {
  return target.set(-x, z, -y, w);
}

export function readMjPos(buffer, index, target = new THREE.Vector3()) {
  const i = index * 3;
  return mjPosToThree(buffer[i], buffer[i + 1], buffer[i + 2], target);
}

export function readMjQuat(buffer, index, target = new THREE.Quaternion()) {
  const i = index * 4;
  return mjQuatToThree(buffer[i], buffer[i + 1], buffer[i + 2], buffer[i + 3], target);
}

export function vec3ToArr(v) {
  return [v.x, v.y, v.z];
}
