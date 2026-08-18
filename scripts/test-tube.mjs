import * as THREE from 'three';
import { pointInTube, tubeBoxIndex } from '../src/plan/bezier-tube.js';
import { setCorridorViolated } from '../src/vis/corridor.js';

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

const boxes = [
  { t: 0, lp: 0, aabb: { min: [-1, -1, -1], max: [1, 1, 1] } },
  { t: 1, lp: 0, aabb: { min: [2, -1, -1], max: [4, 1, 1] } },
  { t: 2, lp: 0, aabb: { min: [5, -1, -1], max: [7, 1, 1] } },
];

assert(tubeBoxIndex(boxes, 0.5) === 0, 'time 0.5 should select box 0');
assert(tubeBoxIndex(boxes, 1) === 1, 'a segment boundary should select the new box');
assert(tubeBoxIndex(boxes, 2) === 2, 'the terminal time should select the terminal box');
assert(pointInTube([0, 0, 0], boxes, 0.5), 'point should be inside active box');
assert(!pointInTube([3, 0, 0], boxes, 0.5), 'a point in a future box is outside the active time slice');
assert(pointInTube([3, 0, 0], boxes, 1), 'point should enter the next box at its boundary');
assert(!pointInTube([8, 0, 0], boxes, 2), 'point outside the terminal box should be rejected');

const corridor = new THREE.Group();
for (let boxIndex = 0; boxIndex < 2; boxIndex++) {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), new THREE.MeshBasicMaterial());
  mesh.name = 'tubeBox';
  mesh.userData.boxIndex = boxIndex;
  corridor.add(mesh);
}
setCorridorViolated(corridor, true, 1);
assert(corridor.children[0].material.color.getHex() === 0x3ec7c2, 'unaffected boxes should stay green');
assert(corridor.children[1].material.color.getHex() === 0xd64545, 'only the active violated box should turn red');
setCorridorViolated(corridor, false);
assert(corridor.children.every((mesh) => mesh.material.color.getHex() === 0x3ec7c2), 'corridor should recover to green');

console.log('all tube membership tests passed');
