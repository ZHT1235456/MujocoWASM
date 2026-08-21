import * as THREE from 'three';

const toVec3 = (p) => new THREE.Vector3(p[0], p[1], p[2]);

export function createCorridorView(scene) {
  const group = new THREE.Group();
  group.name = 'corridorView';
  scene.add(group);
  return group;
}

function disposeGroup(group) {
  const doomed = [...group.children];
  for (const child of doomed) {
    group.remove(child);
    child.traverse((obj) => {
      if (obj.geometry) obj.geometry.dispose();
      if (obj.material) {
        if (Array.isArray(obj.material)) obj.material.forEach((m) => m.dispose());
        else obj.material.dispose();
      }
    });
  }
}

export function drawTree(group, nodes, visible) {
  const old = group.getObjectByName('rrtTree');
  if (old) {
    group.remove(old);
    old.geometry?.dispose();
    old.material?.dispose();
  }
  if (!visible || !nodes?.length) return;
  const positions = [];
  for (const node of nodes) {
    if (node.parent < 0) continue;
    const a = node.p;
    const b = nodes[node.parent].p;
    positions.push(a[0], a[1], a[2], b[0], b[1], b[2]);
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  const line = new THREE.LineSegments(
    geo,
    new THREE.LineBasicMaterial({ color: 0x6ec8ff, transparent: true, opacity: 0.85 })
  );
  line.name = 'rrtTree';
  group.add(line);
}

export function drawCorridor(group, samples, radii, options = {}) {
  disposeGroup(group);
  const hasPath = samples && samples.length >= 2;
  const hasBoxes = !!options.boxes?.length;
  if (!hasPath && !hasBoxes) return;

  const showCenter = options.centerline !== false;
  const showTube = options.corridor !== false;
  const violated = options.violated ?? false;
  const tubeColor = violated ? 0xd64545 : 0x3ec7c2;

  if (showCenter && hasPath) {
    const pts = samples.map(toVec3);
    const curve = new THREE.CatmullRomCurve3(pts);
    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(curve.getPoints(Math.min(400, samples.length * 2))),
      new THREE.LineBasicMaterial({ color: 0xf7941e })
    );
    line.name = 'centerline';
    group.add(line);
  }

  if (showTube) {
    if (options.boxes?.length) {
      drawBoxes(group, options.boxes, tubeColor);
    } else {
      const pts = samples.map(toVec3);
      const curve = new THREE.CatmullRomCurve3(pts);
      const meanR = radii.reduce((a, b) => a + b, 0) / radii.length;
      const tube = new THREE.Mesh(
        new THREE.TubeGeometry(curve, Math.min(240, samples.length), Math.max(0.12, meanR * 0.92), 14, false),
        new THREE.MeshPhysicalMaterial({
          color: tubeColor,
          transparent: true,
          opacity: 0.16,
          roughness: 0.35,
          metalness: 0.05,
          side: THREE.DoubleSide,
          depthWrite: false,
        })
      );
      tube.name = 'tube';
      group.add(tube);

      for (let i = 0; i < samples.length; i += Math.max(1, Math.floor(samples.length / 18))) {
        const tangent = new THREE.Vector3();
        if (i < samples.length - 1) {
          tangent.set(
            samples[i + 1][0] - samples[i][0],
            samples[i + 1][1] - samples[i][1],
            samples[i + 1][2] - samples[i][2]
          ).normalize();
        } else {
          tangent.set(
            samples[i][0] - samples[i - 1][0],
            samples[i][1] - samples[i - 1][1],
            samples[i][2] - samples[i - 1][2]
          ).normalize();
        }
        const ring = new THREE.Mesh(
          new THREE.TorusGeometry(radii[i], 0.012, 8, 28),
          new THREE.MeshBasicMaterial({ color: tubeColor, transparent: true, opacity: 0.55 })
        );
        ring.position.set(...samples[i]);
        ring.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), tangent);
        group.add(ring);
      }
    }
  }
}

function drawBoxes(group, boxes, color) {
  for (let boxIndex = 0; boxIndex < boxes.length; boxIndex++) {
    const box = boxes[boxIndex];
    const aabb = box.aabb;
    if (!aabb) continue;
    const size = [aabb.max[0] - aabb.min[0], aabb.max[1] - aabb.min[1], aabb.max[2] - aabb.min[2]];
    const center = [
      0.5 * (aabb.min[0] + aabb.max[0]),
      0.5 * (aabb.min[1] + aabb.max[1]),
      0.5 * (aabb.min[2] + aabb.max[2]),
    ];
    const geo = new THREE.BoxGeometry(size[0], size[1], size[2]);
    const mesh = new THREE.Mesh(
      geo,
      new THREE.MeshPhysicalMaterial({
        color,
        transparent: true,
        opacity: 0.13,
        roughness: 0.4,
        metalness: 0.04,
        side: THREE.DoubleSide,
        depthWrite: false,
      })
    );
    mesh.position.set(...center);
    mesh.name = 'tubeBox';
    mesh.userData.boxIndex = boxIndex;
    group.add(mesh);

    const edges = new THREE.LineSegments(
      new THREE.EdgesGeometry(geo),
      new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.5 })
    );
    edges.position.set(...center);
    edges.name = 'tubeBoxEdge';
    edges.userData.boxIndex = boxIndex;
    group.add(edges);
  }
}

export function setCorridorViolated(group, violated, boxIndex = -1) {
  group.traverse((obj) => {
    if (obj.material && obj.name !== 'centerline') {
      const isViolatedBox = violated && (boxIndex < 0 || obj.userData.boxIndex === boxIndex);
      if (obj.material.color) obj.material.color.setHex(isViolatedBox ? 0xd64545 : 0x3ec7c2);
    }
  });
}
