import { safeRectAsymmetric } from '../src/plan/asymmetric-hyperrect.js';
import { contains, safeRect, volume } from '../src/plan/hyperrect.js';

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function assertAabb(actual, expected, message) {
  assert(actual, `${message}: expected an AABB`);
  assert(
    actual.min.every((value, axis) => Math.abs(value - expected.min[axis]) < 1e-9) &&
      actual.max.every((value, axis) => Math.abs(value - expected.max[axis]) < 1e-9),
    `${message}: got ${JSON.stringify(actual)}, expected ${JSON.stringify(expected)}`
  );
}

function interiorsOverlap(a, b) {
  return [0, 1, 2].every((axis) => a.min[axis] < b.max[axis] && a.max[axis] > b.min[axis]);
}

function assertSafe(rect, point, domain, obstacles, message) {
  assert(rect.min.every((value, axis) => value >= domain.min[axis]), `${message}: below domain`);
  assert(rect.max.every((value, axis) => value <= domain.max[axis]), `${message}: above domain`);
  assert(contains(rect, point), `${message}: does not contain the given point`);
  assert(obstacles.every((obstacle) => !interiorsOverlap(rect, obstacle)), `${message}: overlaps an obstacle interior`);
}

const domain = { min: [0, 0, 0], max: [10, 10, 10] };

const empty = safeRectAsymmetric([2, 5, 5], domain, []);
assertAabb(empty, domain, 'an empty workspace should return the full domain');

const point = [1, 5, 5];
const oneSided = [{ min: [4, 0, 0], max: [5, 10, 10] }];
const asymmetric = safeRectAsymmetric(point, domain, oneSided);
const symmetric = safeRect(point, domain, oneSided, 0.99);
assertAabb(asymmetric, { min: [0, 0, 0], max: [4, 10, 10] }, 'one-sided obstacle result');
assert(volume(asymmetric) > volume(symmetric), 'the asymmetric box should improve on the symmetric box');
assertSafe(asymmetric, point, domain, oneSided, 'one-sided obstacle result');

const walls = [
  { min: [1, 0, 0], max: [2, 10, 10] },
  { min: [8, 0, 0], max: [9, 10, 10] },
  { min: [0, 2, 0], max: [10, 3, 10] },
  { min: [0, 7, 0], max: [10, 8, 10] },
  { min: [0, 0, 0], max: [10, 10, 1] },
  { min: [0, 0, 9], max: [10, 10, 10] },
];
const enclosed = safeRectAsymmetric([5, 5, 5], domain, walls);
assertAabb(enclosed, { min: [2, 3, 1], max: [8, 7, 9] }, 'multi-obstacle exact result');
assertSafe(enclosed, [5, 5, 5], domain, walls, 'multi-obstacle exact result');

const clipped = safeRectAsymmetric([5, 5, 5], domain, [
  { min: [20, 20, 20], max: [21, 21, 21] },
  { min: [-2, -2, -2], max: [0, 12, 12] },
]);
assertAabb(clipped, domain, 'zero-volume and out-of-domain obstacle clipping');

const touchingObstacle = [{ min: [4, 0, 0], max: [6, 10, 10] }];
const touching = safeRectAsymmetric([4, 5, 5], domain, touchingObstacle);
assertAabb(touching, { min: [0, 0, 0], max: [4, 10, 10] }, 'boundary contact result');
assertSafe(touching, [4, 5, 5], domain, touchingObstacle, 'boundary contact result');

assert(
  safeRectAsymmetric([5, 5, 5], domain, [{ min: [4, 4, 4], max: [6, 6, 6] }]) === null,
  'a point in an obstacle interior should be rejected'
);
assert(safeRectAsymmetric([11, 5, 5], domain, []) === null, 'a point outside the domain should be rejected');

console.log('all asymmetric hyperrectangle tests passed');
