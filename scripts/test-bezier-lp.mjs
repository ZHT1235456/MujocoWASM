import { planSafeTube } from '../src/plan/rrt-tube.js';
import { bezierLpInTube } from '../src/plan/bezier-lp.js';

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

// This deterministic tube has a tight C4 transition.  It used to reach the
// old 16-attempt cutoff (4.86x duration) and incorrectly report infeasibility.
let seed = 11;
Math.random = () => {
  seed = (1664525 * seed + 1013904223) >>> 0;
  return seed / 0x100000000;
};

const planned = await planSafeTube([-8, 1.4, -8], [8, 1.8, 8], {
  nv: 5000,
  margin0: 0.18,
  alphaV: 4,
});
assert(planned.ok, planned.message || 'regression tube planning failed');

const trajectory = await bezierLpInTube(planned.boxes, { np: 9 });
assert(trajectory.ok, trajectory.message || 'extended LP duration search failed');
assert(trajectory.attempts > 16, `regression no longer exercises the old cutoff: ${trajectory.attempts}`);
assert(trajectory.timeScale > 4.86, `expected duration scale above old limit, got ${trajectory.timeScale}`);

console.log('ok extended Algorithm 2 duration search', {
  boxes: planned.boxes.length,
  attempts: trajectory.attempts,
  timeScale: +trajectory.timeScale.toFixed(3),
});
