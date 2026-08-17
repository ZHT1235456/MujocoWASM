import { DRONE_RADIUS } from '../world.js';
import { collides, segmentCollides, dist, lerp } from './collide.js';

function nearest(nodes, p) {
  let best = 0;
  let bestD = Infinity;
  for (let i = 0; i < nodes.length; i++) {
    const d = dist(nodes[i].p, p);
    if (d < bestD) {
      bestD = d;
      best = i;
    }
  }
  return best;
}

function steer(from, to, step) {
  const d = dist(from, to);
  if (d <= step) return to.slice();
  const t = step / d;
  return lerp(from, to, t);
}

function nearIndices(nodes, p, radius) {
  const ids = [];
  const r2 = radius * radius;
  for (let i = 0; i < nodes.length; i++) {
    const q = nodes[i].p;
    const dx = q[0] - p[0];
    const dy = q[1] - p[1];
    const dz = q[2] - p[2];
    if (dx * dx + dy * dy + dz * dz <= r2) ids.push(i);
  }
  return ids;
}

/**
 * 3D RRT*。状态为 (x,y,z)，Y-up。
 */
export function planRrtStar(start, goal, options = {}) {
  const iters = options.iters ?? 2500;
  const step = options.step ?? 0.65;
  const goalBias = options.goalBias ?? 0.18;
  const radius = options.robotRadius ?? DRONE_RADIUS + 0.08;
  const bounds = options.bounds;
  const gamma = options.gamma ?? 2.2;

  const nodes = [{ p: start.slice(), parent: -1, cost: 0 }];
  let goalIndex = -1;

  const sample = () => {
    if (Math.random() < goalBias) return goal.slice();
    return [
      bounds.min[0] + Math.random() * (bounds.max[0] - bounds.min[0]),
      bounds.min[1] + Math.random() * (bounds.max[1] - bounds.min[1]),
      bounds.min[2] + Math.random() * (bounds.max[2] - bounds.min[2]),
    ];
  };

  for (let k = 0; k < iters; k++) {
    const xRand = sample();
    const nIdx = nearest(nodes, xRand);
    const xNew = steer(nodes[nIdx].p, xRand, step);
    if (collides(xNew, radius) || segmentCollides(nodes[nIdx].p, xNew, radius)) continue;

    const n = nodes.length;
    const r = Math.min(gamma * Math.pow(Math.log(n + 1) / (n + 1), 1 / 3) * 6, 2.4);
    const near = nearIndices(nodes, xNew, Math.max(r, step * 1.4));
    if (!near.includes(nIdx)) near.push(nIdx);

    let bestParent = nIdx;
    let bestCost = nodes[nIdx].cost + dist(nodes[nIdx].p, xNew);
    for (const i of near) {
      if (segmentCollides(nodes[i].p, xNew, radius)) continue;
      const c = nodes[i].cost + dist(nodes[i].p, xNew);
      if (c < bestCost) {
        bestCost = c;
        bestParent = i;
      }
    }

    const newIndex = nodes.length;
    nodes.push({ p: xNew, parent: bestParent, cost: bestCost });

    for (const i of near) {
      if (i === bestParent) continue;
      if (segmentCollides(xNew, nodes[i].p, radius)) continue;
      const c = bestCost + dist(xNew, nodes[i].p);
      if (c + 1e-6 < nodes[i].cost) {
        nodes[i].parent = newIndex;
        nodes[i].cost = c;
      }
    }

    if (dist(xNew, goal) < step * 1.2 && !segmentCollides(xNew, goal, radius)) {
      const c = bestCost + dist(xNew, goal);
      if (goalIndex < 0) {
        goalIndex = nodes.length;
        nodes.push({ p: goal.slice(), parent: newIndex, cost: c });
      } else if (c < nodes[goalIndex].cost) {
        nodes[goalIndex].parent = newIndex;
        nodes[goalIndex].cost = c;
      }
    }
  }

  if (goalIndex < 0) {
    const nearestGoal = nearest(nodes, goal);
    if (!segmentCollides(nodes[nearestGoal].p, goal, radius) && dist(nodes[nearestGoal].p, goal) < 2.5) {
      goalIndex = nodes.length;
      nodes.push({
        p: goal.slice(),
        parent: nearestGoal,
        cost: nodes[nearestGoal].cost + dist(nodes[nearestGoal].p, goal),
      });
    }
  }

  if (goalIndex < 0) {
    return { ok: false, nodes, path: [], edges: nodes };
  }

  const path = [];
  let cur = goalIndex;
  while (cur >= 0) {
    path.push(nodes[cur].p);
    cur = nodes[cur].parent;
  }
  path.reverse();
  return { ok: true, nodes, path, cost: nodes[goalIndex].cost };
}

export function shortcutPath(path, radius = DRONE_RADIUS + 0.08) {
  if (path.length < 3) return path;
  const out = [path[0]];
  let i = 0;
  while (i < path.length - 1) {
    let j = path.length - 1;
    while (j > i + 1 && segmentCollides(path[i], path[j], radius)) j--;
    out.push(path[j]);
    i = j;
  }
  return out;
}
