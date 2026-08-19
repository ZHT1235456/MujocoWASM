import { WORLD, DRONE_RADIUS } from '../world-scene.js';
import {
  aabbFromCenterSize,
  aabbFromCenterRadius,
  contains,
  containsInterior,
  centerOf,
  closestPoint,
  infDist,
  isValidAabb,
  radiusOf,
  inflateAabb,
  intersectAabb,
  deflateAabb,
  safeRect,
  sampleFree,
  sampleInAabb,
} from './hyperrect.js';
import { safeRectAsymmetric } from './asymmetric-hyperrect.js';

const RECT_MODES = new Set(['symmetric', 'asymmetric']);

function obstacleAabbs() {
  return WORLD.obstacles.map((obs) => aabbFromCenterSize(obs.pos, obs.size));
}

export function trackingBound(t, margin0, decay) {
  return margin0 * Math.exp(-decay * Math.max(0, t));
}

function setsAtTime(t, margin0, decay) {
  const lp = trackingBound(t, margin0, decay);
  const pad = lp + DRONE_RADIUS;
  const domain = deflateAabb(
    { min: WORLD.bounds.min.slice(), max: WORLD.bounds.max.slice() },
    pad
  );
  const obstacles = obstacleAabbs().map((obs) => inflateAabb(obs, pad));
  return { domain, obstacles, lp, pad };
}

function targetAabb(goal, half, t, margin0, decay) {
  const lp = trackingBound(t, margin0, decay);
  const raw = aabbFromCenterRadius(goal, half);
  const deflated = deflateAabb(raw, Math.min(lp, Math.min(half[0], half[1], half[2]) * 0.45));
  return isValidAabb(deflated) ? deflated : raw;
}

function safeRectForMode(point, sets, alpha, rectMode) {
  return rectMode === 'asymmetric'
    ? safeRectAsymmetric(point, sets.domain, sets.obstacles)
    : safeRect(point, sets.domain, sets.obstacles, alpha);
}

function obstacleContainsPoint(obstacle, point, rectMode) {
  return rectMode === 'asymmetric' ? containsInterior(obstacle, point) : contains(obstacle, point);
}

function tryExtend(nodes, parentIdx, xNew, alphaV, margin0, decay, alpha, rectMode) {
  const parent = nodes[parentIdx];
  const step = Math.hypot(xNew[0] - parent.p[0], xNew[1] - parent.p[1], xNew[2] - parent.p[2]);
  if (step < 1e-3) return -1;
  const tNew = parent.t + step / Math.max(0.15, alphaV);
  const nSets = setsAtTime(tNew, margin0, decay);
  if (!contains(nSets.domain, xNew) || nSets.obstacles.some((o) => obstacleContainsPoint(o, xNew, rectMode))) return -1;
  const nrect = safeRectForMode(xNew, nSets, alpha, rectMode);
  if (!isValidAabb(nrect)) return -1;
  const idx = nodes.length;
  nodes.push({ p: xNew, t: tNew, parent: parentIdx, rect: nrect });
  return idx;
}

function connectExactGoal(nodes, startIdx, goal, alphaV, margin0, decay, alpha, rectMode) {
  let current = startIdx;
  for (let hop = 0; hop < 48; hop++) {
    if (infDist(nodes[current].p, goal) < 1e-7) return current;
    const rect = nodes[current].rect;
    const next = contains(rect, goal) ? goal.slice() : closestPoint(goal, rect);
    const nextIdx = tryExtend(nodes, current, next, alphaV, margin0, decay, alpha, rectMode);
    if (nextIdx < 0) return -1;
    if (infDist(next, goal) < 1e-7) {
      nodes[nextIdx].t = Math.max(nodes[nextIdx].t, nodes[current].t + 4);
    }
    current = nextIdx;
  }
  return infDist(nodes[current].p, goal) < 1e-7 ? current : -1;
}

/**
 * Non-symmetric maximum boxes can place their anchor on a face, leaving the
 * direct goal projection equal to the anchor. Bridge through an overlap with a
 * conservative goal-centered safe box so the final LP still receives a
 * connected box chain.
 */
function bridgeExactGoal(nodes, goal, alphaV, margin0, decay, alpha, rectMode) {
  let best = null;
  for (let index = 0; index < nodes.length; index++) {
    const node = nodes[index];
    const goalSets = setsAtTime(node.t, margin0, decay);
    const goalRect = safeRectForMode(goal, goalSets, alpha, rectMode);
    if (!isValidAabb(goalRect)) continue;
    const overlap = intersectAabb(node.rect, goalRect);
    if (!isValidAabb(overlap, 1e-7)) continue;

    let bridge = closestPoint(goal, overlap);
    let step = Math.hypot(...bridge.map((value, axis) => value - node.p[axis]));
    if (step < 1e-3 && infDist(bridge, goal) > 1e-7) {
      bridge = centerOf(overlap);
      step = Math.hypot(...bridge.map((value, axis) => value - node.p[axis]));
    }
    if (step < 1e-3 && infDist(bridge, goal) > 1e-7) continue;

    const remaining = Math.hypot(...goal.map((value, axis) => value - bridge[axis]));
    const score = node.t + (step + remaining) / Math.max(0.15, alphaV);
    if (!best || score < best.score) best = { index, bridge, goalRect, score };
  }

  if (!best) return -1;
  let parent = best.index;
  if (infDist(best.bridge, goal) > 1e-7) {
    const from = nodes[parent];
    const step = Math.hypot(...best.bridge.map((value, axis) => value - from.p[axis]));
    const bridgeIndex = nodes.length;
    nodes.push({
      p: best.bridge,
      t: from.t + step / Math.max(0.15, alphaV),
      parent,
      rect: best.goalRect,
      preserveRect: true,
    });
    parent = bridgeIndex;
  }

  const from = nodes[parent];
  const step = Math.hypot(...goal.map((value, axis) => value - from.p[axis]));
  const goalIndex = nodes.length;
  nodes.push({
    p: goal.slice(),
    t: from.t + Math.max(4, step / Math.max(0.15, alphaV)),
    parent,
    rect: best.goalRect,
    preserveRect: true,
  });
  return goalIndex;
}

/**
 * 论文 Algorithm 1：在安全超矩形邻域上生长 RRT，得到带时间戳的盒子走廊。
 */
export async function planSafeTube(start, goal, options = {}) {
  const Nv = options.nv ?? 500;
  const alpha = options.alpha ?? 0.99;
  const alphaV = options.alphaV ?? 4;
  const cSample = options.cSample ?? 0.8;
  const margin0 = options.margin0 ?? 0.18;
  const decay = options.decay ?? 0.12;
  const targetHalf = options.targetHalf ?? [0.55, 0.45, 0.55];
  const rectMode = options.rectMode ?? 'symmetric';

  if (!RECT_MODES.has(rectMode)) {
    return { ok: false, nodes: [], boxes: [], rectMode, message: `未知安全盒模式：${rectMode}` };
  }

  const startSets = setsAtTime(0, margin0, decay);
  if (
    !contains(startSets.domain, start) ||
    startSets.obstacles.some((o) => obstacleContainsPoint(o, start, rectMode))
  ) {
    return { ok: false, nodes: [], boxes: [], rectMode, message: '起点不在收缩后的自由空间内' };
  }

  const nodes = [{ p: start.slice(), t: 0, parent: -1, rect: safeRectForMode(start, startSets, alpha, rectMode) }];
  if (!isValidAabb(nodes[0].rect)) {
    return { ok: false, nodes, boxes: [], rectMode, message: '起点安全超矩形为空，请减小误差裕度' };
  }

  const yieldFn = options.yieldFn;
  let goalIndex = -1;
  for (let i = 1; i <= Nv; i++) {
    if (yieldFn && i % 160 === 0) await yieldFn();
    const fromFree = i <= cSample * Nv;
    const xs = fromFree
      ? sampleFree(startSets.domain, startSets.obstacles)
      : sampleInAabb(targetAabb(goal, targetHalf, 0, margin0, decay));

    let best = 0;
    let bestD = Infinity;
    for (let j = 0; j < nodes.length; j++) {
      const rect = nodes[j].rect;
      if (!isValidAabb(rect)) continue;
      const d = infDist(xs, closestPoint(xs, rect));
      if (d < bestD) {
        bestD = d;
        best = j;
      }
    }

    const xNew = closestPoint(xs, nodes[best].rect);
    const idx = tryExtend(nodes, best, xNew, alphaV, margin0, decay, alpha, rectMode);
    if (idx < 0) continue;

    const target = targetAabb(goal, targetHalf, nodes[idx].t, margin0, decay);
    if (contains(target, nodes[idx].p) && contains(nodes[idx].rect, goal)) {
      goalIndex = idx;
      break;
    }

    if (i % 35 === 0) {
      let gBest = 0;
      let gD = Infinity;
      for (let j = 0; j < nodes.length; j++) {
        const d = infDist(nodes[j].p, goal);
        if (d < gD) {
          gD = d;
          gBest = j;
        }
      }
      const gIdx = tryExtend(
        nodes,
        gBest,
        closestPoint(goal, nodes[gBest].rect),
        alphaV,
        margin0,
        decay,
        alpha,
        rectMode
      );
      if (
        gIdx >= 0 &&
        contains(targetAabb(goal, targetHalf, nodes[gIdx].t, margin0, decay), nodes[gIdx].p) &&
        contains(nodes[gIdx].rect, goal)
      ) {
        goalIndex = gIdx;
        break;
      }
    }
  }

  if (goalIndex < 0) {
    let best = 0;
    let bestD = Infinity;
    for (let j = 0; j < nodes.length; j++) {
      const d = infDist(nodes[j].p, goal);
      if (d < bestD) {
        bestD = d;
        best = j;
      }
    }
    const attached = tryExtend(
      nodes,
      best,
      closestPoint(goal, nodes[best].rect),
      alphaV,
      margin0,
      decay,
      alpha,
      rectMode
    );
    if (attached >= 0 && infDist(nodes[attached].p, goal) < 1.25) goalIndex = attached;
    else if (bestD < 1.2) goalIndex = best;
  }

  if (goalIndex < 0) {
    return { ok: false, nodes, boxes: [], rectMode, message: '未连到目标，请增加迭代或放宽裕度' };
  }

  goalIndex = connectExactGoal(nodes, goalIndex, goal, alphaV, margin0, decay, alpha, rectMode);
  if (goalIndex < 0 && rectMode === 'asymmetric') {
    goalIndex = bridgeExactGoal(nodes, goal, alphaV, margin0, decay, alpha, rectMode);
  }
  if (goalIndex < 0) {
    return { ok: false, nodes, boxes: [], rectMode, message: '已接近目标，但无法生成到精确终点的安全盒链' };
  }
  const goalParent = nodes[goalIndex].parent;
  if (goalParent >= 0) nodes[goalIndex].t = Math.max(nodes[goalIndex].t, nodes[goalParent].t + 4);

  const path = [];
  let cur = goalIndex;
  while (cur >= 0) {
    path.push(nodes[cur]);
    cur = nodes[cur].parent;
  }
  path.reverse();

  const boxes = path.map((node, i) => {
    const sets = setsAtTime(node.t, margin0, decay);
    const rect =
      i === path.length - 1 || node.preserveRect
        ? node.rect
        : safeRectForMode(node.p, sets, alpha, rectMode) || node.rect;
    return {
      p: node.p.slice(),
      t: node.t,
      r: radiusOf(rect),
      aabb: rect,
      lp: trackingBound(node.t, margin0, decay),
    };
  });

  return {
    ok: true,
    nodes: nodes.map((n) => ({ p: n.p, parent: n.parent, t: n.t })),
    boxes,
    path: boxes.map((b) => b.p),
    times: boxes.map((b) => b.t),
    duration: boxes[boxes.length - 1].t,
    minClearance: Math.min(...boxes.map((b) => Math.min(...b.r))),
    alphaV,
    margin0,
    rectMode,
  };
}
