/** 轴对齐超矩形与论文 Lemma 8 / 9 / Corollary 4 的集合运算。 */

export function aabbFromCenterSize(pos, size) {
  return {
    min: [pos[0] - size[0] / 2, pos[1] - size[1] / 2, pos[2] - size[2] / 2],
    max: [pos[0] + size[0] / 2, pos[1] + size[1] / 2, pos[2] + size[2] / 2],
  };
}

export function aabbFromCenterRadius(c, r) {
  return {
    min: [c[0] - r[0], c[1] - r[1], c[2] - r[2]],
    max: [c[0] + r[0], c[1] + r[1], c[2] + r[2]],
  };
}

export function centerOf(aabb) {
  return [
    0.5 * (aabb.min[0] + aabb.max[0]),
    0.5 * (aabb.min[1] + aabb.max[1]),
    0.5 * (aabb.min[2] + aabb.max[2]),
  ];
}

export function radiusOf(aabb) {
  return [
    0.5 * (aabb.max[0] - aabb.min[0]),
    0.5 * (aabb.max[1] - aabb.min[1]),
    0.5 * (aabb.max[2] - aabb.min[2]),
  ];
}

export function inflateAabb(aabb, m) {
  const pad = typeof m === 'number' ? [m, m, m] : m;
  return {
    min: [aabb.min[0] - pad[0], aabb.min[1] - pad[1], aabb.min[2] - pad[2]],
    max: [aabb.max[0] + pad[0], aabb.max[1] + pad[1], aabb.max[2] + pad[2]],
  };
}

export function deflateAabb(aabb, m) {
  return inflateAabb(aabb, typeof m === 'number' ? -m : m.map((v) => -v));
}

export function contains(aabb, p) {
  return (
    p[0] >= aabb.min[0] &&
    p[0] <= aabb.max[0] &&
    p[1] >= aabb.min[1] &&
    p[1] <= aabb.max[1] &&
    p[2] >= aabb.min[2] &&
    p[2] <= aabb.max[2]
  );
}

/** Whether p lies in the strict interior of an axis-aligned box. */
export function containsInterior(aabb, p) {
  return (
    p[0] > aabb.min[0] &&
    p[0] < aabb.max[0] &&
    p[1] > aabb.min[1] &&
    p[1] < aabb.max[1] &&
    p[2] > aabb.min[2] &&
    p[2] < aabb.max[2]
  );
}

export function intersectAabb(a, b) {
  const min = [Math.max(a.min[0], b.min[0]), Math.max(a.min[1], b.min[1]), Math.max(a.min[2], b.min[2])];
  const max = [Math.min(a.max[0], b.max[0]), Math.min(a.max[1], b.max[1]), Math.min(a.max[2], b.max[2])];
  if (min[0] > max[0] || min[1] > max[1] || min[2] > max[2]) return null;
  return { min, max };
}

export function volume(aabb) {
  const r = radiusOf(aabb);
  return 8 * r[0] * r[1] * r[2];
}

export function isValidAabb(aabb, minR = 1e-4) {
  if (!aabb) return false;
  const r = radiusOf(aabb);
  return r[0] > minR && r[1] > minR && r[2] > minR;
}

/** 无穷范数投影到盒子上（论文 CP）。 */
export function closestPoint(p, aabb) {
  return [
    Math.min(aabb.max[0], Math.max(aabb.min[0], p[0])),
    Math.min(aabb.max[1], Math.max(aabb.min[1], p[1])),
    Math.min(aabb.max[2], Math.max(aabb.min[2], p[2])),
  ];
}

export function infDist(a, b) {
  return Math.max(Math.abs(a[0] - b[0]), Math.abs(a[1] - b[1]), Math.abs(a[2] - b[2]));
}

/** Lemma 8：盒子内以 v 为中心的最大内接超矩形。 */
export function maxInnerRect(v, aabb) {
  const c = centerOf(aabb);
  const r = radiusOf(aabb);
  const nr = [r[0] - Math.abs(c[0] - v[0]), r[1] - Math.abs(c[1] - v[1]), r[2] - Math.abs(c[2] - v[2])];
  if (nr[0] <= 0 || nr[1] <= 0 || nr[2] <= 0) return null;
  return aabbFromCenterRadius(v, nr);
}

/**
 * Lemma 9 的排除板：在最大无穷范数间隙的维度上切一刀，
 * 使该板与障碍盒子不相交（α < 1）。
 */
export function excludingSlab(x, obstacle, alpha) {
  if (contains(obstacle, x)) return null;
  const y = closestPoint(x, obstacle);
  const d = infDist(x, y);
  if (d <= 1e-9) return null;
  let axis = 0;
  let best = Math.abs(x[0] - y[0]);
  for (let i = 1; i < 3; i++) {
    const di = Math.abs(x[i] - y[i]);
    if (di > best) {
      best = di;
      axis = i;
    }
  }
  const half = alpha * d;
  const min = [-1e6, -1e6, -1e6];
  const max = [1e6, 1e6, 1e6];
  min[axis] = x[axis] - half;
  max[axis] = x[axis] + half;
  return { min, max };
}

/**
 * Corollary 4：时刻 t 上、包含 y 且避开膨胀障碍的安全超矩形。
 */
export function safeRect(y, domain, obstacles, alpha) {
  if (!contains(domain, y)) return null;
  for (const obs of obstacles) {
    if (contains(obs, y)) return null;
  }
  let rect = maxInnerRect(y, domain);
  if (!rect) return null;
  for (const obs of obstacles) {
    const slab = excludingSlab(y, obs, alpha);
    if (!slab) return null;
    rect = intersectAabb(rect, slab);
    if (!isValidAabb(rect)) return null;
  }
  return rect;
}

export function sampleInAabb(aabb) {
  return [
    aabb.min[0] + Math.random() * (aabb.max[0] - aabb.min[0]),
    aabb.min[1] + Math.random() * (aabb.max[1] - aabb.min[1]),
    aabb.min[2] + Math.random() * (aabb.max[2] - aabb.min[2]),
  ];
}

export function sampleFree(domain, obstacles, tries = 40) {
  for (let i = 0; i < tries; i++) {
    const p = sampleInAabb(domain);
    if (obstacles.every((obs) => !contains(obs, p))) return p;
  }
  return sampleInAabb(domain);
}
