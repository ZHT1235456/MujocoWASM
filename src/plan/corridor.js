import { DRONE_RADIUS } from '../world.js';
import { collides, clearance } from './collide.js';

/**
 * 沿采样曲线膨胀安全走廊：每个点二分最大无碰半径。
 */
export function inflateCorridor(samples, options = {}) {
  const rMin = options.rMin ?? DRONE_RADIUS + 0.05;
  const rMax = options.rMax ?? 0.8;
  const radii = [];
  let minR = Infinity;

  for (const p of samples) {
    if (collides(p, rMin)) {
      radii.push(rMin);
      minR = Math.min(minR, 0);
      continue;
    }
    let lo = rMin;
    let hi = rMax;
    for (let i = 0; i < 10; i++) {
      const mid = 0.5 * (lo + hi);
      if (collides(p, mid)) hi = mid;
      else lo = mid;
    }
    const cap = Math.min(lo, clearance(p) * 0.98);
    const r = Math.max(rMin, Math.min(rMax, cap));
    radii.push(r);
    minR = Math.min(minR, r);
  }

  return { radii, minClearance: minR === Infinity ? 0 : minR };
}

export function corridorRadiusAt(s, sTable, radii) {
  if (!sTable?.length) return radii[0] ?? 0.3;
  if (s <= sTable[0]) return radii[0];
  if (s >= sTable[sTable.length - 1]) return radii[radii.length - 1];
  let i = 1;
  while (i < sTable.length && sTable[i] < s) i++;
  const t = (s - sTable[i - 1]) / Math.max(1e-6, sTable[i] - sTable[i - 1]);
  return radii[i - 1] * (1 - t) + radii[i] * t;
}

export function pointOnPath(s, sTable, samples, tangents) {
  if (s <= 0) return { p: samples[0], t: tangents[0] };
  const last = sTable.length - 1;
  if (s >= sTable[last]) return { p: samples[last], t: tangents[last] };
  let i = 1;
  while (i < sTable.length && sTable[i] < s) i++;
  const u = (s - sTable[i - 1]) / Math.max(1e-6, sTable[i] - sTable[i - 1]);
  return {
    p: [
      samples[i - 1][0] + (samples[i][0] - samples[i - 1][0]) * u,
      samples[i - 1][1] + (samples[i][1] - samples[i - 1][1]) * u,
      samples[i - 1][2] + (samples[i][2] - samples[i - 1][2]) * u,
    ],
    t: [
      tangents[i - 1][0] + (tangents[i][0] - tangents[i - 1][0]) * u,
      tangents[i - 1][1] + (tangents[i][1] - tangents[i - 1][1]) * u,
      tangents[i - 1][2] + (tangents[i][2] - tangents[i - 1][2]) * u,
    ],
  };
}
