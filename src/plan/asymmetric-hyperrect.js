import { contains, containsInterior, intersectAabb } from './hyperrect.js';

const AXES = [0, 1, 2];
const VOLUME_EPSILON = 1e-12;

function uniqueSorted(values) {
  return [...new Set(values)].sort((a, b) => a - b);
}

function hasInterior(aabb) {
  return AXES.every((axis) => aabb.max[axis] > aabb.min[axis]);
}

function clipObstacles(domain, obstacles) {
  const clipped = [];
  for (const obstacle of obstacles) {
    const intersection = intersectAabb(domain, obstacle);
    if (intersection && hasInterior(intersection)) clipped.push(intersection);
  }
  return clipped;
}

function eventBounds(point, domain, obstacles, axis) {
  const lower = [domain.min[axis]];
  const upper = [domain.max[axis]];
  for (const obstacle of obstacles) {
    const obstacleUpper = obstacle.max[axis];
    const obstacleLower = obstacle.min[axis];
    if (obstacleUpper >= domain.min[axis] && obstacleUpper <= point[axis]) lower.push(obstacleUpper);
    if (obstacleLower >= point[axis] && obstacleLower <= domain.max[axis]) upper.push(obstacleLower);
  }
  return { lower: uniqueSorted(lower), upper: uniqueSorted(upper) };
}

function obstacleOrder(a, b, scanAxis, spanAxis) {
  return (
    a.min[scanAxis] - b.min[scanAxis] ||
    a.max[scanAxis] - b.max[scanAxis] ||
    a.min[spanAxis] - b.min[spanAxis] ||
    a.max[spanAxis] - b.max[spanAxis]
  );
}

/** Exact O(N^2) two-dimensional scan from the asymmetric-box derivation. */
function solve2d(point, domain, obstacles, spanAxis, scanAxis) {
  const events = eventBounds(point, domain, obstacles, scanAxis);
  const ordered = obstacles.slice().sort((a, b) => obstacleOrder(a, b, scanAxis, spanAxis));
  let best = null;
  let bestArea = -Infinity;

  for (const lower of events.lower) {
    let spanLower = domain.min[spanAxis];
    let spanUpper = domain.max[spanAxis];
    let pointer = 0;
    let infeasible = false;

    for (const upper of events.upper) {
      while (pointer < ordered.length && ordered[pointer].min[scanAxis] < upper) {
        const obstacle = ordered[pointer++];
        if (obstacle.max[scanAxis] <= lower) continue;

        if (obstacle.max[spanAxis] <= point[spanAxis]) {
          spanLower = Math.max(spanLower, obstacle.max[spanAxis]);
        } else if (obstacle.min[spanAxis] >= point[spanAxis]) {
          spanUpper = Math.min(spanUpper, obstacle.min[spanAxis]);
        } else {
          infeasible = true;
        }
      }

      if (infeasible) break;
      const area = (spanUpper - spanLower) * (upper - lower);
      if (area > bestArea + VOLUME_EPSILON) {
        bestArea = area;
        best = { spanLower, spanUpper, scanLower: lower, scanUpper: upper };
      }
    }
  }

  return best ? { ...best, area: bestArea } : null;
}

function selectOuterAxis(point, domain, obstacles) {
  let bestAxis = 0;
  let bestEvents = Infinity;
  for (const axis of AXES) {
    const events = eventBounds(point, domain, obstacles, axis);
    const count = events.lower.length * events.upper.length;
    if (count < bestEvents) {
      bestEvents = count;
      bestAxis = axis;
    }
  }
  return bestAxis;
}

/**
 * Return the maximum-volume safe AABB containing point. The box may touch the
 * boundary of the domain or an obstacle, but their strict interiors never overlap.
 */
export function safeRectAsymmetric(point, domain, obstacles) {
  if (!domain || !contains(domain, point) || !hasInterior(domain)) return null;
  const clipped = clipObstacles(domain, obstacles ?? []);
  if (clipped.some((obstacle) => containsInterior(obstacle, point))) return null;

  const outerAxis = selectOuterAxis(point, domain, clipped);
  const [spanAxis, scanAxis] = AXES.filter((axis) => axis !== outerAxis);
  const outerEvents = eventBounds(point, domain, clipped, outerAxis);
  let best = null;
  let bestVolume = -Infinity;

  for (const outerLower of outerEvents.lower) {
    for (const outerUpper of outerEvents.upper) {
      const active = clipped.filter(
        (obstacle) => outerLower < obstacle.max[outerAxis] && outerUpper > obstacle.min[outerAxis]
      );
      const crossSection = solve2d(point, domain, active, spanAxis, scanAxis);
      if (!crossSection) continue;

      const volume = crossSection.area * (outerUpper - outerLower);
      if (volume <= bestVolume + VOLUME_EPSILON) continue;

      const min = domain.min.slice();
      const max = domain.max.slice();
      min[outerAxis] = outerLower;
      max[outerAxis] = outerUpper;
      min[spanAxis] = crossSection.spanLower;
      max[spanAxis] = crossSection.spanUpper;
      min[scanAxis] = crossSection.scanLower;
      max[scanAxis] = crossSection.scanUpper;
      best = { min, max };
      bestVolume = volume;
    }
  }

  return best && hasInterior(best) ? best : null;
}
