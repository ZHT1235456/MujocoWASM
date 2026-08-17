import * as THREE from 'three';
import { WORLD, DRONE_RADIUS } from '../world.js';

function aabbFromObstacle(obs, inflate = 0) {
  return {
    min: [
      obs.pos[0] - obs.size[0] / 2 - inflate,
      obs.pos[1] - obs.size[1] / 2 - inflate,
      obs.pos[2] - obs.size[2] / 2 - inflate,
    ],
    max: [
      obs.pos[0] + obs.size[0] / 2 + inflate,
      obs.pos[1] + obs.size[1] / 2 + inflate,
      obs.pos[2] + obs.size[2] / 2 + inflate,
    ],
  };
}

export function pointInAabb(p, aabb) {
  return (
    p[0] >= aabb.min[0] &&
    p[0] <= aabb.max[0] &&
    p[1] >= aabb.min[1] &&
    p[1] <= aabb.max[1] &&
    p[2] >= aabb.min[2] &&
    p[2] <= aabb.max[2]
  );
}

export function inBounds(p, bounds = WORLD.bounds, radius = 0) {
  return (
    p[0] >= bounds.min[0] + radius &&
    p[0] <= bounds.max[0] - radius &&
    p[1] >= bounds.min[1] + radius &&
    p[1] <= bounds.max[1] - radius &&
    p[2] >= bounds.min[2] + radius &&
    p[2] <= bounds.max[2] - radius
  );
}

export function collides(p, radius = DRONE_RADIUS, obstacles = WORLD.obstacles) {
  if (!inBounds(p, WORLD.bounds, 0.02)) return true;
  if (p[1] - radius < 0.05) return true;
  for (const obs of obstacles) {
    if (pointInAabb(p, aabbFromObstacle(obs, radius))) return true;
  }
  return false;
}

export function segmentCollides(a, b, radius = DRONE_RADIUS, step = 0.12) {
  const dx = b[0] - a[0];
  const dy = b[1] - a[1];
  const dz = b[2] - a[2];
  const len = Math.hypot(dx, dy, dz);
  const n = Math.max(2, Math.ceil(len / step));
  for (let i = 0; i <= n; i++) {
    const t = i / n;
    if (collides([a[0] + dx * t, a[1] + dy * t, a[2] + dz * t], radius)) return true;
  }
  return false;
}

export function clearance(p, obstacles = WORLD.obstacles) {
  let min = p[1];
  for (const obs of obstacles) {
    const aabb = aabbFromObstacle(obs, 0);
    const dx = Math.max(aabb.min[0] - p[0], 0, p[0] - aabb.max[0]);
    const dy = Math.max(aabb.min[1] - p[1], 0, p[1] - aabb.max[1]);
    const dz = Math.max(aabb.min[2] - p[2], 0, p[2] - aabb.max[2]);
    min = Math.min(min, Math.hypot(dx, dy, dz));
  }
  return min;
}

export function dist(a, b) {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

export function lerp(a, b, t) {
  return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
}

export function toVec3(p) {
  return new THREE.Vector3(p[0], p[1], p[2]);
}
