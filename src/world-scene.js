/** 场景与机体常数（不依赖视觉模型）。 */

export const DRONE_RADIUS = 0.28;
export const MASS = 0.85;
export const GRAVITY = 9.81;
export const HOVER_THRUST = (MASS * GRAVITY) / 4;
export const YAW_GEAR = 0.018;

export const WORLD = {
  bounds: {
    min: [-10, 0.35, -10],
    max: [10, 6.0, 10],
  },
  start: [-8.0, 1.4, -8.0],
  goal: [8.0, 1.8, 8.0],
  obstacles: [
    { name: 'tower_c', pos: [0.0, 1.7, 0.2], size: [2.2, 3.4, 2.0] },
    { name: 'block_a', pos: [-3.6, 1.4, -2.2], size: [2.4, 2.8, 1.8] },
    { name: 'block_b', pos: [3.8, 1.8, -3.0], size: [2.0, 3.6, 2.2] },
    { name: 'wall_n', pos: [-1.2, 1.1, 4.4], size: [5.5, 2.2, 1.1] },
    { name: 'pillar_1', pos: [5.4, 1.6, 2.2], size: [1.3, 3.2, 1.3] },
    { name: 'pillar_2', pos: [-6.2, 1.3, 1.6], size: [1.4, 2.6, 1.6] },
    { name: 'low_bar', pos: [2.2, 0.7, 6.2], size: [4.0, 1.4, 1.2] },
    { name: 'gate_l', pos: [-5.0, 1.5, -5.4], size: [1.0, 3.0, 2.6] },
    { name: 'gate_r', pos: [-2.4, 1.5, -5.4], size: [1.0, 3.0, 2.6] },
  ],
};
