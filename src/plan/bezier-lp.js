import GLPK from 'glpk.js';

const GRAVITY = 9.81;

function binom(n, k) {
  if (k < 0 || k > n) return 0;
  let value = 1;
  for (let i = 1; i <= k; i++) value *= (n + 1 - i) / i;
  return value;
}

function fallingFactorial(n, order) {
  let value = 1;
  for (let i = 0; i < order; i++) value *= n - i;
  return value;
}

function cpName(segment, point, axis) {
  return `c_${segment}_${point}_${axis}`;
}

function devName(segment, point, axis) {
  return `d_${segment}_${point}_${axis}`;
}

function differenceTerms(segment, n, order, axis, atEnd, scale = 1) {
  const terms = [];
  for (let j = 0; j <= order; j++) {
    const point = atEnd ? n - order + j : j;
    const coefficient = (j % 2 === order % 2 ? 1 : -1) * binom(order, j) * scale;
    terms.push({ name: cpName(segment, point, axis), coef: coefficient });
  }
  return terms;
}

function linearPoint(a, b, u) {
  return [
    a[0] + (b[0] - a[0]) * u,
    a[1] + (b[1] - a[1]) * u,
    a[2] + (b[2] - a[2]) * u,
  ];
}

function buildProblem(glpk, boxes, durations, options) {
  const n = options.np;
  const { vmax, amax, epsilon } = options;
  const subjectTo = [];
  const bounds = [];
  const objectiveVars = [];
  let row = 0;
  const add = (prefix, vars, type, lb, ub) => {
    subjectTo.push({ name: `${prefix}_${row++}`, vars, bnds: { type, lb, ub } });
  };
  const fixed = (prefix, vars, value = 0) => add(prefix, vars, glpk.GLP_FX, value, value);

  for (let segment = 0; segment < boxes.length - 1; segment++) {
    const box = boxes[segment].aabb;
    for (let point = 0; point <= n; point++) {
      const nominal = linearPoint(boxes[segment].p, boxes[segment + 1].p, point / n);
      for (let axis = 0; axis < 3; axis++) {
        const name = cpName(segment, point, axis);
        let lb = box.min[axis];
        let ub = box.max[axis];
        if (segment === boxes.length - 2 && point === n) {
          lb = Math.max(lb, boxes.at(-1).aabb.min[axis]);
          ub = Math.min(ub, boxes.at(-1).aabb.max[axis]);
        }
        bounds.push({ name, type: glpk.GLP_DB, lb, ub });
        const dev = devName(segment, point, axis);
        bounds.push({ name: dev, type: glpk.GLP_LO, lb: 0, ub: 0 });
        objectiveVars.push({ name: dev, coef: 1 });
        add('dev_pos', [{ name, coef: 1 }, { name: dev, coef: -1 }], glpk.GLP_UP, 0, nominal[axis]);
        add('dev_neg', [{ name, coef: -1 }, { name: dev, coef: -1 }], glpk.GLP_UP, 0, -nominal[axis]);
      }
    }
  }

  // Paper (77)-(78): exact initial position, velocity, acceleration, and jerk.
  for (let axis = 0; axis < 3; axis++) {
    fixed('initial_position', [{ name: cpName(0, 0, axis), coef: 1 }], boxes[0].p[axis]);
    for (let order = 1; order <= 3; order++) fixed('initial_derivative', differenceTerms(0, n, order, axis, false));
  }

  // Paper (81)-(85): C4 continuity at every segment junction.
  for (let segment = 0; segment < boxes.length - 2; segment++) {
    for (let axis = 0; axis < 3; axis++) {
      for (let order = 0; order <= 4; order++) {
        const factor = fallingFactorial(n, order);
        const vars = [
          ...differenceTerms(segment, n, order, axis, true, factor / durations[segment] ** order),
          ...differenceTerms(segment + 1, n, order, axis, false, -factor / durations[segment + 1] ** order),
        ];
        fixed('continuity', vars);
      }
    }
  }

  // Paper (86)-(88): componentwise velocity, acceleration and positive-thrust bounds.
  for (let segment = 0; segment < boxes.length - 1; segment++) {
    const velocityScale = n / durations[segment];
    const accelerationScale = (n * (n - 1)) / durations[segment] ** 2;
    for (let axis = 0; axis < 3; axis++) {
      for (let point = 0; point < n; point++) {
        add('velocity', [
          { name: cpName(segment, point + 1, axis), coef: velocityScale },
          { name: cpName(segment, point, axis), coef: -velocityScale },
        ], glpk.GLP_DB, -vmax[axis], vmax[axis]);
      }
      for (let point = 0; point < n - 1; point++) {
        const vars = [
          { name: cpName(segment, point + 2, axis), coef: accelerationScale },
          { name: cpName(segment, point + 1, axis), coef: -2 * accelerationScale },
          { name: cpName(segment, point, axis), coef: accelerationScale },
        ];
        const gravity = axis === 1 ? GRAVITY : 0; // Three.js is Y-up.
        add('acceleration', vars, glpk.GLP_DB, -amax[axis] - gravity, amax[axis] - gravity);
        if (axis === 1) add('positive_thrust', vars, glpk.GLP_LO, epsilon - GRAVITY, 0);
      }
    }
  }

  // Demo extension: arrive at rest because it switches to a terminal hover.
  const lastSegment = boxes.length - 2;
  for (let axis = 0; axis < 3; axis++) {
    for (let order = 1; order <= 3; order++) fixed('terminal_derivative', differenceTerms(lastSegment, n, order, axis, true));
  }

  return {
    name: 'safe_bezier_trajectory',
    objective: { direction: glpk.GLP_MIN, name: 'path_deviation', vars: objectiveVars },
    subjectTo,
    bounds,
  };
}

function extractTrajectory(vars, boxes, durations, np, timeScale) {
  const segments = [];
  const times = [0];
  let t = 0;
  for (let segment = 0; segment < boxes.length - 1; segment++) {
    const ctrl = [];
    for (let point = 0; point <= np; point++) ctrl.push([0, 1, 2].map((axis) => vars[cpName(segment, point, axis)]));
    const t0 = t;
    t += durations[segment];
    times.push(t);
    segments.push({ ctrl, t0, t1: t, delta: durations[segment], box: boxes[segment] });
  }
  return { ok: true, segments, duration: t, np, times, timeScale, method: 'lp' };
}

/** Paper Algorithm 2: solve the LP, then reduce alpha_v by alpha_s if infeasible. */
export async function bezierLpInTube(boxes, options = {}) {
  if (!boxes || boxes.length < 2) return { ok: false, segments: [], message: '安全管至少需要两个盒子' };
  const np = options.np ?? 9;
  const alphaS = options.alphaS ?? 0.9;
  const maxAttempts = options.maxAttempts ?? 16;
  const solverOptions = {
    np,
    vmax: options.vmax ?? [2, 2, 2],
    amax: options.amax ?? [1, 10, 1],
    epsilon: options.epsilon ?? 0.05,
  };
  const baseDurations = boxes.slice(0, -1).map((box, i) => Math.max(0.12, boxes[i + 1].t - box.t));
  const glpk = await GLPK();

  for (let attempt = 0; attempt < maxAttempts; attempt++) {
    const timeScale = alphaS ** -attempt;
    const durations = baseDurations.map((delta) => delta * timeScale);
    const solved = await glpk.solve(buildProblem(glpk, boxes, durations, solverOptions), {
      msglev: glpk.GLP_MSG_OFF,
      presol: true,
      tmlim: 8,
    });
    if (solved.result.status === glpk.GLP_OPT || solved.result.status === glpk.GLP_FEAS) {
      return extractTrajectory(solved.result.vars, boxes, durations, np, timeScale);
    }
  }
  return { ok: false, segments: [], message: `Algorithm 2 LP 在 ${maxAttempts} 次时长缩放后仍不可行` };
}
