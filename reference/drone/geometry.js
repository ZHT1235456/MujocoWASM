import * as THREE from 'three';

/**
 * 程序化建模基础工具。
 *
 * 整机的曲面外壳（机身、机臂、起落架、桨叶）都由同一套流程生成：
 *   1. 用「关键站位表」描述截面参数沿轴向的变化；
 *   2. 用单调三次插值把关键站位加密成若干实际截面；
 *   3. 每个截面按超椭圆采样成一圈顶点；
 *   4. 相邻两圈顶点缝合成四边形带，两端按需封盖。
 */

const TMP_A = new THREE.Vector3();
const TMP_B = new THREE.Vector3();

/**
 * 超椭圆（squircle）采样。
 * exp = 2 时是标准椭圆；exp 越大截面越接近圆角矩形，用来表现注塑外壳的方中带圆。
 */
export function superellipsePoint(angle, halfWidth, halfHeight, exp) {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  const k = 2 / exp;
  const x = Math.sign(c) * Math.pow(Math.abs(c), k) * halfWidth;
  const y = Math.sign(s) * Math.pow(Math.abs(s), k) * halfHeight;
  return [x, y];
}

export function sampleSuperellipseRing(sec, segments) {
  const pts = [];
  for (let j = 0; j <= segments; j++) {
    const angle = (j / segments) * Math.PI * 2;
    const upper = Math.sin(angle) >= 0;
    const halfH = upper ? sec.hTop : sec.hBot;
    const exp = upper ? sec.expTop ?? 3 : sec.expBot ?? 3;
    const [sx, sy] = superellipsePoint(angle, sec.w, halfH, exp);
    pts.push([sx + (sec.xOff ?? 0), sy + (sec.yOff ?? 0)]);
  }
  return pts;
}

function resampleClosed(pts, segments) {
  const closed = [...pts];
  if (closed.length < 2) return sampleSuperellipseRing({ w: 0.01, hTop: 0.01, hBot: 0.01, yOff: 0 }, segments);
  const first = closed[0];
  const last = closed[closed.length - 1];
  if (Math.hypot(first[0] - last[0], first[1] - last[1]) > 1e-9) closed.push([first[0], first[1]]);

  const lengths = [];
  let total = 0;
  for (let i = 0; i < closed.length - 1; i++) {
    const d = Math.hypot(closed[i + 1][0] - closed[i][0], closed[i + 1][1] - closed[i][1]);
    lengths.push(d);
    total += d;
  }
  if (total < 1e-9) return sampleSuperellipseRing({ w: 0.01, hTop: 0.01, hBot: 0.01, yOff: 0 }, segments);

  const out = [];
  for (let j = 0; j <= segments; j++) {
    let target = (j / segments) * total;
    if (j === segments) target = total;
    let acc = 0;
    let x = closed[0][0];
    let y = closed[0][1];
    for (let i = 0; i < lengths.length; i++) {
      const next = acc + lengths[i];
      if (target <= next || i === lengths.length - 1) {
        const u = lengths[i] < 1e-12 ? 0 : (target - acc) / lengths[i];
        x = closed[i][0] + (closed[i + 1][0] - closed[i][0]) * u;
        y = closed[i][1] + (closed[i + 1][1] - closed[i][1]) * u;
        break;
      }
      acc = next;
    }
    out.push([x, y]);
  }
  return out;
}

function filletedLoop(corners, radii, arcSegs = 5) {
  const n = corners.length;
  const pts = [];
  for (let i = 0; i < n; i++) {
    const prev = corners[(i + n - 1) % n];
    const curr = corners[i];
    const next = corners[(i + 1) % n];
    const l1 = Math.hypot(prev[0] - curr[0], prev[1] - curr[1]);
    const l2 = Math.hypot(next[0] - curr[0], next[1] - curr[1]);
    const radius = Array.isArray(radii) ? radii[i] : radii;
    const r = Math.min(radius, l1 * 0.42, l2 * 0.42);
    if (r < 1e-5 || l1 < 1e-9 || l2 < 1e-9) {
      pts.push(curr);
      continue;
    }
    const p1 = [
      curr[0] + ((prev[0] - curr[0]) / l1) * r,
      curr[1] + ((prev[1] - curr[1]) / l1) * r,
    ];
    const p2 = [
      curr[0] + ((next[0] - curr[0]) / l2) * r,
      curr[1] + ((next[1] - curr[1]) / l2) * r,
    ];
    pts.push(p1);
    for (let k = 1; k < arcSegs; k++) {
      const t = k / arcSegs;
      const omt = 1 - t;
      pts.push([
        omt * omt * p1[0] + 2 * omt * t * curr[0] + t * t * p2[0],
        omt * omt * p1[1] + 2 * omt * t * curr[1] + t * t * p2[1],
      ]);
    }
    pts.push(p2);
  }
  return pts;
}

function raySegmentHit(ox, oy, dx, dy, ax, ay, bx, by) {
  const ex = bx - ax;
  const ey = by - ay;
  const denom = dx * ey - dy * ex;
  if (Math.abs(denom) < 1e-12) return null;
  const t = ((ax - ox) * ey - (ay - oy) * ex) / denom;
  const s = ((ax - ox) * dy - (ay - oy) * dx) / denom;
  if (t > 1e-8 && s >= -1e-5 && s <= 1 + 1e-5) return t;
  return null;
}

function sampleClosedByAngle(poly, segments, ox, oy) {
  const closed = [...poly];
  const first = closed[0];
  const last = closed[closed.length - 1];
  if (Math.hypot(first[0] - last[0], first[1] - last[1]) > 1e-9) closed.push([first[0], first[1]]);

  const out = [];
  for (let j = 0; j <= segments; j++) {
    const ang = (j / segments) * Math.PI * 2;
    const dx = Math.cos(ang);
    const dy = Math.sin(ang);
    let bestT = Infinity;
    let hx = ox + dx * 0.01;
    let hy = oy + dy * 0.01;
    for (let i = 0; i < closed.length - 1; i++) {
      const t = raySegmentHit(ox, oy, dx, dy, closed[i][0], closed[i][1], closed[i + 1][0], closed[i + 1][1]);
      if (t != null && t < bestT) {
        bestT = t;
        hx = ox + dx * t;
        hy = oy + dy * t;
      }
    }
    out.push([hx, hy]);
  }
  return out;
}

/**
 * 机身截面：顶面比最大宽度内收一圈，再向下落一个台阶。
 * 始终保持同一套 8 角拓扑，按极角采样，避免沿轴向换拓扑时出现折痕。
 */
export function steppedShellRing(sec, segments) {
  const yOff = sec.yOff ?? 0;
  const yTop = yOff + sec.hTop;
  const yBot = yOff - sec.hBot;
  const w = sec.w;
  const inset = Math.max(sec.inset ?? 0, 0.0005);
  const step = Math.max(sec.step ?? 0, 0.0005);
  const topFillet = sec.topFillet ?? 0.004;

  const ySh = yTop - Math.min(step, sec.hTop * 0.55);
  const xi = Math.max(w * 0.3, w - inset);
  const rStep = Math.min(0.0022, inset * 0.16 + 0.0005, step * 0.5 + 0.0005);
  const rBot = Math.min(sec.botFillet ?? 0.0075, w * 0.42, sec.hBot * 0.72);
  const rTop = Math.min(topFillet, xi * 0.72, sec.hTop * 0.7);

  const corners = [
    [w, yBot],
    [w, ySh],
    [xi, ySh],
    [xi, yTop],
    [-xi, yTop],
    [-xi, ySh],
    [-w, ySh],
    [-w, yBot],
  ];
  const radii = [rBot, rStep, rStep, rTop, rTop, rStep, rStep, rBot];
  const dense = filletedLoop(corners, radii, 8);
  return sampleClosedByAngle(dense, segments, 0, yOff);
}

/**
 * 单调三次（Fritsch–Carlson）插值。
 * 相比 Catmull-Rom 不会在关键站位之间过冲，机身不会鼓出多余的包。
 */
function monotoneSlopes(xs, ys) {
  const n = xs.length;
  const delta = new Array(n - 1);
  for (let i = 0; i < n - 1; i++) {
    delta[i] = (ys[i + 1] - ys[i]) / (xs[i + 1] - xs[i]);
  }
  const m = new Array(n);
  m[0] = delta[0];
  m[n - 1] = delta[n - 2];
  for (let i = 1; i < n - 1; i++) {
    if (delta[i - 1] * delta[i] <= 0) {
      m[i] = 0;
    } else {
      const w1 = 2 * (xs[i + 1] - xs[i]) + (xs[i] - xs[i - 1]);
      const w2 = (xs[i + 1] - xs[i]) + 2 * (xs[i] - xs[i - 1]);
      m[i] = (w1 + w2) / (w1 / delta[i - 1] + w2 / delta[i]);
    }
  }
  return m;
}

function evalMonotone(xs, ys, ms, x) {
  const n = xs.length;
  if (x <= xs[0]) return ys[0];
  if (x >= xs[n - 1]) return ys[n - 1];
  let lo = 0;
  let hi = n - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (xs[mid] <= x) lo = mid;
    else hi = mid;
  }
  const h = xs[hi] - xs[lo];
  const t = (x - xs[lo]) / h;
  const t2 = t * t;
  const t3 = t2 * t;
  return (
    (2 * t3 - 3 * t2 + 1) * ys[lo] +
    (t3 - 2 * t2 + t) * h * ms[lo] +
    (-2 * t3 + 3 * t2) * ys[hi] +
    (t3 - t2) * h * ms[hi]
  );
}

/**
 * 把关键站位表编译成「参数 -> 截面参数对象」的函数。
 * keys: [{ t, ...channels }]，所有关键帧必须含相同的通道名。
 */
export function compileProfile(keys) {
  const ts = keys.map((k) => k.t);
  const channels = Object.keys(keys[0]).filter((k) => k !== 't');
  const curves = {};
  for (const name of channels) {
    const ys = keys.map((k) => k[name]);
    curves[name] = { ys, ms: monotoneSlopes(ts, ys) };
  }
  return (t) => {
    const out = {};
    for (const name of channels) {
      out[name] = evalMonotone(ts, curves[name].ys, curves[name].ms, t);
    }
    return out;
  };
}

/**
 * 沿给定轴向路径放样出一段壳体。
 *
 * @param {object}   opts
 * @param {function} opts.path      t∈[0,1] -> THREE.Vector3，截面中心的行进路径
 * @param {function} opts.section   t∈[0,1] -> { w, hTop, hBot, expTop, expBot, yOff, roll }
 * @param {number}   opts.steps     轴向切片数
 * @param {number}   opts.segments  每个截面的周向采样数
 * @param {boolean}  opts.capStart  起始端是否封盖
 * @param {boolean}  opts.capEnd    末端是否封盖
 * @param {boolean}  opts.softCapStart  起始端盖与筒身共顶点，法线连续（纺锤收口）
 * @param {THREE.Vector3} opts.up   参考上方向（固定参考系，避免 Frenet 标架自旋）
 */
export function loft({
  path,
  section,
  steps = 48,
  segments = 48,
  capStart = true,
  capEnd = true,
  softCapStart = false,
  up = new THREE.Vector3(0, 1, 0),
  ring,
}) {
  const positions = [];
  const uvs = [];
  const indices = [];
  const rings = [];

  const eps = 1e-4;
  for (let i = 0; i <= steps; i++) {
    const t = i / steps;
    const center = path(t);
    // 数值差分求切向，路径可以是任意函数而不必是 THREE.Curve
    const ahead = path(Math.min(1, t + eps));
    const behind = path(Math.max(0, t - eps));
    const tangent = TMP_A.copy(ahead).sub(behind).normalize();
    if (tangent.lengthSq() < 1e-12) tangent.set(0, 0, 1);

    const right = new THREE.Vector3().crossVectors(up, tangent).normalize();
    if (right.lengthSq() < 1e-12) right.set(1, 0, 0);
    const localUp = new THREE.Vector3().crossVectors(tangent, right).normalize();

    const sec = section(t);
    const roll = sec.roll ?? 0;
    if (roll !== 0) {
      right.applyAxisAngle(tangent, roll);
      localUp.applyAxisAngle(tangent, roll);
    }

    const ringPts = ring ? ring(sec, segments) : sampleSuperellipseRing(sec, segments);
    const ringIdx = [];
    const base = positions.length / 3;
    for (let j = 0; j <= segments; j++) {
      const [x, y] = ringPts[j];
      TMP_B.copy(center).addScaledVector(right, x).addScaledVector(localUp, y);
      positions.push(TMP_B.x, TMP_B.y, TMP_B.z);
      uvs.push(j / segments, t);
      ringIdx.push(base + j);
    }
    rings.push({ ring: ringIdx, center: center.clone(), tangent: tangent.clone() });
  }

  for (let i = 0; i < steps; i++) {
    const a = rings[i].ring;
    const b = rings[i + 1].ring;
    // 绕序要让法线朝外：截面按 (right, localUp) 逆时针排布、路径沿切向前进，
    // 因此同一四边形必须按 a[j] → a[j+1] → b[j] 的顺序缠绕
    for (let j = 0; j < segments; j++) {
      indices.push(a[j], a[j + 1], b[j]);
      indices.push(a[j + 1], b[j + 1], b[j]);
    }
  }

  // 端盖默认另起顶点形成硬边；soft 时复用筒身顶点，让收口处法线连续
  const addCap = (ringInfo, flip, soft = false) => {
    const { ring, center } = ringInfo;
    const centerIndex = positions.length / 3;
    positions.push(center.x, center.y, center.z);
    uvs.push(0.5, 0.5);
    const rim = [];
    if (soft) {
      for (let j = 0; j <= segments; j++) rim.push(ring[j]);
    } else {
      for (let j = 0; j <= segments; j++) {
        const src = ring[j] * 3;
        rim.push(positions.length / 3);
        positions.push(positions[src], positions[src + 1], positions[src + 2]);
        uvs.push(j / segments, flip ? 0 : 1);
      }
    }
    for (let j = 0; j < segments; j++) {
      if (flip) indices.push(centerIndex, rim[j + 1], rim[j]);
      else indices.push(centerIndex, rim[j], rim[j + 1]);
    }
  };

  if (capStart) addCap(rings[0], true, softCapStart);
  if (capEnd) addCap(rings[steps], false);

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geometry.setAttribute('uv', new THREE.Float32BufferAttribute(uvs, 2));
  geometry.setIndex(indices);
  geometry.computeVertexNormals();
  return geometry;
}

/** 直线路径工具：从 a 到 b。 */
export function linePath(a, b) {
  const from = a.clone();
  const dir = b.clone().sub(a);
  return (t) => from.clone().addScaledVector(dir, t);
}

/** Catmull-Rom 路径工具：给一串控制点即可得到平滑走线（机臂、起落架用）。 */
export function splinePath(points) {
  const curve = new THREE.CatmullRomCurve3(points.map((p) => p.clone()), false, 'catmullrom', 0.5);
  return (t) => curve.getPoint(t);
}

/**
 * 圆角矩形轮廓，用于挤出各类面板、盖板、散热栅。
 */
export function roundedRectShape(width, height, radius) {
  const w = width / 2;
  const h = height / 2;
  const r = Math.min(radius, w, h);
  const shape = new THREE.Shape();
  shape.moveTo(-w + r, -h);
  shape.lineTo(w - r, -h);
  shape.quadraticCurveTo(w, -h, w, -h + r);
  shape.lineTo(w, h - r);
  shape.quadraticCurveTo(w, h, w - r, h);
  shape.lineTo(-w + r, h);
  shape.quadraticCurveTo(-w, h, -w, h - r);
  shape.lineTo(-w, -h + r);
  shape.quadraticCurveTo(-w, -h, -w + r, -h);
  return shape;
}

/** 带倒角的圆角板，常用来做外壳上的凸起面板与嵌片。 */
export function roundedPlate(width, height, depth, radius, bevel = depth * 0.28) {
  const geometry = new THREE.ExtrudeGeometry(roundedRectShape(width, height, radius), {
    depth: Math.max(depth - bevel * 2, 1e-4),
    bevelEnabled: bevel > 0,
    bevelThickness: bevel,
    bevelSize: bevel,
    bevelSegments: 3,
    curveSegments: 16,
  });
  geometry.translate(0, 0, -depth / 2 + bevel);
  geometry.computeVertexNormals();
  return geometry;
}

/** 圆角长方体（比 BoxGeometry 更贴近注塑件的观感）。 */
export function roundedBox(width, height, depth, radius, segments = 4) {
  const geometry = new THREE.BoxGeometry(width, height, depth, segments, segments, segments);
  const pos = geometry.attributes.position;
  const half = new THREE.Vector3(width / 2, height / 2, depth / 2);
  const r = Math.min(radius, half.x, half.y, half.z);
  const inner = new THREE.Vector3(half.x - r, half.y - r, half.z - r);
  const v = new THREE.Vector3();
  const clamped = new THREE.Vector3();
  for (let i = 0; i < pos.count; i++) {
    v.fromBufferAttribute(pos, i);
    clamped.set(
      THREE.MathUtils.clamp(v.x, -inner.x, inner.x),
      THREE.MathUtils.clamp(v.y, -inner.y, inner.y),
      THREE.MathUtils.clamp(v.z, -inner.z, inner.z)
    );
    v.sub(clamped).normalize().multiplyScalar(r).add(clamped);
    pos.setXYZ(i, v.x, v.y, v.z);
  }
  geometry.computeVertexNormals();
  return geometry;
}

/** 给几何体整体染色（配合 vertexColors 使用，便于把多个零件合并成一次 draw call）。 */
export function paint(geometry, color) {
  const c = new THREE.Color(color);
  const count = geometry.attributes.position.count;
  const colors = new Float32Array(count * 3);
  for (let i = 0; i < count; i++) {
    colors[i * 3] = c.r;
    colors[i * 3 + 1] = c.g;
    colors[i * 3 + 2] = c.b;
  }
  geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  return geometry;
}
