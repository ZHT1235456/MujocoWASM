import loadMujoco from '@mujoco/mujoco';
import wasmUrl from '@mujoco/mujoco/mujoco.wasm?url';
import { buildMjcf } from '../world.js';

let mujoco = null;

export async function initMujoco() {
  mujoco = await loadMujoco({
    locateFile: (path) => (String(path).endsWith('.wasm') ? wasmUrl : path),
  });
  return mujoco;
}

function loadModelFromXml(xml) {
  if (mujoco.MjModel.from_xml_string) return mujoco.MjModel.from_xml_string(xml);
  if (mujoco.MjModel.loadFromXML) {
    try {
      mujoco.FS.mkdir('/working');
    } catch {
      /* exists */
    }
    mujoco.FS.writeFile('/working/quadrotor.xml', xml);
    return mujoco.MjModel.loadFromXML('/working/quadrotor.xml');
  }
  throw new Error('无法加载 MjModel');
}

export function createSim(start) {
  const xml = buildMjcf(start);
  const model = loadModelFromXml(xml);
  const data = new mujoco.MjData(model);
  mujoco.mj_forward(model, data);
  return { mujoco, model, data, xml };
}

export function resetSim(sim, start) {
  sim.model.delete?.();
  sim.data.delete?.();
  const next = createSim(start);
  sim.model = next.model;
  sim.data = next.data;
  sim.xml = next.xml;
  return sim;
}

export function stepSim(sim, thrustsOrFn, dtRender) {
  const { mujoco, model, data } = sim;
  const timestep = model.opt?.timestep ?? model.option?.timestep ?? 0.002;
  const ctrl = data.ctrl;
  const dt = Math.min(Math.max(0, dtRender), 1 / 30);
  const target = data.time + dt;
  let guard = 0;
  while (data.time < target && guard++ < 16) {
    const thrusts = typeof thrustsOrFn === 'function' ? thrustsOrFn(timestep) : thrustsOrFn;
    for (let i = 0; i < 4; i++) ctrl[i] = thrusts[i] ?? 0;
    mujoco.mj_step(model, data);
  }
}
