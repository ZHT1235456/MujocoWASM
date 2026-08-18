function syncSliderFill(el) {
  if (!el) return;
  const min = Number(el.min);
  const max = Number(el.max);
  const value = Number(el.value);
  const pct = max === min ? 0 : ((value - min) / (max - min)) * 100;
  el.style.setProperty('--fill', `${pct}%`);
}

export function bindPanel(app) {
  const $ = (id) => document.getElementById(id);

  const readStart = () => [
    Number($('c-sx').value),
    Number($('c-sy').value),
    Number($('c-sz').value),
  ];
  const readGoal = () => [
    Number($('c-gx').value),
    Number($('c-gy').value),
    Number($('c-gz').value),
  ];

  for (const id of ['c-iters', 'c-radius', 'c-speed']) syncSliderFill($(id));

  $('c-iters').addEventListener('input', () => {
    $('v-iters').textContent = $('c-iters').value;
    syncSliderFill($('c-iters'));
    app.planningConfigChanged?.();
  });
  $('c-radius').addEventListener('input', () => {
    $('v-radius').textContent = `${(Number($('c-radius').value) / 100).toFixed(2)} m`;
    syncSliderFill($('c-radius'));
    app.planningConfigChanged?.();
  });
  $('c-speed').addEventListener('input', () => {
    $('v-speed').textContent = `${(Number($('c-speed').value) / 10).toFixed(1)} m/s`;
    syncSliderFill($('c-speed'));
    app.planningConfigChanged?.();
  });

  $('c-plan').addEventListener('click', () => {
    app.plan(readStart(), readGoal(), {
      iters: Number($('c-iters').value),
      rMax: Number($('c-radius').value) / 100,
      speed: Number($('c-speed').value) / 10,
    });
  });
  $('c-fly').addEventListener('click', () => app.fly(Number($('c-speed').value) / 10));
  $('c-pause').addEventListener('click', () => app.togglePause());
  $('c-reset').addEventListener('click', () => app.reset(readStart()));

  let startTimer = 0;
  const pushStart = () => app.moveStart?.(readStart());
  for (const id of ['c-sx', 'c-sy', 'c-sz']) {
    $(id).addEventListener('change', pushStart);
    $(id).addEventListener('input', () => {
      clearTimeout(startTimer);
      startTimer = setTimeout(pushStart, 180);
    });
  }
  for (const id of ['c-gx', 'c-gy', 'c-gz']) {
    const pushGoal = () => app.moveGoal?.(readGoal());
    $(id).addEventListener('input', pushGoal);
    $(id).addEventListener('change', pushGoal);
  }

  $('c-corridor').addEventListener('change', (e) => app.setShow({ corridor: e.target.checked }));
  $('c-centerline').addEventListener('change', (e) => app.setShow({ centerline: e.target.checked }));
  $('c-tree').addEventListener('change', (e) => app.setShow({ tree: e.target.checked }));
  $('c-wire').addEventListener('change', (e) => app.setShow({ wire: e.target.checked }));
  $('c-axes').addEventListener('change', (e) => app.setShow({ axes: e.target.checked }));
  $('c-follow').addEventListener('change', (e) => {
    app.setFollow?.(e.target.checked);
  });

  return { readStart, readGoal };
}

let statusHideTimer = null;

export function setStatus(text, hideAfterMs = 0) {
  const el = document.getElementById('status');
  if (!el) return;
  if (statusHideTimer) {
    clearTimeout(statusHideTimer);
    statusHideTimer = null;
  }
  el.textContent = text;
  if (hideAfterMs > 0) {
    statusHideTimer = setTimeout(() => {
      el.textContent = '';
      statusHideTimer = null;
    }, hideAfterMs);
  }
}

let pauseReady = false;
let flyReady = false;

export function setPauseEnabled(enabled) {
  pauseReady = enabled;
  const el = document.getElementById('c-pause');
  const panel = document.getElementById('panel');
  if (el) el.disabled = !enabled || !!panel?.classList.contains('is-busy');
}

export function setFlyEnabled(enabled) {
  flyReady = enabled;
  const el = document.getElementById('c-fly');
  const panel = document.getElementById('panel');
  if (el) el.disabled = !enabled || !!panel?.classList.contains('is-busy');
}

export function setBusy(busy, message = '正在规划路径…') {
  const overlay = document.getElementById('busy');
  const panel = document.getElementById('panel');
  const label = document.getElementById('busy-text');
  if (label) label.textContent = message;
  if (overlay) overlay.hidden = !busy;
  panel?.classList.toggle('is-busy', busy);
  for (const el of panel?.querySelectorAll('button, input') ?? []) {
    if (el.id === 'c-pause') {
      el.disabled = busy || !pauseReady;
      continue;
    }
    if (el.id === 'c-fly') {
      el.disabled = busy || !flyReady;
      continue;
    }
    const configLocked = panel.classList.contains('config-locked') && el.hasAttribute('data-config-control');
    el.disabled = busy || configLocked;
  }
}

export function setConfigLocked(locked) {
  const panel = document.getElementById('panel');
  const state = document.getElementById('config-lock-state');
  panel?.classList.toggle('config-locked', locked);
  if (state) state.textContent = locked ? '已锁定' : '起飞后锁定';
  for (const el of panel?.querySelectorAll('[data-config-control]') ?? []) {
    el.disabled = locked || panel.classList.contains('is-busy');
  }
}

export function waitFrame() {
  return new Promise((resolve) => requestAnimationFrame(() => requestAnimationFrame(resolve)));
}

export function setMetrics({ length, clearance, time, inside }) {
  if (length != null) document.getElementById('m-length').textContent = `${length.toFixed(2)} m`;
  if (clearance != null) document.getElementById('m-clearance').textContent = `${clearance.toFixed(2)} m`;
  if (time != null) document.getElementById('m-time').textContent = `${time.toFixed(2)} s`;
  if (inside != null) document.getElementById('m-inside').textContent = inside;
}

export function clearPlanMetrics() {
  document.getElementById('m-length').textContent = '—';
  document.getElementById('m-clearance').textContent = '—';
  document.getElementById('m-time').textContent = '0.00 s';
  document.getElementById('m-inside').textContent = '待重新规划';
}
