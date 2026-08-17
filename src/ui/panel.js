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

  $('c-iters').addEventListener('input', () => {
    $('v-iters').textContent = $('c-iters').value;
  });
  $('c-radius').addEventListener('input', () => {
    $('v-radius').textContent = `${(Number($('c-radius').value) / 100).toFixed(2)} m`;
  });
  $('c-speed').addEventListener('input', () => {
    $('v-speed').textContent = `${(Number($('c-speed').value) / 10).toFixed(1)} m/s`;
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
    app.follow = e.target.checked;
  });

  return { readStart, readGoal };
}

export function setStatus(text) {
  const el = document.getElementById('status');
  if (el) el.textContent = text;
}

export function setBusy(busy, message = '正在规划路径…') {
  const overlay = document.getElementById('busy');
  const panel = document.getElementById('panel');
  const label = document.getElementById('busy-text');
  if (label) label.textContent = message;
  if (overlay) overlay.hidden = !busy;
  panel?.classList.toggle('is-busy', busy);
  for (const el of panel?.querySelectorAll('button, input') ?? []) {
    el.disabled = busy;
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
