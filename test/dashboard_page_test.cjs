// Behavior test for tools/dashboard/index.html's wired keyboard control,
// loading the WHOLE page script in a vm with a permissive DOM stub (canvas,
// layout and stream code just hits no-op proxies). Covers the page-side
// items from Codex's review of e4a70a5: key-repeat suppression, release
// handling when focus has moved into a text box, Enter/Space, the Falcon
// slider path, and resync/reset on firmware-reported disarm.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const html = fs.readFileSync(path.join(__dirname, '../tools/dashboard/index.html'), 'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];

// Anything not explicitly modeled: every property is another callable stub.
function universal() {
  const store = {};
  return new Proxy(function () {}, {
    get(_, key) {
      if (key in store) return store[key];
      if (key === Symbol.toPrimitive) return () => 0;
      if (key === 'length') return 0;
      return (store[key] = universal());
    },
    set(_, key, value) { store[key] = value; return true; },
    apply() { return universal(); },
  });
}

function makeElement(id, tagName = 'DIV', type = '') {
  const real = {
    id, tagName, type, textContent: '', value: '0', disabled: false, isContentEditable: false, handlers: {},
    addEventListener(name, fn) { this.handlers[name] = fn; },
    blur() {},
  };
  return new Proxy(real, {
    get(target, key) {
      if (key in target) return target[key];
      return (target[key] = universal());
    },
  });
}

const elements = {};
const getElement = (id) => (elements[id] ??= makeElement(id, id === 'flywheelSlider' ? 'INPUT' : 'DIV', id === 'flywheelSlider' ? 'range' : ''));
elements.flywheelSlider = makeElement('flywheelSlider', 'INPUT', 'range');
elements.streamUrl = makeElement('streamUrl', 'INPUT', 'url');

const windowHandlers = {};
const documentHandlers = {};
let intervalFns = [];
const ctx = {
  console, TextEncoder, TextDecoder, Promise, Set, Map, Number, Math, JSON, Date, Object, Array, String,
  performance: { now: () => 0 },
  setInterval: (fn) => { intervalFns.push(fn); return intervalFns.length; },
  clearInterval() {}, setTimeout: () => 0, requestAnimationFrame: () => 0,
  localStorage: { getItem: () => null, setItem() {}, removeItem() {} },
  navigator: { serial: { addEventListener() {} } },
  alert() {},
  document: {
    getElementById: getElement,
    querySelectorAll: () => [],
    querySelector: () => universal(),
    createElement: () => universal(),
    addEventListener: (name, fn) => { documentHandlers[name] = fn; },
    hidden: false,
  },
  window: { addEventListener: (name, fn) => { windowHandlers[name] = fn; } },
};
vm.createContext(ctx);
vm.runInContext(script, ctx);

const writes = [];
ctx.writes = writes;
const connectFakePort = () => vm.runInContext(
  'writer = { write: async (b) => writes.push(new TextDecoder().decode(b).trim()), releaseLock() {} };' +
  'port = { close: async () => {} }; reader = { cancel: async () => {} };',
  ctx);
connectFakePort();

const body = { tagName: 'BODY', type: '' };
const key = (k, { repeat = false, target = body } = {}) =>
  ({ key: k, repeat, target, preventDefault() {}, stopPropagation() {} });
const run = (code) => vm.runInContext(code, ctx);
const armed = () => run('controlArmed');
const event = (name) => run(`handleLine("$EVENT,${name}")`);
const last = () => writes.at(-1);

(async () => {
  // Disarmed by default: movement and R/F/L send nothing.
  await windowHandlers.keydown(key('w'));
  await windowHandlers.keydown(key('r'));
  assert.equal(writes.length, 0);
  // Initial disabled state comes from the markup itself (not modeled by
  // the DOM stub), so check the source directly.
  assert.match(html, /<button id="enableControl" disabled>/);
  assert.match(html, /<input id="flywheelSlider"[^>]*\bdisabled\b/);

  // Enable goes through the button, and only the firmware's
  // OUTPUTS_ENABLED actually arms the page.
  await elements.enableControl.handlers.click();
  assert.equal(last(), 'E');
  assert.equal(armed(), false);
  event('OUTPUTS_ENABLED');
  assert.equal(armed(), true);
  assert.equal(elements.flywheelSlider.disabled, false);

  // W press / auto-repeat / release.
  writes.length = 0;
  await windowHandlers.keydown(key('w'));
  await windowHandlers.keydown(key('w', { repeat: true }));
  await windowHandlers.keydown(key('w', { repeat: true }));
  assert.deepEqual([...writes], ['T,1,1']);
  await windowHandlers.keyup(key('w'));
  assert.equal(last(), 'T,0,0');

  // A = left forward, right backward (Jeremy's explicit polarity).
  await windowHandlers.keydown(key('a'));
  assert.equal(last(), 'T,1,-1');
  await windowHandlers.keyup(key('a'));

  // R/F/L: single press sends once; held-key auto-repeat sends nothing.
  writes.length = 0;
  for (const k of ['r', 'f', 'l']) {
    await windowHandlers.keydown(key(k));
    await windowHandlers.keydown(key(k, { repeat: true }));
    await windowHandlers.keydown(key(k, { repeat: true }));
  }
  assert.deepEqual([...writes], ['R', 'F', 'L']);

  // Arrows drive Rotate/Angle; release stops.
  writes.length = 0;
  await windowHandlers.keydown(key('ArrowRight'));
  await windowHandlers.keydown(key('ArrowUp'));
  assert.deepEqual([...writes], ['ROT,1', 'ANG,1']);
  await windowHandlers.keyup(key('ArrowRight'));
  await windowHandlers.keyup(key('ArrowUp'));
  assert.deepEqual([...writes].slice(-2), ['ROT,0', 'ANG,0']);

  // Typing in a real text box never drives the robot...
  writes.length = 0;
  for (const k of ['w', 'r', 'f', 'l', 'ArrowUp']) {
    await windowHandlers.keydown(key(k, { target: elements.streamUrl }));
  }
  assert.equal(writes.length, 0);
  assert.equal(armed(), true);

  // ...but a key held BEFORE focus moved there is still released by its
  // keyup (Codex: otherwise the heartbeat keeps resending the old value).
  await windowHandlers.keydown(key('d'));
  assert.equal(last(), 'T,-1,1');
  await windowHandlers.keyup(key('d', { target: elements.streamUrl }));
  assert.equal(last(), 'T,0,0');

  // Focus jumping into a text box mid-hold also releases it.
  await windowHandlers.keydown(key('s'));
  assert.equal(last(), 'T,-1,-1');
  await documentHandlers.focusin({ target: elements.streamUrl });
  assert.equal(last(), 'T,0,0');

  // Falcon slider -> FLY command, and resent by the heartbeat while non-zero.
  elements.flywheelSlider.value = '40';
  await elements.flywheelSlider.handlers.input();
  assert.equal(last(), 'FLY,0.40');
  writes.length = 0;
  await run('sendHeartbeat()');
  assert.deepEqual([...writes], ['FLY,0.40']);

  // Enter while the slider has focus still stops (a range input isn't a
  // typing target), and resets the slider to 0.
  writes.length = 0;
  await windowHandlers.keydown(key('Enter', { target: elements.flywheelSlider }));
  assert.equal(last(), 'X');
  assert.equal(armed(), false);
  assert.equal(elements.flywheelSlider.value, '0');
  assert.equal(run('flywheelRatio'), 0);

  // Disarmed: heartbeat sends nothing, slider input is ignored.
  writes.length = 0;
  await run('sendHeartbeat()');
  elements.flywheelSlider.value = '70';
  await elements.flywheelSlider.handlers.input();
  assert.equal(writes.length, 0);
  assert.equal(elements.flywheelSlider.value, '0');

  // Neutral + armed heartbeat is a bare keepalive.
  event('OUTPUTS_ENABLED');
  await run('sendHeartbeat()');
  assert.equal(last(), 'K');

  // Firmware-side disarms (watchdog, RC takeover) reset the page too.
  for (const name of ['COMMAND_TIMEOUT', 'DISABLED_RC_LINK_ACTIVE']) {
    event('OUTPUTS_ENABLED');
    elements.flywheelSlider.value = '50';
    await elements.flywheelSlider.handlers.input();
    await windowHandlers.keydown(key('w'));
    event(name);
    assert.equal(armed(), false, name);
    assert.equal(run('flywheelRatio'), 0, name);
    assert.equal(run('heldKeys.size'), 0, name);
  }

  // Telemetry that reports wiredArmed=0 also resyncs the page.
  event('OUTPUTS_ENABLED');
  const tel = ['TEL', 1, 0, 0, 0, 0, 0, 1500, 1500, 1500, 1500, 1500, 0, 0, 1,
    'NaN', 'NaN', 'NaN', 0, 1500, 1500, 0, 0, 0, 0, 0, 1500, 0.0].join(',');
  run(`handleLine(${JSON.stringify(tel)})`);
  assert.equal(armed(), false);
  assert.match(elements.falconPwm.textContent, /1500 us \(0\.0%\)/);

  // Window blur while armed stops.
  event('OUTPUTS_ENABLED');
  writes.length = 0;
  await windowHandlers.blur();
  assert.equal(last(), 'X');
  assert.equal(armed(), false);

  // Codex review afed238 [P1]: with the feed (R) and the Falcon running,
  // focus moves into the stream URL box -- Enter there must still stop.
  event('OUTPUTS_ENABLED');
  await windowHandlers.keydown(key('r'));
  elements.flywheelSlider.value = '40';
  await elements.flywheelSlider.handlers.input();
  await documentHandlers.focusin({ target: elements.streamUrl });
  writes.length = 0;
  await windowHandlers.keydown(key('Enter', { target: elements.streamUrl }));
  assert.equal(last(), 'X');
  assert.equal(armed(), false);
  assert.equal(run('flywheelRatio'), 0);
  writes.length = 0;
  await run('sendHeartbeat()');
  assert.equal(writes.length, 0);  // no keepalive keeps anything alive

  // Disarmed: Enter/Space in a text box are ordinary typing again.
  await windowHandlers.keydown(key('Enter', { target: elements.streamUrl }));
  await windowHandlers.keydown(key(' ', { target: elements.streamUrl }));
  assert.equal(writes.length, 0);
  assert.notEqual(run('port'), null);

  // Same scenario with Space from the text box: stop, then disconnect.
  event('OUTPUTS_ENABLED');
  await windowHandlers.keydown(key('r'));
  writes.length = 0;
  await windowHandlers.keydown(key(' ', { target: elements.streamUrl }));
  assert.equal(writes[0], 'X');
  assert.equal(run('port'), null);
  assert.equal(armed(), false);
  connectFakePort();

  // Space: stop first, then disconnect.
  event('OUTPUTS_ENABLED');
  writes.length = 0;
  await windowHandlers.keydown(key(' '));
  assert.equal(writes[0], 'X');
  assert.equal(run('port'), null);
  assert.equal(run('writer'), null);

  console.log('dashboard page wired-control behavior tests passed');
})().catch((e) => { console.error(e); process.exitCode = 1; });
