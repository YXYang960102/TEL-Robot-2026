// Behavior test for tools/dashboard/index.html's keyboard control (wired
// through the Mega's USB, or wireless through the ground STM32),
// loading the WHOLE page script in a vm with a permissive DOM stub (canvas,
// layout and stream code just hits no-op proxies). Covers the page-side
// items from Codex's review of e4a70a5: key-repeat suppression, release
// handling when focus has moved into a text box, Enter/Space, the Falcon
// slider path, and resync/reset on firmware-reported disarm. Plus the
// wireless-round additions: enable is pending until the robot confirms,
// the page only arms for an enable it asked for, R is gone, and the ground
// station's $WIRELESS / $GROUND_EVENT / $RFLINK lines.
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
const clock = { t: 0 };
const ctx = {
  console, TextEncoder, TextDecoder, Promise, Set, Map, Number, Math, JSON, Date, Object, Array, String,
  performance: { now: () => clock.t },
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
const line = (text) => run(`handleLine(${JSON.stringify(text)})`);
// Full enable handshake: the button sends E, the robot confirms.
const arm = async () => { await elements.enableControl.handlers.click(); event('OUTPUTS_ENABLED'); };
const tel = (armedSlot, owner) => ['TEL', 1, 0, 0, 0, 0, 0, 1500, 1500, 1500, 1500, 1500, 0, 0, 1,
  'NaN', 'NaN', 'NaN', armedSlot, 1500, 1500, owner, 0, 0, 0, 0, 1500, 0.0].join(',');

async function emergencyStopViaEnter() {
  await windowHandlers.keydown(key('Enter'));
}

(async () => {
  // Disarmed by default: movement and F/L send nothing.
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
  // R controls the horizontal belt independently of the F tower pulse.
  writes.length = 0;
  for (const k of ['r', 'f', 'l']) {
    await windowHandlers.keydown(key(k));
    await windowHandlers.keydown(key(k, { repeat: true }));
    await windowHandlers.keydown(key(k, { repeat: true }));
  }
  assert.deepEqual([...writes], ['R', 'F', 'L']);

  // Arrows: left/right turret, up/down elevation, both hold-to-move;
  // release stops. With no angle telemetry yet (older firmware, or the
  // wireless ground station) the exact-angle widgets are off.
  writes.length = 0;
  await windowHandlers.keydown(key('ArrowRight'));
  await windowHandlers.keydown(key('ArrowUp'));
  assert.deepEqual([...writes], ['ROT,1', 'ANG,1']);
  await windowHandlers.keyup(key('ArrowRight'));
  await windowHandlers.keyup(key('ArrowUp'));
  assert.deepEqual([...writes].slice(-2), ['ROT,0', 'ANG,0']);
  assert.equal(elements.angleSlider.disabled, true);

  // The Mega reports the angle fields (TEL slots 29-31): the exact-angle
  // widgets turn on; the arrows stay the hold-to-move jog.
  run('handleLine(' + JSON.stringify(['TEL', 1, 0, 0, 0, 0, 0, 1500, 1500, 1500, 1500, 1500, 0, 3, 1,
    'NaN', 'NaN', 'NaN', 1, 1500, 1500, 1, 0, 0, 0, 0, 1500, 0.0, 1500, 135.0, 0.0, 270.0].join(',')) + ')');
  assert.equal(elements.angleSlider.disabled, false);
  writes.length = 0;
  await windowHandlers.keydown(key('ArrowUp'));
  await windowHandlers.keydown(key('ArrowUp', { repeat: true }));
  assert.deepEqual([...writes], ['ANG,1']);              // held, not repeated
  await windowHandlers.keyup(key('ArrowUp'));
  assert.equal(last(), 'ANG,0');
  assert.ok(!writes.some((w) => w.startsWith('ANGS')));
  // Slider and number box send an absolute target.
  elements.angleSlider.value = '90';
  await elements.angleSlider.handlers.input();
  assert.equal(last(), 'ANGT,90');
  elements.angleInput.value = '47.5';
  await elements.angleSet.handlers.click();
  assert.equal(last(), 'ANGT,47.5');
  writes.length = 0;
  elements.angleInput.value = '';
  await elements.angleSet.handlers.click();
  assert.equal(writes.length, 0);                       // empty box: nothing sent

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
  await arm();
  await run('sendHeartbeat()');
  assert.equal(last(), 'K');

  // Firmware-side disarms (watchdog, RC link lost, corrupt line) reset the
  // page too.
  for (const name of ['OUTPUTS_DISABLED', 'COMMAND_TIMEOUT', 'DISABLED_RC_LINK_LOST', 'DISABLED_CORRUPT_COMMAND']) {
    await arm();
    elements.flywheelSlider.value = '50';
    await elements.flywheelSlider.handlers.input();
    await windowHandlers.keydown(key('w'));
    event(name);
    assert.equal(armed(), false, name);
    assert.equal(run('flywheelRatio'), 0, name);
    assert.equal(run('heldKeys.size'), 0, name);
  }

  // Telemetry that reports armed=0 also resyncs the page.
  await arm();
  line(tel(0, 0));
  assert.equal(armed(), false);
  assert.match(elements.falconPwm.textContent, /1500 us \(0\.0%\)/);

  // Armed, but by the wireless source: this USB page must not count as
  // armed (it would heartbeat and send motion the Mega rejects).
  await arm();
  line(tel(1, 2));
  assert.equal(armed(), false);
  assert.match(elements.controlOwner.textContent, /wireless/);
  // ...and telemetry alone never arms a page that didn't ask.
  line(tel(1, 1));
  assert.equal(armed(), false);

  // Window blur while armed stops.
  await arm();
  writes.length = 0;
  await windowHandlers.blur();
  assert.equal(last(), 'X');
  assert.equal(armed(), false);

  // Codex review afed238 [P1]: with the chassis and the Falcon running,
  // focus moves into the stream URL box -- Enter there must still stop.
  await arm();
  await windowHandlers.keydown(key('f'));
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
  await arm();
  await windowHandlers.keydown(key('w'));
  writes.length = 0;
  await windowHandlers.keydown(key(' ', { target: elements.streamUrl }));
  assert.equal(writes[0], 'X');
  assert.equal(run('port'), null);
  assert.equal(armed(), false);
  connectFakePort();

  // Space: stop first, then disconnect.
  await arm();
  writes.length = 0;
  await windowHandlers.keydown(key(' '));
  assert.equal(writes[0], 'X');
  assert.equal(run('port'), null);
  assert.equal(run('writer'), null);

  // ---------------- enable handshake ----------------
  connectFakePort();
  run('setConnection(true)');

  // An OUTPUTS_ENABLED this page never asked for (the Mega prints the
  // wireless operator's enable to USB too) does not arm it.
  event('OUTPUTS_ENABLED');
  assert.equal(armed(), false);

  // Pending: heartbeat keeps the link alive with K, motion keys do nothing,
  // a second click doesn't resend E.
  writes.length = 0;
  await elements.enableControl.handlers.click();
  await elements.enableControl.handlers.click();
  assert.deepEqual([...writes], ['E']);
  assert.equal(run('enablePending'), true);
  await windowHandlers.keydown(key('w'));
  await run('sendHeartbeat()');
  assert.deepEqual([...writes], ['E', 'K']);
  await windowHandlers.keyup(key('w'));

  // Enter during pending cancels it.
  await windowHandlers.keydown(key('Enter'));
  assert.equal(last(), 'X');
  assert.equal(run('enablePending'), false);
  writes.length = 0;
  await run('sendHeartbeat()');
  assert.equal(writes.length, 0);

  // Rejected by the robot (pins unconfirmed / other source owns it) on
  // the Mega's USB: pending ends, and NO X (a wired X would disarm the
  // wireless operator who owns the robot).
  line(tel(1, 2));
  assert.equal(run('linkKind'), 'robot');
  for (const name of ['ENABLE_REJECTED_PINS_UNCONFIRMED', 'ENABLE_REJECTED_OTHER_SOURCE']) {
    writes.length = 0;
    await elements.enableControl.handlers.click();
    event(name);
    await Promise.resolve();
    assert.equal(run('enablePending'), false, name);
    assert.equal(armed(), false, name);
    assert.deepEqual([...writes], ['E'], name);
  }

  // No confirmation at all within 3 s on the Mega link: give up, no X.
  writes.length = 0;
  clock.t = 1000;
  await elements.enableControl.handlers.click();
  clock.t = 3900;
  await run('sendHeartbeat()');
  assert.equal(run('enablePending'), true);
  clock.t = 4100;
  await run('sendHeartbeat()');
  assert.equal(run('enablePending'), false);
  assert.deepEqual([...writes], ['E', 'K']);

  // Wired telemetry confirms a pending enable even if the event line was
  // lost.
  await elements.enableControl.handlers.click();
  line(tel(1, 1));
  assert.equal(armed(), true);
  await emergencyStopViaEnter();

  // ---------------- ground station (wireless) ----------------
  connectFakePort();
  run('setConnection(true)');
  line('$RFLINK,1,100,98,45');
  assert.equal(run('linkKind'), 'ground');
  assert.match(elements.rfLinkState.textContent, /up 100% \/ down 98%/);
  line('$WIRELESS,1,0,0,1,0,1,1,80');
  assert.match(elements.wirelessState.textContent, /disarmed/);

  // Robot confirms via $WIRELESS (even if the $EVENT was coalesced away).
  clock.t = 10000;
  writes.length = 0;
  await elements.enableControl.handlers.click();
  line('$GROUND_EVENT,ENABLE_REQUESTED');
  assert.equal(armed(), false);
  line('$WIRELESS,1,1,2,1,0,1,1,60');
  assert.equal(armed(), true);
  assert.match(elements.controlOwner.textContent, /wireless/);

  // Stale status: warn only, keep driving (the robot's own RC-loss
  // failsafe is what stops it).
  line('$WIRELESS,0,0,0,0,0,0,0,1500');
  assert.equal(armed(), true);
  assert.match(elements.wirelessState.textContent, /no robot status/);

  // Fresh status saying disarmed -> page disarms.
  line('$WIRELESS,1,0,0,1,0,1,1,40');
  assert.equal(armed(), false);

  // Wired source owns the robot -> wireless page doesn't count as armed.
  await arm();
  assert.equal(armed(), true);
  line('$WIRELESS,1,1,1,1,0,1,1,40');
  assert.equal(armed(), false);

  // Ground-side release (PC timeout etc.) disarms the page.
  for (const name of ['PC_TIMEOUT', 'ENABLE_RELEASED', 'RELEASED_CORRUPT_COMMAND', 'COMMAND_TOO_LONG']) {
    await arm();
    line(`$GROUND_EVENT,${name}`);
    assert.equal(armed(), false, name);
  }

  // Ground link: a rejected or unconfirmed enable sends X so the ground
  // drops the arm channel it is still holding.
  writes.length = 0;
  await elements.enableControl.handlers.click();
  event('ENABLE_REJECTED_PINS_UNCONFIRMED');
  await new Promise((r) => setImmediate(r));
  assert.deepEqual([...writes], ['E', 'X']);
  writes.length = 0;
  clock.t = 20000;
  await elements.enableControl.handlers.click();
  clock.t = 23500;
  await run('sendHeartbeat()');
  assert.deepEqual([...writes], ['E', 'X']);
  assert.equal(run('enablePending'), false);

  // ---------------- operator modes (M) ----------------
  connectFakePort();
  run('setConnection(true)');
  clock.t = 30000;
  await arm();
  writes.length = 0;
  await windowHandlers.keydown(key('m'));
  await windowHandlers.keydown(key('m', { repeat: true }));
  assert.deepEqual([...writes], ['M']);               // one press, no auto-repeat
  // The robot confirms; the page shows it and locks the Falcon slider
  // (the shot table drives the flywheel in the auto modes).
  event('MODE_SEMI_AUTO');
  assert.match(elements.controlArmedBadge.textContent, /半自動/);
  assert.match(elements.operatorMode.textContent, /SEMI AUTO/);
  assert.equal(elements.flywheelSlider.disabled, true);
  writes.length = 0;
  elements.flywheelSlider.value = '60';
  await elements.flywheelSlider.handlers.input();
  assert.equal(writes.length, 0);
  assert.equal(elements.flywheelSlider.value, '0');
  // F is still sent as-is (the robot decides: fire now or FIRE_NOT_READY).
  await windowHandlers.keydown(key('f'));
  assert.equal(last(), 'F');
  event('MODE_FULL_AUTO');
  assert.match(elements.operatorMode.textContent, /FULL AUTO/);
  event('MODE_FULL_MANUAL');
  assert.equal(elements.flywheelSlider.disabled, false);
  // TEL slot 13 is mode + 1: 2 = semi auto; 0 (older firmware) is ignored.
  const telMode = (slot13) => ['TEL', 1, 0, 0, 0, 0, 0, 1500, 1500, 1500, 1500, 1500, 0, slot13, 1,
    'NaN', 'NaN', 'NaN', 1, 1500, 1500, 1, 0, 0, 0, 0, 1500, 0.0].join(',');
  line(telMode(2));
  assert.match(elements.operatorMode.textContent, /SEMI AUTO/);
  line(telMode(0));
  assert.match(elements.operatorMode.textContent, /SEMI AUTO/);   // not "FULL AUTO"
  // Any disarm drops the page back to full manual, like the firmware.
  await windowHandlers.keydown(key('Enter'));
  assert.equal(armed(), false);
  assert.match(elements.operatorMode.textContent, /FULL MANUAL/);
  // Ground station: $WIRELESS turns the exact-angle widgets off (the radio
  // carries only up/stop/down); the arrows' ANG jog is unchanged.
  // $WIRELESS field 10 carries the mode (-1 = not reported).
  connectFakePort();
  run('setConnection(true)');
  clock.t = 40000;
  await elements.enableControl.handlers.click();
  line('$WIRELESS,1,1,2,1,0,1,1,50,2');
  assert.equal(armed(), true);
  assert.match(elements.operatorMode.textContent, /FULL MANUAL/);
  line('$WIRELESS,1,1,2,1,0,1,1,50,0');
  assert.match(elements.operatorMode.textContent, /FULL AUTO/);
  line('$WIRELESS,1,1,2,1,0,1,1,50,-1');
  assert.match(elements.operatorMode.textContent, /FULL AUTO/);   // unknown: unchanged
  assert.equal(elements.angleSlider.disabled, true);
  writes.length = 0;
  await windowHandlers.keydown(key('ArrowUp'));
  assert.equal(last(), 'ANG,1');
  await windowHandlers.keyup(key('ArrowUp'));
  assert.equal(last(), 'ANG,0');

  // Slot 28 (appended): elevation pulse; older lines without it are fine.
  line(['TEL', 1, 0, 0, 0, 0, 0, 1500, 1500, 1500, 1500, 1500, 0, 3, 1,
    'NaN', 'NaN', 'NaN', 0, 1500, 1500, 0, 0, 0, 0, 0, 1500, 0.0, 2310].join(','));
  assert.equal(elements.anglePulse.textContent, '2310 us');
  // Slots 29-31: target angle and range -> label and slider limits.
  line(['TEL', 1, 0, 0, 0, 0, 0, 1500, 1500, 1500, 1500, 1500, 0, 3, 1,
    'NaN', 'NaN', 'NaN', 0, 1500, 1500, 0, 0, 0, 0, 0, 1500, 0.0, 2310, 243.5, 0.0, 270.0].join(','));
  assert.equal(elements.anglePulse.textContent, '243.5° · 2310 us');
  assert.equal(elements.angleSlider.max, '270');
  assert.equal(elements.angleSlider.value, '244');
  assert.match(elements.angleValue.textContent, /243\.5°/);

  // ---------------- gamepad (Gamepad API) ----------------
  connectFakePort();
  run('setConnection(true)');
  clock.t = 50000;
  let pads = [];
  ctx.navigator.getGamepads = () => pads;
  const pad = (mapping, axes, pressedButtons = []) => ({
    connected: true, id: mapping === 'standard' ? 'Xbox 360 Controller' : 'Flight Stick', mapping, axes,
    buttons: Array.from({ length: 17 }, (_, i) => ({ pressed: pressedButtons.includes(i), value: 0 })),
  });
  const poll = () => run('pollGamepad()');

  await poll();
  assert.match(elements.gamepadState.textContent, /none/);

  // Disarmed: the stick and fire button send nothing.
  pads = [pad('standard', [1, 0, 0, 0], [3])];
  writes.length = 0;
  await poll();
  assert.equal(writes.length, 0);
  assert.match(elements.gamepadState.textContent, /Xbox/);
  assert.match(elements.gamepadInputs.textContent, /按鈕 \[3\]/);

  // Enabled with the stick already pushed: ignored until it is centered once.
  pads = [pad('standard', [1, 0, 0, 0])];
  await arm();
  writes.length = 0;
  await poll(); await poll();
  assert.equal(writes.length, 0);
  pads = [pad('standard', [0, 0, 0, 0])];
  await poll();
  // Stick right = turn right = same as D (T,-1,1); deadzone-rescaled analog.
  pads = [pad('standard', [1, 0, 0, 0])];
  await poll();
  assert.equal(last(), 'T,-1,1');
  pads = [pad('standard', [0.575, 0, 0, 0])];   // half way past the deadzone
  await poll();
  assert.equal(last(), 'T,-0.5,0.5');
  pads = [pad('standard', [0, -1, 0, 0])];      // stick up = forward
  await poll();
  assert.equal(last(), 'T,1,1');
  pads = [pad('standard', [0.1, -0.1, 0, 0])];  // inside the deadzone = stop
  await poll();
  assert.equal(last(), 'T,0,0');

  // The keyboard still works while the stick is centered.
  await windowHandlers.keydown(key('w'));
  assert.equal(last(), 'T,1,1');
  await windowHandlers.keyup(key('w'));
  assert.equal(last(), 'T,0,0');

  // Fire (Xbox Y = button 3): one F per press, none while held.
  writes.length = 0;
  pads = [pad('standard', [0, 0, 0, 0], [3])];
  await poll(); await poll(); await poll();
  pads = [pad('standard', [0, 0, 0, 0])];
  await poll();
  pads = [pad('standard', [0, 0, 0, 0], [3])];
  await poll();
  assert.deepEqual([...writes], ['F', 'F']);

  // Unplugged while driving: the drive drops to zero.
  pads = [pad('standard', [-1, 0, 0, 0])];
  await poll();
  assert.equal(last(), 'T,1,-1');
  pads = [];
  await poll();
  assert.equal(last(), 'T,0,0');

  // A non-standard pad (flight stick): axes 0/1, fire = button 0.
  writes.length = 0;
  pads = [pad('', [0, 0], [0])];
  await poll();
  assert.deepEqual([...writes], ['F']);
  assert.match(elements.gamepadState.textContent, /flight stick/);

  // Disarm clears the gamepad drive.
  pads = [pad('', [0, -1])];
  await poll();
  assert.equal(last(), 'T,1,1');
  await windowHandlers.keydown(key('Enter'));
  writes.length = 0;
  await poll();
  assert.equal(writes.length, 0);

  console.log('dashboard page control behavior tests passed');
})().catch((e) => { console.error(e); process.exitCode = 1; });
