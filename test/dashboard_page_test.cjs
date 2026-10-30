// Behavior test for tools/dashboard/index.html's operator control (wired
// through the Mega's USB, or wireless through the ground STM32),
// loading the WHOLE page script in a vm with a permissive DOM stub (canvas,
// and layout code just hits no-op proxies). Covers the page-side
// items from Codex's review of e4a70a5: key-repeat suppression, release
// handling when focus has moved into a text box, Enter/Space, the Falcon
// slider path, and resync/reset on firmware-reported disarm. Plus the
// wireless-round additions: enable is pending until the robot confirms,
// the page only arms for an enable it asked for, and the ground station's
// $WIRELESS / $GROUND_EVENT / $RFLINK lines. Control table v2 (2026-10-16):
// 1-9 / 0 -> TGT, mode + auto-path menus applied after Enable, no keyboard
// drive/fire (gamepad only), and
// the target / auto-path readouts.
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
// A real text box on the page (the elevation angle number box).
elements.angleInput = makeElement('angleInput', 'INPUT', 'number');

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
// The menus' markup defaults (the DOM stub doesn't parse HTML).
elements.startMode.value = '2';       // full manual
elements.autoPathSelect.value = '0';  // None

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
  // Gamepad stub (browser Gamepad API). Xbox = "standard" mapping.
  let pads = [];
  ctx.navigator.getGamepads = () => pads;
  const pad = (mapping, axes, pressedButtons = []) => ({
    connected: true, id: mapping === 'standard' ? 'Xbox 360 Controller' : 'Flight Stick', mapping, axes,
    buttons: Array.from({ length: 17 }, (_, i) => ({ pressed: pressedButtons.includes(i), value: 0 })),
  });
  const xbox = (axes = [0, 0, 0, 0], buttons = []) => pad('standard', axes, buttons);
  const XB = { A: 0, B: 1, X: 2, Y: 3, LB: 4, RB: 5, UP: 12, DOWN: 13, LEFT: 14, RIGHT: 15 };
  const poll = () => run('pollGamepad()');
  // Press and release Xbox buttons (sticks centered).
  const press = async (...buttons) => { pads = [xbox([0, 0, 0, 0], buttons)]; await poll(); pads = [xbox()]; await poll(); };
  const start = () => elements.startMatch.handlers.click();

  // Disarmed by default: keys and the pad send nothing.
  for (const k of ['ArrowUp', '5', '0', 'm']) await windowHandlers.keydown(key(k));
  pads = [xbox([0, -1, 1, 0], [XB.LB, XB.UP, XB.Y])];
  await poll();
  pads = [xbox()];
  await poll();
  assert.equal(writes.length, 0);
  // Initial disabled state comes from the markup itself (not modeled by
  // the DOM stub), so check the source directly.
  assert.match(html, /<button id="enableControl" disabled>/);
  assert.match(html, /<button id="startMatch" disabled>/);
  assert.match(html, /<input id="flywheelSlider"[^>]*\bdisabled\b/);

  // Enable goes through the button, and only the firmware's
  // OUTPUTS_ENABLED actually arms the page.
  await elements.enableControl.handlers.click();
  assert.equal(last(), 'E');
  assert.equal(armed(), false);
  event('OUTPUTS_ENABLED');
  assert.equal(armed(), true);

  // ---- Before Y: the robot holds still (Jeremy, 2026-10-17) ----
  assert.equal(run('matchStarted'), false);
  assert.equal(elements.startMatch.disabled, false);
  assert.equal(elements.flywheelSlider.disabled, true);
  assert.match(elements.matchState.textContent, /按 Y 開始/);
  writes.length = 0;
  pads = [xbox([0, -1, 1, 0], [XB.UP, XB.RIGHT, XB.LB])];  // stick, D-pad, fire
  await poll(); await poll();
  pads = [xbox()];
  await poll();
  elements.flywheelSlider.value = '40';
  await elements.flywheelSlider.handlers.input();
  assert.equal(writes.length, 0);
  assert.equal(elements.flywheelSlider.value, '0');
  await run('sendHeartbeat()');
  assert.deepEqual([...writes], ['K']);                  // just the keepalive
  // Digits still pick a target (no motion); repeats send nothing.
  writes.length = 0;
  for (const k of ['1', '5', '9', '0']) {
    await windowHandlers.keydown(key(k));
    await windowHandlers.keydown(key(k, { repeat: true }));
  }
  assert.deepEqual([...writes], ['TGT,1', 'TGT,5', 'TGT,9', 'TGT,0']);   // 0 = back to the AI's pick
  // Q W E: the bonus holes 10 / 11 / 12 (2026-10-27); repeats send nothing.
  writes.length = 0;
  for (const k of ['q', 'w', 'e']) {
    await windowHandlers.keydown(key(k));
    await windowHandlers.keydown(key(k, { repeat: true }));
  }
  assert.deepEqual([...writes], ['TGT,10', 'TGT,11', 'TGT,12']);
  // M, the arrows and the other old letter keys are gone from the keyboard.
  writes.length = 0;
  for (const k of ['m', 'ArrowUp', 'ArrowLeft', 'r', 's', 'a', 'd', 'f', 'l']) {
    await windowHandlers.keydown(key(k));
  }
  assert.equal(writes.length, 0);
  assert.equal(windowHandlers.keyup, undefined);       // no held keyboard keys any more
  // X / A / B before the start only pick the mode Y will apply.
  await press(XB.A);
  assert.equal(elements.startMode.value, '1');
  await press(XB.X);
  assert.equal(elements.startMode.value, '0');
  await press(XB.B);
  assert.equal(elements.startMode.value, '2');
  assert.equal(writes.length, 0);

  // ---- Y: start (menus: full manual, no path) ----
  clock.t = 1000;
  await press(XB.Y);
  assert.equal(run('matchStarted'), true);
  assert.equal(writes.length, 0);                        // manual + None: nothing to send
  assert.equal(elements.matchTimer.textContent, '3:00');
  assert.match(elements.matchState.textContent, /比賽中/);
  assert.equal(elements.startMatch.disabled, true);
  assert.equal(elements.flywheelSlider.disabled, false);
  clock.t = 1000 + 60000;
  run('renderMatchTimer()');
  assert.equal(elements.matchTimer.textContent, '2:00');
  clock.t = 1000 + 179500;
  run('renderMatchTimer()');
  assert.equal(elements.matchTimer.textContent, '0:01');
  // A second Y does nothing.
  await press(XB.Y);
  assert.equal(writes.length, 0);

  // D-pad: right/left turret, up/down elevation, hold-to-move; release
  // stops. With no angle telemetry yet the exact-angle widgets are off.
  pads = [xbox([0, 0, 0, 0], [XB.RIGHT, XB.UP])];
  await poll(); await poll();                            // held: not repeated
  assert.deepEqual([...writes], ['ROT,1', 'ANG,1']);
  pads = [xbox()];
  await poll();
  assert.deepEqual([...writes].slice(-2), ['ROT,0', 'ANG,0']);
  assert.equal(elements.angleSlider.disabled, true);

  // The Mega reports the angle fields (TEL slots 29-31): the exact-angle
  // widgets turn on; the D-pad stays the hold-to-move jog.
  run('handleLine(' + JSON.stringify(['TEL', 1, 0, 0, 0, 0, 0, 1500, 1500, 1500, 1500, 1500, 0, 3, 1,
    'NaN', 'NaN', 'NaN', 1, 1500, 1500, 1, 0, 0, 0, 0, 1500, 0.0, 1500, 135.0, 0.0, 270.0].join(',')) + ')');
  assert.equal(elements.angleSlider.disabled, false);
  writes.length = 0;
  pads = [xbox([0, 0, 0, 0], [XB.DOWN])];
  await poll();
  assert.deepEqual([...writes], ['ANG,-1']);
  pads = [xbox()];
  await poll();
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

  // Typing in a real text box never drives the robot.
  writes.length = 0;
  for (const k of ['5', 'm']) await windowHandlers.keydown(key(k, { target: elements.angleInput }));
  assert.equal(writes.length, 0);
  assert.equal(armed(), true);

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
  // Disarm ends the match for this Enable (the countdown itself runs on).
  assert.equal(run('matchStarted'), false);
  assert.match(elements.matchState.textContent, /倒數中/);

  // Disarmed: heartbeat sends nothing, slider input is ignored.
  writes.length = 0;
  await run('sendHeartbeat()');
  elements.flywheelSlider.value = '70';
  await elements.flywheelSlider.handlers.input();
  assert.equal(writes.length, 0);
  assert.equal(elements.flywheelSlider.value, '0');

  // A new Enable waits for Y again; the countdown restarts from 3:00.
  await arm();
  assert.equal(run('matchStarted'), false);
  await run('sendHeartbeat()');
  assert.equal(last(), 'K');
  clock.t = 400000;
  run('renderMatchTimer()');
  assert.equal(elements.matchTimer.textContent, '0:00');   // the old one ran out
  assert.match(html, /#matchTimer\.done/);
  await start();
  assert.equal(elements.matchTimer.textContent, '3:00');
  clock.t = 400000 + 200000;
  run('renderMatchTimer()');
  assert.equal(elements.matchTimer.textContent, '0:00');   // at 0: just shown
  assert.match(elements.matchState.textContent, /時間到/);
  assert.equal(armed(), true);                              // nothing is disabled by it

  // Firmware-side disarms (watchdog, RC link lost, corrupt line) reset the
  // page too.
  for (const name of ['OUTPUTS_DISABLED', 'COMMAND_TIMEOUT', 'DISABLED_RC_LINK_LOST', 'DISABLED_CORRUPT_COMMAND']) {
    await arm();
    await start();
    elements.flywheelSlider.value = '50';
    await elements.flywheelSlider.handlers.input();
    pads = [xbox([0, 0, 0, 0], [XB.RIGHT])];
    await poll();
    event(name);
    assert.equal(armed(), false, name);
    assert.equal(run('flywheelRatio'), 0, name);
    assert.equal(run('rotateAxis'), 0, name);
    assert.equal(run('matchStarted'), false, name);
    pads = [xbox()];
    await poll();
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

  // Codex review afed238 [P1]: with the Falcon running, focus in a text
  // box -- Enter there must still stop.
  await arm();
  await start();
  elements.flywheelSlider.value = '40';
  await elements.flywheelSlider.handlers.input();
  writes.length = 0;
  await windowHandlers.keydown(key('Enter', { target: elements.angleInput }));
  assert.equal(last(), 'X');
  assert.equal(armed(), false);
  assert.equal(run('flywheelRatio'), 0);
  writes.length = 0;
  await run('sendHeartbeat()');
  assert.equal(writes.length, 0);  // no keepalive keeps anything alive

  // Disarmed: Enter/Space in a text box are ordinary typing again.
  await windowHandlers.keydown(key('Enter', { target: elements.angleInput }));
  await windowHandlers.keydown(key(' ', { target: elements.angleInput }));
  assert.equal(writes.length, 0);
  assert.notEqual(run('port'), null);

  // Same scenario with Space from the text box: stop, then disconnect.
  await arm();
  writes.length = 0;
  await windowHandlers.keydown(key(' ', { target: elements.angleInput }));
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

  // Pending: heartbeat keeps the link alive with K, keys and Start do
  // nothing, a second click doesn't resend E.
  writes.length = 0;
  await elements.enableControl.handlers.click();
  await elements.enableControl.handlers.click();
  assert.deepEqual([...writes], ['E']);
  assert.equal(run('enablePending'), true);
  await windowHandlers.keydown(key('5'));
  await start();
  await run('sendHeartbeat()');
  assert.deepEqual([...writes], ['E', 'K']);
  assert.equal(run('matchStarted'), false);

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

  // No confirmation at all within 5 s on the Mega link: give up, no X.
  writes.length = 0;
  clock.t = 1000000;
  await elements.enableControl.handlers.click();
  clock.t = 1004900;
  await run('sendHeartbeat()');
  assert.equal(run('enablePending'), true);
  clock.t = 1005100;
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
  assert.match(elements.rfLinkState.textContent, /up 100% \/ down 98% · RSSI -45 dBm/);
  line('$RFLINK,1,97,97,232');                     // signed byte from this ELRS TX
  assert.match(elements.rfLinkState.textContent, /RSSI -24 dBm/);
  line('$WIRELESS,1,0,0,1,0,1,1,80');
  assert.match(elements.wirelessState.textContent, /disarmed/);

  // Robot confirms via $WIRELESS (even if the $EVENT was coalesced away).
  // The menus picked before Enable wait for Y: the mode, then the path.
  clock.t = 1100000;
  writes.length = 0;
  elements.startMode.value = '1';                  // semi auto
  elements.autoPathSelect.value = '3';             // right 1
  await elements.enableControl.handlers.click();
  line('$GROUND_EVENT,ENABLE_REQUESTED');
  assert.equal(armed(), false);
  line('$WIRELESS,1,1,2,1,0,1,1,60');
  assert.equal(armed(), true);
  await new Promise((r) => setImmediate(r));
  assert.deepEqual([...writes], ['E']);            // Enabled, but nothing until Y
  await press(XB.Y);
  assert.deepEqual([...writes], ['E', 'MODE,1', 'AUTO,3']);
  assert.match(elements.controlOwner.textContent, /wireless/);
  // The robot reports the mode: the menu follows it while enabled.
  line('$WIRELESS,1,1,2,1,0,1,1,60,1');
  assert.equal(elements.startMode.value, '1');

  // No target/auto telemetry through the ground station: the page shows
  // what it sent, once the robot's event comes back over the uplink.
  assert.match(elements.targetHole.textContent, /none/);
  assert.match(elements.autoPath.textContent, /not started/);
  writes.length = 0;
  await windowHandlers.keydown(key('7'));
  assert.equal(last(), 'TGT,7');
  assert.match(elements.targetHole.textContent, /none/);     // not confirmed yet
  event('TARGET_SELECTED');
  assert.equal(elements.targetHole.textContent, '7 · 右下');
  event('AUTO_PATH_STARTED');                                 // the menu's path, sent at Y
  assert.match(elements.autoPath.textContent, /^已啟動 右一/);
  assert.match(elements.autoPathsUsed.textContent, /本頁送出紀錄：右一/);
  // During the match the mode menu and X / A / B switch right away.
  writes.length = 0;
  elements.startMode.value = '0';
  await elements.startMode.handlers.change();
  await press(XB.B);
  assert.deepEqual([...writes], ['MODE,0', 'MODE,2']);
  // Changing the path menu sends nothing (next Enable + Y).
  elements.autoPathSelect.value = '1';
  elements.autoPathSelect.handlers.change();
  assert.equal(writes.length, 2);
  elements.startMode.value = '2';
  elements.autoPathSelect.value = '0';
  // $AIM (the robot's aim frame over the radio): distance, elevation pulse
  // and the target line, without needing the Mega's USB.
  line('$AIM,1,352,1620,10,10,1,-15');
  assert.equal(elements.distance.textContent, '3.52 m');
  assert.match(elements.distanceRaw.textContent, /^3520 mm/);
  assert.equal(elements.anglePulse.textContent, '1620 us');
  assert.equal(elements.aimState.textContent, '視覺 #10 已看到 ✓ 3.52 m · 微調 水平 -60 px');  // 4 px units since 2026-10-30
  line('$AIM,1,0,1620,10,0,0,0');
  assert.equal(elements.distance.textContent, '--');
  assert.equal(elements.aimState.textContent, '視覺 #10 看不到 · 微調 水平 0 px');
  line('$AIM,1,410,1500,0,7,1,0');
  assert.equal(elements.aimState.textContent, 'AI 自選 #7 4.10 m · 微調 水平 0 px');
  line('$AIM,0,0,0,0,0,0,0');
  assert.equal(elements.distance.textContent, '--');
  assert.match(elements.distanceRaw.textContent, /收不到機器人視覺資料/);
  line('$AIM,1,abc');                                         // malformed: ignored
  assert.match(elements.distanceRaw.textContent, /收不到機器人視覺資料/);
  assert.equal(run('linkKind'), 'ground');
  // $VISION (the robot's vision frame): tx / ty in half pixels.
  line('$VISION,1,1290,-125,41,1,10');
  assert.equal(elements.tx.textContent, '-62.5');
  assert.equal(elements.ty.textContent, '20.5');
  assert.equal(elements.falconPwm.textContent, '1290 us (42.0%)');  // the Falcon's real pulse
  line('$VISION,1,0,0,0,0,0');                     // no valid target
  assert.equal(elements.tx.textContent, '--');
  line('$VISION,0,0,0,0,0,0');                     // stale
  assert.equal(elements.ty.textContent, '--');
  line('$VISION,1,x');                             // malformed: ignored
  assert.equal(run('linkKind'), 'ground');
  // The vision panel keeps only tx, ty and distance.
  for (const gone of ['targetId', 'valid', 'shooterReady', 'operatorMode', 'warnBits']) {
    assert.doesNotMatch(html, new RegExp(`id="${gone}"`), gone);
  }

  // RB (turret back to 0 deg, 2026-10-31) is wired only: over the ground
  // station it only explains that.
  writes.length = 0;
  await press(XB.RB);
  assert.equal(writes.length, 0);
  assert.match(run('log.textContent'), /RB 轉台回 0° 只支援有線/);

  // Stale status: warn only, keep driving (the robot's own RC-loss
  // failsafe is what stops it).
  line('$WIRELESS,0,0,0,0,0,0,0,1500');
  assert.equal(armed(), true);
  assert.match(elements.wirelessState.textContent, /no robot status/);

  // Status saying disarmed: it may be seconds old over ELRS, so the page
  // stays armed (keeps heartbeating) and logs the mismatch once; the
  // robot's event is what disarms it.
  elements.log.textContent = '';
  line('$WIRELESS,1,0,0,1,0,1,1,40');
  line('$WIRELESS,1,0,0,1,0,1,1,40');
  assert.equal(armed(), true);
  assert.equal((run('log.textContent').match(/robot status says disarmed/g) || []).length, 1);
  event('DISABLED_RC_LINK_LOST');
  assert.equal(armed(), false);

  // Same for a status naming the wired source: logged, not disarmed.
  await arm();
  line('$WIRELESS,1,1,1,1,0,1,1,40');
  assert.equal(armed(), true);
  assert.match(run('log.textContent'), /robot status says armed \(owner wired/);
  event('OUTPUTS_DISABLED');
  assert.equal(armed(), false);

  // RF log: lost / back once each, LQ low once until it recovers.
  elements.log.textContent = '';
  line('$RFLINK,0,0,0,0');
  line('$RFLINK,0,0,0,0');
  line('$RFLINK,1,40,90,50');
  line('$RFLINK,1,45,90,50');
  line('$RFLINK,1,95,95,50');
  line('$RFLINK,1,30,95,50');
  const rfLog = run('log.textContent');
  assert.equal((rfLog.match(/RF link lost/g) || []).length, 1);
  assert.equal((rfLog.match(/RF link back/g) || []).length, 1);
  assert.equal((rfLog.match(/RF LQ low/g) || []).length, 2);

  // Ground-side release (PC timeout etc.) disarms the page.
  for (const name of ['PC_TIMEOUT', 'ENABLE_RELEASED', 'RELEASED_CORRUPT_COMMAND', 'COMMAND_TOO_LONG']) {
    await arm();
    line(`$GROUND_EVENT,${name}`);
    assert.equal(armed(), false, name);
  }
  // The ground's stage-1 hold and its resume are log-only: still armed.
  await arm();
  line('$GROUND_EVENT,PC_HOLD');
  line('$GROUND_EVENT,PC_RESUMED,640');
  assert.equal(armed(), true);
  assert.match(elements.log.textContent, /ground: PC_RESUMED,640/);
  // Stall evidence: a late heartbeat tick is logged as a page stall; a
  // normal tick is not; nothing is logged while disarmed.
  elements.log.textContent = '';
  clock.t = 500000;
  await run('sendHeartbeat()');
  clock.t = 500075;
  await run('sendHeartbeat()');
  assert.doesNotMatch(elements.log.textContent, /PC_GAP/);
  clock.t = 500700;
  await run('sendHeartbeat()');
  assert.match(elements.log.textContent, /PC_GAP 625ms/);
  line('$GROUND_EVENT,PC_TIMEOUT');
  assert.equal(armed(), false);
  elements.log.textContent = '';
  clock.t = 509000;
  await run('sendHeartbeat()');
  // ...and the first tick after a new Enable doesn't count the idle time.
  await arm();
  clock.t = 512000;
  await run('sendHeartbeat()');
  assert.doesNotMatch(elements.log.textContent, /PC_GAP/);
  // A slow serial write is logged as USB backpressure.
  ctx.clock = clock;
  run('writer = { write: () => { clock.t += 400; return Promise.resolve(); }, releaseLock() {} }');
  await run('sendControl("K")');
  assert.match(elements.log.textContent, /WRITE_SLOW 400ms/);
  connectFakePort();
  line('$GROUND_EVENT,ENABLE_RELEASED');

  // A new Enable allows one more auto path and starts on the AI's own
  // target pick; the record stays.
  assert.match(elements.autoPath.textContent, /not started/);
  assert.match(elements.autoPathsUsed.textContent, /右一/);
  assert.match(elements.targetHole.textContent, /none/);

  // Ground link: a rejected or unconfirmed enable sends X so the ground
  // drops the arm channel it is still holding.
  writes.length = 0;
  await elements.enableControl.handlers.click();
  event('ENABLE_REJECTED_PINS_UNCONFIRMED');
  await new Promise((r) => setImmediate(r));
  assert.deepEqual([...writes], ['E', 'X']);
  writes.length = 0;
  clock.t = 1200000;
  await elements.enableControl.handlers.click();
  clock.t = 1205500;
  await run('sendHeartbeat()');
  assert.deepEqual([...writes], ['E', 'X']);
  assert.equal(run('enablePending'), false);

  // ---------------- operator modes (X / A / B) ----------------
  connectFakePort();
  run('setConnection(true)');
  clock.t = 1300000;
  await arm();
  await start();
  writes.length = 0;
  pads = [xbox([0, 0, 0, 0], [XB.A])];
  await poll(); await poll();                          // held: one press
  pads = [xbox()];
  await poll();
  assert.deepEqual([...writes], ['MODE,1']);
  // The robot confirms; the page shows it and locks the Falcon slider
  // (the shot table drives the flywheel in the auto modes).
  event('MODE_SEMI_AUTO');
  assert.match(elements.controlArmedBadge.textContent, /半自動/);
  assert.match(run('MODE_NAMES[operatorMode]'), /SEMI AUTO/);
  assert.equal(elements.flywheelSlider.disabled, true);
  writes.length = 0;
  elements.flywheelSlider.value = '60';
  await elements.flywheelSlider.handlers.input();
  assert.equal(writes.length, 0);
  assert.equal(elements.flywheelSlider.value, '0');
  // Hole keys still go out in the auto modes (semi-auto aims at the hole).
  await windowHandlers.keydown(key('6'));
  assert.equal(last(), 'TGT,6');
  event('MODE_FULL_AUTO');
  assert.match(run('MODE_NAMES[operatorMode]'), /FULL AUTO/);
  event('MODE_FULL_MANUAL');
  assert.equal(elements.flywheelSlider.disabled, false);
  // TEL slot 13 is mode + 1: 2 = semi auto; 0 (older firmware) is ignored.
  const telMode = (slot13) => ['TEL', 1, 0, 0, 0, 0, 0, 1500, 1500, 1500, 1500, 1500, 0, slot13, 1,
    'NaN', 'NaN', 'NaN', 1, 1500, 1500, 1, 0, 0, 0, 0, 1500, 0.0].join(',');
  line(telMode(2));
  assert.match(run('MODE_NAMES[operatorMode]'), /SEMI AUTO/);
  line(telMode(0));
  assert.match(run('MODE_NAMES[operatorMode]'), /SEMI AUTO/);   // not "FULL AUTO"
  // Any disarm drops the page back to full manual, like the firmware.
  await windowHandlers.keydown(key('Enter'));
  assert.equal(armed(), false);
  assert.match(run('MODE_NAMES[operatorMode]'), /FULL MANUAL/);
  // Ground station: $WIRELESS turns the exact-angle widgets off (the radio
  // carries only up/stop/down); the D-pad's ANG jog is unchanged.
  // $WIRELESS field 10 carries the mode (-1 = not reported).
  connectFakePort();
  run('setConnection(true)');
  clock.t = 1400000;
  await elements.enableControl.handlers.click();
  line('$WIRELESS,1,1,2,1,0,1,1,50,2');
  assert.equal(armed(), true);
  assert.match(run('MODE_NAMES[operatorMode]'), /FULL MANUAL/);
  line('$WIRELESS,1,1,2,1,0,1,1,50,0');
  assert.match(run('MODE_NAMES[operatorMode]'), /FULL AUTO/);
  line('$WIRELESS,1,1,2,1,0,1,1,50,-1');
  assert.match(run('MODE_NAMES[operatorMode]'), /FULL AUTO/);   // unknown: unchanged
  assert.equal(elements.angleSlider.disabled, true);
  await start();
  writes.length = 0;
  pads = [xbox([0, 0, 0, 0], [XB.UP])];
  await poll();
  assert.equal(last(), 'ANG,1');
  pads = [xbox()];
  await poll();
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

  // Slots 32-34: target hole, auto path this Enable, paths used since boot.
  const telTarget = (hole, path, mask) => ['TEL', 1, 0, 0, 0, 0, 0, 1500, 1500, 1500, 1500, 1500, 0, 3, 1,
    'NaN', 'NaN', 'NaN', 0, 1500, 1500, 0, 0, 0, 0, 0, 1500, 0.0, 2310, 243.5, 0.0, 270.0, hole, path, mask].join(',');
  line(telTarget(5, 1, (1 << 1) | (1 << 4)));
  assert.equal(elements.targetHole.textContent, '5 · 中');
  assert.match(elements.autoPath.textContent, /^已啟動 左一/);
  assert.equal(elements.autoPathsUsed.textContent, '開機後已用：左一、右二');
  line(telTarget(0, 0, 1 << 4));
  assert.match(elements.targetHole.textContent, /none/);
  assert.match(elements.autoPath.textContent, /not started/);
  assert.equal(elements.autoPathsUsed.textContent, '開機後已用：右二');
  // Slots 35-37: vision id sent to the Orin + fine-adjust; slots 5/6 say
  // whether that hole is in view.
  const telAim = (frameId, valid, visionId, aim, elev) => ['TEL', 1, 0, 0, 3500, frameId, valid, 1500, 1500, 1500, 1500, 1500, 0, 2, 1,
    'NaN', 'NaN', 'NaN', 0, 1500, 1500, 0, 0, 0, 0, 0, 1500, 0.0, 2310, 243.5, 0.0, 270.0, 1, 0, 0, visionId, aim, elev].join(',');
  line(telAim(10, 1, 10, 5, 0));
  assert.equal(elements.aimState.textContent, '視覺 #10 已看到 ✓ 3.50 m · 微調 水平 +5 px · 仰角 0 µs');
  // The vision panel: meters, plus the raw mm.
  assert.equal(elements.distance.textContent, '3.50 m');
  assert.match(elements.distanceRaw.textContent, /^3500 mm/);
  line(telAim(0, 0, 10, -15, 20));
  assert.equal(elements.aimState.textContent, '視覺 #10 看不到 · 微調 水平 -15 px · 仰角 +20 µs');
  // No valid target: "--", never "0 m".
  assert.equal(elements.distance.textContent, '--');
  assert.match(elements.distanceRaw.textContent, /看不到目標/);
  line(telAim(11, 1, 0, 0, 0));
  assert.equal(elements.aimState.textContent, 'AI 自選 #11 3.50 m · 微調 水平 0 px · 仰角 0 µs');
  // Older firmware (no slots 35-37) leaves it alone.
  line(telTarget(0, 0, 1 << 4));
  assert.match(elements.aimState.textContent, /^AI 自選/);

  // With the robot reporting, events no longer overwrite it from the page.
  event('TARGET_SELECTED');
  assert.match(elements.targetHole.textContent, /none/);

  // ---------------- gamepad driving ----------------
  connectFakePort();
  run('setConnection(true)');
  clock.t = 1500000;
  pads = [];
  await poll();
  assert.match(elements.gamepadState.textContent, /none/);

  // Disarmed: the sticks, LB and Y send nothing.
  pads = [xbox([0, -1, 1, 0], [XB.LB, XB.Y])];
  writes.length = 0;
  await poll();
  assert.equal(writes.length, 0);
  assert.match(elements.gamepadState.textContent, /Xbox/);
  assert.match(elements.gamepadInputs.textContent, /按鈕 \[3,4\]/);
  pads = [xbox()];
  await poll();

  // Enabled and started with the stick already pushed: ignored until it
  // is centered once.
  await arm();
  pads = [xbox([0, 0, 1, 0], [XB.Y])];        // Y while holding the turn stick
  await poll();
  assert.equal(run('matchStarted'), true);
  writes.length = 0;
  pads = [xbox([0, 0, 1, 0])];
  await poll(); await poll();
  assert.equal(writes.length, 0);
  pads = [xbox()];
  await poll();
  // Right stick X = turn: right = left wheel forward (flipped after the
  // 2026-10-17 robot test); deadzone-rescaled analog, then x TURN_SPEED_SCALE
  // 0.6 (2026-10-27: turning slower, forward / back untouched).
  pads = [xbox([0, 0, 1, 0])];
  await poll();
  assert.equal(last(), 'T,0.6,-0.6');
  pads = [xbox([0, 0, 0.575, 0])];            // half way past the deadzone
  await poll();
  assert.equal(last(), 'T,0.3,-0.3');
  pads = [xbox([0, -1, 0, 0])];               // left stick up = forward
  await poll();
  assert.equal(last(), 'T,1,1');
  pads = [xbox([0.1, -0.1, 0.1, 0])];         // inside the deadzone = stop
  await poll();
  assert.equal(last(), 'T,0,0');
  // Left stick X and right stick Y drive nothing.
  writes.length = 0;
  pads = [xbox([1, 0, 0, 1])];
  await poll();
  assert.equal(writes.length, 0);
  pads = [xbox()];
  await poll();

  // Fire (LB): one F per press, none while held; Y never fires.
  writes.length = 0;
  pads = [xbox([0, 0, 0, 0], [XB.LB])];
  await poll(); await poll(); await poll();
  pads = [xbox()];
  await poll();
  pads = [xbox([0, 0, 0, 0], [XB.LB])];
  await poll();
  pads = [xbox([0, 0, 0, 0], [XB.Y])];
  await poll();
  assert.deepEqual([...writes], ['F', 'F']);
  pads = [xbox()];
  await poll();

  // Unplugged while driving and jogging: everything drops to zero.
  pads = [xbox([0, 0, -1, 0], [XB.LEFT])];
  await poll();
  assert.ok(writes.includes('T,-0.6,0.6') && writes.includes('ROT,-1'));
  pads = [];
  await poll();
  assert.ok(writes.slice(-2).includes('T,0,0') && writes.slice(-2).includes('ROT,0'));

  // A non-standard pad (flight stick): axes 0/1, fire = button 0; no
  // start or mode buttons (the Start button / menu do that).
  writes.length = 0;
  pads = [pad('', [0, 0], [0])];
  await poll();
  assert.deepEqual([...writes], ['F']);
  assert.match(elements.gamepadState.textContent, /flight stick/);

  // The team's pad (Vendor 11c1, D-input): its own profile -- Y (4)
  // starts, right stick X (axis 2) turns, the hat (axis 9) jogs, RB (7) fires.
  await windowHandlers.keydown(key('Enter'));
  connectFakePort();
  run('setConnection(true)');
  const pad11 = (axes, pressed = []) => {
    const a = Array(10).fill(0); a[9] = 1.29;            // hat released
    axes.forEach((v, i) => { if (v !== undefined) a[i] = v; });
    return { connected: true, id: 'Controller (Vendor: 11c1 Product: 9101)', mapping: '', axes: a,
      buttons: Array.from({ length: 16 }, (_, i) => ({ pressed: pressed.includes(i), value: 0 })) };
  };
  pads = [pad11([])];
  await poll();
  assert.match(elements.gamepadState.textContent, /Xbox \(11c1\)/);
  await arm();
  writes.length = 0;
  pads = [pad11([], [4])]; await poll();                // Y: start
  pads = [pad11([])]; await poll();
  assert.equal(run('matchStarted'), true);
  pads = [pad11([undefined, undefined, 1])]; await poll(); // right stick right
  assert.equal(last(), 'T,0.6,-0.6');
  pads = [pad11([0, -1, 0])]; await poll();             // left stick up
  assert.equal(last(), 'T,1,1');
  pads = [pad11([])]; await poll();
  writes.length = 0;
  pads = [pad11([undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, undefined, -1])];
  await poll();                                          // hat up
  assert.deepEqual([...writes], ['ANG,1']);
  pads = [pad11([])]; await poll();
  assert.equal(last(), 'ANG,0');
  writes.length = 0;
  pads = [pad11([], [6])]; await poll();                // LB fires
  pads = [pad11([], [9])]; await poll();                // RT: Falcon +1% (full manual)
  pads = [pad11([], [9])]; await poll();                // held: nothing more
  pads = [pad11([])]; await poll();
  pads = [pad11([], [9])]; await poll();                // again: +1%
  pads = [pad11([], [8])]; await poll();                // LT: -1%
  assert.deepEqual([...writes], ['F', 'FLY,0.01', 'FLY,0.02', 'FLY,0.01']);
  writes.length = 0;
  pads = [pad11([], [3])]; await poll();                // X: full auto
  assert.deepEqual([...writes], ['MODE,0']);
  line('$EVENT,MODE_FULL_AUTO');
  // In the autos LT / RT are an offset on the shot table (FLYO).
  writes.length = 0;
  pads = [pad11([], [9])]; await poll();
  pads = [pad11([])]; await poll();
  pads = [pad11([], [9])]; await poll();
  pads = [pad11([])]; await poll();
  pads = [pad11([], [8])]; await poll();
  assert.deepEqual([...writes], ['FLYO,1', 'FLYO,2', 'FLYO,1']);
  assert.match(elements.flywheelSliderValue.textContent, /射擊表轉速微調 \+1%/);
  // A new hole starts from its own table value: offset back to 0.
  writes.length = 0;
  await windowHandlers.keydown(key('5'));
  assert.deepEqual([...writes], ['TGT,5', 'FLYO,0']);
  // Clamped at +-20.
  for (let i = 0; i < 25; i++) { pads = [pad11([], [9])]; await poll(); pads = [pad11([])]; await poll(); }
  assert.equal(run('flywheelOffsetPct'), 20);
  // A mode change drops it (the robot does too).
  line('$EVENT,MODE_SEMI_AUTO');
  assert.equal(run('flywheelOffsetPct'), 0);
  pads = [pad11([])]; await poll();

  // Disarm clears the gamepad drive.
  pads = [pad('', [0, -1])];
  await poll();
  assert.equal(last(), 'T,1,1');
  await windowHandlers.keydown(key('Enter'));
  writes.length = 0;
  await poll();
  assert.equal(writes.length, 0);

  // ---- Test mode + the S-key shot log (2026-10-26) ----
  pads = [];
  await poll();
  await arm();
  await start();
  writes.length = 0;
  elements.startMode.value = '3';                    // menu: test mode
  await elements.startMode.handlers.change();
  assert.equal(last(), 'MODE,3');
  event('MODE_TEST');
  assert.equal(run('operatorMode'), 3);
  assert.match(run('MODE_NAMES[operatorMode]'), /TEST/);
  assert.equal(elements.flywheelSlider.disabled, false); // the operator's flywheel, as in manual
  elements.flywheelSlider.value = '46';
  await elements.flywheelSlider.handlers.input();
  assert.equal(last(), 'FLY,0.46');
  // LT / RT step the slider itself (not FLYO) in test mode.
  await run('nudgeFlywheel(1)');
  assert.equal(last(), 'FLY,0.47');
  await run('nudgeFlywheel(-1)');
  assert.equal(last(), 'FLY,0.46');
  assert.equal(run('holeForVisionId(10)'), 1);
  assert.equal(run('holeForVisionId(8)'), 5);
  assert.equal(run('holeForVisionId(6)'), 9);
  assert.equal(run('holeForVisionId(0)'), 0);
  assert.equal(run('holeForVisionId(1)'), 10);          // bonus left / middle / right
  assert.equal(run('holeForVisionId(3)'), 12);
  assert.equal(run('holeForVisionId(13)'), 0);

  // Wired TEL with every slot: hole 5 = vision id 8, Falcon 1730 us = 46%.
  const telShot = ({ distance = 2500, targetId = 8, valid = 1, selected = 8, falcon = 1730, angle = 2063, mode = 3 } = {}) =>
    ['TEL', 1, 0, 0, distance, targetId, valid, 1500, 1500, 1500, 1500, 1500, 0, mode + 1, 0,
      'NaN', 'NaN', 'NaN', 1, 1500, 1500, 1, 0, 0, 2, 1, falcon, Math.abs(falcon - 1500) / 5, angle,
      'NaN', 'NaN', 'NaN', 5, 0, 0, selected, 0, 0, 20].join(',');
  const shots = () => run('shotRecords.length');
  run('shotRecords = []');
  writes.length = 0;
  for (const mm of [2480, 2700, 2500, 2520, 2490]) { clock.t += 100; line(telShot({ distance: mm })); }
  assert.equal(run('operatorMode'), 3);               // TEL slot 13 = 3 + 1
  await windowHandlers.keydown(key('s'));
  assert.equal(shots(), 1);
  assert.equal(writes.length, 0);                     // S never talks to the robot
  const rec = run('shotRecords[0]');
  assert.equal(rec.hole, 5);
  assert.equal(rec.distanceCm, 250);                  // median of 2480/2490/2500/2520/2700
  assert.equal(rec.elevationUs, 2063);
  assert.equal(rec.flywheelPct, 46);
  assert.equal(rec.mode, 3);
  assert.equal(rec.falconUs, 1730);
  assert.equal(rec.samples, 5);
  // Held S doesn't repeat.
  await windowHandlers.keydown(key('s', { repeat: true }));
  assert.equal(shots(), 1);
  // Same hole, same distance: replaces it (re-measurement).
  elements.flywheelSlider.value = '48';
  await elements.flywheelSlider.handlers.input();
  clock.t += 100; line(telShot({ falcon: 1740 }));
  await windowHandlers.keydown(key('s'));
  assert.equal(shots(), 1);
  assert.equal(run('shotRecords[0].flywheelPct'), 48);
  // Still ramping: refused.
  elements.flywheelSlider.value = '60';
  await elements.flywheelSlider.handlers.input();
  clock.t += 100; line(telShot({ falcon: 1700 }));    // 40% actual vs 60% commanded
  await windowHandlers.keydown(key('s'));
  assert.equal(shots(), 1);
  assert.match(elements.log.textContent, /飛輪還在變速/);
  // The camera lost the hole for longer than the window: refused.
  clock.t += 100; line(telShot({ falcon: 1800, valid: 0 }));
  clock.t += 1600; line(telShot({ falcon: 1800, valid: 0 }));
  await windowHandlers.keydown(key('s'));
  assert.equal(shots(), 1);
  assert.match(elements.log.textContent, /鏡頭沒看到洞口 5/);
  // A frame of another hole never counts.
  clock.t += 100; line(telShot({ falcon: 1800, targetId: 9, distance: 3000 }));
  await windowHandlers.keydown(key('s'));
  assert.equal(shots(), 1);
  // No hole selected (the AI's pick): refused.
  clock.t += 100; line(telShot({ falcon: 1800, selected: 0, targetId: 8, distance: 3000 }));
  await windowHandlers.keydown(key('s'));
  assert.equal(shots(), 1);
  assert.match(elements.log.textContent, /先按 1–9 或 Q W E 選洞口/);
  // Still moving (2.5 m -> 3 m inside the window): refused.
  for (const mm of [2500, 2600, 2700, 2800, 2900]) { clock.t += 100; line(telShot({ falcon: 1800, distance: mm })); }
  await windowHandlers.keydown(key('s'));
  assert.equal(shots(), 1);
  assert.match(elements.log.textContent, /距離還不穩（1\.5 秒內在 2\.50–2\.90 m 之間）/);
  // A second point once it stands at 3.00 m, 60%.
  clock.t += 1600;
  clock.t += 100; line(telShot({ falcon: 1800, distance: 3004 }));
  await windowHandlers.keydown(key('s'));
  assert.equal(shots(), 2);
  // The CSV: the Excel's five columns first, exactly apply.py's headers.
  const csv = run('shotLogCsv()').split('\r\n');
  assert.equal(csv[0], '洞口,距離(m),仰角(us),轉速,轉速單位,模式,紀錄時間,Falcon實際(us),距離樣本數');
  assert.match(csv[1], /^5,2\.50,2063,48,%,TEST \/ 測試,\d{4}-\d\d-\d\d \d\d:\d\d:\d\d,1740,\d+$/);
  assert.match(csv[2], /^5,3\.00,2063,60,%,/);
  // The autos: the flywheel comes from the Falcon's actual pulse.
  clock.t += 1600;                                     // the 3 m frames age out of the window
  for (const mm of [4000, 4010]) { clock.t += 100; line(telShot({ falcon: 1731, distance: mm, mode: 1 })); }
  assert.equal(run('operatorMode'), 1);
  await windowHandlers.keydown(key('s'));
  assert.equal(run('shotRecords.at(-1).distanceCm'), 401);
  assert.equal(run('shotRecords.at(-1).flywheelPct'), 46.2);
  // Undo removes the newest.
  await elements.shotLogUndo.handlers.click();
  assert.equal(shots(), 2);
  // Clear needs two clicks.
  await elements.shotLogClear.handlers.click();
  assert.equal(shots(), 2);
  await elements.shotLogClear.handlers.click();
  assert.equal(shots(), 0);
  // Over the radio: $AIM (distance cm, elevation, selected / frame id) and
  // $VISION (Falcon pulse), as the ground station relays them.
  clock.t += 1600;
  run('setOperatorMode(3)');
  elements.flywheelSlider.value = '46';
  await elements.flywheelSlider.handlers.input();
  line('$AIM,1,312,1980,4,4,1,0');                    // hole 3 (vision id 4), 3.12 m
  line('$VISION,1,1730,0,0,1,4');
  await windowHandlers.keydown(key('s'));
  assert.equal(shots(), 1);
  assert.deepEqual([run('shotRecords[0].hole'), run('shotRecords[0].distanceCm'),
    run('shotRecords[0].elevationUs'), run('shotRecords[0].flywheelPct')], [3, 312, 1980, 46]);
  // Stale radio data (the ground's fresh = 0) is never used.
  clock.t += 1600;
  line('$AIM,0,0,0,0,0,0,0');
  await windowHandlers.keydown(key('s'));
  assert.equal(shots(), 1);
  await windowHandlers.keydown(key('Enter'));

  // Target board: the hole in the camera's valid frame is marked "seen",
  // its readouts mirror the Manual Control ones; a bonus hole records as
  // hole 10-12.
  clock.t += 3000;
  await arm();
  await start();
  run('setOperatorMode(3)');
  elements.flywheelSlider.value = '55';
  await elements.flywheelSlider.handlers.input();
  for (const mm of [3500, 3510, 3505]) {
    clock.t += 100; line(telShot({ distance: mm, targetId: 2, selected: 2, falcon: 1775, angle: 2300 }));
  }
  assert.equal(run('seenHole'), 11);
  assert.equal(elements.boardElevation.textContent, '2300 us');
  assert.equal(elements.boardFlywheel.textContent, '1775 us (55.0%)');
  assert.equal(elements.boardFlywheelCmd.textContent, '指令 55%');
  assert.equal(elements.boardDistance.textContent, '3.50 m');      // the latest frame (3505 mm)
  run('shotRecords = []');
  await windowHandlers.keydown(key('s'));
  assert.equal(run('shotRecords[0].hole'), 11);
  clock.t += 100; line(telShot({ valid: 0, targetId: 0, selected: 2, falcon: 1775 }));
  assert.equal(run('seenHole'), 0);
  // Over the radio the same, from $AIM.
  line('$AIM,1,280,2100,10,10,1,0');
  assert.equal(run('seenHole'), 1);
  line('$AIM,0,0,0,0,0,0,0');
  assert.equal(run('seenHole'), 0);
  run('shotRecords = []');

  // TEL slot 39 (2026-10-27): Mega <-> bridge lines dropped since boot.
  // (Each TEL line is also echoed into the log, hence the counts.)
  const telDrop = (n) => telShot({ falcon: 1490 }) + `,${n}`;
  const drops = () => (elements.log.textContent.match(/UART_DROP/g) || []).length;
  clock.t += 3000; line(telDrop(0));
  assert.match(elements.rfLinkState.textContent, /丟棄 0 行/);
  const before = drops();
  clock.t += 100; line(telDrop(7));
  assert.equal(drops(), before + 1);
  assert.match(elements.log.textContent, /UART_DROP \+7（共 7 行/);
  clock.t += 100; line(telDrop(9));                   // within 2 s: not logged again
  assert.equal(drops(), before + 1);
  clock.t += 2100; line(telDrop(12));
  assert.equal(drops(), before + 2);
  assert.match(elements.log.textContent, /UART_DROP \+5（共 12 行/);
  assert.match(elements.rfLinkState.textContent, /丟棄 12 行/);
  await windowHandlers.keydown(key('Enter'));

  // TEL slots 40-44 (2026-10-27): fire conditions and the turret AS5600.
  const telFull = (ready, shotCm, raw, status, deg10, mode = 1) =>
    telShot({ mode }) + `,0,${ready},${shotCm},${raw},${status},${deg10}`;
  run('setOperatorMode(1)');
  clock.t += 100; line(telFull(5, 305, 1234, 3, -125));
  assert.match(elements.boardReady.textContent, /轉台鎖定 ✓ · 飛輪轉穩 ✗ · 有射擊值 ✓ · 等待中 · 射擊距離（穩定後）3\.05 m/);
  assert.equal(elements.boardTurret.textContent, '-12.5°');
  assert.equal(elements.boardTurretRaw.textContent, 'raw 1234 · OK');
  clock.t += 100; line(telFull(15, 305, 1234, 0x01, 0));
  assert.match(elements.boardReady.textContent, /可發射/);
  assert.match(elements.boardTurretRaw.textContent, /沒偵測到磁鐵/);
  assert.equal(elements.boardTurret.textContent, '--');            // no magnet: no angle
  clock.t += 100; line(telFull(0, 0, 0, 0x20, 0));
  assert.equal(elements.boardTurret.textContent, '--');
  assert.match(elements.boardTurretRaw.textContent, /沒讀到感測器/);
  clock.t += 100; line(telFull(0, 0, 4000, 0x13, 853, 2));
  assert.match(elements.boardReady.textContent, /只在半自動／全自動/);
  assert.match(elements.boardTurretRaw.textContent, /圈數不可信/);
  assert.equal(elements.boardTurret.textContent, '--');            // turns lost: no angle
  // AS5600 magnet strength (TEL 50-51): AGC, MAGNITUDE and their 10 s span.
  const telMagnet = (status, agc, mag) =>
    telFull(0, 0, 159, status, 0) + `,0,0,-1,0,4,${agc},${mag}`;
  clock.t += 100; line(telMagnet(0x07, 255, 900));
  assert.equal(elements.boardTurretMagnet.textContent,
    '磁場 AGC 255（10 秒內 255–255，越大越弱）· 強度 900（900–900）· 太弱：磁鐵再靠近');
  clock.t += 100; line(telMagnet(0x03, 140, 1800));
  assert.equal(elements.boardTurretMagnet.textContent,
    '磁場 AGC 140（10 秒內 140–255，越大越弱）· 強度 1800（900–1800）');
  clock.t += 11000; line(telMagnet(0x03, 150, 1700));             // older samples drop out
  assert.match(elements.boardTurretMagnet.textContent, /10 秒內 150–150/);
  clock.t += 100; line(telMagnet(0x03, -1, -1));
  assert.equal(elements.boardTurretMagnet.textContent, '磁場：讀不到 AGC');
  // "Too weak" with the magnet seen: a warning, the angle still shows (2026-10-31).
  clock.t += 100; line(telFull(0, 0, 159, 0x07, 0));
  assert.equal(elements.boardTurret.textContent, '0.0°');
  assert.equal(elements.boardTurretRaw.textContent, 'raw 159 · 磁鐵太遠（讀值可用，只警告）');
  clock.t += 100; line(telFull(0, 0, 159, 0x05, 0));            // too weak, no magnet seen
  assert.equal(elements.boardTurretRaw.textContent, 'raw -- · 沒偵測到磁鐵');  // no raw either
  assert.equal(elements.boardTurret.textContent, '--');            // no magnet: no angle
  // A ground on another channel table: the event, plus what to do.
  event('LINK_VERSION_MISMATCH');
  assert.match(elements.log.textContent, /通道表版本不一致/);
  // Board colors: several detected holes (v3 mask), selected wins blue.
  run('seenMaskHoles = (1 << 4) | (1 << 5) | (1 << 11); renderSeen()');
  run('targetHole = 5; renderTargetAndAuto()');
  assert.equal(run('seenMaskHoles'), (1 << 4) | (1 << 5) | (1 << 11));

  // Orin v3 detected holes: $SEEN (radio) and TEL slots 45-46 (USB).
  line('$SEEN,1,' + ((1 << 3) | (1 << 4) | (1 << 1)) + ',1');   // Orin ids 4, 5, 2
  assert.equal(run('seenMaskHoles'), (1 << 3) | (1 << 6) | (1 << 11)); // holes 3, 6, 11
  line('$SEEN,1,0,2');
  assert.match(elements.targetBoardState.textContent, /推算位置/);
  line('$SEEN,0,4095,1');                                         // stale: nothing lit
  assert.equal(run('seenMaskHoles'), 0);
  clock.t += 100; line(telFull(0, 0, 0, 3, 0) + ',' + (1 << 7) + ',1');  // Orin id 8 = hole 5
  assert.equal(run('seenMaskHoles'), 1 << 5);
  // 2026-10-28: the depth ring (TEL 47-48) ...
  clock.t += 100; line(telFull(0, 0, 0, 3, 0) + ',' + (1 << 7) + ',1,62,' + 0b11100011);
  assert.deepEqual({ ...run('ringShown') }, { percent: 62, sectors: 0b11100011 });
  assert.equal(elements.ringValue.textContent, '有效 62%');
  assert.equal(elements.ringDetail.textContent, '下、左下、左側沒有深度');
  clock.t += 100; line(telFull(0, 0, 0, 3, 0) + ',0,0,-1,0');
  assert.equal(elements.ringValue.textContent, '--');
  // ... and over the radio, with the turret angle (no raw value there).
  line('$SEEN,1,16,1,-503,3,12,1');
  assert.equal(elements.boardTurret.textContent, '-50.3°');
  assert.equal(elements.boardTurretRaw.textContent, '無線 · OK');
  assert.match(elements.ringDetail.textContent, /右下、下、左下、左、左上、上、右上側沒有深度 · 低於 20%/);
  line('$SEEN,1,16,1,0,32,100,255');
  assert.match(elements.boardTurretRaw.textContent, /無線 · 沒讀到感測器/);
  assert.equal(elements.ringDetail.textContent, '整圈都有深度');
  line('$SEEN,0,0,0,0,0,-1,0');
  assert.equal(elements.boardTurret.textContent, '--');
  assert.equal(elements.boardTurretRaw.textContent, '無線：沒有回傳');
  assert.equal(elements.ringValue.textContent, '--');
  line('$SEEN,1,16,1');                                           // older ground: board only
  assert.equal(elements.boardTurretRaw.textContent, '無線：沒有回傳');
  // 2026-10-31: the turret guard -- TEL 49 and the 9th $AIM field.
  clock.t += 100; line(telFull(0, 0, 0, 3, 0) + ',0,0,-1,0,5');
  assert.equal(elements.boardTurretGuard.textContent, '到右限位 +36°（只能往左）');
  line('$AIM,1,352,1620,10,10,1,-15,0');
  assert.match(elements.boardTurretGuard.textContent, /^位置未知：轉台鎖住/);
  line('$AIM,0,0,0,0,0,0,0,0');
  assert.equal(elements.boardTurretGuard.textContent, '--');
  // 「設為歸位點」: only on the Mega's USB and disabled.
  writes.length = 0;
  await run('setTurretZero()');                                     // the ground is the link now
  assert.equal(writes.length, 0);
  assert.match(run('log.textContent'), /只能用 USB 有線連 Mega/);
  clock.t += 100; line(telFull(0, 0, 0, 3, 0) + ',0,0,-1,0,4');     // the Mega's USB again
  assert.equal(elements.boardTurretGuard.textContent, '正常 · 限位 −136°～+36°');
  clock.t += 100; line(telFull(0, 0, 0, 3, 0) + ',0,0,-1,0,7');     // guard switched off
  assert.match(elements.boardTurretGuard.textContent, /^轉台保護關閉/);
  run('controlArmed = true');
  await run('setTurretZero()');
  assert.equal(writes.length, 0);
  assert.match(run('log.textContent'), /要先 Disable/);
  run('controlArmed = false');
  await run('setTurretZero()');
  assert.equal(last(), 'TZERO');
  event('TURRET_ZERO_SET');
  assert.match(run('log.textContent'), /已設定歸位點/);
  event('TURRET_ZERO_REJECTED');
  assert.match(run('log.textContent'), /設定歸位點被拒絕/);

  console.log('dashboard page control behavior tests passed');
})().catch((e) => { console.error(e); process.exitCode = 1; });

// The realtime trace and the vision stream panels were removed (Jeremy,
// 2026-10-16).
assert.doesNotMatch(html, /data-panel="(trace|stream)"|id="(chart|streamUrl|streamImage)"/);

// The mode and auto-path menus live in their own panel (Jeremy, 2026-10-16),
// not inside the manual control panel.
{
  const setup = html.slice(html.indexOf('data-panel="setup"'), html.indexOf('data-panel="control"'));
  assert.match(setup, /id="startMode"/);
  assert.match(setup, /id="autoPathSelect"/);
  const control = html.slice(html.indexOf('data-panel="control"'));
  assert.doesNotMatch(control, /id="startMode"|id="autoPathSelect"/);
}

// Enable, the Falcon slider and the elevation set live in their own panel;
// the safety panel is gone (Jeremy, 2026-10-16).
{
  const operate = html.slice(html.indexOf('data-panel="operate"'), html.indexOf('data-panel="control"'));
  for (const id of ['enableControl', 'controlArmedBadge', 'flywheelSlider', 'angleSlider', 'angleInput', 'angleSet']) {
    assert.match(operate, new RegExp(`id="${id}"`), id);
  }
  assert.doesNotMatch(html.slice(html.indexOf('data-panel="control"')), /id="(enableControl|flywheelSlider|angleSet)"/);
  assert.doesNotMatch(html, /data-panel="safety"|id="risk"/);
}

// Manual Control readout order (Jeremy, 2026-10-17): row 1 target hole,
// Falcon, elevation, wireless; row 2 the rest; last the wheel pulses and
// the gamepad (4 per row).
{
  const control = html.slice(html.indexOf('data-panel="control"'));
  const grid = control.slice(0, control.indexOf('</section>'));
  const order = [...grid.matchAll(/class="value" id="([A-Za-z]+)"/g)].map((m) => m[1]);
  assert.deepEqual(order, ['targetHole', 'falconPwm', 'anglePulse', 'wirelessState',
    'controlOwner', 'towerState', 'autoPath', 'shooterAxisMode', 'chassisPulses', 'gamepadState']);
}
