// Focused host test for tools/dashboard/index.html's pure control
// functions: dpadAxes (Xbox D-pad -> turret / elevation), the Xbox button
// map, and targetForKey (keys 1-9) -- Jeremy, 2026-10-17.
//
// The dashboard page itself (field drawing, draggable panel layout) is
// exercised as a whole in dashboard_page_test.cjs; this extracts just the
// pure functions by source text and evaluates them in isolation, where the
// easy-to-get-wrong mappings live: the D-pad signs, which Xbox button does
// what, and Jeremy's hole numbering (columns left to right, each bottom-up).
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const html = fs.readFileSync(path.join(__dirname, '../tools/dashboard/index.html'), 'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];

function extractFunction(name) {
  const marker = `function ${name}(`;
  const start = script.indexOf(marker);
  assert.ok(start >= 0, `could not find function ${name} in tools/dashboard/index.html`);
  let depth = 0;
  let i = script.indexOf('{', start);
  const bodyStart = i;
  for (; i < script.length; i++) {
    if (script[i] === '{') depth++;
    else if (script[i] === '}') {
      depth--;
      if (depth === 0) break;
    }
  }
  return script.slice(start, i + 1);
}

const src = [
  `const HAT_DIRECTIONS = ${script.match(/const HAT_DIRECTIONS = (\[[^\n]*\]);/)[1]};`,
  extractFunction('hatAxes'),
  extractFunction('dpadAxes'),
  extractFunction('targetForKey'),
].join('\n');

const ctx = vm.createContext({ Math, Number });
vm.runInContext(`${src}\nresult = { dpadAxes, hatAxes, targetForKey };`, ctx);
const { dpadAxes, hatAxes, targetForKey } = ctx.result;

// The Xbox profile as written in the page (standard Gamepad API mapping:
// A 0, B 1, X 2, Y 3, LB 4, RB 5, D-pad 12-15; axes LX 0, LY 1, RX 2).
const xbox = JSON.parse(JSON.stringify(vm.runInContext(
  `(${script.match(/const GAMEPAD_PROFILES = (\{[\s\S]*?\n    \});/)[1]}).standard`, vm.createContext({}))));
assert.equal(xbox.forwardAxis, 1);                 // left stick Y
assert.equal(xbox.turnAxis, 2);                    // right stick X
assert.equal(xbox.fireButton, 5);                  // RB
assert.equal(xbox.homeButton, 4);                  // LB
assert.equal(xbox.startButton, 3);                 // Y
assert.deepEqual(xbox.modeButtons, { 0: 1, 1: 2, 2: 0 }); // A semi, B manual, X full auto
assert.deepEqual(xbox.dpad, { up: 12, down: 13, left: 14, right: 15 });

// D-pad: [turret, elevation], right/up = +1, both of a pair cancel.
const d = (...buttons) => { const [r, a] = dpadAxes(new Set(buttons), xbox.dpad); return [r, a]; };
assert.deepEqual(d(), [0, 0]);
assert.deepEqual(d(15), [1, 0]);
assert.deepEqual(d(14), [-1, 0]);
assert.deepEqual(d(12), [0, 1]);
assert.deepEqual(d(13), [0, -1]);
assert.deepEqual(d(14, 15), [0, 0]);
assert.deepEqual(d(12, 15), [1, 1]);
assert.deepEqual(d(5, 3), [0, 0]);                 // other buttons: nothing
{ const [r, a] = dpadAxes(new Set([12]), null); assert.deepEqual([r, a], [0, 0]); } // no D-pad (flight stick)

// The team's pad (Vendor 11c1, D-input, no "standard" mapping): its map
// as read off on the robot 2026-10-17, and the D-pad hat on axis 9.
const pad11c1 = JSON.parse(JSON.stringify(vm.runInContext(
  `(${script.match(/const GAMEPAD_PROFILES = (\{[\s\S]*?\n    \});/)[1]}).vendor11c1`, vm.createContext({}))));
assert.equal(pad11c1.forwardAxis, 1);
assert.equal(pad11c1.turnAxis, 2);
assert.equal(pad11c1.fireButton, 7);                   // RB
assert.equal(pad11c1.homeButton, 6);                   // LB
assert.equal(pad11c1.startButton, 4);                  // Y
assert.deepEqual(pad11c1.modeButtons, { 0: 1, 1: 2, 3: 0 }); // A semi, B manual, X full auto
assert.deepEqual(pad11c1.dpad, { hatAxis: 9 });
const hat = (v) => { const [r, a] = hatAxes(v); return [r, a]; };
assert.deepEqual(hat(-1), [0, 1]);                     // up (measured -1)
assert.deepEqual(hat(-0.43), [1, 0]);                  // right (measured -0.43)
assert.deepEqual(hat(0.14), [0, -1]);                  // down (measured 0.14)
assert.deepEqual(hat(0.71), [-1, 0]);                  // left (measured 0.71)
assert.deepEqual(hat(-0.71), [1, 1]);                  // up-right
assert.deepEqual(hat(1), [-1, 1]);                     // up-left
assert.deepEqual(hat(1.29), [0, 0]);                   // released
assert.deepEqual(hat(undefined), [0, 0]);
{ const axes = Array(10).fill(0); axes[9] = 0.71;
  const [r, a] = dpadAxes(new Set(), pad11c1.dpad, axes); assert.deepEqual([r, a], [-1, 0]); }

// Keys 1-9 -> holes 1-9; anything else -> 0 (not a hole key). The hole
// layout itself (1 left-bottom ... 9 right-top) is the firmware's number.
for (let n = 1; n <= 9; n++) assert.equal(targetForKey(String(n)), n);
for (const k of ['0', '10', 'q', '', 'enter', ' ']) assert.equal(targetForKey(k), 0, k);

console.log('dashboard control key functions: all asserts passed');
