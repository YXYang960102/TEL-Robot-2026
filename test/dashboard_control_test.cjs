// Focused host test for tools/dashboard/index.html's wired-control command
// functions (tankCommand/rotateCommand), added for the
// 2026-10-11 dev integration (docs/codex-handoff.md).
//
// The dashboard page itself (1600+ lines: canvas field/chart drawing,
// draggable/resizable panel layout persisted to localStorage, MJPEG stream
// handling) is not loaded wholesale in a VM here the way the smaller
// vision_rotate.html bench page is in vision_rotate_page_test.cjs -- most
// of that code runs at top-level load time and depends on a real
// canvas/ResizeObserver/localStorage environment that isn't worth mocking
// for what this round actually needs verified. Instead this extracts just
// the three pure command functions by source text and evaluates them in
// isolation against a fake `heldKeys` Set, which is where the one
// genuinely easy-to-get-wrong piece of logic lives: the A/D polarity
// Jeremy specified (A = left wheel forward + right wheel backward, the
// opposite of the usual "nose" convention) and the combined-key clamping.
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

const heldKeys = new Set();
const src = [
  extractFunction('tankCommand'),
  extractFunction('rotateCommand'),
].join('\n');

const ctx = vm.createContext({ heldKeys, Math });
vm.runInContext(`${src}\nresult = { tankCommand, rotateCommand };`, ctx);
const { tankCommand, rotateCommand } = ctx.result;

function setKeys(...keys) {
  heldKeys.clear();
  keys.forEach((k) => heldKeys.add(k));
}

// tankCommand() returns an array from the vm context's own Array realm;
// cross-realm arrays trip up assert.deepEqual's reference checks even when
// structurally identical, so pull values out into plain local arrays first.
function tank() {
  const [left, right] = tankCommand();
  return [left, right];
}

// Neutral: nothing held.
setKeys();
assert.deepEqual(tank(), [0, 0]);
assert.equal(rotateCommand(), 0);

// W alone: both wheels forward.
setKeys('w');
assert.deepEqual(tank(), [1, 1]);

// S alone: both wheels backward.
setKeys('s');
assert.deepEqual(tank(), [-1, -1]);

// A alone: Jeremy's explicit polarity -- left wheel forward, right wheel
// backward (opposite the usual "nose points left" convention). This is
// the one sign that must NOT get silently flipped.
setKeys('a');
assert.deepEqual(tank(), [1, -1]);

// D alone: mirror of A.
setKeys('d');
assert.deepEqual(tank(), [-1, 1]);

// W+A combined: forward bias plus the A pivot, clamped to [-1, 1].
setKeys('w', 'a');
assert.deepEqual(tank(), [1, 0]);

// Opposing keys cancel (W+S, A+D).
setKeys('w', 's');
assert.deepEqual(tank(), [0, 0]);
setKeys('a', 'd');
assert.deepEqual(tank(), [0, 0]);

// Rotate axis: right arrow = +1, left arrow = -1, both held = 0.
setKeys('arrowright');
assert.equal(rotateCommand(), 1);
setKeys('arrowleft');
assert.equal(rotateCommand(), -1);
setKeys('arrowleft', 'arrowright');
assert.equal(rotateCommand(), 0);

// (Elevation is no longer a held axis: arrows up/down send ANGS steps --
// covered by dashboard_page_test.cjs.)

console.log('dashboard wired-control command functions: all asserts passed');
