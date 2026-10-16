// Focused host test for tools/dashboard/index.html's pure control-key
// functions (rotateCommand/angleCommand for the arrows, targetForKey for
// 1-9 -- control table v2, 2026-10-16).
//
// The dashboard page itself (canvas field/chart drawing, draggable panel
// layout, MJPEG stream) is exercised as a whole in dashboard_page_test.cjs;
// this extracts just the pure functions by source text and evaluates them
// in isolation against a fake `heldKeys` Set, where the easy-to-get-wrong
// mappings live: the arrow signs and Jeremy's hole numbering (columns left
// to right, each from the bottom up).
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
  extractFunction('rotateCommand'),
  extractFunction('angleCommand'),
  extractFunction('targetForKey'),
].join('\n');

const ctx = vm.createContext({ heldKeys, Math, Number });
vm.runInContext(`${src}\nresult = { rotateCommand, angleCommand, targetForKey };`, ctx);
const { rotateCommand, angleCommand, targetForKey } = ctx.result;

function setKeys(...keys) {
  heldKeys.clear();
  keys.forEach((k) => heldKeys.add(k));
}

// Neutral: nothing held.
setKeys();
assert.equal(rotateCommand(), 0);
assert.equal(angleCommand(), 0);

// Rotate axis: right arrow = +1, left arrow = -1, both held = 0.
setKeys('arrowright');
assert.equal(rotateCommand(), 1);
setKeys('arrowleft');
assert.equal(rotateCommand(), -1);
setKeys('arrowleft', 'arrowright');
assert.equal(rotateCommand(), 0);

// Elevation hold-to-move jog (arrows up/down, every firmware and the
// radio): up = +1, down = -1.
setKeys('arrowup');
assert.equal(angleCommand(), 1);
setKeys('arrowdown');
assert.equal(angleCommand(), -1);

// Keys 1-9 -> holes 1-9; anything else -> 0 (not a hole key). The hole
// layout itself (1 left-bottom ... 9 right-top) is the firmware's number.
for (let n = 1; n <= 9; n++) assert.equal(targetForKey(String(n)), n);
for (const k of ['0', '10', 'q', '', 'enter', ' ']) assert.equal(targetForKey(k), 0, k);

console.log('dashboard control key functions: all asserts passed');
