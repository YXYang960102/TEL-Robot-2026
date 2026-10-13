const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const html = fs.readFileSync(path.join(__dirname, '../tools/shooter_keyboard_test/vision_rotate.html'), 'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
const handlers = {}, elements = {};
const writes = [];
let now=100,tick;
const ctx = vm.createContext({TextEncoder, TextDecoder, Promise, Set, Number,
  performance:{now:()=>now}, navigator:{}, setTimeout, setInterval:fn=>{tick=fn;},
  document:{getElementById:id=>elements[id]??=( {} ), addEventListener:()=>{}},
  window:{addEventListener:(name,fn)=>handlers[name]=fn}});
vm.runInContext(script,ctx);
ctx.writes = writes;
vm.runInContext('writer={write:async b=>writes.push(new TextDecoder().decode(b))};ready=true;lastTelemetry=100;',ctx);
const key = (code,repeat=false)=>({code,repeat,preventDefault(){},stopPropagation(){}});
(async()=>{
  // Enable, confirm, then the 50ms tick pings "P" (never a direction command).
  await elements.enable.onclick();
  assert.equal(writes.at(-1),'E\n');
  assert.equal(vm.runInContext('armed',ctx),false);
  vm.runInContext('telemetry("$VROTATE,5,1,1500,1,1,0,NONE,0,0,0,0,0,5,0,0,0,0,0,0")',ctx);
  assert.equal(vm.runInContext('armed',ctx),true);
  tick();await vm.runInContext('chain',ctx);
  assert.equal(writes.at(-1),'P\n');

  // Old v4 firmware is not mistaken for v5 (field count/version differ).
  vm.runInContext('ready=false',ctx);
  vm.runInContext('telemetry("$VROTATE,4,1,1500,1,1,0,NONE,0,0,0,0,0,5,0,0,0,0,0")',ctx);
  assert.equal(vm.runInContext('ready',ctx),false);
  vm.runInContext('ready=true',ctx);

  // Vision panel reflects the latest telemetry regardless of armed state,
  // including the v4 dt/P/D/raw-output/saturated fields (deadband retired
  // 2026-10-10, see docs/codex-handoff.md -- no more "zone" text to check).
  vm.runInContext('telemetry("$VROTATE,5,1,1650,0,0,0,NONE,4,-30,900,3,1,12,58,20,-20,-12,1,-12")',ctx);
  assert.match(elements.vision.textContent,/tx：4/);
  assert.match(elements.vision.textContent,/鎖定中/);
  assert.match(elements.vision.textContent,/dt：58 ms/);
  assert.match(elements.vision.textContent,/P 分量：20 μs/);
  assert.match(elements.vision.textContent,/I 分量（tx 在 IZone 內才累積）：-12 μs/);
  assert.match(elements.vision.textContent,/D 分量（濾波後）：-20 μs/);
  assert.match(elements.vision.textContent,/夾限前輸出：-12 μs/);
  assert.match(elements.vision.textContent,/是否飽和：是/);

  // Explicit stop via Enter.
  handlers.keydown(key('Enter'));await vm.runInContext('chain',ctx);
  assert.equal(writes.at(-1),'X\n');assert.equal(vm.runInContext('armed',ctx),false);

  // Firmware-reported VISION_TIMEOUT surfaces a distinct, readable reason.
  await elements.enable.onclick();
  vm.runInContext('telemetry("$VROTATE,5,1,1500,0,0,0,NONE,0,0,0,0,0,5,0,0,0,0,0,0")',ctx);
  assert.equal(vm.runInContext('armed',ctx),true);
  vm.runInContext('telemetry("$VROTATE,5,0,1500,0,0,0,VISION_TIMEOUT,0,0,0,0,0,900,0,0,0,0,0,0")',ctx);
  assert.match(elements.status.textContent,/Orin 鏡頭連結逾時/);

  // noLimits checkbox flow, same enable/confirm/race-guard shape as rotate.html.
  elements.noLimits.checked=true;
  await elements.enable.onclick();assert.equal(writes.at(-1),'E_NO_LIMITS\n');
  elements.noLimits.checked=false;elements.noLimits.onchange();
  await vm.runInContext('chain',ctx);assert.equal(writes.at(-1),'X\n');
  assert.equal(vm.runInContext('armed',ctx),false);

  // Reproduce the original race: stale disabled telemetry arrives before the
  // enable write itself is confirmed sent -- must not flip armed early.
  writes.length=0;elements.noLimits.checked=true;
  const enabling=elements.enable.onclick();
  vm.runInContext('telemetry("$VROTATE,5,0,1500,0,0,0,NONE,0,0,0,0,0,5,0,0,0,0,0,0")',ctx);
  await enabling;assert.equal(writes[0],'E_NO_LIMITS\n');
  assert.equal(vm.runInContext('pendingEnable',ctx),true);
  tick();await vm.runInContext('chain',ctx);assert.equal(writes.at(-1),'P\n');
  vm.runInContext('telemetry("$VROTATE,5,1,1500,0,0,1,NONE,0,0,0,0,0,5,0,0,0,0,0,0")',ctx);
  assert.equal(vm.runInContext('armed',ctx),true);
  // Firmware disables on its own (e.g. USB tab watchdog) -> page reflects it.
  vm.runInContext('telemetry("$VROTATE,5,0,1500,0,0,0,TIMEOUT,0,0,0,0,0,900,0,0,0,0,0,0")',ctx);
  assert.match(elements.status.textContent,/分頁逾時/);
  await vm.runInContext('chain',ctx);

  // Enable-confirmation timeout still fires if the firmware never confirms.
  await elements.enable.onclick();
  now=1101;vm.runInContext('lastTelemetry=1101',ctx);tick();
  await vm.runInContext('chain',ctx);
  assert.equal(vm.runInContext('pendingEnable',ctx),false);
  assert.match(elements.status.textContent,/1 秒/);

  console.log('vision_rotate page controls/protocol/vision-panel tests passed');
})().catch(e=>{console.error(e);process.exitCode=1;});
