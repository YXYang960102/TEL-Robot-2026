const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const html = fs.readFileSync(path.join(__dirname, '../tools/shooter_keyboard_test/rotate.html'), 'utf8');
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
  handlers.keydown(key('KeyE'));
  assert.equal(vm.runInContext('armed',ctx),false);
  await elements.enable.onclick();
  assert.equal(writes.at(-1),'E\n');
  assert.equal(vm.runInContext('armed',ctx),false);
  handlers.keydown(key('ArrowRight'));await vm.runInContext('chain',ctx);
  assert.equal(writes.at(-1),'E\n');
  vm.runInContext('telemetry("$ROTATE,2,1,1500,1,1,0,NONE")',ctx);
  handlers.keydown(key('ArrowRight'));await vm.runInContext('chain',ctx);
  assert.equal(writes.at(-1),'M,0,1\n');
  handlers.keyup(key('ArrowRight'));await vm.runInContext('chain',ctx);
  assert.equal(writes.at(-1),'M,0,0\n');
  handlers.keydown(key('Enter'));await vm.runInContext('chain',ctx);
  assert.equal(writes.at(-1),'X\n');assert.equal(vm.runInContext('armed',ctx),false);
  await elements.enable.onclick();handlers.blur();await vm.runInContext('chain',ctx);
  assert.equal(writes.at(-1),'X\n');
  vm.runInContext('ready=false;telemetry("$TEL,1,1,0");',ctx);
  assert.equal(vm.runInContext('ready',ctx),false);
  vm.runInContext('telemetry("$ROTATE,1,0,1500,1,1");',ctx);
  assert.equal(vm.runInContext('ready',ctx),false);
  vm.runInContext('telemetry("$ROTATE,2,0,1500,1,1,0");',ctx);
  assert.equal(vm.runInContext('ready',ctx),true);
  elements.noLimits.checked=true;
  await elements.enable.onclick();assert.equal(writes.at(-1),'E_NO_LIMITS\n');
  elements.noLimits.checked=false;elements.noLimits.onchange();
  await vm.runInContext('chain',ctx);assert.equal(writes.at(-1),'X\n');
  assert.equal(vm.runInContext('armed',ctx),false);
  // Reproduce original race: old disabled telemetry before enable write.
  writes.length=0;elements.noLimits.checked=true;
  const enabling=elements.enable.onclick();
  vm.runInContext('telemetry("$ROTATE,2,0,1500,1,1,0")',ctx);
  await enabling;assert.equal(writes[0],'E_NO_LIMITS\n');
  assert.equal(vm.runInContext('pendingEnable',ctx),true);
  assert.equal(elements.noLimits.checked,true);
  tick();await vm.runInContext('chain',ctx);assert.equal(writes.at(-1),'M,0,0\n');
  vm.runInContext('telemetry("$ROTATE,2,1,1500,1,1,1,NONE")',ctx);
  assert.equal(vm.runInContext('armed',ctx),true);
  vm.runInContext('telemetry("$ROTATE,2,0,1500,1,1,0,JOG_CAP")',ctx);
  assert.match(elements.status.textContent,/400 ms/);
  await vm.runInContext('chain',ctx);
  await elements.enable.onclick();
  now=1101;vm.runInContext('lastTelemetry=1101',ctx);tick();
  await vm.runInContext('chain',ctx);
  assert.equal(vm.runInContext('pendingEnable',ctx),false);
  assert.match(elements.status.textContent,/1 秒/);
  // Stale pending jog cannot run after stop increments the write generation.
  writes.length=0;
  vm.runInContext('armed=true;send("M,0,1");stop();',ctx);
  await vm.runInContext('chain',ctx);assert.deepEqual(writes,['X\n']);
  console.log('rotate page controls/protocol/write-generation tests passed');
})().catch(e=>{console.error(e);process.exitCode=1;});
