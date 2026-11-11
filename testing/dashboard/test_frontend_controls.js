/* Run with: node --test testing/dashboard/test_frontend_controls.js
 * Entire dashboard script, fake DOM and transport: no network or robot commands.
 */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const {test} = require('node:test');

const script = fs.readFileSync(path.join(__dirname, '../../tools/dashboard/static/app.js'), 'utf8');
const flush = () => new Promise(resolve => setImmediate(resolve));

function robotState(overrides = {}) {
  return {mode: 'demo', connection: {status: 'connected'}, car: {status: 'online'},
    sections: {}, control: {can_start: true, can_stop: true, can_reset: false,
      pending: false, reason: 'Ready to start.', reasons: {}, last_request: null,
      observed: {fresh: true, state: 'off', running: false, stopped: true}, ...overrides}};
}

async function dashboard(initial = robotState()) {
  const nodes = new Map(), timers = new Map(), calls = [];
  let timerId = 0, latest = initial, pollError = null;
  class Node {
    constructor() {
      this.value = ''; this.dataset = {}; this.style = {}; this.listeners = {};
      this.attributes = {}; this.disabled = false; this.hidden = false; this.open = false;
      this.classList = {toggle() {}, add() {}};
    }
    addEventListener(type, fn) { this.listeners[type] = fn; }
    setAttribute(key, value) { this.attributes[key] = value; }
    append() {}
    replaceChildren() {}
    click() { return this.disabled ? undefined : this.listeners.click?.(); }
  }
  const get = id => { if (!nodes.has(id)) nodes.set(id, new Node()); return nodes.get(id); };
  const document = {getElementById: get, querySelector: () => ({content: 'test-token'}),
    querySelectorAll: () => [], createElement: () => new Node(),
    createDocumentFragment: () => new Node(), documentElement: new Node()};
  const window = {dispatchEvent() {}, addEventListener() {},
    matchMedia: () => ({addEventListener() {}})};
  const fetch = (url, options = {}) => {
    if (url === '/api/state') {
      return pollError ? Promise.reject(pollError) : Promise.resolve({ok: true, json: async () => latest});
    }
    assert.equal(url, '/api/control', 'Tests must not invoke another command API');
    return new Promise((resolve, reject) => {
      calls.push({url, options, action: JSON.parse(options.body).action,
        reply: value => resolve({ok: true, json: async () => ({ok: true, state: value})}), reject});
      options.signal.addEventListener('abort', () => reject(Object.assign(new Error('Aborted'), {name: 'AbortError'})));
    });
  };
  vm.runInNewContext(script, {document, window, fetch, AbortController,
    CustomEvent: class {}, localStorage: {getItem() {}, setItem() {}},
    setTimeout: (fn, ms) => { const id = ++timerId; timers.set(id, {fn, ms}); return id; },
    clearTimeout: id => timers.delete(id), requestAnimationFrame() {}});
  await flush();
  return {get, calls,
    async poll(next, error = null) {
      latest = next; pollError = error;
      const timer = [...timers].find(([, value]) => value.ms === 750);
      assert.ok(timer, 'Poll callback scheduled'); timers.delete(timer[0]);
      await timer[1].fn(); await flush();
    },
    async timeout() {
      const timer = [...timers].find(([, value]) => value.ms === 5000);
      assert.ok(timer, 'Control timeout scheduled'); timers.delete(timer[0]);
      timer[1].fn(); await flush();
    }};
}

test('STOP overtakes in-flight START; a late START response cannot replace STOP', async () => {
  const ui = await dashboard();
  const start = ui.get('control-start').click();
  assert.equal(ui.get('control-start').disabled, true);
  assert.equal(ui.get('control-stop').disabled, false);
  ui.get('control-start').click();
  const stop = ui.get('control-stop').click();
  assert.deepEqual(ui.calls.map(call => call.action), ['start', 'stop']);
  assert.equal(ui.calls[1].options.headers['X-Dashboard-Token'], 'test-token');
  ui.calls[1].reply(robotState({last_request: {id: 2, action: 'stop', status: 'observed', detail: 'Stopped/braked observed.'}}));
  await stop;
  ui.calls[0].reply(robotState({pending: true, last_request: {id: 1, action: 'start', status: 'queued'}}));
  await start;
  assert.match(ui.get('control-feedback').textContent, /^Stop request · confirmed by robot/);
  assert.equal(ui.get('control-stop').disabled, false);
});

test('queued or executed is not a running confirmation; stale car still allows STOP', async () => {
  const ui = await dashboard();
  const start = ui.get('control-start').click();
  ui.calls[0].reply(robotState({can_start: false, pending: true,
    last_request: {id: 1, action: 'start', status: 'queued', detail: 'Waiting for controller state.'}}));
  await start;
  assert.match(ui.get('control-feedback').textContent, /queued; awaiting robot/);
  assert.equal(ui.get('control-observed').textContent, 'Robot reports: Off');
  assert.equal(ui.get('control-start').disabled, true);
  assert.equal(ui.get('control-stop').disabled, false);
  await ui.poll(robotState({can_start: false, pending: true,
    last_request: {id: 1, action: 'start', status: 'executed'},
    observed: {fresh: false, state: 'off', running: false, stopped: false}}));
  assert.match(ui.get('control-feedback').textContent, /executed; awaiting observed state/);
  assert.match(ui.get('control-observed').textContent, /state unconfirmed/);
  assert.equal(ui.get('control-stop').disabled, false);
});

test('fault reset is explicit and never starts the robot automatically', async () => {
  const ui = await dashboard(robotState({can_start: false, can_reset: true,
    observed: {fresh: true, state: 'fault', stopped: true}}));
  assert.equal(ui.get('control-reset').hidden, false);
  assert.equal(ui.calls.length, 0);
  const reset = ui.get('control-reset').click();
  assert.equal(ui.get('control-stop').disabled, false);
  ui.calls[0].reply(robotState({last_request: {id: 1, action: 'reset', status: 'observed'}}));
  await reset;
  assert.deepEqual(ui.calls.map(call => call.action), ['reset']);
  assert.equal(ui.get('control-reset').hidden, true);
  assert.equal(ui.get('control-start').disabled, false);
});

test('lost response remains unconfirmed until polling recovers the robot outcome', async () => {
  const ui = await dashboard();
  const start = ui.get('control-start').click();
  await ui.timeout(); await start;
  assert.match(ui.get('control-feedback').textContent, /outcome is unconfirmed/);
  assert.equal(ui.get('control-stop').disabled, false);
  await ui.poll(robotState({can_start: false,
    last_request: {id: 3, action: 'start', status: 'observed', detail: 'Running observed.'},
    observed: {fresh: true, state: 'following', running: true, stopped: false}}));
  assert.match(ui.get('control-feedback').textContent, /confirmed by robot/);
  assert.equal(ui.get('control-observed').textContent, 'Robot reports: Following');
});

test('local service loss disables remote controls and identifies the physical stop', async () => {
  const ui = await dashboard();
  await ui.poll(null, new Error('Service unavailable'));
  for (const action of ['start', 'stop', 'reset']) assert.equal(ui.get(`control-${action}`).disabled, true);
  assert.match(ui.get('control-feedback').textContent, /use GP21 or the power switch to stop/);
  assert.equal(ui.calls.length, 0);
});

test('fresh controller state supplies Overview after restart and overrides old state events', async () => {
  const ui = await dashboard(robotState({observed: {fresh: true, state: 'off', reason: 'OK', running: false, stopped: true}}));
  assert.equal(ui.get('robot-state').textContent, 'Off');
  assert.match(ui.get('robot-state-detail').textContent, /Live controller report/);
  assert.match(ui.get('robot-state-detail').textContent, /Motors stopped and braked/);
  assert.equal(ui.get('robot-state').dataset.active, 'false');
  const fault = robotState({observed: {fresh: true, state: 'fault', reason: 'motor / encoder fault', running: false, stopped: true}});
  fault.sections.state = {valid: true, data: {state: 'line_following'}, received_at: '2026-11-10T11:09:44Z', freshness: 'last_report', age_s: 1200};
  await ui.poll(fault);
  assert.equal(ui.get('robot-state').textContent, 'Fault');
  assert.equal(ui.get('robot-state').className, 'error-text');
  assert.match(ui.get('robot-state-detail').textContent, /Reason: motor \/ encoder fault/);
  assert.equal(ui.get('robot-state').dataset.active, 'false');
  await ui.poll(robotState({observed: {fresh: true, state: 'following', reason: 'OK', running: true, stopped: false}}));
  assert.equal(ui.get('robot-state').textContent, 'Following');
  assert.equal(ui.get('robot-state').dataset.active, 'true');
  assert.equal(ui.calls.length, 0);
});

test('stale or offline controller uses the existing historical Overview fallback', async () => {
  const historical = robotState({observed: {fresh: false, state: 'following', running: false, stopped: false}});
  historical.sections.state = {valid: true, data: {state: 'stopped'}, received_at: '2026-11-10T11:09:44Z', freshness: 'last_report', age_s: 1200};
  const ui = await dashboard(historical);
  assert.equal(ui.get('robot-state').textContent, 'Stopped');
  assert.match(ui.get('robot-state-detail').textContent, /Last report/);
  await ui.poll({...historical, control: {...historical.control, observed: {fresh: true, state: 'following', running: true}}});
  assert.equal(ui.get('robot-state').textContent, 'Following');
  await ui.poll(null, new Error('Service unavailable'));
  assert.equal(ui.get('robot-state').textContent, 'Stopped');
  assert.match(ui.get('robot-state-detail').textContent, /Service offline/);
});

test('initial readiness guidance appears once without duplicating request feedback', async () => {
  const state = robotState(); state.mode = 'live';
  const ui = await dashboard(state);
  assert.equal(ui.get('control-feedback').textContent, 'No control request sent from this dashboard.');
  assert.match(ui.get('control-guidance').textContent, /Ready to start/);
  assert.equal(ui.get('control-feedback').textContent.includes('Ready to start'), false);
});
