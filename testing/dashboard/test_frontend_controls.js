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
      this.children = [];
      this.classList = {toggle() {}, add() {}};
    }
    addEventListener(type, fn) { this.listeners[type] = fn; }
    setAttribute(key, value) { this.attributes[key] = value; }
    append(...children) { this.children.push(...children); }
    replaceChildren(...children) { this.children = children; }
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

test('face width estimate comes only from bounds on the last usable scan', async () => {
  const state = robotState();
  const scan = {valid: true, freshness: 'last_report', received_at: '2026-11-23T02:56:20Z',
    data: {valid: true, width_mm: 209, face_width_min_mm: 123, face_width_max_mm: 177}};
  state.sections.scan = scan;
  const ui = await dashboard(state);
  const rows = () => ui.get('readings-body').children[0].children;
  const estimate = () => rows().find(row => row.children[0].textContent.includes('estimated face width'))?.children[1].textContent;
  assert.equal(estimate(), '≈150 mm (123–177 mm)');
  scan.stale = true;
  await ui.poll(state);
  assert.equal(estimate(), 'Unavailable while disconnected');
  scan.stale = false;
  delete scan.data.face_width_min_mm;
  delete scan.data.face_width_max_mm;
  await ui.poll(state);
  assert.equal(estimate(), 'No estimate in this scan');
});

test('discarded hump events show original firmware run, measurements and downloadable CSV', async () => {
  const state = robotState();
  const name = 'hump-abc12342-r00009-000000.csv';
  state.hump_events = {latest: {firmware_run: 9, event_sequence: 2, received_at_utc: '2026-11-24T05:00:00Z',
    rejection_reason: 'reverse_travel', crossing_travel_um: 7800, peak_um: 9200, firmware_dropped_events: 1},
    files: [name], error: null};
  const ui = await dashboard(state);
  assert.equal(ui.get('hump-discards').hidden, false);
  assert.match(ui.get('hump-discard-summary').textContent, /Run 9, event 2.*Reverse travel.*7.80 mm.*9.20 mm.*drops 1/);
  assert.equal(ui.get('hump-discard-files').children[0].children[0].children[0].href, '/api/imu/' + name);
});

test('run history shows run summaries and keeps downloads in the Buddy sections', async () => {
  const started_at = '2026-11-13T07:34:22Z';
  const runs = [
    {id: 'active', started_at, active: true, ended_at: null, hump_count: 0, peak_mm: null, has_details: true},
    {id: 'pickup', started_at, active: false, ended_at: '2026-11-13T07:34:25Z', hump_count: 2, peak_mm: 14,
      end_reason: 'Lift detected', has_details: true},
    {id: 'legacy', started_at, active: false, ended_at: null, hump_count: null, peak_mm: 10, has_details: false},
  ];
  const state = robotState();
  state.run_log = {runs, current_id: null, error: null};
  const ui = await dashboard(state);
  const rows = ui.get('run-log-body').children[0].children;
  assert.equal(ui.get('imu-captures').hidden, true);
  assert.equal(ui.get('run-count').textContent, '3');
  assert.deepEqual(rows.map(row => [1, 3, 4, 5].map(i => row.children[i].textContent)),
    [['Running', '0', '—', '—'], ['Ended', '2', '14', 'Lift detected'], ['End unconfirmed', 'Unavailable', '10', '—']]);
  assert.ok(rows.every(row => row.children.length === 6));
  assert.equal(ui.get('run-history-toggle').hidden, true);
});

test('run history expands from five rows to the full recent list and collapses again', async () => {
  const runs = Array.from({length: 20}, (_, index) => ({
    id: String(index), started_at: '2026-11-13T07:34:22Z', hump_count: index,
  }));
  const state = {...robotState(), run_log: {runs}};
  const ui = await dashboard(state);
  const toggle = ui.get('run-history-toggle');
  const rows = () => ui.get('run-log-body').children[0].children;
  assert.equal(rows().length, 5);
  assert.equal(toggle.hidden, false);
  assert.equal(toggle.textContent, 'Show more');
  assert.equal(toggle.attributes['aria-expanded'], 'false');

  toggle.click();
  assert.equal(rows().length, 20);
  assert.equal(toggle.textContent, 'Show less');
  assert.equal(toggle.attributes['aria-expanded'], 'true');
  await ui.poll({...state, run_log: {runs: [{...runs[0], hump_count: 8}, ...runs.slice(1)]}});
  assert.equal(rows().length, 20);
  assert.equal(rows()[0].children[3].textContent, '8');
  toggle.click();
  assert.equal(rows().length, 5);
});

test('barcode captures expand independently of run history', async () => {
  const captures = Array.from({length: 20}, (_, index) => ({
    id: String(index), name: `Capture ${index}`, complete: true,
  }));
  const state = {...robotState(), traces: captures, run_log: {runs: captures}};
  const ui = await dashboard(state);
  const toggle = ui.get('barcode-captures-toggle');
  const rows = () => ui.get('runs-body').children[0].children;
  assert.equal(rows().length, 5);
  assert.equal(toggle.textContent, 'Show more');

  toggle.click();
  assert.equal(rows().length, 20);
  assert.equal(rows()[19].children[0].textContent, 'Capture 19');
  assert.equal(toggle.attributes['aria-expanded'], 'true');
  assert.equal(ui.get('run-log-body').children[0].children.length, 5);
  toggle.click();
  assert.equal(rows().length, 5);
  await ui.poll({...state, traces: captures.slice(0, 4)});
  assert.equal(toggle.hidden, true);
});

test('buddy CSV sections follow the selected run and show missing telemetry', async () => {
  const started_at = '2026-11-13T07:34:22Z';
  const current = {id: 'new run', started_at, buddy_entries: {1: 3, 2: 6, 3: 12, 4: 1}};
  const older = {id: 'older', started_at, hump_count: 2, peak_mm: 14, has_details: false};
  const state = robotState();
  state.run_log = {runs: [current, older]};
  const ui = await dashboard(state);
  const picker = ui.get('buddy-run-picker');
  const buddies = [1, 2, 3, 4, 5];
  assert.equal(ui.get('buddy-run-logs').hidden, false);
  assert.equal(picker.value, 'new run');
  assert.notEqual(picker.children[0].textContent, picker.children[1].textContent);
  assert.deepEqual(buddies.map(id => ui.get(`buddy-run-download-${id}`).href),
    buddies.map(id => `/api/runs/new%20run/buddy/${id}.csv`));
  assert.deepEqual(buddies.map(id => ui.get(`buddy-run-count-${id}`).textContent),
    ['3 entries', '6 entries', '12 entries', '1 entry', 'No samples recorded']);

  picker.value = 'older';
  picker.listeners.change();
  assert.equal(ui.get('buddy-run-count-4').textContent, 'Hump summary available');
  assert.equal(ui.get('buddy-run-download-4').href, '/api/runs/older/buddy/4.csv');
  await ui.poll({...state, run_log: {runs: [{id: 'next', started_at}, current, older]}});
  assert.equal(picker.value, 'older');
  assert.equal(ui.get('buddy-run-download-5').href, '/api/runs/older/buddy/5.csv');
  await ui.poll({...state, run_log: {runs: [current]}});
  assert.equal(picker.value, 'new run');
  await ui.poll({...state, run_log: {runs: []}});
  assert.equal(ui.get('buddy-run-logs').hidden, true);
});

test('raw IMU captures expose a CSV download for each firmware run', async () => {
  const name = 'imu-20261117T080000000Z-10203040-r00012-12345678.csv';
  const state = robotState();
  state.imu_captures = {captures: [{name, run: 12, received_at: '2026-11-17T08:00:00Z', limited: true}]};
  const ui = await dashboard(state);
  const row = ui.get('imu-capture-body').children[0].children[0];
  const download = row.children[3].children[0];
  assert.equal(ui.get('imu-captures').hidden, false);
  assert.equal(row.children[1].textContent, '12');
  assert.equal(row.children[2].textContent, 'Size limit reached');
  assert.equal(download.href, `/api/imu/${name}`);
  assert.equal(download.download, name);
});

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
