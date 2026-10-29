/* Browser display only. All robot commands use the existing MQTT setup APIs. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const token = document.querySelector('meta[name="dashboard-token"]').content;
  const actions = ['forward', 'left', 'right', 'uturn'];
  const pageNames = ['overview', 'sensors', 'debug', 'runs'];
  let state = null;
  let page = 'overview';
  let buddy = '1';
  let transportError = '';
  let mapDirty = false;
  let mapEditing = false;
  let trace = null;
  let traceRequest = 0;
  let traceLoading = false;
  let importing = false;
  let activePosts = new Set();
  let lastEventKey = '';
  let lastRunsKey = '';
  let lastMapResult = '';
  let lastConnection = '';
  let initialConnectionFields = false;
  let initialDemoSelection = false;
  let pollController = null;
  let packetPaused = false;
  let packetKey = '';
  const number = value => typeof value === 'number' && Number.isFinite(value);
  const shown = value => value === null || value === undefined || value === '' ? '—' : String(value);
  const readable = value => typeof value === 'string' ? value.replaceAll('_', ' ') : shown(value);
  const titleCase = value => readable(value).replace(/^./, letter => letter.toUpperCase());

  function element(tag, text, className) {
    const item = document.createElement(tag);
    if (text !== undefined) item.textContent = String(text);
    if (className) item.className = className;
    return item;
  }

  function age(seconds) {
    if (!number(seconds)) return 'Never';
    if (seconds < 1) return 'Just now';
    if (seconds < 60) return `${Math.floor(seconds)} s ago`;
    if (seconds < 3600) return `${Math.floor(seconds / 60)} min ago`;
    return `${Math.floor(seconds / 3600)} h ago`;
  }

  function received(section) {
    if (!section || !section.received_at) return 'Not received';
    if (transportError) return 'Service offline';
    if (section.freshness === 'cached') return 'Cached · age unknown';
    if (!section.valid) return 'Invalid';
    if (section.freshness === 'last_report') return `${section.stale ? 'Disconnected · ' : ''}Last report · ${age(section.age_s)}`;
    return `${section.stale ? 'Stale · ' : ''}${age(section.age_s)}`;
  }

  function section(name) { return state?.sections?.[name] || null; }
  function field(name, key) {
    const item = section(name);
    if (!item?.valid) return null;
    return item.data?.[key] ?? null;
  }
  function connected() {
    return !transportError && state?.connection?.status === 'connected';
  }
  function faultText() {
    const active = field('faults', 'active');
    if (!Array.isArray(active)) return 'Unknown';
    return active.length ? active.map(readable).join(', ') : 'None reported';
  }
  function formatTime(value) {
    const date = new Date(value);
    return Number.isFinite(date.getTime()) ? date.toLocaleTimeString([], {hour: '2-digit', minute: '2-digit', second: '2-digit'}) : '—';
  }

  async function api(path, options = {}) {
    const response = await fetch(path, {credentials: 'same-origin', cache: 'no-store', ...options});
    let result;
    try { result = await response.json(); }
    catch (_) { throw new Error(`Local service returned an unreadable response (${response.status}).`); }
    if (!response.ok || result.error || result.ok === false) {
      throw new Error(result.error || `Request failed (${response.status}).`);
    }
    return result;
  }

  async function post(path, data = {}) {
    if (activePosts.has(path)) throw new Error('This request is already being sent.');
    activePosts.add(path);
    renderControls();
    try {
      const result = await api(path, {method: 'POST', headers: {'Content-Type': 'application/json', 'X-Dashboard-Token': token}, body: JSON.stringify(data)});
      if (result.state) { state = result.state; transportError = ''; render(); }
      return result;
    } finally {
      activePosts.delete(path);
      renderControls();
    }
  }

  function feedback(id, status, message) {
    $(id).dataset.status = status;
    $(id).textContent = message;
  }

  function applyAppearance(choice) {
    const selected = ['light', 'dark'].includes(choice) ? choice : 'system';
    $('appearance').value = selected;
    document.documentElement.style.colorScheme = selected === 'system' ? 'light dark' : selected;
    try { localStorage.setItem('robot-dashboard-appearance', selected); } catch (_) {}
    drawTrace();
    window.dispatchEvent(new CustomEvent('dashboard:appearance'));
  }

  function showPage(next, focus = false) {
    if (!pageNames.includes(next)) return;
    page = next;
    for (const name of pageNames) {
      $(`page-${name}`).hidden = name !== page;
      $(`tab-${name}`).setAttribute('aria-selected', String(name === page));
      $(`tab-${name}`).tabIndex = name === page ? 0 : -1;
    }
    if (focus) $(`tab-${page}`).focus();
    if (page === 'runs') requestAnimationFrame(drawTrace);
  }

  function renderHeader() {
    const connection = state?.connection || {};
    const car = state?.car || {};
    const offline = Boolean(transportError);
    const connectionStatus = offline ? 'error' : connection.status || 'disconnected';
    $('broker-status').textContent = offline ? 'Local service offline' : `Broker ${connectionStatus}`;
    $('broker-dot').className = `status-dot ${connectionStatus === 'connected' ? 'good' : ['error', 'connecting'].includes(connectionStatus) ? 'warning' : ''}`;
    $('car-status').textContent = offline ? 'Car status unknown' : car.status === 'online' ? `Car online · ${age(car.age_s)}` : car.status === 'stale' ? `Car stale · ${age(car.age_s)}` : 'Car status unknown';
    $('car-dot').className = `status-dot ${!offline && car.status === 'online' ? 'good' : !offline && car.status === 'stale' ? 'warning' : ''}`;
    $('server-alert').hidden = !offline;
    $('server-alert').textContent = offline ? `${transportError} Displayed readings are last known values. Reconnecting automatically…` : '';
    const demo = state?.mode === 'demo';
    if (demo && !initialDemoSelection && state.demo_scenario) {
      $('demo-scenario').value = state.demo_scenario;
      initialDemoSelection = true;
    }
    $('demo-banner').hidden = !demo;
    $('connection-panel').hidden = demo;
    $('nav-context').textContent = demo ? 'Simulation · local only' : connection.host ? `${connection.host}:${connection.port || 1883}` : 'Local telemetry console';
    $('connection-detail').textContent = connection.error || (connectionStatus === 'connected' ? `${connection.host}:${connection.port} · connected` : connectionStatus === 'connecting' ? 'Connecting…' : 'Connect to the MQTT broker on your laptop');
    if (!initialConnectionFields && connection.host) {
      if (document.activeElement !== $('broker-host')) $('broker-host').value = connection.host;
      if (document.activeElement !== $('broker-port')) $('broker-port').value = connection.port || 1883;
      initialConnectionFields = true;
    }
    if (connectionStatus === 'connected' && lastConnection !== 'connected') $('connection-panel').open = false;
    if (connectionStatus === 'error' && lastConnection !== 'error') $('connection-panel').open = true;
    lastConnection = connectionStatus;
    const counters = state?.counters || {};
    $('footer-received').textContent = `${shown(counters.received ?? 0)} messages received · ${shown(counters.malformed ?? 0)} malformed`;
  }

  function renderOverview() {
    const reported = field('state', 'state');
    const stateSection = section('state');
    $('robot-state').textContent = reported ? titleCase(reported) : 'Waiting for data';
    $('robot-state').className = reported === 'error' ? 'error-text' : '';
    $('robot-state').dataset.active = String(['line_following', 'turning_left', 'turning_right', 'uturn', 'avoiding_obstacle'].includes(reported));
    const diagnostic=state?.diagnostics?.controller;
    const diagnosticReason=diagnostic?.latest?.data?.reason;
    const detailedReasons=['OK','IR sensor fault','Motor / encoder fault','Motion request rejected','Line lost','Missing direction / duplicate barcode','Barcode not mapped','Controller timeout','Motion report fault','Bad input','Terrain / IMU fault'];
    $('robot-state-detail').textContent = reported ? `${received(stateSection)}. ${diagnostic?.received ? `Controller: ${detailedReasons[diagnosticReason]} · ${diagnostic.stale?'stale · ':''}${age(diagnostic.age_s)}.` : 'Detailed controller reason requires updated diagnostic firmware.'}` : 'No valid state message received yet.';
    const rows = [
      ['Left wheel speed', field('motion', 'left_mms'), 'mm/s', 'motion'],
      ['Right wheel speed', field('motion', 'right_mms'), 'mm/s', 'motion'],
      ['Last decoded letter', field('barcode', 'letter'), '', 'barcode'],
      ['Assigned action', field('barcode', 'action') === null ? null : titleCase(field('barcode', 'action')), '', 'barcode'],
      ['Faults', faultText(), '', 'faults']
    ];
    if (section('heading')?.received_at) rows.push(['Heading', field('heading', 'heading_deg'), '°', 'heading']);
    if (section('hump')?.received_at) rows.push(['Humps reported', field('hump', 'count'), '', 'hump'], ['Reported hump height', field('hump', 'height_mm'), 'mm', 'hump']);
    if (section('obstacle')?.received_at) rows.push(['Obstacle distance', field('obstacle', 'distance_mm'), 'mm', 'obstacle'], ['Obstacle width', field('obstacle', 'width_mm'), 'mm', 'obstacle'], ['Selected route', readable(field('obstacle', 'route')), '', 'obstacle']);
    const body = document.createDocumentFragment();
    for (const [label, value, unit, source] of rows) {
      const tr = element('tr');
      const item = section(source);
      if (item?.received_at && (item.stale || transportError)) tr.className = 'stale';
      if (source === 'faults' && Array.isArray(field('faults', 'active')) && field('faults', 'active').length) tr.classList.add('has-fault');
      tr.append(element('td', label), element('td', shown(value)), element('td', unit), element('td', received(item)));
      body.append(tr);
    }
    $('readings-body').replaceChildren(body);
  }

  function mapDraft() {
    return Object.fromEntries(actions.map(action => [action, $(`map-${action}`).value.trim().toUpperCase()]));
  }
  function mapValidation() {
    const letters = Object.values(mapDraft());
    if (letters.some(letter => !/^[A-Z]$/.test(letter))) return 'Enter one letter, A–Z, for every action.';
    if (new Set(letters).size !== 4) return 'Each action needs a different letter.';
    return '';
  }
  function renderControls() {
    const online = connected() && state?.car?.status === 'online';
    const commands = state?.commands || {};
    const mapping = commands.map || {};
    const locked = commands.map_lock === 'locked';
    const pendingMap = mapping.status === 'pending' || activePosts.has('/api/map');
    const pendingSnapshot = commands.snapshot?.status === 'pending' || activePosts.has('/api/snapshot');
    const invalid = mapValidation();
    $('map-button').disabled = !online || pendingMap || pendingSnapshot || Boolean(invalid);
    $('map-button').textContent = pendingMap ? 'Waiting for reply…' : locked ? 'Retry after robot reboot' : 'Send map';
    $('snapshot-button').disabled = !online || pendingSnapshot || pendingMap;
    $('snapshot-button').textContent = pendingSnapshot ? 'Waiting for snapshot…' : 'Request snapshot';
    $('connect-button').disabled = !state || Boolean(transportError) || activePosts.has('/api/connect') || state?.connection?.status === 'connecting';
    $('disconnect-button').disabled = Boolean(transportError) || !['connected', 'connecting'].includes(state?.connection?.status) || activePosts.has('/api/disconnect');
    actions.forEach(action => { $(`map-${action}`).disabled = pendingMap; });
    $('map-guidance').textContent = locked ? 'Previously locked. Reboot the car before retrying; the robot will accept or reject the request.' : !online ? 'Connect and wait for recent car traffic before sending.' : invalid || 'Ready to send. Wait for the car’s reply.';
    $('map-guidance').classList.toggle('warning-text', online && mapDirty && Boolean(invalid));
    $('map-status-label').textContent = locked ? 'Locked' : pendingMap ? 'Awaiting reply' : mapDirty ? 'Unsaved draft' : mapping.status === 'accepted' ? 'OK reply received' : 'Local draft';
    $('map-lock-note').textContent = locked ? 'The dashboard observed a closed configuration or an active run. A failed retry must not unlock it.' : 'Firmware locks the map after the first accepted START. The current lock status has not been confirmed.';
    $('trace-file').disabled = importing || Boolean(transportError);
  }

  function renderCommands() {
    const commands = state?.commands || {};
    const mapping = commands.map || {};
    const key = JSON.stringify([mapping.status, mapping.detail, mapping.letters]);
    if (key !== lastMapResult) {
      if (mapping.letters && !mapEditing && !mapDirty) actions.forEach(action => { $(`map-${action}`).value = mapping.letters[action] || ''; });
      if (mapping.status === 'accepted') {
        if (mapping.letters && actions.every(action => mapDraft()[action] === mapping.letters[action])) mapDirty = false;
        feedback('map-feedback', 'accepted', 'OK reply received. Verify using one command sender; the protocol does not read back the active map.');
      } else if (mapping.status && mapping.status !== 'idle') {
        feedback('map-feedback', mapping.status, mapping.detail || ({pending: 'Sent; waiting for the car’s reply.', rejected: 'The car rejected this map.', timeout: 'No reply received. The active map is unknown.', error: 'Map request failed.'})[mapping.status] || mapping.status);
      }
      lastMapResult = key;
    }
    const snapshot = commands.snapshot || {};
    if (snapshot.status && snapshot.status !== 'idle') feedback('snapshot-feedback', snapshot.status, snapshot.detail || ({pending: 'Sent; waiting for snapshot reply.', accepted: 'Snapshot received. Cached values may not be fresh sensor samples.', rejected: 'Snapshot rejected.', timeout: 'No snapshot reply received.', error: 'Snapshot request failed.'})[snapshot.status] || snapshot.status);
    else if (!activePosts.has('/api/snapshot')) feedback('snapshot-feedback', 'idle', connected() ? 'Request all available telemetry sections.' : 'Connect to request the latest available readings.');
    renderControls();
  }

  function debugDefinition() {
    const source = name => received(section(name));
    const valueUnit = (name, key, unit) => field(name, key) === null ? '—' : `${field(name, key)}${unit ? ` ${unit}` : ''}`;
    const ir = state?.diagnostics?.ir;
    const controller = state?.diagnostics?.controller;
    const cd = controller?.latest?.data;
    const cdAge = cd ? `${controller.stale ? 'Stale · ' : ''}${age(controller.age_s)} · diagnostic snapshot` : 'Requires updated IR_DEBUG=1 firmware';
    const driveStates = ['Off','Following','Braking','Manoeuvre','Reacquiring','Halted','Fault'];
    const reasons = ['OK','IR sensor fault','Motor / encoder fault','Motion request rejected','Line lost','Missing direction / duplicate barcode','Barcode not mapped','Controller timeout','Motion report fault','Bad input','Terrain / IMU fault'];
    const pendingActions = ['None','Left','Right','Straight','U-turn'];
    const faultGroup = names => {
      const active = field('faults', 'active');
      return Array.isArray(active) ? active.filter(name => names.includes(name)).map(readable).join(', ') || 'None reported' : 'Unknown';
    };
    const commonState = ['Reported robot state', field('state', 'state') === null ? '—' : titleCase(field('state', 'state')), source('state'), 'state'];
    const definitions = {
      '1': {
        title: 'Buddy 1 · Wi-Fi / MQTT', description: 'Check whether messages and command replies are reaching the laptop.',
        rows: [
          ['Laptop → broker', transportError ? 'Local service offline' : titleCase(state?.connection?.status || 'unknown'), 'Laptop connection'],
          ['Car heartbeat', transportError || !number(state?.car?.heartbeat_age_s) ? 'Unknown' : connected() && state.car.heartbeat_age_s <= 15 ? 'Recent' : 'Stale', age(state?.car?.heartbeat_age_s)],
          ['Received messages', shown(state?.counters?.received ?? 0), 'This dashboard session'],
          ['Payload bytes received', shown(state?.counters?.bytes_received ?? 0), 'Application bytes; excludes MQTT headers'],
          ['Malformed messages', shown(state?.counters?.malformed ?? 0), 'Laptop decoder'],
          ['Map request / reply', titleCase(state?.commands?.map?.status || 'idle'), 'Setup command'],
          ['Configuration lock', state?.commands?.map_lock === 'locked' ? 'Locked' : 'Not confirmed', 'Observed run / BUSY reply'],
          ['Snapshot request / reply', titleCase(state?.commands?.snapshot?.status || 'idle'), 'Setup command'],
          ['Wi-Fi fault bit', faultGroup(['wifi_loss']), source('faults'), 'faults']
        ],
        unavailable: 'Wi-Fi association, DHCP/IP acquisition, MCU retry counts, outbox occupancy and RAM usage are not carried in current telemetry. Broker connection describes the laptop’s connection; it does not prove the car is online.'
      },
      '2': {
        title: 'Buddy 2 · Motion control', description: 'Compare the reported wheel speeds and inspect motion-related fault flags.',
        rows: [
          ['Left reported speed', valueUnit('motion', 'left_mms', 'mm/s'), source('motion'), 'motion'],
          ['Right reported speed', valueUnit('motion', 'right_mms', 'mm/s'), source('motion'), 'motion'],
          ['Left / right PID setpoint', cd?.motion_valid ? `${cd.setpoint_left} / ${cd.setpoint_right} mm/s` : '—', cdAge,'controller'],
          ['Left / right motor duty', cd?.motion_valid ? `${cd.duty_left} / ${cd.duty_right} %` : '—', cdAge,'controller'],
          ['Left / right encoder counts', cd?.motion_valid ? `${cd.encoder_left} / ${cd.encoder_right}` : '—', cdAge,'controller'],
          ['Applied brake + stillness', cd?.motion_available ? (cd.braked ? 'Acknowledged' : 'Not acknowledged') : '—', cdAge,'controller'],
          ['Detailed motion state', cd?.motion_available ? ['IDLE','VELOCITY','DRIVE','TURN','COMPLETE','FAULT'][cd.motion_state] : '—',cdAge,'controller'],
          commonState,
          ['Motor / encoder fault bits', faultGroup(['motor_stall', 'encoder_fault']), source('faults'), 'faults']
        ],
        unavailable: 'Updated IR_DEBUG=1 firmware supplies coherent ramped PID setpoint, measured speed, last signed motor duty, encoder counts and brake acknowledgement at up to 5 Hz. The setpoint is not the raw speed request. Wheel reports use 2 mm/s steps. PID component terms and stop latency are still unavailable; zero speed alone does not acknowledge braking.'
      },
      '3': {
        title: 'Buddy 3 · Line / barcode', description: 'Inspect decoded letters, assigned actions and the sensor trace behind a missed barcode.',
        rows: [
          ['Last decoded letter', shown(field('barcode', 'letter')), source('barcode'), 'barcode'],
          ['Assigned action', field('barcode', 'action') === null ? '—' : titleCase(field('barcode', 'action')), source('barcode'), 'barcode'],
          ['Detailed controller state', cd ? driveStates[cd.drive_state] : '—',cdAge,'controller'],
          ['Controller reason', cd ? reasons[cd.reason] : '—',cdAge,'controller'],
          ['Pending action', cd ? pendingActions[cd.pending] : '—',cdAge,'controller'],
          ['Line-position error', cd && (cd.line_flags & 1) ? `${cd.line_error} / ±100` : '—',cdAge,'controller'],
          ['Junction candidate', cd ? (cd.line_flags & 4 ? 'Yes' : 'No') : '—',cdAge,'controller'],
          ['Barcode / junction / line-lost events', cd ? `${cd.barcode_events} / ${cd.junction_events} / ${cd.lost_events}` : '—','Since boot; saturates at 65535','controller'],
          ['Last / run-max IR sampling gap', cd ? `${(cd.sample_gap_us/1000).toFixed(2)} ms / ${cd.run_max_gap_us ? (cd.run_max_gap_us/1000).toFixed(2)+' ms' : 'not recorded'}` : '—',cdAge,'controller'],
          commonState,
          ['IR sensor fault bit', faultGroup(['ir_sensor_fault']), source('faults'), 'faults'],
          ['Received barcode captures', shown(state?.traces?.length ?? 0), 'Incoming + imported traces']
          ,['IR preview', ir?.latest ? `${ir.stale ? 'Stale' : 'Receiving'} · ${age(ir.age_s)}` : 'Not received', 'Optional IR_DEBUG=1 firmware']
        ],
        unavailable: 'The updated diagnostic firmware adds line position, pending action, controller reason, calibration and event counts at up to 5 Hz. A sampled candidate is not a junction event; use the cumulative count. Both front sensors may read white when the line runs between them. These observations cannot prove original-line identity after avoidance. Use Sensors for 20 Hz IR curves and Runs for high-rate recorded barcode evidence.'
      },
      '4': {
        title: 'Buddy 4 · IMU / terrain', description: 'Hump count, run peak height and the IMU fault bit, published by firmware built with S4=1.',
        rows: [
          ['Heading', valueUnit('heading', 'heading_deg', '°'), source('heading'), 'heading'],
          ['Hump count', shown(field('hump', 'count')), source('hump'), 'hump'],
          ['Reported hump height', valueUnit('hump', 'height_mm', 'mm'), source('hump'), 'hump'],
          ['IMU fault bit', faultGroup(['imu_fault']), source('faults'), 'faults']
        ],
        unavailable: 'Firmware built with S4=1 publishes the hump count, the highest hump peak of the current run and the IMU fault bit; default builds (S4=0) publish none of them. The heading topic stays empty and reserved for Buddy 4: the encoder heading used for the route map is in car/telemetry/pose. Pitch/roll, raw accelerometer/gyro, motion events and calibration validity are not published. Empty graphs do not mean zero height or a healthy IMU.'
      },
      '5': {
        title: 'Buddy 5 · Obstacle / recovery', description: 'Obstacle summary, no-echo fault and avoidance state, published by firmware built with S5=1.',
        rows: [
          ['Reported distance', valueUnit('obstacle', 'distance_mm', 'mm'), source('obstacle'), 'obstacle'],
          ['Reported width', valueUnit('obstacle', 'width_mm', 'mm'), source('obstacle'), 'obstacle'],
          ['Selected route', field('obstacle', 'route') === null ? '—' : titleCase(field('obstacle', 'route')), source('obstacle'), 'obstacle'],
          ['No-echo fault bit', faultGroup(['no_echo']), source('faults'), 'faults'],
          commonState
        ],
        unavailable: 'Firmware built with S5=1 (and S5_HARDWARE_CONFIRMED=1) publishes the closest distance, width and chosen route of each completed scan, the no-echo fault bit and the "avoiding obstacle" state; default builds (S5=0) publish none of them. Servo angles, individual scan samples and left/right clearance are not published. No scan profile is invented from one summary distance. Detecting a line alone does not prove it is the original line.'
      }
    };
    return definitions[buddy];
  }

  function renderDebug() {
    const definition = debugDefinition();
    $('debug-title').textContent = definition.title;
    $('debug-description').textContent = definition.description;
    $('debug-availability').textContent = definition.unavailable;
    const body = document.createDocumentFragment();
    for (const [label, value, source, name] of definition.rows) {
      const tr = element('tr');
      if ((name === 'controller' && (state?.diagnostics?.controller?.stale || transportError)) || (name && section(name)?.received_at && (section(name)?.stale || transportError))) tr.className = 'stale';
      tr.append(element('td', label), element('td', value), element('td', source));
      body.append(tr);
    }
    $('debug-body').replaceChildren(body);
    $('debug-trace-link').hidden = buddy !== '3' || !state?.traces?.length;
    $('debug-sensor-link').hidden = buddy !== '3';
    $('packet-inspector').hidden = buddy !== '1';
    renderPackets();
    document.querySelectorAll('[data-buddy]').forEach(button => button.setAttribute('aria-pressed', String(button.dataset.buddy === buddy)));
    window.dispatchEvent(new CustomEvent('dashboard:buddy',{detail:buddy}));
  }

  function renderPackets() {
    if (buddy !== '1' || !$('packet-inspector').open || packetPaused) return;
    const packets = (state?.diagnostics?.raw_messages || []).slice(0, 8);
    const key = JSON.stringify(packets);
    if (key === packetKey) return;
    packetKey = key;
    const fragment = document.createDocumentFragment();
    if (!packets.length) fragment.append(element('li', 'No packets received.', 'muted'));
    packets.forEach(packet => {
      const row = element('li');
      const heading = element('div', undefined, 'packet-heading');
      const wireVersion = Number.parseInt((packet.hex || '').slice(0, 2), 16);
      heading.append(element('time', formatTime(packet.time)), element('span', `${(packet.hex || '').length / 2} bytes · ${packet.decode_error ? 'Rejected' : `Decoded v${wireVersion}`}`, packet.decode_error ? 'warning-text' : ''));
      row.append(heading, element('p', packet.topic, 'packet-topic'), element('pre', `HEX  ${packet.hex || '—'}\n${packet.decode_error ? 'ERROR  ' + packet.decode_error : JSON.stringify(packet.decoded, null, 2)}`));
      fragment.append(row);
    });
    $('packet-list').replaceChildren(fragment);
  }

  function buddyMatches(event) {
    const tag = String(event.buddy ?? '').toLowerCase();
    return !tag || ['all', 'system', 'controller'].includes(tag) || tag === buddy || tag === `buddy ${buddy}` || tag === `buddy${buddy}`;
  }
  function renderEventList(target, events) {
    const fragment = document.createDocumentFragment();
    if (!events.length) fragment.append(element('li', 'No matching events received.', 'empty-line'));
    for (const event of events) {
      const item = element('li');
      item.dataset.level = event.level || 'info';
      const time = element('time', formatTime(event.time));
      if (event.time) time.dateTime = String(event.time);
      const tag = event.buddy ? (/^\d$/.test(String(event.buddy)) ? `Buddy ${event.buddy}` : String(event.buddy)) : 'System';
      item.append(time, element('span', tag, 'event-buddy'), element('span', event.message || '', 'event-message'));
      fragment.append(item);
    }
    $(target).replaceChildren(fragment);
  }
  function renderEvents() {
    const events = Array.isArray(state?.events) ? state.events : [];
    const key = JSON.stringify([events, buddy, $('event-level').value]);
    if (key === lastEventKey) return;
    lastEventKey = key;
    const newest = [...events].sort((a, b) => String(b.time).localeCompare(String(a.time)));
    renderEventList('overview-events', newest.slice(0, 7));
    renderEventList('debug-events', newest.filter(event => buddyMatches(event) && ($('event-level').value === 'all' || ['warning', 'warn', 'error'].includes(event.level))).slice(0, 70));
  }

  function renderRuns() {
    const traces = Array.isArray(state?.traces) ? state.traces : [];
    $('run-count').textContent = String(traces.length);
    const key = JSON.stringify([traces, trace?.id]);
    if (key === lastRunsKey) return;
    lastRunsKey = key;
    const body = document.createDocumentFragment();
    if (!traces.length) {
      const row = element('tr');
      const cell = element('td', 'No captures yet. Import a saved trace, or wait for the car to upload one after a run.', 'empty-line');
      cell.colSpan = 6;
      row.append(cell); body.append(row);
    }
    for (const entry of traces) {
      const row = element('tr');
      row.classList.toggle('selected', entry.id === trace?.id);
      const summary = entry.summary || {};
      const captureStatus = entry.truncated ? 'Truncated' : entry.complete ? 'Complete' : 'Incomplete';
      row.append(element('td', entry.name), element('td', shown(summary.letters)), element('td', shown(summary.decoded)), element('td', shown(summary.rejected)), element('td', captureStatus, entry.complete && !entry.truncated ? '' : 'warning-text'));
      const action = element('td');
      const button = element('button', 'Inspect');
      button.type = 'button';
      button.setAttribute('aria-label', `Inspect ${entry.name}`);
      button.addEventListener('click', () => openTrace(entry.id));
      action.append(button); row.append(action); body.append(row);
    }
    $('runs-body').replaceChildren(body);
    const latest = traces.find(entry => entry.id === trace?.id);
    if (page === 'runs' && latest && !traceLoading &&
        (latest.sample_count !== trace.sample_count || latest.complete !== trace.complete || latest.truncated !== trace.truncated)) {
      openTrace(latest.id);
    }
  }

  function render() {
    renderHeader(); renderOverview(); renderCommands(); renderDebug(); renderEvents(); renderRuns();
    window.dispatchEvent(new CustomEvent('dashboard:state', {detail: state}));
  }

  async function openTrace(id) {
    const request = ++traceRequest;
    const priorEpisode = trace?.id === id ? $('trace-episode').value : null;
    traceLoading = true;
    showPage('runs');
    feedback('import-feedback', 'pending', 'Loading capture…');
    try {
      const result = await api(`/api/traces/${encodeURIComponent(id)}`);
      if (request !== traceRequest) return;
      trace = result.trace || result;
      $('trace-detail').hidden = false;
      $('trace-title').textContent = trace.name || 'Barcode capture';
      const summary = trace.summary || {};
      $('trace-summary').textContent = `${trace.complete ? 'Capture complete' : 'Capture incomplete'} · ${shown(summary.decoded)} decoded · ${shown(summary.rejected)} rejected · letters ${shown(summary.letters)} · ${shown(summary.not_stored)} capture opportunities not stored · ${shown(summary.timing_error)} timing errors · ${shown(summary.overflow)} overflows`;
      const warnings = Array.isArray(trace.warnings) ? trace.warnings.filter(item => typeof item === 'string') : [];
      if (!trace.complete) warnings.unshift('This capture is incomplete. Samples or the final summary may be missing.');
      if (trace.truncated) warnings.push('The capture was truncated; the chart cannot show omitted records.');
      if (trace.missing_records) warnings.push('Missing or malformed capture records.');
      if (number(summary.not_stored) && summary.not_stored > 0) warnings.push(`${summary.not_stored} capture opportunities did not fit in the robot’s capture buffer.`);
      $('trace-warning').hidden = !warnings.length;
      $('trace-warning').textContent = [...new Set(warnings)].join(' ');
      $('trace-download').href = `/api/traces/${encodeURIComponent(id)}/download`;
      const episodes = [...new Set((trace.samples || []).map(sample => sample.episode))];
      $('trace-episode').replaceChildren();
      for (const episode of episodes) {
        const option = element('option', `Episode ${shown(episode)}`);
        option.value = String(episode);
        $('trace-episode').append(option);
      }
      $('trace-episode').disabled = episodes.length <= 1;
      if (priorEpisode && episodes.some(episode => String(episode) === priorEpisode)) $('trace-episode').value = priorEpisode;
      feedback('import-feedback', 'idle', '');
      renderRuns();
      requestAnimationFrame(drawTrace);
    } catch (error) {
      if (request === traceRequest) feedback('import-feedback', 'error', error.message);
    } finally {
      if (request === traceRequest) traceLoading = false;
    }
  }

  function drawTrace() {
    if (!trace || page !== 'runs' || $('trace-detail').hidden) return;
    const canvas = $('trace-canvas');
    const context = canvas.getContext('2d');
    if (!context) return;
    const width = canvas.clientWidth || 640;
    const height = canvas.clientHeight || 270;
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = Math.round(width * ratio); canvas.height = Math.round(height * ratio);
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    context.clearRect(0, 0, width, height);
    const samples = (trace.samples || []).filter(sample => String(sample.episode) === $('trace-episode').value);
    const signal = $('trace-signal').value;
    const valid = samples.filter(sample => number(sample.t_ms) && number(sample[signal]));
    $('trace-plot-empty').hidden = valid.length > 0;
    $('trace-plot-note').textContent = `${samples.length.toLocaleString()} samples · ${signal === 'raw' ? 'ADC counts' : 'normalised darkness'}`;
    $('trace-range').textContent = '';
    if (!valid.length) { canvas.setAttribute('aria-label', 'No timestamped sensor samples available in this episode.'); return; }
    const first = Math.min(...valid.map(sample => sample.t_ms));
    const last = Math.max(...valid.map(sample => sample.t_ms));
    const end = last > first ? last : first + 1;
    const maxValue = signal === 'raw' ? 4095 : 1000;
    const left = 49, right = 15, top = 19, bottom = 37;
    const plotWidth = Math.max(1, width - left - right), plotHeight = height - top - bottom;
    const x = time => left + (time - first) / (end - first) * plotWidth;
    const y = value => top + plotHeight - value / maxValue * plotHeight;
    const style = getComputedStyle(document.documentElement);
    const plotLine = getComputedStyle(canvas.parentElement).borderTopColor;
    const plotAccent = getComputedStyle(document.querySelector('.main-tabs button[aria-selected="true"]')).borderBottomColor;
    // Read resolved ordinary properties: custom light-dark() values may stay unevaluated.
    context.strokeStyle = plotLine;
    context.fillStyle = style.color;
    context.lineWidth = 1;
    context.font = '11px ui-monospace, monospace';
    context.textAlign = 'right';
    for (let index = 0; index <= 4; index++) {
      const value = maxValue * index / 4;
      const yy = y(value);
      context.beginPath(); context.moveTo(left, yy); context.lineTo(width - right, yy); context.stroke();
      context.fillText(String(Math.round(value)), left - 8, yy + 3);
    }
    for (let index = 0; index <= 4; index++) {
      const time = first + (end - first) * index / 4;
      context.textAlign = index === 0 ? 'left' : index === 4 ? 'right' : 'center';
      context.fillText(`${((time - first) / 1000).toFixed(2)} s`, x(time), height - 14);
    }
    context.save();
    context.beginPath(); context.rect(left, top, plotWidth, plotHeight); context.clip();
    context.strokeStyle = plotAccent;
    context.lineWidth = 1.3;
    context.beginPath();
    let previous = null;
    for (const sample of samples) {
      if (!number(sample.t_ms) || !number(sample[signal])) { previous = null; continue; }
      const xx = x(sample.t_ms), yy = y(sample[signal]);
      if (previous === null || sample.t_ms < previous) context.moveTo(xx, yy);
      else context.lineTo(xx, yy);
      previous = sample.t_ms;
    }
    context.stroke(); context.restore();
    if (valid.length === 1) {
      context.fillStyle = plotAccent;
      context.beginPath(); context.arc(x(valid[0].t_ms), y(valid[0][signal]), 2, 0, Math.PI * 2); context.fill();
    }
    const min = Math.min(...valid.map(sample => sample[signal]));
    const max = Math.max(...valid.map(sample => sample[signal]));
    const unplotted = samples.length - valid.length;
    $('trace-range').textContent = `Recorded range ${min}–${max} ${signal === 'raw' ? 'ADC counts' : '/ 1000'} over ${((last - first) / 1000).toFixed(2)} s. Time is relative to the first timestamped sample in this episode.${unplotted ? ` ${unplotted} samples have unknown time or value; the line breaks across them.` : ''}`;
    canvas.setAttribute('aria-label', `Episode ${$('trace-episode').value}: ${signal === 'raw' ? 'raw ADC' : 'normalised darkness'} ranges from ${min} to ${max} over ${((last - first) / 1000).toFixed(2)} seconds. ${unplotted} samples cannot be plotted.`);
  }

  async function poll() {
    pollController = new AbortController();
    const deadline = setTimeout(() => pollController?.abort(), 4500);
    try {
      state = await api('/api/state', {signal: pollController.signal});
      transportError = '';
      render();
    } catch (error) {
      transportError = error.name === 'AbortError' ? 'The local dashboard service is not responding.' : 'Cannot reach the local dashboard service.';
      render();
    } finally {
      clearTimeout(deadline);
      pollController = null;
      setTimeout(poll, 750);
    }
  }

  document.querySelectorAll('[data-page]').forEach(button => {
    button.addEventListener('click', () => showPage(button.dataset.page));
    button.addEventListener('keydown', event => {
      const index = pageNames.indexOf(page);
      if (event.key === 'ArrowRight' || event.key === 'ArrowLeft') {
        event.preventDefault(); showPage(pageNames[(index + (event.key === 'ArrowRight' ? 1 : -1) + pageNames.length) % pageNames.length], true);
      } else if (event.key === 'Home' || event.key === 'End') {
        event.preventDefault(); showPage(pageNames[event.key === 'Home' ? 0 : pageNames.length - 1], true);
      }
    });
  });
  document.querySelectorAll('[data-buddy]').forEach(button => button.addEventListener('click', () => { buddy = button.dataset.buddy; renderDebug(); renderEvents(); }));
  $('view-debug-button').addEventListener('click', () => showPage('debug', true));
  $('open-sensors-button').addEventListener('click', () => showPage('sensors', true));
  $('event-level').addEventListener('change', renderEvents);
  $('appearance').addEventListener('change', () => applyAppearance($('appearance').value));
  $('packet-inspector').addEventListener('toggle', renderPackets);
  $('packet-pause').addEventListener('click', () => {
    packetPaused = !packetPaused;
    $('packet-pause').setAttribute('aria-pressed', String(packetPaused));
    $('packet-pause').textContent = packetPaused ? 'Resume packet view' : 'Pause packet view';
    renderPackets();
  });
  $('connection-form').addEventListener('submit', async event => {
    event.preventDefault();
    try {
      await post('/api/connect', {host: $('broker-host').value.trim(), port: Number($('broker-port').value)});
      $('connection-note').textContent = 'Connection requested. Waiting for broker status…';
    } catch (error) { $('connection-note').textContent = error.message; }
  });
  $('disconnect-button').addEventListener('click', async () => {
    try { await post('/api/disconnect'); $('connection-note').textContent = 'Disconnected from the broker.'; }
    catch (error) { $('connection-note').textContent = error.message; }
  });
  $('demo-form').addEventListener('submit', async event => {
    event.preventDefault();
    try { await post('/api/demo', {scenario: $('demo-scenario').value}); }
    catch (error) { feedback('snapshot-feedback', 'error', error.message); }
  });
  actions.forEach(action => {
    const input = $(`map-${action}`);
    input.addEventListener('input', () => { input.value = input.value.toUpperCase(); mapDirty = true; renderControls(); });
    input.addEventListener('focus', () => { mapEditing = true; });
    input.addEventListener('blur', () => { mapEditing = false; });
  });
  $('map-form').addEventListener('submit', async event => {
    event.preventDefault();
    const error = mapValidation();
    if (error) { feedback('map-feedback', 'error', error); return; }
    if (!connected()) return;
    try { feedback('map-feedback', 'pending', 'Sending map…'); await post('/api/map', mapDraft()); }
    catch (failure) { feedback('map-feedback', 'error', failure.message); }
  });
  $('snapshot-button').addEventListener('click', async () => {
    if (!connected()) return;
    try { feedback('snapshot-feedback', 'pending', 'Sending snapshot request…'); await post('/api/snapshot'); }
    catch (error) { feedback('snapshot-feedback', 'error', error.message); }
  });
  $('save-log-button').addEventListener('click', () => {
    const document = {saved_at: new Date().toISOString(), mode: state?.mode || 'unknown', selected_buddy: buddy, note: 'Laptop-received telemetry and bounded recent event history; not an MCU diagnostic log.', state};
    const blob = new Blob([JSON.stringify(document, null, 2)], {type: 'application/json'});
    const url = URL.createObjectURL(blob);
    const anchor = element('a'); anchor.href = url; anchor.download = `robot_monitor_${new Date().toISOString().replaceAll(':', '-')}.json`;
    anchor.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
  });
  $('trace-file').addEventListener('change', async event => {
    const file = event.target.files?.[0];
    if (!file) return;
    if (file.size > 2 * 1024 * 1024) { feedback('import-feedback', 'error', 'Choose a trace smaller than 2 MiB.'); event.target.value = ''; return; }
    importing = true; renderControls();
    try {
      feedback('import-feedback', 'pending', `Importing ${file.name}…`);
      const result = await post('/api/traces/import', {name: file.name, text: await file.text()});
      const id = result.trace?.id || result.id;
      if (id) await openTrace(id);
      else feedback('import-feedback', 'accepted', 'Trace imported. Select Inspect when it appears in the list.');
    } catch (error) { feedback('import-feedback', 'error', error.message); }
    finally { importing = false; event.target.value = ''; renderControls(); }
  });
  $('inspect-latest-button').addEventListener('click', () => { const entry = state?.traces?.[0]; if (entry) openTrace(entry.id); });
  $('trace-episode').addEventListener('change', drawTrace);
  $('trace-signal').addEventListener('change', drawTrace);
  if ('ResizeObserver' in window) new ResizeObserver(() => { if (page === 'runs' && trace) drawTrace(); }).observe($('trace-canvas').parentElement);
  else window.addEventListener('resize', drawTrace);
  window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', drawTrace);
  let savedAppearance = 'system';
  try { savedAppearance = localStorage.getItem('robot-dashboard-appearance') || 'system'; } catch (_) {}
  applyAppearance(savedAppearance);
  render();
  poll();
})();
