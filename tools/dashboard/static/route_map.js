/* Route map from car/telemetry/pose. All values come from /api/state (route). */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const number = value => typeof value === 'number' && Number.isFinite(value);
  let route = {runs: []}, selected = null;

  // Theme tokens use light-dark(), which a canvas cannot parse: let the page
  // resolve each one to an rgb() colour through a hidden probe element.
  const probe = document.createElement('span');
  probe.hidden = true; document.body.append(probe);
  const css = name => { probe.style.color = `var(${name})`; return getComputedStyle(probe).color; };
  const colours = () => ({
    ink: getComputedStyle(document.documentElement).color,
    grid: getComputedStyle($('route-canvas').parentElement).borderTopColor,
    path: css('--accent'), barcode: css('--accent'), hump: css('--good'),
    obstacle: css('--danger'), turn: css('--warn'),
    end: getComputedStyle(document.documentElement).color
  });
  const metres = cm => `${(cm / 100).toFixed(2)} m`;
  const clock = iso => {
    const date = new Date(iso);
    return Number.isFinite(date.getTime()) ? date.toLocaleTimeString([], {hour: '2-digit', minute: '2-digit', second: '2-digit'}) : '—';
  };
  const current = () => route.runs.find(run => run.id === selected) || route.runs[0] || null;

  function niceStep(span) {
    const raw = span / 5, power = 10 ** Math.floor(Math.log10(Math.max(raw, 1)));
    for (const m of [1, 2, 5, 10]) if (raw <= m * power) return m * power;
    return 10 * power;
  }

  function marker(context, kind, x, y, c) {
    context.fillStyle = c[kind] || c.ink; context.strokeStyle = c[kind] || c.ink;
    context.beginPath();
    if (kind === 'hump' || kind === 'end') context.rect(x - 5, y - 5, 10, 10);
    else if (kind === 'obstacle') { context.moveTo(x, y - 7); context.lineTo(x + 7, y); context.lineTo(x, y + 7); context.lineTo(x - 7, y); context.closePath(); }
    else context.arc(x, y, kind === 'turn' ? 4.5 : 5.5, 0, 2 * Math.PI);
    context.fill();
  }

  function draw() {
    const canvas = $('route-canvas'), context = canvas.getContext('2d'), run = current();
    if (!context) return;
    const width = canvas.clientWidth, height = canvas.clientHeight;
    if (width < 10 || height < 10) return;
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = Math.round(width * ratio); canvas.height = Math.round(height * ratio);
    context.setTransform(ratio, 0, 0, ratio, 0, 0); context.clearRect(0, 0, width, height);
    $('route-empty').hidden = Boolean(run && run.points.length);
    if (!run || !run.points.length) { canvas.setAttribute('aria-label', 'Route map: no route received'); return; }
    const c = colours(), pad = 34;
    const xs = run.points.map(p => p.x).concat(run.markers.map(m => m.x));
    const ys = run.points.map(p => p.y).concat(run.markers.map(m => m.y));
    let minX = Math.min(...xs), maxX = Math.max(...xs), minY = Math.min(...ys), maxY = Math.max(...ys);
    const span = Math.max(maxX - minX, maxY - minY, 40);            // at least 40 cm across
    const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
    const scale = Math.min((width - 2 * pad) / span, (height - 2 * pad) / span); // px per cm, equal axes
    const X = x => width / 2 + (x - cx) * scale, Y = y => height / 2 - (y - cy) * scale;
    // Grid and scale bar.
    const step = niceStep(span);
    context.strokeStyle = c.grid; context.lineWidth = 1;
    for (let gx = Math.floor((cx - width / 2 / scale) / step) * step; X(gx) <= width; gx += step) {
      context.beginPath(); context.moveTo(X(gx), 0); context.lineTo(X(gx), height); context.stroke();
    }
    for (let gy = Math.floor((cy - height / 2 / scale) / step) * step; Y(gy) >= 0; gy += step) {
      context.beginPath(); context.moveTo(0, Y(gy)); context.lineTo(width, Y(gy)); context.stroke();
    }
    context.fillStyle = c.ink; context.font = '11px ui-monospace, monospace'; context.textAlign = 'left';
    context.fillText(`grid ${step >= 100 ? (step / 100) + ' m' : step + ' cm'}`, 10, height - 10);
    // Path.
    context.strokeStyle = c.path; context.lineWidth = 2; context.lineJoin = 'round';
    context.beginPath();
    run.points.forEach((p, i) => i ? context.lineTo(X(p.x), Y(p.y)) : context.moveTo(X(p.x), Y(p.y)));
    context.stroke();
    // Start.
    context.fillStyle = c.ink; context.font = '600 11px -apple-system, sans-serif';
    context.beginPath(); context.arc(X(0), Y(0), 4, 0, 2 * Math.PI); context.fill();
    context.fillText(run.joined_mid_run ? 'Joined' : 'START', X(0) + 7, Y(0) - 7);
    // Markers.
    run.markers.forEach(m => marker(context, m.kind, X(m.x), Y(m.y), c));
    // Current position and heading (0 = START direction, anticlockwise +).
    const last = run.points[run.points.length - 1];
    if (run.active && number(run.heading_deg)) {
      const a = run.heading_deg * Math.PI / 180, lx = X(last.x), ly = Y(last.y);
      const dx = -Math.sin(a), dy = -Math.cos(a);           // canvas y grows downward
      context.fillStyle = c.path; context.beginPath();
      context.moveTo(lx + dx * 11, ly + dy * 11);
      context.lineTo(lx - dy * 6 - dx * 5, ly + dx * 6 - dy * 5);
      context.lineTo(lx + dy * 6 - dx * 5, ly - dx * 6 - dy * 5);
      context.closePath(); context.fill();
    }
    const counts = run.markers.reduce((acc, m) => (acc[m.kind] = (acc[m.kind] || 0) + 1, acc), {});
    canvas.setAttribute('aria-label', `Route map, run ${run.id}: ${run.path_m} m driven, ${run.points.length} points, ` +
      `${counts.barcode || 0} barcodes, ${counts.hump || 0} humps, ${counts.obstacle || 0} obstacles, ${counts.turn || 0} turns.`);
  }

  function render() {
    const runSelect = $('route-run'), run = current();
    runSelect.replaceChildren(...(route.runs.length ? route.runs.map(r => {
      const option = document.createElement('option');
      option.value = String(r.id);
      option.textContent = `Run ${r.id}${r.active ? ' · live' : ''} · ${clock(r.started_at)}`;
      return option;
    }) : [Object.assign(document.createElement('option'), {textContent: 'None yet'})]));
    runSelect.disabled = route.runs.length < 2;
    if (run) runSelect.value = String(run.id);
    $('route-summary').textContent = run
      ? `${run.active ? 'Live' : 'Ended'} · ${run.path_m.toFixed(2)} m driven · net ${run.distance_m.toFixed(2)} m along the route · heading ${run.heading_deg}° from START` +
        (run.joined_mid_run ? ' · dashboard joined mid-run, earlier path unknown' : '') +
        (run.dropped_points ? ` · ${run.dropped_points} later points not stored` : '')
      : 'Waiting for car/telemetry/pose. The route starts at the next START.';
    const rows = run ? run.markers.slice().reverse().map(m => {
      const tr = document.createElement('tr');
      for (const text of [m.label, metres(m.d), clock(m.time)]) {
        const td = document.createElement('td'); td.textContent = text; tr.append(td);
      }
      return tr;
    }) : [];
    if (!rows.length) {
      const tr = document.createElement('tr'), td = document.createElement('td');
      td.colSpan = 3; td.className = 'empty-line'; td.textContent = 'No events on this route yet.';
      tr.append(td); rows.push(tr);
    }
    $('route-events').replaceChildren(...rows);
    draw();
  }

  $('route-run').addEventListener('change', event => { selected = Number(event.target.value); render(); });
  window.addEventListener('dashboard:state', event => {
    const next = event.detail?.route;
    if (!next || !Array.isArray(next.runs)) return;
    // Follow the newest run unless the user picked an older one.
    if (selected !== null && route.runs[0] && selected === route.runs[0].id) selected = null;
    route = next; render();
  });
  window.addEventListener('dashboard:appearance', draw);
  window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', draw);
  if ('ResizeObserver' in window) new ResizeObserver(draw).observe($('route-canvas').parentElement);
})();
