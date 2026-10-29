/* Observation-only wireless IR graphs. All values come from /api/sensors. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const channels = ['left', 'right', 'barcode'];
  let live = null, frozen = null, paused = false, loading = false, timer, error = '';
  const number = value => typeof value === 'number' && Number.isFinite(value);
  const visible = () => !$('page-sensors').hidden;
  const source = () => paused ? frozen : live;
  const ago = seconds => number(seconds) ? `${seconds.toFixed(1)} s ago` : 'never';

  function samples() {
    const points = source()?.samples || [];
    const end = points.length ? points[points.length - 1].t_ms : 0;
    const start = Math.max(0, end - Number($('sensor-window').value) * 1000);
    return {points: points.filter(point => point.t_ms >= start), start, end: Math.max(start + 1, end)};
  }

  function plot(channel, series) {
    const canvas = $(`sensor-${channel}-canvas`), context = canvas.getContext('2d');
    if (!context) return;
    const width = canvas.clientWidth, height = canvas.clientHeight;
    if (width < 10 || height < 10) return;
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = Math.round(width * ratio); canvas.height = Math.round(height * ratio);
    context.setTransform(ratio, 0, 0, ratio, 0, 0); context.clearRect(0, 0, width, height);
    const signal = $('sensor-signal').value, max = signal === 'raw' ? 4095 : 1000;
    const left = 45, right = 12, top = 12, bottom = 40, plotWidth = width - left - right, plotHeight = height - top - bottom;
    const x = t => left + (t - series.start) / (series.end - series.start) * plotWidth;
    const y = value => top + plotHeight * (1 - value / max);
    const ink = getComputedStyle(document.documentElement).color;
    const grid = getComputedStyle(canvas.parentElement).borderTopColor;
    const accent = getComputedStyle($('tab-sensors')).borderBottomColor;
    const white = getComputedStyle(document.querySelector('.surface-white')).backgroundColor;
    const black = getComputedStyle(document.querySelector('.surface-black')).backgroundColor;
    context.font = '11px ui-monospace, monospace'; context.fillStyle = ink;
    context.strokeStyle = grid; context.lineWidth = 1;
    for (const value of [0, max / 2, max]) {
      context.beginPath(); context.moveTo(left, y(value)); context.lineTo(width - right, y(value)); context.stroke();
      context.textAlign = 'right'; context.fillText(String(Math.round(value)), left - 7, y(value) + 4);
    }
    const ticks = width < 450 ? 2 : 4;
    for (let i = 0; i <= ticks; i++) {
      const time = series.start + (series.end - series.start) * i / ticks;
      context.textAlign = i === 0 ? 'left' : i === ticks ? 'right' : 'center';
      context.fillText(`${((time - series.end) / 1000).toFixed(1)} s`, x(time), height - 9);
    }
    let drawing = false, values = [];
    context.save(); context.beginPath(); context.rect(left, top, plotWidth, plotHeight); context.clip();
    context.strokeStyle = accent; context.lineWidth = 1.5; context.beginPath();
    series.points.forEach(point => {
      const sensor = point.sensors[channel], value = sensor?.[signal];
      if (!number(value)) { drawing = false; return; }
      values.push(value);
      if (!drawing || point.gap_before) context.moveTo(x(point.t_ms), y(value));
      else context.lineTo(x(point.t_ms), y(value));
      drawing = true;
    });
    context.stroke(); context.restore();
    // The strip reports the firmware's actual classification, not a browser
    // threshold or a guess based on the curve. Gaps are deliberately empty.
    for (let i = 0; i < series.points.length - 1; i++) {
      const point = series.points[i], next = series.points[i + 1];
      const sensor = point.sensors[channel];
      if (!sensor?.valid || next.gap_before || !['white', 'black'].includes(sensor.surface)) continue;
      context.fillStyle = sensor.surface === 'white' ? white : black;
      context.fillRect(x(point.t_ms), height - 31, Math.max(0, x(next.t_ms) - x(point.t_ms)), 8);
    }
    const latest = source()?.latest?.sensors?.[channel];
    const value = latest?.[signal];
    $(`sensor-${channel}-reading`).textContent = `${number(value) ? value : '—'} ${signal === 'raw' ? 'ADC' : '/1000'} · ${latest?.surface || 'unavailable'}${paused ? ' · paused' : ''}`;
    $(`sensor-${channel}-empty`).hidden = values.length > 0;
    $(`sensor-${channel}-empty`).textContent = live?.received ? 'No valid readings for this channel in this window.' : 'Waiting for optional IR_DEBUG=1 firmware.';
    canvas.setAttribute('aria-label', `${channel} IR ${signal}: ${values.length} valid samples${values.length ? `, range ${Math.min(...values)} to ${Math.max(...values)}` : ''}; ${paused ? 'paused' : 'rolling'} ${(series.end - series.start) / 1000} second window.`);
  }

  function draw() {
    if (!visible()) return;
    const series = samples();
    channels.forEach(channel => plot(channel, series));
    const status = $('sensor-stream-status');
    status.className = 'muted';
    if (error) {
      status.textContent = error + ' Displayed samples are last known values.';
      status.classList.add('warning-text');
    } else if (!live?.received) {
      status.textContent = 'No IR stream received. Live graphs require the optional IR_DEBUG=1 firmware.';
    } else {
      status.textContent = `${live.source === 'simulation' ? 'Simulated' : 'Wireless'} IR preview · ${live.stale ? 'stale' : 'receiving'} · last packet ${ago(live.age_s)}${paused ? ' · graphs paused' : ''}`;
      if (live.stale) status.classList.add('warning-text');
    }
    $('sensor-save').disabled = !series.points.length;
    $('sensor-buffer-note').textContent = `${live?.received || 0} preview frames received · ${live?.skipped_previews || 0} preview frames skipped/not observed · ${live?.clock_restarts || 0} source-clock restarts. Up to 1200 frames are retained on the laptop. Pause freezes the view while reception continues.`;
  }

  async function poll() {
    if (loading) return;
    if (!visible()) { timer = setTimeout(poll, 1000); return; }
    loading = true;
    const controller = new AbortController(), timeout = setTimeout(() => controller.abort(), 3000);
    try {
      const response = await fetch('/api/sensors', {cache: 'no-store', signal: controller.signal});
      if (!response.ok) throw new Error('IR service unavailable');
      live = await response.json(); error = '';
    } catch (_) { error = 'Cannot reach the live sensor service.'; }
    finally {
      clearTimeout(timeout); loading = false; draw();
      timer = setTimeout(poll, visible() ? 300 : 1000);
    }
  }
  function wake() {
    if (!visible()) return;
    clearTimeout(timer); draw(); poll();
  }
  $('sensor-pause').addEventListener('click', () => {
    paused = !paused;
    frozen = paused ? live : null;
    $('sensor-pause').setAttribute('aria-pressed', String(paused));
    $('sensor-pause').textContent = paused ? 'Resume graphs' : 'Pause graphs';
    draw();
  });
  $('sensor-signal').addEventListener('change', draw);
  $('sensor-window').addEventListener('change', draw);
  $('sensor-save').addEventListener('click', () => {
    const data = source(), series = samples();
    if (!data || !series.points.length) return;
    const columns = ['source', 'time_ms', 'device_clock_us', 'sequence', 'gap_before', ...channels.flatMap(c => [`${c}_raw`, `${c}_darkness`, `${c}_surface`, `${c}_raw_valid`, `${c}_normalised_valid`])];
    const rows = series.points.map(point => [data.source, point.t_ms, point.timestamp_us, point.sequence, point.gap_before,
      ...channels.flatMap(c => {const s = point.sensors[c]; return [s.raw ?? '', s.darkness ?? '', s.surface, s.raw_valid, s.valid];})].join(','));
    const blob = new Blob([columns.join(',') + '\n' + rows.join('\n') + '\n'], {type: 'text/csv'});
    const url = URL.createObjectURL(blob), anchor = document.createElement('a');
    anchor.href = url; anchor.download = `ir-sensors-${data.source}-${new Date().toISOString().replaceAll(':', '-')}.csv`;
    document.body.append(anchor); anchor.click(); anchor.remove(); setTimeout(() => URL.revokeObjectURL(url), 1000);
  });
  document.querySelectorAll('[data-page]').forEach(button => button.addEventListener('click', wake));
  $('open-sensors-button').addEventListener('click', wake);
  window.addEventListener('dashboard:appearance', draw);
  window.addEventListener('dashboard:state', event => {
    const barcode = event.detail?.sections?.barcode;
    $('sensor-barcode-result').textContent = barcode?.valid ? `Last decoded letter: ${barcode.data.letter} → ${barcode.data.action}. Received separately from sensor frames; this does not confirm a completed turn.` : 'Last decoded letter: not received';
  });
  if ('ResizeObserver' in window) {
    const observer = new ResizeObserver(draw);
    channels.forEach(c => observer.observe($(`sensor-${c}-canvas`).parentElement));
  }
  window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', draw);
  poll();
})();
