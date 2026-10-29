/* Read-only charts. Host histories and MCU snapshots have separate clocks. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const numeric = v => typeof v === 'number' && Number.isFinite(v);
  const make = (tag,text,cls) => {const n=document.createElement(tag); if(text!==undefined)n.textContent=text;if(cls)n.className=cls;return n;};
  const active = () => !$('page-debug').hidden;
  let data=null, state=null, frozen=null, paused=false, loading=false, timer, buddy='1', built='', charts=[], error='';
  const view = () => paused ? frozen : data;
  const series = (label,key,kind='measured') => ({label,key,kind});
  function definitions(v) {
    const ctrl=v?.controller?.samples || [], reports=v?.telemetry || {};
    const controller=(title,lines,unit,valid,range) => ({title,lines,unit,points:ctrl.map(p=>({t:p.t_ms,gap:p.gap_before,d:p.data})),valid,range,clock:'MCU source time · up to 5 Hz'});
    const report=(title,name,lines,unit,range,discrete=false) => ({title,lines,unit,points:(reports[name]||[]).map(p=>({t:p.t_ms,d:p.data})),range,discrete,clock:'Laptop receive time · telemetry reports, not continuous sensor samples'});
    if(buddy==='1') return [
      {title:'Messages reaching this laptop',lines:[series('Received','messages')],unit:'msg/s',points:(v?.rates||[]).map(p=>({t:p.t_ms,d:p})),clock:'Completed one-second buckets · includes diagnostic traffic'},
      {title:'Application payload reaching this laptop',lines:[series('Payload','bytes')],unit:'bytes/s',points:(v?.rates||[]).map(p=>({t:p.t_ms,d:p})),clock:'Excludes MQTT, TCP and Wi-Fi headers; not a RAM measurement'}];
    if(buddy==='2') {
      if(!v?.controller?.received) return [report('Left wheel · measured only','motion',[series('Measured','left_mms')],'mm/s'),report('Right wheel · measured only','motion',[series('Measured','right_mms')],'mm/s')];
      return [controller('Left wheel speed',[series('PID setpoint','setpoint_left','target'),series('Measured','measured_left')],'mm/s',d=>d.motion_valid),
        controller('Right wheel speed',[series('PID setpoint','setpoint_right','target'),series('Measured','measured_right')],'mm/s',d=>d.motion_valid),
        controller('Signed motor duty',[series('Left','duty_left','target'),series('Right','duty_right')],'%',d=>d.motion_valid,[-100,100])];
    }
    if(buddy==='3') return [controller('Line-position error',[series('Error','line_error')],'±100',d=>d.line_flags&1,[-100,100])];
    if(buddy==='4') return [report('Heading reports','heading',[series('Heading','heading_deg')],'°',[0,360]),report('Reported hump height','hump',[series('Height','height_mm')],'mm',null,true)];
    return [report('Obstacle distance reports','obstacle',[series('Distance','distance_mm')],'mm',null,true),report('Obstacle width reports','obstacle',[series('Width','width_mm')],'mm',null,true)];
  }
  function layout(defs) {
    const key=buddy+':'+defs.map(d=>d.title).join('|');
    if(key===built)return;
    built=key;charts=[];const list=$('buddy-chart-list');list.replaceChildren();
    defs.forEach((def,index)=>{
      const section=make('section',undefined,'buddy-chart');section.append(make('h4',def.title));
      const legend=make('div',undefined,'chart-legend');
      def.lines.forEach(line=>{const label=make('span',undefined,line.kind);label.append(make('i'),document.createTextNode(line.label));legend.append(label);});
      section.append(legend);
      const box=make('div',undefined,'sensor-plot'),canvas=make('canvas'),empty=make('p','Not received / unavailable');
      canvas.setAttribute('role','img');canvas.setAttribute('aria-label',def.title);box.append(canvas,empty);section.append(box);
      const note=make('p','', 'buddy-chart-note');section.append(note);list.append(section);charts.push({canvas,empty,note,index});
    });
  }
  function plot(chart,def) {
    const {canvas,empty,note}=chart,ctx=canvas.getContext('2d');if(!ctx)return;
    const width=canvas.clientWidth,height=canvas.clientHeight;if(width<10)return;
    const ratio=Math.min(window.devicePixelRatio||1,2);canvas.width=width*ratio;canvas.height=height*ratio;ctx.setTransform(ratio,0,0,ratio,0,0);
    const windowMs=Number($('buddy-window').value)*1000;
    const end=def.points.length ? def.points.at(-1).t : windowMs,start=Math.max(0,end-windowMs);
    const points=def.points.filter(p=>p.t>=start),values=[];
    points.forEach(p=>def.lines.forEach(l=>{if(p.d && (!def.valid||def.valid(p.d)) && numeric(p.d[l.key]))values.push(p.d[l.key]);}));
    const lo=def.range?.[0] ?? Math.min(0,...values), hi=def.range?.[1] ?? Math.max(1,...values)*1.08;
    const top=12,left=48,bottom=28,right=12, pw=width-left-right,ph=height-top-bottom;
    const x=t=>left+(t-start)/Math.max(1,end-start)*pw,y=v=>top+ph*(1-(v-lo)/Math.max(1,hi-lo));
    const css=getComputedStyle(document.documentElement),ink=css.color,grid=getComputedStyle(canvas.parentElement).borderTopColor;
    // Resolve theme variables through actual DOM colors (light-dark supported).
    const targetInk=getComputedStyle($('tab-debug')).borderBottomColor;
    const measuredLegend=document.querySelector('.chart-legend .measured i');
    const measuredInk=measuredLegend ? getComputedStyle(measuredLegend).borderTopColor : targetInk;
    ctx.font='10px ui-monospace,monospace';ctx.fillStyle=ink;ctx.strokeStyle=grid;ctx.lineWidth=1;
    for(const v of [lo,(lo+hi)/2,hi]){ctx.beginPath();ctx.moveTo(left,y(v));ctx.lineTo(width-right,y(v));ctx.stroke();ctx.textAlign='right';ctx.fillText(String(Math.round(v)),left-6,y(v)+3);}
    for(let i=0;i<=2;i++){const t=start+(end-start)*i/2;ctx.textAlign=i===0?'left':i===2?'right':'center';ctx.fillText(((t-end)/1000).toFixed(1)+' s',x(t),height-7);}
    ctx.save();ctx.beginPath();ctx.rect(left,top,pw,ph);ctx.clip();
    def.lines.forEach(l=>{
      ctx.strokeStyle=l.kind==='target'?targetInk:measuredInk;ctx.fillStyle=ctx.strokeStyle;ctx.lineWidth=1.6;ctx.setLineDash(l.kind==='target'?[5,4]:[]);
      let previous=null;ctx.beginPath();
      points.forEach(p=>{
        const value=p.d && (!def.valid||def.valid(p.d)) ? p.d[l.key] : null;
        if(!numeric(value)){previous=null;return;}
        const gap=p.gap || (previous && p.t-previous.t>(def.clock.startsWith('MCU')?600:3000));
        const wrap=l.key==='heading_deg' && previous && Math.abs(value-previous.value)>180;
        if(def.discrete || !previous || gap || wrap)ctx.moveTo(x(p.t),y(value));else ctx.lineTo(x(p.t),y(value));
        if(def.discrete || points.length===1){ctx.fillRect(x(p.t)-2,y(value)-2,4,4);}
        previous={t:p.t,value};
      });ctx.stroke();
    });ctx.restore();
    empty.hidden=Boolean(values.length);
    empty.textContent=def.clock.startsWith('MCU')?'Requires updated IR_DEBUG=1 firmware.':'No valid telemetry reports received.';
    note.textContent=`${def.clock} · ${def.unit}${values.length ? ` · observed range ${Math.min(...values)}–${Math.max(...values)}` : ''}${paused?' · paused':''}`;
    canvas.setAttribute('aria-label',`${def.title}: ${values.length} plotted values${values.length?`, range ${Math.min(...values)} to ${Math.max(...values)} ${def.unit}`:', unavailable'}; ${paused?'paused':'rolling'} view.`);
  }
  function extra(v) {
    const target=$('buddy-extra');target.replaceChildren();
    if(buddy==='1'){
      target.append(make('h4','Laptop → broker connection history'));
      const list=make('ul',undefined,'connection-history');
      (v?.connections||[]).slice(-12).reverse().forEach(p=>list.append(make('li',`${Math.max(0,((v.now_ms-p.t_ms)/1000)).toFixed(1)} s ago · ${p.status}`)));
      if(!list.childNodes.length)list.append(make('li','No connection transitions recorded.'));
      target.append(list,make('p','This is the laptop connection, not the Pico’s association/DHCP history. MCU queue depth, publish failures, free RAM and stack high-water marks are not published.', 'future-signal'));
    }else if(buddy==='2'){
      target.append(make('p','Speed graphs: dashed = ramped PID setpoint; solid = measured speed from the same snapshot. Duty graph: dashed = left, solid = right. Resolution is 1 mm/s and 1% duty. PID component terms and measured stop latency are unavailable. Invalid or contended snapshots leave gaps.', 'buddy-chart-note'));
    }else if(buddy==='3'){
      const d=v?.controller?.latest?.data;
      target.append(make('h4','Active calibration references'));
      if(!d){target.append(make('p','Not available until the updated diagnostic firmware reports calibration.', 'future-signal'));return;}
      const table=make('table',undefined,'calibration-table'),head=make('tr');
      ['Sensor','Raw white / black','White / black limits','Accepted'].forEach(t=>head.append(make('th',t)));
      const thead=make('thead');thead.append(head);table.append(thead);const body=make('tbody');
      [['left',1,'Left · GP27'],['right',2,'Right · GP26'],['barcode',0,'Barcode · GP28']].forEach(([c,bit,label])=>{
        const row=make('tr');[label,`${d[c+'_white']} / ${d[c+'_black']}`,`${d[c+'_white_limit']} / ${d[c+'_black_limit']} /1000`,d.accepted_calibration&(1<<bit)?'Yes':'No'].forEach(t=>row.append(make('td',t)));body.append(row);
      });table.append(body);const wrap=make('div',undefined,'table-wrap');wrap.append(table);target.append(wrap);
      const name=['Barcode','Left','Right'][d.cal_sensor];
      target.append(make('p',`${d.collecting?'Collecting':'Last collection'} · ${name}: white ${d.white_count?`${d.white_min}–${d.white_max}`:'not sampled'} (n=${d.white_count}), black ${d.black_count?`${d.black_min}–${d.black_max}`:'not sampled'} (n=${d.black_count}). Active references are not the full observed ranges.`, 'buddy-chart-note'));
      target.append(make('p','Line error: negative = line left, positive = line right. Both white can be normal in the current between-sensors mounting. Diagnostic states are sampled at 5 Hz; event counters preserve brief events but are not exact event timestamps.', 'buddy-chart-note'));
    }else if(buddy==='4')target.append(make('p','Pitch / roll, raw acceleration / gyro and estimate-validity diagnostics: not published. Hump height is shown as individual reports, not a continuous height signal. Current course firmware has no Buddy 4 telemetry adapter.', 'future-signal'));
    else target.append(make('p','Distance versus scan angle, per-angle no-echo, avoidance/recovery state and original-line confirmation: not published. A summary distance cannot be turned into a scan profile. No-echo is shown separately in the fault table, never guessed from a zero distance.', 'future-signal'));
  }
  function draw(){
    if(!active())return;
    const v=view(),defs=definitions(v);layout(defs);charts.forEach((c,i)=>plot(c,defs[i]));extra(v);
    const control=data?.controller;
    const status=$('buddy-graph-status');status.className='table-note';
    if(error){status.textContent=error+' Last-known plots retained.';status.classList.add('warning-text');}
    else if(['2','3'].includes(buddy)){
      status.textContent=control?.received ? `${data.mode==='demo'?'SIMULATED':'Wireless'} controller diagnostics · ${control.stale?'stale':'receiving'} · last ${control.age_s?.toFixed(1)} s ago${paused?' · graphs paused':''}` : 'No controller diagnostic packet received. Existing measured-speed reports remain available when supplied.';
      if(control?.stale)status.classList.add('warning-text');
    }else {
      const disconnected=state?.connection?.status!=='connected';
      status.textContent=`${data?.mode==='demo'?'SIMULATED · ':''}${disconnected?'Broker disconnected · ':''}Laptop receive-time histories · no cached snapshot points${paused?' · graphs paused':''}`;
      if(disconnected)status.classList.add('warning-text');
    }
  }
  async function poll(){
    if(loading)return;if(!active()){timer=setTimeout(poll,1000);return;}
    loading=true;const abort=new AbortController(),timeout=setTimeout(()=>abort.abort(),3000);
    try{const response=await fetch('/api/debug',{cache:'no-store',signal:abort.signal});if(!response.ok)throw new Error();data=await response.json();error='';}
    catch(_){error='Cannot reach the diagnostic service.';}
    finally{clearTimeout(timeout);loading=false;draw();timer=setTimeout(poll,active()?750:1000);}
  }
  function wake(){if(!active())return;clearTimeout(timer);draw();poll();}
  window.addEventListener('dashboard:buddy',event=>{buddy=event.detail;wake();});
  window.addEventListener('dashboard:state',event=>{state=event.detail;draw();});
  window.addEventListener('dashboard:appearance',draw);window.addEventListener('resize',draw);
  $('buddy-window').addEventListener('change',draw);
  $('buddy-pause').addEventListener('click',()=>{paused=!paused;frozen=paused?data:null;$('buddy-pause').textContent=paused?'Resume graphs':'Pause graphs';$('buddy-pause').setAttribute('aria-pressed',String(paused));draw();});
  $('buddy-save').addEventListener('click',()=>{
    if(!view())return;
    const blob=new Blob([JSON.stringify({format:'robot-debug-evidence-v1',mode:view().mode,saved_at:new Date().toISOString(),graphs:view(),latest_status:state},null,2)],{type:'application/json'});
    const url=URL.createObjectURL(blob),a=make('a');a.href=url;a.download=`buddy-${buddy}-debug-${view().mode}-${Date.now()}.json`;document.body.append(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(url),1000);
  });
  poll();
})();
