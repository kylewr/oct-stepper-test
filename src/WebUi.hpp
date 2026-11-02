#pragma once

#include <Arduino.h>

const char WEB_UI[] PROGMEM = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>Stepper Test Bench</title>
<style>
:root{color-scheme:dark;--bg:#090d13;--panel:#121a25;--line:#29394d;--text:#edf5ff;--muted:#91a5bd;--cyan:#39d6ff;--green:#50e39a;--amber:#ffc45b;--red:#ff596f}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:14px/1.4 ui-monospace,SFMono-Regular,Consolas,monospace}.shell{max-width:1500px;margin:auto;padding:16px}.top{display:flex;align-items:center;gap:10px;flex-wrap:wrap;position:sticky;top:0;z-index:5;padding:13px 0;background:#090d13ef}.top h1{font:700 19px system-ui;margin:0}.grow{flex:1}.status{display:flex;align-items:center;gap:7px;color:var(--muted)}.dot{width:9px;height:9px;border-radius:50%;background:var(--red)}.dot.on{background:var(--green)}
button,input,select{font:inherit}button{border:1px solid var(--line);border-radius:9px;background:#1a2635;color:var(--text);padding:10px 13px;cursor:pointer;touch-action:none}button:hover{border-color:#55789f}button:disabled{opacity:.35;cursor:not-allowed}.primary{background:#0b3c4e;border-color:#187b99;color:#b1efff}.warn{background:#4b340c;border-color:#956b20;color:#ffe1a5}.danger{background:#51131e;border-color:#aa3044;color:#ffd7dc;font-weight:800}
.bench-note{color:var(--muted);margin:0 0 12px}.axis-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(330px,1fr));gap:14px}.axis-grid.focus-mode{grid-template-columns:minmax(500px,1fr) 270px;align-items:start}.axis-grid.focus-mode .axis-card.focused{grid-column:1;grid-row:1/span 4}.axis-grid.focus-mode .axis-card:not(.focused){grid-column:2}.axis-grid.focus-mode .axis-card:not(.focused) .axis-controls{display:none}.axis-grid.focus-mode .axis-card:not(.focused) .readout{grid-template-columns:1fr;margin-bottom:0}.axis-grid.focus-mode .axis-card:not(.focused) .metric:nth-child(3){display:none}.card{background:var(--panel);border:1px solid var(--line);border-radius:15px;padding:15px}.hardware{margin-top:14px}.head{display:flex;align-items:center;gap:8px;flex-wrap:wrap;margin-bottom:12px}.head h2{font:700 18px system-ui;margin:0}.head .focus-button{margin-left:auto;padding:6px 9px;font-size:11px}.chip{padding:4px 8px;border-radius:99px;background:#202d3e;color:var(--muted);font-size:11px}.chip.running{color:var(--green);background:#103d31}.row{display:flex;gap:9px;align-items:end;flex-wrap:wrap}.field{display:flex;flex-direction:column;gap:5px;flex:1;min-width:105px}.field label,.control-label{font-size:11px;text-transform:uppercase;letter-spacing:.06em;color:var(--muted)}input,select{width:100%;min-width:0;padding:9px;border:1px solid var(--line);border-radius:8px;background:#0a111a;color:var(--text)}input[type=range]{padding:0;border:0;background:transparent;accent-color:var(--cyan)}
.readout{display:grid;grid-template-columns:repeat(3,1fr);gap:7px;margin:10px 0 13px}.metric{background:#0b121c;border-radius:9px;padding:9px}.metric b{display:block;font-size:17px;color:#fff;overflow-wrap:anywhere}.metric small{color:var(--muted)}.slider-control{margin:10px 0}.slider-head{display:flex;justify-content:space-between;gap:8px;margin-bottom:5px}.slider-value{color:var(--cyan)}.slider-inputs{display:grid;grid-template-columns:1fr 92px;gap:9px;align-items:center}.continuous{display:grid;grid-template-columns:1fr auto 1fr;gap:8px;margin:12px 0}.continuous button{height:50px;font-size:15px}.continuous button.active{background:#103d31;border-color:#29996f;color:var(--green)}.move{display:grid;grid-template-columns:1fr auto;gap:8px;align-items:end}.subtle{color:var(--muted);font-size:12px}
details{border-top:1px solid var(--line);margin-top:14px;padding-top:11px}summary{cursor:pointer;color:#c6d7e9;font-weight:700;margin-bottom:12px}.actions{display:flex;gap:8px;margin-top:11px;flex-wrap:wrap}.pin-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:9px;margin-top:12px}.pins{padding:11px;background:#0b121c;border-radius:9px}.pins strong{display:block;margin-bottom:8px}.check{display:flex;align-items:center;gap:7px;margin-top:10px}.check input{width:auto}.toast{position:fixed;right:18px;bottom:18px;display:none;max-width:360px;padding:12px 15px;border-radius:10px;background:#42141d;border:1px solid var(--red);z-index:10}
@media(max-width:720px){.shell{padding:10px}.top h1{width:100%}.axis-grid,.axis-grid.focus-mode{grid-template-columns:1fr}.axis-grid.focus-mode .axis-card.focused,.axis-grid.focus-mode .axis-card:not(.focused){grid-column:1;grid-row:auto}.card{padding:13px}.move{grid-template-columns:1fr auto}}
</style>
</head>
<body><main class="shell">
<header class="top">
  <h1>STEPPER TEST BENCH</h1>
  <div class="status"><i id="connDot" class="dot"></i><span id="connText">offline</span></div>
  <div class="grow"></div>
  <button id="drivers" data-command>Enable drivers</button>
  <button id="estop" class="danger" data-command>STOP ALL</button>
</header>

<p class="bench-note">12 V starting profile: 400 steps/s, 800 steps/s², 1200 steps/s default ceiling. Set the correct effective steps/revolution to make the RPM display meaningful.</p>
<section id="axisGrid" class="axis-grid"></section>

<details class="card hardware" id="hardwarePanel">
  <summary id="hardwareSummary">Arduino shield</summary>
  <p class="subtle" id="hardwareHelp"></p>
  <div class="row">
    <div class="field"><label>Active axes</label><select id="hardwareAxisCount"><option>1</option><option>2</option><option>3</option><option>4</option></select></div>
  </div>
  <div id="directPinConfig">
    <div class="row">
      <div class="field"><label>Shared ENABLE GPIO</label><input id="enablePin" type="number" min="0" max="21"></div>
      <label class="check"><input id="enableActiveLow" type="checkbox"> ENABLE is active-low</label>
    </div>
    <div id="pinGrid" class="pin-grid"></div>
  </div>
  <div class="actions"><button id="saveHardware" class="primary" data-command>Save &amp; reboot</button></div>
</details>
</main><div id="toast" class="toast"></div>

<script>
(()=>{'use strict';
let socket,sequence=0,profiles=[],axisStates=[],hardware=null,driversEnabled=false,bridgeBackend=false,focusAxis=null;
const liveTimers=[];
const $=selector=>document.querySelector(selector);
const number=(value,fallback=0)=>Number.isFinite(Number(value))?Number(value):fallback;
const escapeHtml=value=>String(value).replace(/[&<>"']/g,char=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[char]));
const axisCard=axis=>document.querySelector(`[data-axis-card="${axis}"]`);

function send(type,axis=null,payload={}){
  if(!socket||socket.readyState!==WebSocket.OPEN)return;
  socket.send(JSON.stringify({v:1,id:String(++sequence),type,...(axis===null?{}:{axis}),payload}));
}

function toast(message){
  const element=$('#toast');
  element.textContent=message;
  element.style.display='block';
  clearTimeout(element.timer);
  element.timer=setTimeout(()=>element.style.display='none',3500);
}

function setConnected(connected){
  $('#connDot').classList.toggle('on',connected);
  $('#connText').textContent=connected?'connected':'offline';
  document.querySelectorAll('[data-command]').forEach(button=>button.disabled=!connected);
}

function connect(){
  const protocol=location.protocol==='https:'?'wss':'ws';
  socket=new WebSocket(`${protocol}://${location.host}/ws`);
  socket.onopen=()=>setConnected(true);
  socket.onclose=()=>{setConnected(false);setTimeout(connect,1500)};
  socket.onerror=()=>socket.close();
  socket.onmessage=event=>handle(JSON.parse(event.data));
}

function handle(message){
  if(message.type==='snapshot'){
    profiles=message.profiles||[];
    axisStates=message.axes||[];
    hardware=message.hardware||hardware;
    bridgeBackend=message.backend==='arduino-uart';
    driversEnabled=!!message.driversEnabled;
    renderAxisCards();
    renderHardware();
    updateAllReadouts();
  }else if(message.type==='telemetry'){
    axisStates=message.axes||[];
    driversEnabled=!!message.driversEnabled;
    updateAllReadouts();
  }else if(message.type==='error'){
    toast(message.message||message.code||'Command failed');
  }else if(message.type==='event'&&message.message){
    toast(message.message);
  }
}

function slider(label,field,value,min,max){
  return `<div class="slider-control">
    <div class="slider-head"><span class="control-label">${label}</span><span class="slider-value" data-value-for="${field}">${Number(value).toLocaleString()}</span></div>
    <div class="slider-inputs">
      <input type="range" data-field="${field}" min="${min}" max="${max}" step="1" value="${value}">
      <input type="number" data-field="${field}" min="${min}" max="${max}" value="${value}">
    </div>
  </div>`;
}

function renderAxisCards(){
  if(focusAxis!==null&&focusAxis>=profiles.length)focusAxis=null;
  $('#axisGrid').innerHTML=profiles.map((profile,axis)=>{
    const cycleLimit=Math.max(1000,Number(profile.defaultCycleSteps)*10);
    const dwellLimit=Math.max(5000,Number(profile.defaultDwellMs)*10);
    return `<section class="card axis-card" data-axis-card="${axis}">
      <div class="head"><h2>${escapeHtml(profile.label)}</h2><span class="chip" data-output="motionState">stopped</span><span class="chip" data-output="referenceState">uncertain</span><button class="focus-button" data-action="focus">${focusAxis===axis?'Restore':'Focus'}</button></div>
      <div class="readout">
        <div class="metric"><b data-output="position">0</b><small>steps</small></div>
        <div class="metric"><b data-output="speedNow">0</b><small>steps/s · <span data-output="rpmNow">0.0</span> RPM</small></div>
        <div class="metric"><b data-output="angle">0.00°</b><small>shaft angle</small></div>
      </div>
      <div class="axis-controls">
      ${slider('Speed · steps/s','speed',profile.defaultSpeed,1,profile.maxSpeed)}
      ${slider('Acceleration · steps/s²','acceleration',profile.defaultAcceleration,1,profile.maxAcceleration)}
      <div class="continuous">
        <button data-action="continuous" data-direction="-1" data-command>◀ RUN</button>
        <button data-action="stop" class="warn" data-command>STOP</button>
        <button data-action="continuous" data-direction="1" data-command>RUN ▶</button>
      </div>
      <div class="move">
        <div class="field"><label>Relative move · steps</label><input data-field="moveSteps" type="number" value="${profile.defaultJogSteps}"></div>
        <button data-action="move" class="primary" data-command>Move</button>
      </div>
      <div class="actions"><button data-action="zero" data-command>Set zero</button></div>
      <details>
        <summary>Cycle test</summary>
        ${slider('Travel · steps','cycleTravel',profile.defaultCycleSteps,1,cycleLimit)}
        ${slider('Dwell · ms','cycleDwell',profile.defaultDwellMs,0,dwellLimit)}
        <div class="field"><label>Round trips</label><input data-field="cycleRepeats" type="number" min="1" value="10"></div>
        <label class="check"><input data-field="cycleInfinite" type="checkbox"> run until stopped</label>
        <div class="actions"><button data-action="cycleStart" class="primary" data-command>Start cycle</button><button data-action="cycleStop" data-command>Stop cycle</button></div>
        <div class="subtle" data-output="cycleStatus">idle</div>
      </details>
      <details>
        <summary>Axis settings</summary>
        <div class="row">
          <div class="field"><label>Name</label><input data-field="profileLabel" maxlength="16" value="${escapeHtml(profile.label)}"></div>
          <div class="field"><label>Steps / revolution</label><input data-field="stepsPerRev" type="number" min="1" value="${profile.stepsPerRevolution}"></div>
          <div class="field"><label>Maximum speed</label><input data-field="maxSpeed" type="number" min="1" value="${profile.maxSpeed}"></div>
          <div class="field"><label>Maximum acceleration</label><input data-field="maxAcceleration" type="number" min="1" value="${profile.maxAcceleration}"></div>
        </div>
        <label class="check"><input data-field="invertDirection" type="checkbox" ${profile.directionInverted?'checked':''}> invert direction</label>
        <div class="actions"><button data-action="saveProfile" class="primary" data-command>Save axis settings</button></div>
      </details>
      </div>
    </section>`;
  }).join('');
  applyFocus();
  profiles.forEach((_,axis)=>refreshControlValues(axisCard(axis)));
  setConnected(!!socket&&socket.readyState===WebSocket.OPEN);
}

function field(card,name){return card.querySelector(`[data-field="${name}"]`)}
function integer(card,name,fallback=0){return Math.trunc(number(field(card,name)?.value,fallback))}
function motionValues(card){return{speed:integer(card,'speed'),acceleration:integer(card,'acceleration')}}

function refreshControlValues(card){
  if(!card)return;
  const axis=Number(card.dataset.axisCard),profile=profiles[axis]||{},steps=Math.max(1,integer(card,'stepsPerRev',profile.stepsPerRevolution||200));
  card.querySelectorAll('[data-value-for]').forEach(output=>{
    const name=output.dataset.valueFor,value=integer(card,name);
    if(name==='speed')output.textContent=`${value.toLocaleString()} · ${(value*60/steps).toFixed(1)} RPM`;
    else if(name==='acceleration')output.textContent=`${value.toLocaleString()} · ${(value*60/steps).toFixed(1)} RPM/s`;
    else output.textContent=value.toLocaleString();
  });
}

function applyFocus(){
  const grid=$('#axisGrid'),focused=focusAxis!==null;
  grid.classList.toggle('focus-mode',focused);
  grid.querySelectorAll('[data-axis-card]').forEach(card=>{
    const active=Number(card.dataset.axisCard)===focusAxis;
    card.classList.toggle('focused',active);
    const button=card.querySelector('[data-action="focus"]');
    if(button)button.textContent=active?'Restore':'Focus';
  });
}

function scheduleLiveMotion(axis,card){
  const direction=Number(card.dataset.continuousDirection||0);
  if(!direction||liveTimers[axis])return;
  liveTimers[axis]=setTimeout(()=>{
    liveTimers[axis]=null;
    const currentCard=axisCard(axis),currentDirection=Number(currentCard?.dataset.continuousDirection||0);
    if(currentCard&&currentDirection)send('jogStart',axis,{direction:currentDirection,...motionValues(currentCard)});
  },80);
}

function updateAllReadouts(){
  $('#drivers').textContent=driversEnabled?'Disable drivers':'Enable drivers';
  profiles.forEach((profile,axis)=>{
    const card=axisCard(axis),state=axisStates[axis];
    if(!card||!state)return;
    const out=name=>card.querySelector(`[data-output="${name}"]`);
    const revolutions=number(state.position)/number(profile.stepsPerRevolution,1);
    const degrees=((revolutions*360)%360+360)%360;
    out('position').textContent=Number(state.position).toLocaleString();
    out('speedNow').textContent=Number(state.speed).toLocaleString();
    out('rpmNow').textContent=(number(state.speed)*60/number(profile.stepsPerRevolution,1)).toFixed(1);
    out('angle').textContent=`${degrees.toFixed(2)}°`;
    const names=['stopped','moving','jogging','stopping'];
    out('motionState').textContent=names[state.mode]||'stopped';
    out('motionState').classList.toggle('running',state.mode!==0);
    out('referenceState').textContent=state.referenced?'zeroed':'uncertain';
    if(state.mode===2)card.dataset.continuousDirection=state.direction>=0?'1':'-1';
    else if(state.mode===0)card.dataset.continuousDirection='0';
    card.querySelectorAll('[data-action="continuous"]').forEach(button=>button.classList.toggle('active',state.mode===2&&Number(button.dataset.direction)===Number(card.dataset.continuousDirection)));
    const cycle=state.cycle||{},phases=['idle','moving out','dwelling out','moving back','dwelling back'];
    out('cycleStatus').textContent=cycle.infinite?`${phases[cycle.phase]||'idle'} · ${cycle.completed||0} / ∞`:`${phases[cycle.phase]||'idle'} · ${cycle.completed||0} / ${cycle.repeats||0}`;
  });
}

function renderHardware(){
  if(!hardware)return;
  $('#hardwareSummary').textContent=bridgeBackend?'Arduino CNC shield':'Hardware pins';
  $('#hardwareHelp').textContent=bridgeBackend
    ?'Independent shield mapping: X D2/D5, Y D3/D6, Z D4/D7, A D12/D13, shared ENABLE D8.'
    :'Edit the direct GPIO mapping. Saving stops motion and reboots the controller.';
  $('#directPinConfig').style.display=bridgeBackend?'none':'';
  $('#hardwareAxisCount').value=hardware.axisCount;
  $('#enablePin').value=hardware.enablePin;
  $('#enableActiveLow').checked=!!hardware.enableActiveLow;
  $('#pinGrid').innerHTML=(hardware.axes||[]).map((pins,index)=>`<div class="pins">
    <strong>Axis ${index+1}</strong><div class="row">
      <div class="field"><label>STEP GPIO</label><input class="step-pin" data-axis="${index}" type="number" min="0" max="21" value="${pins.stepPin}"></div>
      <div class="field"><label>DIR GPIO</label><input class="direction-pin" data-axis="${index}" type="number" min="0" max="21" value="${pins.directionPin}"></div>
    </div></div>`).join('');
}

$('#axisGrid').addEventListener('input',event=>{
  const card=event.target.closest('[data-axis-card]'),name=event.target.dataset.field;
  if(!card||!name)return;
  if(event.target.type==='range'||event.target.type==='number'){
    card.querySelectorAll(`[data-field="${name}"]`).forEach(input=>{if(input!==event.target)input.value=event.target.value});
    refreshControlValues(card);
    if(name==='speed'||name==='acceleration')scheduleLiveMotion(Number(card.dataset.axisCard),card);
  }
});

$('#axisGrid').addEventListener('click',event=>{
  const button=event.target.closest('button[data-action]');
  if(!button)return;
  const card=button.closest('[data-axis-card]'),axis=Number(card.dataset.axisCard),action=button.dataset.action;
  if(action==='focus'){
    focusAxis=focusAxis===axis?null:axis;
    applyFocus();
    return;
  }
  if(action==='continuous'){
    const direction=Number(button.dataset.direction);
    if(Number(card.dataset.continuousDirection)===direction){
      card.dataset.continuousDirection='0';
      send('jogStop',axis);
    }else{
      card.dataset.continuousDirection=String(direction);
      send('jogStart',axis,{direction,...motionValues(card)});
    }
  }
  if(action==='move'){
    card.dataset.continuousDirection='0';
    send('moveRelative',axis,{steps:integer(card,'moveSteps'),...motionValues(card)});
  }
  if(action==='stop'){
    card.dataset.continuousDirection='0';
    send('stopAxis',axis);
  }
  if(action==='zero')send('zeroPosition',axis);
  if(action==='cycleStart'){
    card.dataset.continuousDirection='0';
    send('startCycle',axis,{travelSteps:Math.abs(integer(card,'cycleTravel')),initialDirection:1,dwellMs:Math.max(0,integer(card,'cycleDwell')),repeats:Math.max(1,integer(card,'cycleRepeats',1)),infinite:!!field(card,'cycleInfinite').checked,...motionValues(card)});
  }
  if(action==='cycleStop')send('stopCycle',axis);
  if(action==='saveProfile'){
    send('updateProfile',axis,{label:field(card,'profileLabel').value,stepsPerRevolution:integer(card,'stepsPerRev'),directionInverted:!!field(card,'invertDirection').checked,maxSpeed:integer(card,'maxSpeed'),maxAcceleration:integer(card,'maxAcceleration'),defaultSpeed:integer(card,'speed'),defaultAcceleration:integer(card,'acceleration'),defaultJogSteps:Math.max(1,Math.abs(integer(card,'moveSteps'))),defaultCycleSteps:Math.max(1,Math.abs(integer(card,'cycleTravel'))),defaultDwellMs:Math.max(0,integer(card,'cycleDwell'))});
  }
});

$('#drivers').onclick=()=>send('setDrivers',null,{enabled:!driversEnabled});
$('#estop').onclick=()=>send('emergencyStop');
$('#saveHardware').onclick=()=>{
  const axes=(hardware?.axes||[]).map((_,index)=>({
    stepPin:Math.trunc(number(document.querySelector(`.step-pin[data-axis="${index}"]`)?.value,-1)),
    directionPin:Math.trunc(number(document.querySelector(`.direction-pin[data-axis="${index}"]`)?.value,-1))
  }));
  send('updateHardware',null,{axisCount:Math.trunc(number($('#hardwareAxisCount').value)),enablePin:Math.trunc(number($('#enablePin').value,-1)),enableActiveLow:$('#enableActiveLow').checked,axes});
};

setConnected(false);
connect();
})();
</script>
</body></html>)HTML";
