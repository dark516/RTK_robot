"use strict";

const model = { revision: -1, sections: {}, calibration: null, connected: false, eventCutoff: 0 };
const $ = id => document.getElementById(id);
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const esc = value => String(value ?? "—").replace(/[&<>"']/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c]));
const label = value => value?.label ?? "—";
const short = value => value ? String(value).slice(0, 12) : "—";
const fixed = (value, digits=2) => Number.isFinite(value) ? Number(value).toFixed(digits) : "—";

let eventSource = null;
let reconnectTimer = null;
let renderQueued = false;
let mapPixels = null;
let mapSignature = "";
const view = { centerX: 0, centerY: 0, ppm: 100, follow: true, dragging: false, lastX: 0, lastY: 0 };
const layers = { grid: true, sites: true };

async function loadSnapshot() {
  try {
    const response = await fetch("/api/snapshot", {cache: "no-store"});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const data = await response.json();
    model.revision = data.revision;
    model.sections = data.sections || {};
    await loadCalibrationOverlay();
    setConnected(true);
    renderAll();
    connectEvents();
  } catch (error) {
    setConnected(false);
    scheduleReconnect();
  }
}

function connectEvents() {
  eventSource?.close();
  eventSource = new EventSource("/api/events");
  eventSource.addEventListener("state", event => {
    const update = JSON.parse(event.data);
    model.revision = update.revision;
    model.sections = update.full ? (update.sections || {}) : {...model.sections, ...(update.sections || {})};
    setConnected(true);
    renderAll();
  });
  eventSource.onopen = () => setConnected(true);
  eventSource.onerror = () => { setConnected(false); eventSource.close(); scheduleReconnect(); };
}

function scheduleReconnect() {
  clearTimeout(reconnectTimer);
  reconnectTimer = setTimeout(loadSnapshot, 1500);
}

function setConnected(connected) {
  model.connected = connected;
  $("disconnect-banner").classList.toggle("visible", !connected);
  const chip = $("connection-chip");
  chip.className = `status-chip ${connected ? "live" : "offline"}`;
  chip.querySelector("span").textContent = connected ? "СВЯЗЬ OK" : "НЕТ СВЯЗИ";
}

function renderAll() {
  renderHeader(); renderMission(); renderMotion(); renderPerception(); renderKnownSigns(); renderFieldStatus();
  renderSources(); renderDiagnostics(); renderEvents(); renderControls(); prepareMap(); scheduleMap();
}

async function loadCalibrationOverlay() {
  try {
    const listResponse = await fetch("/api/calibration/drafts", {cache:"no-store"});
    if (!listResponse.ok) return;
    const listPayload = await listResponse.json();
    const listing = listPayload.data || listPayload;
    const summary = listing.active || listing.drafts?.[0];
    if (!summary?.id) return;
    const response = await fetch(`/api/calibration/drafts/${encodeURIComponent(summary.id)}`, {cache:"no-store"});
    if (!response.ok) return;
    const draftPayload = await response.json();
    model.calibration = calibrationOverlay(draftPayload.data || draftPayload);
  } catch (_) { model.calibration = null; }
}

function calibrationOverlay(document) {
  const transform = document?.field?.field_to_map || {};
  const mapPoint = point => {
    const yaw=Number(transform.yaw_rad)||0,c=Math.cos(yaw),s=Math.sin(yaw);
    return [(Number(transform.x_m)||0)+c*point[0]-s*point[1],(Number(transform.y_m)||0)+s*point[0]+c*point[1]];
  };
  const editable = item => item?.provenance === "preset" ? mapPoint(item.nominal) : item?.active;
  const geometry = document?.geometry || {}, size=document?.field?.size_m || [4,4];
  return {
    id:document.id, name:document.name || document.id, lifecycle:document.lifecycle || "draft", transform,
    outline:[[0,0],[size[0],0],[size[0],size[1]],[0,size[1]]].map(mapPoint),
    cells:(geometry.cells||[]).map(cell=>({...cell, points:[[cell.bounds[0],cell.bounds[1]],[cell.bounds[2],cell.bounds[1]],[cell.bounds[2],cell.bounds[3]],[cell.bounds[0],cell.bounds[3]]].map(mapPoint)})),
    platforms:(geometry.platforms||[]).map(item=>({...item,points:item.polygon?.provenance === "preset" ? (item.polygon.nominal||[]).map(mapPoint) : item.polygon?.active||[]})),
    roads:(geometry.roads||[]).map(item=>({...item,points:(item.points||[]).map(editable)})),
    observations:(geometry.observation_points||[]).map(item=>({...item,point:editable(item.position)})),
    sites:(geometry.sign_sites||[]).map(item=>({...item,point:editable(item.position)})),
  };
}

function renderHeader() {
  const mission = model.sections.mission || {};
  const map = model.sections.map_state || {};
  $("run-id").textContent = short(mission.run_id || map.run_id);
  $("map-revision").textContent = Number.isFinite(map.revision) ? map.revision : "—";
}

function renderMission() {
  const m = model.sections.mission;
  $("mission-live").textContent = m ? "LIVE" : "НЕТ ДАННЫХ";
  $("mission-phase").textContent = label(m?.phase);
  $("mission-execution").textContent = label(m?.execution_state);
  $("mission-objective").textContent = short(m?.objective_id);
  $("mission-route").textContent = short(m?.route_id);
  $("mission-segment").textContent = short(m?.active_segment_id);
  $("mission-generation").textContent = m?.goal_generation ?? "—";
  const done = m?.serviced_stop_ids?.length || 0, required = m?.required_unique_bus_stops;
  $("stops-count").textContent = `${done} / ${required ?? "—"}`;
  $("stops-progress").style.width = `${required ? clamp(done / required * 100, 0, 100) : 0}%`;
  $("serviced-stops").textContent = done ? m.serviced_stop_ids.join(" · ") : "Ещё не обслужены";
  $("dwell-value").textContent = `${fixed(m?.current_dwell_s, 1)} с`;
  $("dwell-progress").style.width = `${clamp((m?.current_dwell_s || 0) / 2 * 100, 0, 100)}%`;
  $("mission-reason").textContent = m?.reason || "Ожидание состояния navigator";
}

function velocity(section) {
  return model.sections[section] || {};
}
function renderMotion() {
  const guard = model.sections.guard || {}, permit = model.sections.permit || {};
  const raw = velocity("raw_command"), final = velocity("final_command"), odom = model.sections.odometry?.twist || {};
  [["raw",raw],["final",final],["odom",odom]].forEach(([id,v]) => {
    $(`${id}-linear`).textContent = fixed(v.linear_x_mps);
    $(`${id}-angular`).textContent = `ω ${fixed(v.angular_z_radps)}`;
  });
  const lamp = $("motion-lamp"); lamp.classList.toggle("go", !!guard.motion_allowed);
  lamp.querySelector("span").textContent = guard.motion_allowed ? "MOTION" : "HOLD";
  $("guard-stopped").textContent = `STOP: ${guard.stopped == null ? "—" : guard.stopped ? "YES" : "NO"}`;
  $("guard-fault").textContent = `FAULT: ${guard.fault_latched == null ? "—" : guard.fault_latched ? "YES" : "NO"}`;
  $("permit-sequence").textContent = `PERMIT: ${permit.enabled ? `#${permit.sequence}` : "OFF"}`;
  $("guard-reason").textContent = guard.reason || "Ожидание motion_guard";
}

function renderPerception() {
  const data = model.sections.detections || {}, detections = data.detections || [];
  $("detection-count").textContent = detections.length;
  $("camera-sequence").textContent = `FRAME ${data.frame_sequence ?? "—"}`;
  const age = data.stamp_s ? Date.now()/1000 - data.stamp_s : null;
  $("camera-age").textContent = `AGE ${fixed(age,1)}s`;
  $("detection-list").innerHTML = detections.length ? detections.map(item => `
    <article class="detection"><i></i><div><b>${esc(label(item.class))}</b><span>${item.range_valid ? `${fixed(item.range_m)} м ± ${fixed(item.range_stddev_m)}` : "дистанция неизвестна"} · ${esc(item.detector_track_id || "без track")}</span></div><strong>${fixed((item.confidence || 0)*100,0)}%</strong></article>`).join("") : '<p class="empty-line">Детекций пока нет</p>';
}

function buildSemanticSigns(field, state, toWorld=fieldToWorld) {
  const records=new Map();
  const physicalKey=(site,face,kind)=>`${site||"?"}:${face||"?"}:${kind?.value??label(kind)}`;
  (state.targets||[]).forEach(target=>{
    const site=target.site_id||target.target_id;
    records.set(physicalKey(site,target.face_id,target.kind),{
      key:`target:${target.target_id}`, name:label(target.kind), site,
      face:target.face_id, state:label(target.state), point:[target.sign_pose?.x_m,target.sign_pose?.y_m], source:"target"
    });
  });
  (state.known_signs||[]).forEach(sign=>{
    const site=(field?.sign_sites||[]).find(item=>item.id===sign.site_id);
    // A known placement is more precise than the target projection and wins
    // when BUS_STOP/PARKING appears in both collections.
    records.set(physicalKey(sign.site_id,sign.face_id,sign.sign_class),{
      key:`known:${sign.site_id}:${sign.face_id}:${label(sign.sign_class)}`,
      name:label(sign.sign_class),site:sign.site_id,face:sign.face_id,
      approach:sign.approach_id,state:"CONFIRMED",
      point:site ? toWorld(site.position) : null,source:"known_sign"
    });
  });
  if ((state.known_signs||[]).length) {
    return [...records.values()];
  }
  const approaches=new Map((state.approaches||[]).map(item=>[item.approach_id,item]));
  (field?.sign_sites||[]).forEach(site=>(site.faces||[]).forEach(face=>{
    const approach=approaches.get(face.controlled_approach_id);
    (approach?.confirmed_sign_classes||[]).forEach(sign=>records.set(
      physicalKey(site.id,face.id,sign),{
        key:`${site.id}:${face.id}:${label(sign)}`,name:label(sign),site:site.id,face:face.id,
        approach:approach.approach_id,state:label(approach.state),point:toWorld(site.position),source:"approach"
      }
    ));
  }));
  return [...records.values()];
}

function semanticSigns() {
  return buildSemanticSigns(model.sections.field, model.sections.map_state||{});
}

function renderKnownSigns() {
  const signs=semanticSigns();
  $("semantic-sign-count").textContent=signs.length;
  $("known-sign-count").textContent=signs.length;
  $("sign-binding-state").textContent=signs.length ? `${signs.length} подтверждено и привязано к полю` : "Ожидание подтверждённого наблюдения";
  $("known-sign-list").innerHTML=signs.length ? signs.map(item=>`
    <article class="known-sign"><span class="sign-glyph">◆</span><div><b>${esc(item.name)}</b><span>${esc(item.site)}${item.face ? ` · ${esc(item.face)}`:""}</span><small>${esc(item.approach || item.source)} · ${esc(item.state)}</small></div></article>`).join("") : '<p class="empty-line">Подтверждённых знаков пока нет</p>';
}

function renderFieldStatus() {
  const calibration=model.calibration, field=model.sections.field, pose=model.sections.pose;
  $("field-preset").textContent=calibration?.name || (field?.available ? "ROS-конфигурация" : "не загружен");
  $("field-grid-state").textContent=calibration ? `5 × 5 · ${calibration.lifecycle}` : "5 × 5 · нет привязки";
  $("localization-state").textContent=pose ? `${fixed(pose.x_m)} / ${fixed(pose.y_m)}` : "нет TF map→base";
}

const SOURCE_NAMES = {map:"Карта",odom:"Одометрия",tf:"TF pose",map_state:"City map",mission:"Navigator",route:"Маршрут",guard:"Guard",permit:"Permit",detections:"Детектор",camera:"Камера",diagnostics:"Диагностика",map_planner_diagnostics:"Привязка знаков",raw_command:"Raw cmd",final_command:"Motor cmd",control_service:"Control"};
function renderSources() {
  const sources = model.sections.sources || {};
  const rows = Object.entries(SOURCE_NAMES).map(([key,name]) => {
    const value = sources[key] || {state:"missing",age_s:null};
    return `<div class="source ${esc(value.state)}"><i></i><span>${esc(name)}</span><b>${Number.isFinite(value.age_s) ? `${fixed(value.age_s,1)}s` : value.state.toUpperCase()}</b></div>`;
  });
  $("source-grid").innerHTML = rows.join("");
  const ready = Object.entries(SOURCE_NAMES).filter(([key]) => ["fresh","ready"].includes(sources[key]?.state)).length;
  $("ready-count").textContent = `${ready} / ${Object.keys(SOURCE_NAMES).length}`;
}

function renderDiagnostics() {
  const rows = [
    ...(model.sections.diagnostics?.status || []),
    ...(model.sections.map_planner_diagnostics?.status || []),
  ];
  $("diagnostic-count").textContent = rows.length;
  $("diagnostic-list").innerHTML = rows.length ? rows.map(row => {
    const details = row.name === "frob_city/map_planner"
      ? Object.entries(row.values || {}).filter(([key]) =>
          key === "detection_state" || key.startsWith("detection_frames_") ||
          key.startsWith("mapping.") || key.startsWith("evidence."))
          .map(([key,value]) => `<span><b>${esc(key)}</b>: ${esc(value)}</span>`).join("")
      : "";
    return `<div class="diagnostic-row"><div><strong>${esc(label(row.level))}</strong> ${esc(row.name)} — ${esc(row.message)}</div>${details ? `<small>${details}</small>` : ""}</div>`;
  }).join("") : '<p class="empty-line">Сообщений нет</p>';
}

function renderEvents() {
  const events = (model.sections.events || []).filter(item => item.time_s >= model.eventCutoff).slice().reverse();
  $("event-list").innerHTML = events.length ? events.map(item => `<li class="${esc(item.level)}"><time>${new Date(item.time_s*1000).toLocaleTimeString("ru-RU")}</time><b>${esc(item.source)}</b><span>${esc(item.message)}</span></li>`).join("") : '<li class="empty-line">Событий пока нет</li>';
}

const ALLOWED_PHASES = {START:["IDLE"],PAUSE:["BOARDING","PARKING_PRIOR","PARKING_SEARCH","PARKING_APPROACH"],RESUME:["PAUSED"],RESET:["IDLE","PAUSED","DONE","ABORTED"],ABORT:["IDLE","BOARDING","PARKING_PRIOR","PARKING_SEARCH","PARKING_APPROACH","PAUSED"]};
function renderControls() {
  const source = model.sections.sources?.control_service;
  const serviceReady = model.connected && source?.state === "ready";
  const phase = label(model.sections.mission?.phase);
  $("control-status").textContent = serviceReady ? `Готов · ${phase}` : "Сервис недоступен";
  document.querySelectorAll("[data-command]").forEach(button => {
    button.disabled = !serviceReady || !(ALLOWED_PHASES[button.dataset.command] || []).includes(phase);
  });
}

async function sendControl(command) {
  if (command === "START" || command === "RESET") {
    if (!confirm(command === "START" ? "Начать автономную миссию?" : "Сбросить текущую попытку?")) return;
  }
  if (command === "ABORT") toast("Отправлена команда немедленной остановки", "warning");
  try {
    const response = await fetch("/api/control", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({command,reason:"web_operator"})});
    const body = await response.json();
    toast(`${command}: ${body.reason || (body.accepted ? "принято" : "отклонено")}`, response.ok ? "info" : "error");
  } catch (error) { toast(`${command}: нет связи с сервером`, "error"); setConnected(false); }
}

function toast(message, level="info") {
  const node = document.createElement("div"); node.className = `toast ${level}`; node.textContent = message;
  $("toast-stack").append(node); setTimeout(() => node.remove(), 5000);
}

function decodeMap(map) {
  const signature = JSON.stringify([map.width,map.height,map.map_load_time_s,map.rle]);
  if (signature === mapSignature) return;
  mapSignature = signature; const values = [];
  (map.rle || []).forEach(([value,count]) => { for (let i=0;i<count;i++) values.push(value); });
  const canvas = document.createElement("canvas"); canvas.width = map.width; canvas.height = map.height;
  const context = canvas.getContext("2d"), image = context.createImageData(map.width,map.height);
  for (let y=0;y<map.height;y++) for (let x=0;x<map.width;x++) {
    const sourceY = map.height - 1 - y, value = values[sourceY*map.width+x] ?? -1, index=(y*map.width+x)*4;
    const color = value < 0 ? 33 : value > 65 ? 13 : Math.round(178 - value*1.25);
    image.data[index]=color; image.data[index+1]=value<0?42:color+5; image.data[index+2]=value<0?47:color+4; image.data[index+3]=255;
  }
  context.putImageData(image,0,0); mapPixels=canvas; fitMap();
}

function mapCorners(map) {
  const w=map.width*map.resolution_m,h=map.height*map.resolution_m,o=map.origin,yaw=o.yaw_rad||0,c=Math.cos(yaw),s=Math.sin(yaw);
  return [[0,0],[w,0],[w,h],[0,h]].map(([x,y]) => [o.x_m+c*x-s*y,o.y_m+s*x+c*y]);
}
function fitMap() {
  const map=model.sections.map, corners=map ? mapCorners(map) : model.calibration?.outline;
  if (!corners?.length) return;
  const xs=corners.map(p=>p[0]), ys=corners.map(p=>p[1]), canvas=$("map-canvas");
  view.centerX=(Math.min(...xs)+Math.max(...xs))/2; view.centerY=(Math.min(...ys)+Math.max(...ys))/2;
  view.ppm=Math.max(10,Math.min(canvas.clientWidth/(Math.max(...xs)-Math.min(...xs)+.5),canvas.clientHeight/(Math.max(...ys)-Math.min(...ys)+.5)));
  scheduleMap();
}
function worldToScreen(x,y) { const c=$("map-canvas"); return [c.width/2+(x-view.centerX)*view.ppm,c.height/2-(y-view.centerY)*view.ppm]; }
function fieldToWorld(point) { const t=model.sections.field?.field_to_map || model.calibration?.transform || {}, yaw=t.yaw_rad||0,c=Math.cos(yaw),s=Math.sin(yaw); return [(t.x_m||0)+c*point[0]-s*point[1],(t.y_m||0)+s*point[0]+c*point[1]]; }
function line(points,color,width=2,dash=[]) { if (!points?.length) return; const ctx=$("map-canvas").getContext("2d"); ctx.beginPath(); points.forEach((p,i)=>{const q=worldToScreen(p[0],p[1]);i?ctx.lineTo(...q):ctx.moveTo(...q);});ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dash);ctx.stroke();ctx.setLineDash([]); }
function polygon(points,fill,stroke) { if (!points?.length)return;const ctx=$("map-canvas").getContext("2d");ctx.beginPath();points.forEach((p,i)=>{const q=worldToScreen(p[0],p[1]);i?ctx.lineTo(...q):ctx.moveTo(...q);});ctx.closePath();ctx.fillStyle=fill;ctx.fill();ctx.strokeStyle=stroke;ctx.lineWidth=1;ctx.stroke(); }
function prepareMap() { const map=model.sections.map;if(map){decodeMap(map);$("map-empty").classList.add("hidden");}else if(model.calibration){$("map-empty").classList.add("hidden");if(view.centerX===0&&view.centerY===0)fitMap();}else $("map-empty").classList.remove("hidden"); }
function scheduleMap(){if(renderQueued)return;renderQueued=true;requestAnimationFrame(()=>{renderQueued=false;drawMap();});}
function drawMap(){
  const canvas=$("map-canvas"),rect=canvas.getBoundingClientRect(),dpr=devicePixelRatio||1;
  if(canvas.width!==Math.floor(rect.width*dpr)||canvas.height!==Math.floor(rect.height*dpr)){canvas.width=Math.floor(rect.width*dpr);canvas.height=Math.floor(rect.height*dpr);view.ppm*=dpr;}
  const ctx=canvas.getContext("2d");ctx.fillStyle="#080d10";ctx.fillRect(0,0,canvas.width,canvas.height);
  const map=model.sections.map;if(map&&mapPixels){const o=worldToScreen(map.origin.x_m,map.origin.y_m);ctx.save();ctx.translate(...o);ctx.rotate(-(map.origin.yaw_rad||0));ctx.scale(view.ppm*map.resolution_m,view.ppm*map.resolution_m);ctx.globalAlpha=.88;ctx.drawImage(mapPixels,0,-map.height);ctx.restore();}
  const field=model.sections.field, calibration=model.calibration;
  if(calibration){
    calibration.roads.forEach(r=>line(r.points,"rgba(85,217,208,.26)",Math.max(2,view.ppm*.06)));
    calibration.platforms.forEach(r=>polygon(r.points,"rgba(244,182,61,.12)","rgba(244,182,61,.58)"));
    if(layers.grid) drawFieldGrid(calibration);
    if(layers.sites) drawFieldSites(calibration);
  } else if(field?.available){
    field.drivable_regions?.forEach(r=>polygon(r.points.map(fieldToWorld),"rgba(85,217,208,.045)","rgba(85,217,208,.18)"));
    field.platforms?.forEach(r=>polygon(r.points.map(fieldToWorld),"rgba(244,182,61,.12)","rgba(244,182,61,.55)"));
    if(layers.grid) drawFallbackGrid(field);
    if(layers.sites) field.sign_sites?.forEach(site=>drawSite(fieldToWorld(site.position),site.id));
  }
  const permit=model.sections.permit;if(permit?.enabled){permit.segment?.allowed_regions?.forEach(r=>polygon(r,"rgba(124,227,161,.12)","rgba(124,227,161,.55)"));line(permit.segment?.path?.poses?.map(p=>[p.x_m,p.y_m]),"#7ce3a1",3);}
  line(model.sections.route?.poses?.map(p=>[p.x_m,p.y_m]),"#55d9d0",3,[9,5]);line((model.sections.trail||[]).map(p=>[p.x_m,p.y_m]),"rgba(244,182,61,.65)",1.5);
  semanticSigns().forEach(sign=>{const calibrated=calibration?.sites.find(site=>site.id===sign.site)?.point, p=calibrated||sign.point;drawSign(p?.[0],p?.[1],sign.name,sign.state);});
  const pose=model.sections.pose;if(pose){if(view.follow){view.centerX=pose.x_m;view.centerY=pose.y_m;}drawRobot(pose);$("robot-coordinates").textContent=`X ${fixed(pose.x_m)} · Y ${fixed(pose.y_m)} · Ψ ${fixed((pose.yaw_rad||0)*180/Math.PI,0)}°`;}
  $("map-scale").textContent=`${fixed(100/view.ppm,2)} м / 100 px`;
}
function drawFieldGrid(calibration){
  calibration.cells.forEach(cell=>{polygon(cell.points,cell.kind==="platform"?"rgba(244,182,61,.035)":"rgba(85,217,208,.018)","rgba(207,229,225,.23)");const centre=cell.points.reduce((a,p)=>[a[0]+p[0]/4,a[1]+p[1]/4],[0,0]),q=worldToScreen(...centre),ctx=$("map-canvas").getContext("2d");ctx.fillStyle="rgba(207,229,225,.58)";ctx.font="700 10px monospace";ctx.textAlign="center";ctx.fillText(cell.id,q[0],q[1]+3);ctx.textAlign="left";});
  line([...calibration.outline,calibration.outline[0]],"rgba(85,217,208,.8)",2);
}
function drawFallbackGrid(field){const size=field.size_m||[4,4],sx=size[0]/5,sy=size[1]/5;for(let row=0;row<5;row++)for(let col=0;col<5;col++){const points=[[col*sx,row*sy],[(col+1)*sx,row*sy],[(col+1)*sx,(row+1)*sy],[col*sx,(row+1)*sy]].map(fieldToWorld);polygon(points,"rgba(85,217,208,.018)","rgba(207,229,225,.23)");}}
function drawFieldSites(calibration){calibration.observations.forEach(item=>drawObservation(item.point,item.id));calibration.sites.forEach(item=>drawSite(item.point,item.id));}
function drawSite(point,id){if(!point?.every(Number.isFinite))return;const ctx=$("map-canvas").getContext("2d"),p=worldToScreen(...point);ctx.beginPath();ctx.arc(p[0],p[1],4,0,Math.PI*2);ctx.fillStyle="#55d9d0";ctx.fill();ctx.strokeStyle="#081013";ctx.lineWidth=1.5;ctx.stroke();}
function drawObservation(point,id){if(!point?.every(Number.isFinite))return;const ctx=$("map-canvas").getContext("2d"),p=worldToScreen(...point);ctx.strokeStyle="rgba(124,227,161,.75)";ctx.lineWidth=1;ctx.strokeRect(p[0]-4,p[1]-4,8,8);}
function drawSign(x,y,name,state){if(!Number.isFinite(x)||!Number.isFinite(y))return;const ctx=$("map-canvas").getContext("2d"),p=worldToScreen(x,y);ctx.save();ctx.translate(...p);ctx.rotate(Math.PI/4);ctx.fillStyle="#ff9254";ctx.fillRect(-6,-6,12,12);ctx.restore();ctx.fillStyle="#fff";ctx.font="600 10px monospace";ctx.fillText(name, p[0]+10,p[1]-8);}
function drawRobot(pose){const ctx=$("map-canvas").getContext("2d"),p=worldToScreen(pose.x_m,pose.y_m),r=12;ctx.save();ctx.translate(...p);ctx.rotate(-(pose.yaw_rad||0));ctx.beginPath();ctx.moveTo(r,0);ctx.lineTo(-r*.7,-r*.65);ctx.lineTo(-r*.45,0);ctx.lineTo(-r*.7,r*.65);ctx.closePath();ctx.fillStyle="#f4b63d";ctx.shadowColor="#f4b63d";ctx.shadowBlur=14;ctx.fill();ctx.restore();}

function setupMapControls(){const canvas=$("map-canvas");[["map-grid","grid"],["map-sites","sites"]].forEach(([id,key])=>$(id).onclick=()=>{layers[key]=!layers[key];$(id).classList.toggle("active",layers[key]);scheduleMap();});$("map-fit").onclick=()=>{view.follow=false;$("map-follow").classList.remove("active");fitMap();};$("map-follow").onclick=()=>{view.follow=!view.follow;$("map-follow").classList.toggle("active",view.follow);scheduleMap();};$("map-zoom-in").onclick=()=>{view.follow=false;view.ppm*=1.25;scheduleMap();};$("map-zoom-out").onclick=()=>{view.follow=false;view.ppm/=1.25;scheduleMap();};canvas.addEventListener("wheel",e=>{e.preventDefault();view.follow=false;view.ppm*=e.deltaY<0?1.12:.89;scheduleMap();},{passive:false});canvas.addEventListener("pointerdown",e=>{view.dragging=true;view.follow=false;view.lastX=e.clientX;view.lastY=e.clientY;canvas.setPointerCapture(e.pointerId);});canvas.addEventListener("pointermove",e=>{if(!view.dragging)return;view.centerX-=(e.clientX-view.lastX)/view.ppm;view.centerY+=(e.clientY-view.lastY)/view.ppm;view.lastX=e.clientX;view.lastY=e.clientY;scheduleMap();});canvas.addEventListener("pointerup",()=>view.dragging=false);new ResizeObserver(scheduleMap).observe(canvas);}
function setupCamera(){const toggle=$("camera-enabled"),image=$("camera-stream"),placeholder=$("camera-placeholder");const apply=()=>{if(toggle.checked){image.src=`/api/camera.mjpg?t=${Date.now()}`;placeholder.classList.remove("hidden");image.onload=()=>placeholder.classList.add("hidden");image.onerror=()=>placeholder.classList.remove("hidden");}else{image.removeAttribute("src");placeholder.classList.remove("hidden");placeholder.querySelector("b").textContent="ПОТОК ВЫКЛЮЧЕН";}};toggle.addEventListener("change",apply);apply();}

if (typeof document !== "undefined") document.addEventListener("DOMContentLoaded",()=>{
  setupMapControls();setupCamera();
  document.querySelectorAll("[data-command]").forEach(button=>button.addEventListener("click",()=>sendControl(button.dataset.command)));
  $("clear-events").onclick=()=>{model.eventCutoff=Date.now()/1000;renderEvents();};
  setInterval(()=>{$("clock").textContent=new Date().toLocaleTimeString("ru-RU");renderPerception();scheduleMap();},1000);
  loadSnapshot();
});

if (typeof module !== "undefined") module.exports = {buildSemanticSigns};
