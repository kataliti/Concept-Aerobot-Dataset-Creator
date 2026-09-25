#!/usr/bin/env python3
import argparse, csv, json, mimetypes, os, re, threading, urllib.parse, webbrowser
from bisect import bisect_left
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

CONCEPT_VLA_DIR = '/home/sed/Desktop/concept-vla'
HOST='127.0.0.1'; PORT=8765; VIDEO_FPS=30.0

ONTOLOGY = [{'id': 1,
  'name_ru': 'взлет',
  'name_en': 'takeoff',
  'states': [{'ru': 'подъем', 'en': 'ascending'}, {'ru': 'на высоте', 'en': 'at_altitude'}]},
 {'id': 2,
  'name_ru': 'посадка',
  'name_en': 'landing',
  'states': [{'ru': 'снижение', 'en': 'descending'}, {'ru': 'на платформе', 'en': 'on_platform'}]},
 {'id': 3,
  'name_ru': 'полет вперед',
  'name_en': 'forward_flight',
  'states': [{'ru': 'движение вперед', 'en': 'moving_forward'},
             {'ru': 'впереди стена', 'en': 'wall_ahead'},
             {'ru': 'замечена платформа', 'en': 'platform_detected'},
             {'ru': 'замечен qr', 'en': 'QR_code_detected'},
             {'ru': 'замечен круг', 'en': 'circle_detected'},
             {'ru': 'замечена фигура', 'en': 'shape_detected'}]},
 {'id': 4,
  'name_ru': 'центровка над qr',
  'name_en': 'centering_over_QR_code',
  'states': [{'ru': 'движение к qr', 'en': 'moving_toward_QR_code'},
             {'ru': 'qr в центре', 'en': 'QR_code_centered'}]},
 {'id': 5,
  'name_ru': 'центровка над платформой',
  'name_en': 'centering_over_platform',
  'states': [{'ru': 'движение к платформе', 'en': 'moving_toward_platform'},
             {'ru': 'платформа в центре', 'en': 'platform_centered'}]},
 {'id': 6,
  'name_ru': 'левый разворот параллельно стене',
  'name_en': 'left_turn_parallel_to_wall',
  'states': [{'ru': 'разворачивание', 'en': 'turning'}, {'ru': 'стена справа', 'en': 'wall_on_the_right'}]},
 {'id': 7,
  'name_ru': 'полет вдоль стены',
  'name_en': 'flight_along_the_wall',
  'states': [{'ru': 'движение вперед', 'en': 'moving_forward'},
             {'ru': 'впереди стена', 'en': 'wall_ahead'},
             {'ru': 'замечена платформа', 'en': 'platform_detected'},
             {'ru': 'замечен qr', 'en': 'QR_code_detected'},
             {'ru': 'справа дверной проем', 'en': 'doorway_on_the_right'},
             {'ru': 'замечен круг', 'en': 'circle_detected'},
             {'ru': 'замечена фигура', 'en': 'shape_detected'}]},
 {'id': 8,
  'name_ru': 'левый разворот над платформой',
  'name_en': 'left_turn_over_platform',
  'states': [{'ru': 'разворачивание', 'en': 'turning'},
             {'ru': 'платформа прямо по курсу', 'en': 'platform_directly_ahead'}]},
 {'id': 9,
  'name_ru': 'правый разворот напротив проема',
  'name_en': 'right_turn_opposite_doorway',
  'states': [{'ru': 'разворачивание', 'en': 'turning'},
             {'ru': 'проем прямо по курсу', 'en': 'doorway_directly_ahead'}]},
 {'id': 10,
  'name_ru': 'подъем до перекладины',
  'name_en': 'ascending_to_crossbar_level',
  'states': [{'ru': 'подъем', 'en': 'ascending'},
             {'ru': 'перед перекладиной', 'en': 'in_front_of_crossbar'}]},
 {'id': 11,
  'name_ru': 'центровка напротив qr',
  'name_en': 'centering_opposite_QR_code',
  'states': [{'ru': 'движение к qr', 'en': 'moving_toward_QR_code'},
             {'ru': 'qr в центре', 'en': 'QR_code_centered'}]},
 {'id': 12,
  'name_ru': 'спуск до нормальной высоты',
  'name_en': 'descending_to_normal_altitude',
  'states': [{'ru': 'снижение', 'en': 'descending'}, {'ru': 'на высоте', 'en': 'at_altitude'}]},
 {'id': 13,
  'name_ru': 'центровка над кругом',
  'name_en': 'centering_over_circle',
  'states': [{'ru': 'движение к кругу', 'en': 'moving_toward_circle'},
             {'ru': 'круг в центре', 'en': 'circle_centered'}]},
 {'id': 14,
  'name_ru': 'центровка над фигурой',
  'name_en': 'centering_over_shape',
  'states': [{'ru': 'движение к фигуре', 'en': 'moving_toward_shape'},
             {'ru': 'фигура в центре', 'en': 'shape_centered'}]},
 {'id': 15,
  'name_ru': 'полет над линией',
  'name_en': 'flight_above_line',
  'states': [{'ru': 'движение вперед', 'en': 'moving_forward'},
             {'ru': 'впереди столб', 'en': 'pillar_ahead'},
             {'ru': 'впереди рамка', 'en': 'frame_ahead'},
             {'ru': 'замечена платформа', 'en': 'platform_detected'}]},
 {'id': 16,
  'name_ru': 'обход столба',
  'name_en': 'pillar_avoidance',
  'states': [{'ru': 'облет', 'en': 'circling_around'}, {'ru': 'столб сзади', 'en': 'pillar_behind'}]},
 {'id': 17,
  'name_ru': 'пролет в рамку',
  'name_en': 'flying_through_frame',
  'states': [{'ru': 'движение к рамке', 'en': 'moving_toward_frame'},
             {'ru': 'рамка позади', 'en': 'frame_behind'}]},
 {'id': 18,
  'name_ru': 'пролет в дверь',
  'name_en': 'passage_door',
  'states': [{'ru': 'в проеме', 'en': 'in_doorway'},
             {'ru': 'проем сзади', 'en': 'doorway_back'}]}]
ONTOLOGY_JSON = ONTOLOGY

HTML=r'''<!doctype html><html lang="ru"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Concept-VLA · разметка</title><style>
body{margin:0;background:#151515;color:#eee;font-family:system-ui}header{padding:10px 14px;border-bottom:1px solid #333;display:flex;gap:20px;flex-wrap:wrap}.grid{display:grid;grid-template-columns:1.5fr .8fr;height:calc(100vh - 48px)}.left,.right{padding:10px;overflow:auto}.right{border-left:1px solid #333}.videos{display:grid;grid-template-columns:1fr 1fr;gap:8px}.box{background:#000;border:1px solid #333}.box h3{font-size:12px;margin:0;padding:5px;background:#222}video{width:100%;display:block;max-height:46vh}button{background:#292929;color:#eee;border:1px solid #555;border-radius:6px;padding:7px;cursor:pointer}button.active{outline:2px solid #62aaff;background:#20334b}button.stateActive{outline:2px solid #67df8c;background:#1e4930}.controls{display:flex;gap:6px;flex-wrap:wrap;margin:8px 0}.programs{display:grid;grid-template-columns:1fr 1fr;gap:5px}.programs button{text-align:left;font-size:12px}.states{display:flex;gap:5px;flex-wrap:wrap}.section{border:1px solid #333;border-radius:7px;margin-bottom:10px;overflow:hidden}.section h2{font-size:14px;margin:0;background:#222;padding:7px}.body{padding:8px}.timebar{display:grid;grid-template-columns:100px 1fr 100px;gap:8px;align-items:center}input{width:100%}.small{font-size:12px;color:#aaa}.current{background:#1f1f1f;padding:8px;border-radius:5px}table{width:100%;border-collapse:collapse;font-size:12px}th,td{padding:5px;border-bottom:1px solid #333;text-align:left}.good{color:#9f9}.bad{color:#f99}.group{padding:7px;border:1px solid #31465b;background:#18232e;border-radius:6px;margin-bottom:8px;font-size:12px}.editing{background:#2b2618}.editBadge{display:none;margin-top:7px;padding:6px 8px;border:1px solid #8c7431;background:#302916;border-radius:5px;color:#ffe09a}.editBadge.on{display:block}.rowActions{white-space:nowrap}.rowActions button{padding:4px 7px;margin-right:4px}.editBtn{border-color:#8c7431}.deleteBtn{border-color:#7d3c3c}
</style></head><body><header><b>Concept-VLA · первичная разметка</b><span id="flight"></span><span id="status">загрузка…</span></header><div class="grid"><div class="left">
<div class="group" id="group"></div>
<div class="videos"><div class="box"><h3>ПЕРЕДНЯЯ КАМЕРА — основной таймлайн</h3><video id="forward" muted preload="metadata"></video></div><div class="box"><h3>НИЖНЯЯ КАМЕРА — синхронизация по timestamp_start</h3><video id="bottom" muted preload="metadata"></video></div></div>
<div class="controls"><button id="play">▶ / ⏸ Пробел</button><button id="m1">−1 кадр</button><button id="p1">+1 кадр</button><button id="m10">−10</button><button id="p10">+10</button><button id="sync">Синхр. нижнюю</button></div>
<div class="timebar"><span id="frame">кадр 0</span><input id="seek" type="range" min="0" max="1" step="1"><span id="ftime">0.0000 с</span></div><div class="small">Разметка сохраняется по реальному <code>timestamp_start</code>. Для копий кадры пересчитываются по ближайшему времени.</div>
<div class="section" style="margin-top:10px"><h2>Текущий сегмент</h2><div class="body"><div class="current" id="label">Выбери одну подпрограмму и одно или несколько состояний.</div><div id="editBadge" class="editBadge"></div><div class="controls"><button id="start">[ начало</button><button id="end">] конец</button><button id="save">Enter сохранить</button><button id="cancelEdit" style="display:none">Отменить редактирование</button><button id="clear">Сбросить</button></div><div class="small" id="marks"></div><div id="msg"></div></div></div>
<div class="section"><h2>Сохранённые сегменты</h2><div class="body"><div class="small" style="margin-bottom:6px">Нажми на строку — перейти к её началу. «Ред.» — изменить существующий сегмент. В group-режиме изменение применяется ко всей группе с пересчётом времени.</div><table><thead><tr><th>#</th><th>Время</th><th>Подпрограмма</th><th>Состояния</th><th>Действия</th></tr></thead><tbody id="segments"></tbody></table></div></div>
</div><div class="right"><div class="section"><h2>Подпрограмма — только одна из 18</h2><div class="body programs" id="programs"></div></div><div class="section"><h2>Состояния — можно выбрать несколько</h2><div class="body states" id="states"></div></div><div class="section"><h2>Клавиши</h2><div class="body small">Space — play/pause<br>←/→ — ±1 кадр<br>Shift+←/→ — ±10<br>[ — начало<br>] — конец<br>Enter — сохранить / применить редактирование<br>Esc — отменить редактирование</div></div></div></div>
<script>
let D,A=[],P=null,S=new Set(),st=null,en=null,editIndex=null;let requestedFrame=null,queuedFrame=null,activeSeekFrame=null,seekWatchdog=null,bottomSeekTimer=null;const f=document.getElementById('forward'),b=document.getElementById('bottom'),seek=document.getElementById('seek');
function near(arr,t){let l=0,h=arr.length;while(l<h){let m=(l+h)>>1;if(arr[m]<t)l=m+1;else h=m}if(l<=0)return 0;if(l>=arr.length)return arr.length-1;return Math.abs(arr[l]-t)<Math.abs(arr[l-1]-t)?l:l-1}
function fi(){if(!D||!D.forward_ts.length||!Number.isFinite(f.currentTime))return 0;return Math.max(0,Math.min(D.forward_ts.length-1,Math.round(f.currentTime*D.video_fps)))}function navFi(){return requestedFrame==null?fi():requestedFrame}function ft(i){return D.forward_ts[Math.max(0,Math.min(D.forward_ts.length-1,i))]}
function safeSeek(v,time){if(!Number.isFinite(time)||v.readyState<1)return false;try{let d=Number.isFinite(v.duration)&&v.duration>0?v.duration:time;v.currentTime=Math.max(0,Math.min(time,Math.max(0,d-0.0001)));return true}catch(e){return false}}
function showPosition(i){if(!D||!D.forward_ts.length)return;i=Math.max(0,Math.min(D.forward_ts.length-1,i));seek.value=i;document.getElementById('frame').textContent='кадр '+i;document.getElementById('ftime').textContent=ft(i).toFixed(4)+' с'}
function sync(i,immediate=false){if(!D||!D.bottom_ts.length)return;clearTimeout(bottomSeekTimer);let run=()=>{if(b.readyState<1)return;let j=near(D.bottom_ts,ft(i));safeSeek(b,j/D.video_fps)};if(immediate)run();else bottomSeekTimer=setTimeout(run,70)}
function pos(){if(requestedFrame==null&&!f.seeking)showPosition(fi())}
function finishForwardSeek(){if(activeSeekFrame==null)return;clearTimeout(seekWatchdog);let completed=activeSeekFrame;activeSeekFrame=null;if(queuedFrame!=null&&queuedFrame!==completed){pumpForwardSeek();return}queuedFrame=null;requestedFrame=null;let i=fi();showPosition(i);sync(i)}
function pumpForwardSeek(){if(activeSeekFrame!=null||queuedFrame==null||f.readyState<1)return;let i=queuedFrame;queuedFrame=null;activeSeekFrame=i;let target=i/D.video_fps;if(Math.abs(f.currentTime-target)<0.0005&&!f.seeking){finishForwardSeek();return}if(!safeSeek(f,target)){activeSeekFrame=null;queuedFrame=i;return}seekWatchdog=setTimeout(finishForwardSeek,2000)}
function go(i){if(!D||!D.forward_ts.length)return;i=Math.max(0,Math.min(D.forward_ts.length-1,Math.round(i)));requestedFrame=i;queuedFrame=i;showPosition(i);pumpForwardSeek()}
function stepFrames(delta){go(navFi()+delta)}
function programs(){let r=document.getElementById('programs');r.innerHTML='';D.ontology.forEach(x=>{let q=document.createElement('button');q.textContent=x.id+'. '+x.name_ru;if(P&&P.id===x.id)q.classList.add('active');q.onclick=()=>selProg(x.id);r.appendChild(q)})}
function selProg(id){P=D.ontology.find(x=>x.id===id);S.clear();programs();states();label()}
function states(){let r=document.getElementById('states');r.innerHTML='';if(!P){r.textContent='Сначала выбери подпрограмму';return}P.states.forEach(x=>{let q=document.createElement('button');q.textContent=x.ru;if(S.has(x.en))q.classList.add('stateActive');q.onclick=()=>{S.has(x.en)?S.delete(x.en):S.add(x.en);states();label()};r.appendChild(q)})}
function stateRu(id,en){let p=D.ontology.find(x=>x.id===id);if(!p)return en;let s=p.states.find(x=>x.en===en);return s?s.ru:en}
function progRu(id){let p=D.ontology.find(x=>x.id===id);return p?p.name_ru:String(id)}
function label(){if(!P){document.getElementById('label').textContent='Выбери одну подпрограмму и одно или несколько состояний.';return}let rr=P.states.filter(x=>S.has(x.en)).map(x=>x.ru);document.getElementById('label').innerHTML='<b>'+P.id+'. '+P.name_ru+'</b><br>Состояния: <b>'+(rr.length?rr.join(', '):'не выбраны')+'</b>'}
function marks(){let x=v=>v==null?'—':'кадр '+v+', t='+ft(v).toFixed(4);document.getElementById('marks').textContent='Начало: '+x(st)+' | Конец: '+x(en)}
function message(t,bad=false){let e=document.getElementById('msg');e.textContent=t;e.className=bad?'bad':'good';setTimeout(()=>{if(e.textContent===t)e.textContent=''},3000)}
function updateEditUi(){let badge=document.getElementById('editBadge'),save=document.getElementById('save'),cancel=document.getElementById('cancelEdit');if(editIndex==null){badge.classList.remove('on');badge.textContent='';save.textContent='Enter сохранить';cancel.style.display='none'}else{let x=A[editIndex];badge.classList.add('on');badge.textContent='РЕДАКТИРОВАНИЕ: сегмент #'+(x?x.segment_id:editIndex+1)+'. Изменения заменят эту строку.';save.textContent='Enter сохранить изменения';cancel.style.display='inline-block'}}
function resetEditor(clearProgram=true){editIndex=null;st=null;en=null;if(clearProgram){P=null;S.clear();programs();states();label()}marks();updateEditUi();renderSeg()}
function editSeg(i){let x=A[i];if(!x)return;editIndex=i;P=D.ontology.find(p=>p.id===x.subprogram_id)||null;S=new Set(x.states||[]);st=near(D.forward_ts,Number(x.start_timestamp));en=near(D.forward_ts,Number(x.end_timestamp));programs();states();label();marks();updateEditUi();renderSeg();go(st);message('Сегмент #'+x.segment_id+' загружен для редактирования')}
async function saveAll(){let r=await fetch('/api/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({segments:A})});if(!r.ok)throw new Error(await r.text());return await r.json()}
async function saveSeg(){if(!P){message('Выбери подпрограмму',true);return}if(S.size===0){message('Выбери хотя бы одно состояние',true);return}if(st==null||en==null){message('Поставь начало и конец',true);return}let a=Math.min(st,en),z=Math.max(st,en),ta=ft(a),tz=ft(z);let ss=P.states.filter(x=>S.has(x.en)).map(x=>x.en);let item={segment_id:editIndex==null?A.length+1:A[editIndex].segment_id,subprogram_id:P.id,subprogram_name:P.name_en,states:ss,start_timestamp:ta,end_timestamp:tz};let wasEdit=editIndex!=null;if(wasEdit)A[editIndex]=item;else A.push(item);A.sort((x,y)=>x.start_timestamp-y.start_timestamp);A.forEach((x,i)=>x.segment_id=i+1);let res=await saveAll();let count=res.updated_flights.length;editIndex=null;renderSeg();updateEditUi();if(wasEdit){st=a;en=z;marks();message('Изменения сохранены: '+count+' набор(ов)')}else{st=Math.min(z+1,D.forward_ts.length-1);en=null;marks();message('Сегмент сохранён: '+count+' набор(ов)')}}
async function deleteSeg(i){let x=A[i];if(!x)return;if(!confirm('Удалить сегмент #'+x.segment_id+'?'))return;A.splice(i,1);A.forEach((y,j)=>y.segment_id=j+1);editIndex=null;await saveAll();resetEditor(false);message('Сегмент удалён')}
function renderSeg(){let t=document.getElementById('segments');t.innerHTML='';A.forEach((x,i)=>{let tr=document.createElement('tr');if(editIndex===i)tr.classList.add('editing');let sr=(x.states||[]).map(s=>stateRu(x.subprogram_id,s)).join(', ');tr.innerHTML='<td>'+x.segment_id+'</td><td>'+Number(x.start_timestamp).toFixed(3)+'–'+Number(x.end_timestamp).toFixed(3)+'</td><td>'+x.subprogram_id+'. '+progRu(x.subprogram_id)+'</td><td>'+sr+'</td><td class="rowActions"><button class="editBtn">✎ Ред.</button><button class="deleteBtn">×</button></td>';tr.onclick=e=>{if(!e.target.closest('button'))go(near(D.forward_ts,x.start_timestamp))};let bs=tr.querySelectorAll('button');bs[0].onclick=e=>{e.stopPropagation();editSeg(i)};bs[1].onclick=async e=>{e.stopPropagation();await deleteSeg(i)};t.appendChild(tr)})}

async function init(){D=await(await fetch('/api/data')).json();document.getElementById('flight').textContent='Показывается: '+D.flight_name;let mode=D.mode==='single'?'ОДИНОЧНАЯ разметка: изменения сохраняются только в '+D.flight_name:'ГРУППОВАЯ разметка: '+D.flight_group.join(', ')+'. Времена автоматически пересчитываются для speed-replay.';document.getElementById('group').textContent=mode;let token=Date.now();f.src='/media/forward?v='+token;b.src='/media/bottom?v='+token;seek.max=D.forward_ts.length-1;document.getElementById('status').textContent=D.forward_ts.length+' передних · '+D.bottom_ts.length+' нижних кадров';A=D.annotations||[];programs();states();label();marks();renderSeg();updateEditUi();f.ontimeupdate=pos;f.onseeked=finishForwardSeek;f.onloadedmetadata=pumpForwardSeek;seek.oninput=()=>go(Number(seek.value));document.getElementById('play').onclick=()=>{if(f.paused){f.play();b.play().catch(()=>{})}else{f.pause();b.pause();sync(navFi())}};document.getElementById('m1').onclick=()=>stepFrames(-1);document.getElementById('p1').onclick=()=>stepFrames(1);document.getElementById('m10').onclick=()=>stepFrames(-10);document.getElementById('p10').onclick=()=>stepFrames(10);document.getElementById('sync').onclick=()=>sync(navFi(),true);document.getElementById('start').onclick=()=>{st=navFi();marks()};document.getElementById('end').onclick=()=>{en=navFi();if(st!=null&&en<st)[st,en]=[en,st];marks()};document.getElementById('save').onclick=saveSeg;document.getElementById('cancelEdit').onclick=()=>{resetEditor(false);message('Редактирование отменено')};document.getElementById('clear').onclick=()=>{st=en=null;marks()};window.onkeydown=e=>{let field=e.target.closest('input,textarea,select,[contenteditable="true"]');if(field&&field!==seek)return;if(e.code==='Space'){e.preventDefault();document.getElementById('play').click()}else if(e.key==='ArrowLeft'){e.preventDefault();stepFrames(e.shiftKey?-10:-1)}else if(e.key==='ArrowRight'){e.preventDefault();stepFrames(e.shiftKey?10:1)}else if(e.key==='['){st=navFi();marks()}else if(e.key===']'){en=navFi();if(st!=null&&en<st)[st,en]=[en,st];marks()}else if(e.key==='Enter'){saveSeg()}else if(e.key==='Escape'&&editIndex!=null){resetEditor(false);message('Редактирование отменено')}};go(0)}init().catch(e=>document.getElementById('status').textContent=e)
</script></body></html>'''


def read_ts(path):
    out=[]
    if not path.exists(): return out
    with open(path,newline='') as f:
        for r in csv.DictReader(f): out.append(float(r['timestamp_start']))
    return out

def nearest_idx(values,t):
    if not values: return None
    i=bisect_left(values,t)
    if i<=0: return 0
    if i>=len(values): return len(values)-1
    return i if abs(values[i]-t)<abs(t-values[i-1]) else i-1

def base_name(name):
    m = re.fullmatch(r'(flight-\d{8}-\d{6})(?:-(\d+))?', name)
    if not m:
        raise ValueError(
            'Flight must be flight-YYYYMMDD-HHMMSS '
            'or flight-YYYYMMDD-HHMMSS-N'
        )
    return m.group(1)


def flight_group(flight):
    """All complete numeric copies sharing one timestamp base."""
    base = base_name(flight.name)
    pat = re.compile(rf'^{re.escape(base)}(?:-(\d+))?$')
    found = []

    for p in flight.parent.iterdir():
        if not p.is_dir():
            continue

        m = pat.fullmatch(p.name)
        if not m:
            continue

        if (
            not (p / 'forward/timestamps.csv').exists()
            or not (p / 'bottom/timestamps.csv').exists()
            or not (p / 'forward/video.mp4').exists()
            or not (p / 'bottom/video.mp4').exists()
        ):
            continue

        idx = 0 if m.group(1) is None else int(m.group(1))
        found.append((idx, p))

    found.sort(key=lambda x: x[0])
    return [p for _, p in found]


def resolve_flight(value):
    """Absolute, cwd-relative, project-relative, or bare flight name."""
    arg = Path(value).expanduser()
    candidates = []

    if arg.is_absolute():
        candidates.append(arg)
    else:
        candidates.extend([
            Path.cwd() / arg,
            Path(CONCEPT_VLA_DIR) / arg,
            Path(CONCEPT_VLA_DIR) / 'datasets' / 'raw' / arg,
        ])

    seen = set()

    for candidate in candidates:
        candidate = candidate.resolve()

        if candidate in seen:
            continue

        seen.add(candidate)

        if candidate.is_dir():
            return candidate

    expected = (
        arg.resolve()
        if arg.is_absolute()
        else (
            Path(CONCEPT_VLA_DIR)
            / 'datasets'
            / 'raw'
            / arg
        ).resolve()
    )
    raise SystemExit(f'Flight directory not found: {expected}')


def canonicalize_segment(s):
    pid = int(s['subprogram_id'])
    p = next(x for x in ONTOLOGY if x['id'] == pid)

    if isinstance(s.get('states'), list):
        states = list(s['states'])
    else:
        old = s.get('state')
        states = []

        if old:
            hit = next(
                (
                    x for x in p['states']
                    if x['en'] == old or x['ru'] == old
                ),
                None,
            )
            states = [hit['en'] if hit else old]

    return {
        'segment_id': int(s['segment_id']),
        'subprogram_id': pid,
        'subprogram_name': p['name_en'],
        'states': states,
        'start_timestamp': float(s['start_timestamp']),
        'end_timestamp': float(s['end_timestamp']),
    }


def local_speed_manifest(flight):
    path = flight / 'augmentation' / 'speed.json'

    if not path.exists():
        return 1.0, None

    try:
        data = json.loads(path.read_text())
        scale = float(data.get('speed_scale', 1.0))

        if not (scale > 0.0):
            raise ValueError('speed_scale must be > 0')

        return scale, data.get('source_flight')

    except Exception as exc:
        print(
            f'[ANNOTATOR WARNING] cannot read {path}: {exc}; '
            'assuming speed_scale=1',
            flush=True,
        )
        return 1.0, None


def cumulative_speed_factor(flight, memo=None, visiting=None):
    """Speed factor relative to the original recording.

    Chained replay_speed datasets multiply their local factors.
    """
    flight = Path(flight).resolve()

    if memo is None:
        memo = {}

    if visiting is None:
        visiting = set()

    if flight in memo:
        return memo[flight]

    if flight in visiting:
        local, _ = local_speed_manifest(flight)
        return local

    visiting.add(flight)
    local, source_name = local_speed_manifest(flight)
    factor = local

    if source_name:
        source = flight.parent / source_name

        if source.exists() and source.resolve() != flight:
            factor = (
                cumulative_speed_factor(
                    source,
                    memo=memo,
                    visiting=visiting,
                )
                * local
            )

    visiting.remove(flight)
    memo[flight] = factor
    return factor


def map_segments_between_flights(segments, source_flight, target_flight):
    """Preserve route progress while mapping between speed clocks."""
    src_factor = cumulative_speed_factor(source_flight)
    dst_factor = cumulative_speed_factor(target_flight)
    ratio = src_factor / dst_factor
    mapped = []

    for segment in segments:
        c = canonicalize_segment(segment)
        c['start_timestamp'] = float(c['start_timestamp']) * ratio
        c['end_timestamp'] = float(c['end_timestamp']) * ratio
        mapped.append(c)

    return mapped, src_factor, dst_factor


def load_group_ann(group, reference_flight):
    """Load any existing group annotation into the selected flight's clock."""
    for annotated_flight in group:
        jp = annotated_flight / 'annotations' / 'subprograms.json'

        if not jp.exists():
            continue

        try:
            raw = json.loads(jp.read_text()).get('segments', [])
            canonical = [canonicalize_segment(s) for s in raw]

            mapped, src_factor, ref_factor = map_segments_between_flights(
                canonical,
                annotated_flight,
                reference_flight,
            )

            if annotated_flight != reference_flight:
                print(
                    '[ANNOTATOR] loaded annotations from '
                    f'{annotated_flight.name}: '
                    f'{src_factor:.6g}x -> {ref_factor:.6g}x',
                    flush=True,
                )

            return mapped

        except Exception as exc:
            print(
                f'[ANNOTATOR WARNING] cannot load {jp}: {exc}',
                flush=True,
            )

    return []


def save_one(
    flight,
    segments,
    *,
    mode='single',
    reference_flight=None,
    reference_speed_factor=1.0,
    target_speed_factor=1.0,
):
    fts = read_ts(flight / 'forward' / 'timestamps.csv')
    bts = read_ts(flight / 'bottom' / 'timestamps.csv')
    d = flight / 'annotations'
    d.mkdir(parents=True, exist_ok=True)
    materialized = []

    for s in segments:
        c = canonicalize_segment(s)
        st = float(c['start_timestamp'])
        et = float(c['end_timestamp'])

        c.update({
            'start_timestamp': round(st, 4),
            'end_timestamp': round(et, 4),
            'start_forward_frame': nearest_idx(fts, st),
            'end_forward_frame': nearest_idx(fts, et),
            'start_bottom_frame': nearest_idx(bts, st),
            'end_bottom_frame': nearest_idx(bts, et),
        })
        materialized.append(c)

    time_basis = (
        'route-progress synchronized timestamp_start; timestamps mapped '
        'between replay-speed clocks; frame indices nearest for this flight'
        if mode == 'group'
        else
        'timestamp_start of this flight; frame indices nearest for this flight'
    )

    payload = {
        'flight': flight.name,
        'ontology_version': 2,
        'language': 'en',
        'annotation_mode': mode,
        'time_basis': time_basis,
        'reference_flight': (
            reference_flight.name
            if reference_flight is not None
            else flight.name
        ),
        'reference_speed_factor': float(reference_speed_factor),
        'flight_speed_factor': float(target_speed_factor),
        'segments': materialized,
    }

    jp = d / 'subprograms.json'
    tmp = jp.with_suffix('.json.tmp')

    with open(tmp, 'w') as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)

    os.replace(tmp, jp)

    cp = d / 'subprograms.csv'
    fields = [
        'segment_id',
        'subprogram_id',
        'subprogram_name',
        'states',
        'start_timestamp',
        'end_timestamp',
        'start_forward_frame',
        'end_forward_frame',
        'start_bottom_frame',
        'end_bottom_frame',
    ]

    with open(cp, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()

        for s in materialized:
            r = dict(s)
            r['states'] = ';'.join(s['states'])
            r['start_timestamp'] = f"{float(s['start_timestamp']):.4f}"
            r['end_timestamp'] = f"{float(s['end_timestamp']):.4f}"
            w.writerow({k: r.get(k) for k in fields})


def save_group(group, segments, reference_flight, mode):
    if mode == 'single':
        factor = cumulative_speed_factor(reference_flight)
        save_one(
            reference_flight,
            segments,
            mode='single',
            reference_flight=reference_flight,
            reference_speed_factor=factor,
            target_speed_factor=factor,
        )
        return

    ref_factor = cumulative_speed_factor(reference_flight)

    for flight in group:
        mapped, _, dst_factor = map_segments_between_flights(
            segments,
            reference_flight,
            flight,
        )

        save_one(
            flight,
            mapped,
            mode='group',
            reference_flight=reference_flight,
            reference_speed_factor=ref_factor,
            target_speed_factor=dst_factor,
        )


class Handler(BaseHTTPRequestHandler):
    def log_message(self,*a): pass
    def send_bytes(self,data,ctype,status=200):
        self.send_response(status); self.send_header('Content-Type',ctype); self.send_header('Content-Length',str(len(data))); self.send_header('Cache-Control','no-store'); self.end_headers(); self.wfile.write(data)
    def send_json(self,obj,status=200): self.send_bytes(json.dumps(obj,ensure_ascii=False).encode(),'application/json; charset=utf-8',status)
    def do_GET(self):
        p=urllib.parse.urlparse(self.path).path
        if p=='/': return self.send_bytes(HTML.encode(),'text/html; charset=utf-8')
        if p=='/api/data':
            s=self.server; return self.send_json({'flight_name':s.flight.name,'flight_group':[p.name for p in s.group],'mode':s.mode,'speed_factors':{p.name:cumulative_speed_factor(p) for p in s.group},'ontology':ONTOLOGY_JSON,'forward_ts':s.forward_ts,'bottom_ts':s.bottom_ts,'video_fps':VIDEO_FPS,'annotations':load_group_ann(s.group,s.flight)})
        if p=='/media/forward': return self.serve_file(self.server.forward_video)
        if p=='/media/bottom': return self.serve_file(self.server.bottom_video)
        self.send_bytes(b'Not found','text/plain',404)
    def serve_file(self,p):
        if not p.exists(): return self.send_bytes(b'Missing video','text/plain',404)
        size=p.stat().st_size; ctype=mimetypes.guess_type(str(p))[0] or 'application/octet-stream'; rh=self.headers.get('Range')
        if rh and rh.startswith('bytes='):
            spec=rh[6:]; a,z=spec.split('-',1); start=int(a) if a else 0; end=int(z) if z else size-1; end=min(end,size-1)
            if start>=size or start>end:
                self.send_response(416); self.send_header('Content-Range',f'bytes */{size}'); self.end_headers(); return
            n=end-start+1; self.send_response(206); self.send_header('Content-Type',ctype); self.send_header('Accept-Ranges','bytes'); self.send_header('Cache-Control','private, max-age=3600'); self.send_header('Content-Range',f'bytes {start}-{end}/{size}'); self.send_header('Content-Length',str(n)); self.end_headers()
            try:
                with open(p,'rb') as f:
                    f.seek(start); left=n
                    while left:
                        chunk=f.read(min(1024*1024,left))
                        if not chunk: break
                        try: self.wfile.write(chunk)
                        except (BrokenPipeError,ConnectionResetError): break
                        left-=len(chunk)
            except (BrokenPipeError,ConnectionResetError): pass
            return
        self.send_response(200); self.send_header('Content-Type',ctype); self.send_header('Content-Length',str(size)); self.send_header('Accept-Ranges','bytes'); self.send_header('Cache-Control','private, max-age=3600'); self.end_headers()
        try:
            with open(p,'rb') as f:
                while True:
                    c=f.read(1024*1024)
                    if not c: break
                    try: self.wfile.write(c)
                    except (BrokenPipeError,ConnectionResetError): break
        except (BrokenPipeError,ConnectionResetError): pass
    def do_POST(self):
        if urllib.parse.urlparse(self.path).path!='/api/save': return self.send_bytes(b'Not found','text/plain',404)
        try:
            n=int(self.headers.get('Content-Length','0')); payload=json.loads(self.rfile.read(n).decode()); save_group(self.server.group,payload.get('segments',[]),self.server.flight,self.server.mode); self.send_json({'ok':True,'updated_flights':[p.name for p in self.server.group]})
        except Exception as e: self.send_json({'ok':False,'error':str(e)},400)

def main():
    ap = argparse.ArgumentParser(
        description=(
            'Concept-VLA annotator: group or single dataset mode. '
            'Group mode maps timestamps for replay_speed datasets.'
        )
    )
    ap.add_argument(
        'flight',
        help=(
            'Absolute path, datasets/raw/... path, project-relative path, '
            'or bare flight-YYYYMMDD-HHMMSS[-N] name'
        ),
    )
    ap.add_argument(
        '--mode',
        choices=('group', 'single'),
        default='group',
        help=(
            'group = all numeric copies sharing the same timestamp base; '
            'single = only the selected dataset (default: group)'
        ),
    )
    ap.add_argument('--host', default=HOST)
    ap.add_argument('--port', type=int, default=PORT)
    ap.add_argument('--no-browser', action='store_true')
    a = ap.parse_args()

    flight = resolve_flight(a.flight)
    base_name(flight.name)

    req = [
        flight / 'forward' / 'video.mp4',
        flight / 'bottom' / 'video.mp4',
        flight / 'forward' / 'timestamps.csv',
        flight / 'bottom' / 'timestamps.csv',
    ]

    for p in req:
        if not p.exists():
            raise SystemExit(f'Missing required file: {p}')

    group = [flight] if a.mode == 'single' else flight_group(flight)

    if not group:
        raise SystemExit('No complete flights found for the selected group')

    s = ThreadingHTTPServer((a.host, a.port), Handler)
    s.flight = flight
    s.group = group
    s.mode = a.mode
    s.forward_video = req[0]
    s.bottom_video = req[1]
    s.forward_ts = read_ts(req[2])
    s.bottom_ts = read_ts(req[3])

    url = f'http://{a.host}:{a.port}/'
    print('[ANNOTATOR] showing:', flight)
    print('[ANNOTATOR] mode:', a.mode)

    if a.mode == 'group':
        print('[ANNOTATOR] group:')

        for p in group:
            print(
                f'  - {p.name} '
                f'(speed factor {cumulative_speed_factor(p):.6g}x)'
            )

        print(
            '[ANNOTATOR] annotations will be mapped by route progress '
            'and written to every flight in the group'
        )
    else:
        print(
            '[ANNOTATOR] annotations will be written ONLY to:',
            flight.name,
        )

    print('[ANNOTATOR] open:', url)
    print('[ANNOTATOR] Ctrl+C to stop')

    if not a.no_browser:
        threading.Timer(.4, lambda: webbrowser.open(url)).start()

    try:
        s.serve_forever()
    except KeyboardInterrupt:
        print('\n[ANNOTATOR] stopped')
    finally:
        s.server_close()


if __name__=='__main__': main()
