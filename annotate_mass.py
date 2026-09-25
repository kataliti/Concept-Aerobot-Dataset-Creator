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

import hashlib
from datetime import datetime

DEFAULT_SAFE_ROOT = f'{CONCEPT_VLA_DIR}/datasets/safe'
DEFAULT_RAW_ROOT = f'{CONCEPT_VLA_DIR}/datasets/raw'
CONFIG_PATH = Path.home() / '.config' / 'concept-vla' / 'annotate_mass.json'
BASE_RE = re.compile(r'^flight-\d{8}-\d{6}$')
COPY_RE = re.compile(r'^(flight-\d{8}-\d{6})-(\d+)$')


def now_iso():
    return datetime.now().astimezone().isoformat(timespec='seconds')


def is_relative_to(path, root):
    try:
        Path(path).resolve().relative_to(Path(root).resolve())
        return True
    except Exception:
        return False


def read_ts(path):
    out = []
    if not path.exists():
        return out
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            try:
                out.append(float(row['timestamp_start']))
            except (KeyError, TypeError, ValueError):
                continue
    return out


def nearest_idx(values, t):
    if not values:
        return None
    i = bisect_left(values, t)
    if i <= 0:
        return 0
    if i >= len(values):
        return len(values) - 1
    return i if abs(values[i] - t) < abs(t - values[i - 1]) else i - 1


def canonicalize_segment(s):
    pid = int(s['subprogram_id'])
    prog = next((x for x in ONTOLOGY if x['id'] == pid), None)
    if prog is None:
        raise ValueError(f'Unknown subprogram_id: {pid}')

    if isinstance(s.get('states'), list):
        states = list(s['states'])
    else:
        old = s.get('state')
        states = []
        if old:
            hit = next((x for x in prog['states'] if x['en'] == old or x['ru'] == old), None)
            states = [hit['en'] if hit else old]

    allowed = {x['en'] for x in prog['states']}
    states = [str(x) for x in states if str(x) in allowed]

    return {
        'segment_id': int(s.get('segment_id', 0) or 0),
        'subprogram_id': pid,
        'subprogram_name': prog['name_en'],
        'states': states,
        'start_timestamp': float(s['start_timestamp']),
        'end_timestamp': float(s['end_timestamp']),
    }


def normalize_segments(segments):
    out = []
    for s in segments:
        c = canonicalize_segment(s)
        if c['end_timestamp'] < c['start_timestamp']:
            c['start_timestamp'], c['end_timestamp'] = c['end_timestamp'], c['start_timestamp']
        out.append(c)
    out.sort(key=lambda x: (x['start_timestamp'], x['end_timestamp']))
    for i, x in enumerate(out, 1):
        x['segment_id'] = i
    return out


def annotation_digest(segments):
    compact = []
    for s in normalize_segments(segments):
        compact.append({
            'subprogram_id': s['subprogram_id'],
            'subprogram_name': s['subprogram_name'],
            'states': s['states'],
            'start_timestamp': round(float(s['start_timestamp']), 6),
            'end_timestamp': round(float(s['end_timestamp']), 6),
        })
    raw = json.dumps(compact, ensure_ascii=False, sort_keys=True, separators=(',', ':')).encode('utf-8')
    return hashlib.sha256(raw).hexdigest()


def annotation_path(flight):
    return Path(flight) / 'annotations' / 'subprograms.json'


def load_annotation_payload(flight):
    jp = annotation_path(flight)
    if not jp.exists():
        return None
    try:
        return json.loads(jp.read_text(encoding='utf-8'))
    except Exception as exc:
        return {'_error': str(exc), 'segments': []}


def load_segments(flight):
    payload = load_annotation_payload(flight)
    if not payload or payload.get('_error'):
        return []
    try:
        return normalize_segments(payload.get('segments', []))
    except Exception as exc:
        print(f'[ANNOTATOR WARNING] cannot parse {annotation_path(flight)}: {exc}', flush=True)
        return []


def local_speed_manifest(flight):
    path = Path(flight) / 'augmentation' / 'speed.json'
    if not path.exists():
        return 1.0, None
    try:
        data = json.loads(path.read_text(encoding='utf-8'))
        scale = float(data.get('speed_scale', 1.0))
        if not scale > 0:
            raise ValueError('speed_scale must be > 0')
        return scale, data.get('source_flight')
    except Exception as exc:
        print(f'[ANNOTATOR WARNING] cannot read {path}: {exc}; assuming 1x', flush=True)
        return 1.0, None


def resolve_parent_path(safe_root, value):
    safe_root = Path(safe_root).expanduser().resolve()
    raw = Path(str(value).strip()).expanduser()
    p = raw.resolve() if raw.is_absolute() else (safe_root / raw).resolve()
    if not is_relative_to(p, safe_root):
        raise ValueError(f'Родительская папка должна находиться внутри safe root: {safe_root}')
    return p


def make_source_index(safe_root, parents):
    idx = {}
    for parent_text in parents:
        try:
            parent = resolve_parent_path(safe_root, parent_text)
        except Exception:
            continue
        if not parent.is_dir():
            continue
        for p in parent.iterdir():
            if p.is_dir() and BASE_RE.fullmatch(p.name):
                idx.setdefault(p.name, p.resolve())
    return idx


def resolve_speed_source(source_name, flight, raw_root, source_index):
    if not source_name:
        return None
    candidates = [Path(flight).parent / source_name, Path(raw_root) / source_name]
    hit = source_index.get(source_name)
    if hit is not None:
        candidates.append(hit)
    for p in candidates:
        try:
            if p.is_dir():
                return p.resolve()
        except Exception:
            pass
    return None


def cumulative_speed_factor(flight, raw_root, source_index, memo=None, visiting=None):
    flight = Path(flight).resolve()
    if memo is None:
        memo = {}
    if visiting is None:
        visiting = set()
    if flight in memo:
        return memo[flight]
    local, source_name = local_speed_manifest(flight)
    if flight in visiting:
        return local
    visiting.add(flight)
    factor = local
    source = resolve_speed_source(source_name, flight, raw_root, source_index)
    if source is not None and source != flight:
        factor = cumulative_speed_factor(source, raw_root, source_index, memo, visiting) * local
    visiting.remove(flight)
    memo[flight] = factor
    return factor


def map_segments_between_flights(segments, source_flight, target_flight, raw_root, source_index):
    src_factor = cumulative_speed_factor(source_flight, raw_root, source_index)
    dst_factor = cumulative_speed_factor(target_flight, raw_root, source_index)
    ratio = src_factor / dst_factor
    mapped = []
    for s in normalize_segments(segments):
        c = dict(s)
        c['start_timestamp'] = float(c['start_timestamp']) * ratio
        c['end_timestamp'] = float(c['end_timestamp']) * ratio
        mapped.append(c)
    return mapped, src_factor, dst_factor


def save_one(flight, segments, *, annotation_mode, reference_flight,
             reference_speed_factor, target_speed_factor, source_annotation_digest):
    flight = Path(flight)
    fts = read_ts(flight / 'forward' / 'timestamps.csv')
    bts = read_ts(flight / 'bottom' / 'timestamps.csv')
    if not fts:
        raise RuntimeError(f'Missing/empty forward timestamps: {flight}')
    if not bts:
        raise RuntimeError(f'Missing/empty bottom timestamps: {flight}')

    d = flight / 'annotations'
    d.mkdir(parents=True, exist_ok=True)
    materialized = []
    for s in normalize_segments(segments):
        c = dict(s)
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
        'route-progress synchronized timestamp_start; timestamps mapped between replay-speed clocks; '
        'frame indices nearest for this flight'
        if annotation_mode == 'propagated_from_source'
        else 'timestamp_start of this source flight; frame indices nearest for this flight'
    )
    payload = {
        'flight': flight.name,
        'ontology_version': 2,
        'language': 'en',
        'annotation_mode': annotation_mode,
        'time_basis': time_basis,
        'reference_flight': Path(reference_flight).name,
        'reference_speed_factor': float(reference_speed_factor),
        'flight_speed_factor': float(target_speed_factor),
        'source_annotation_digest': str(source_annotation_digest),
        'updated_at': now_iso(),
        'segments': materialized,
    }

    jp = d / 'subprograms.json'
    tmp = jp.with_suffix('.json.tmp')
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    os.replace(tmp, jp)

    cp = d / 'subprograms.csv'
    fields = ['segment_id', 'subprogram_id', 'subprogram_name', 'states',
              'start_timestamp', 'end_timestamp', 'start_forward_frame', 'end_forward_frame',
              'start_bottom_frame', 'end_bottom_frame']
    with open(cp, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for s in materialized:
            r = dict(s)
            r['states'] = ';'.join(s['states'])
            r['start_timestamp'] = f"{float(s['start_timestamp']):.4f}"
            r['end_timestamp'] = f"{float(s['end_timestamp']):.4f}"
            w.writerow({k: r.get(k) for k in fields})
    return payload


def save_source_annotation(source, segments, raw_root, source_index):
    source = Path(source).resolve()
    normalized = normalize_segments(segments)
    digest = annotation_digest(normalized)
    factor = cumulative_speed_factor(source, raw_root, source_index)
    return save_one(source, normalized,
                    annotation_mode='source_manual', reference_flight=source,
                    reference_speed_factor=factor, target_speed_factor=factor,
                    source_annotation_digest=digest)


def find_raw_copies(raw_root, base):
    raw_root = Path(raw_root)
    found = []
    if not raw_root.is_dir():
        return []
    pat = re.compile(rf'^{re.escape(base)}-(\d+)$')
    for p in raw_root.iterdir():
        if not p.is_dir():
            continue
        m = pat.fullmatch(p.name)
        if m:
            found.append((int(m.group(1)), p.resolve()))
    found.sort(key=lambda x: x[0])
    return [p for _, p in found]


def propagate_source(source, raw_root, source_index):
    source = Path(source).resolve()
    if not annotation_path(source).exists():
        raise RuntimeError('У исходного flight ещё нет сохранённой разметки')
    segments = load_segments(source)
    digest = annotation_digest(segments)
    copies = find_raw_copies(raw_root, source.name)
    src_factor = cumulative_speed_factor(source, raw_root, source_index)
    updated, errors = [], []
    for target in copies:
        try:
            mapped, _, dst_factor = map_segments_between_flights(
                segments, source, target, raw_root, source_index)
            save_one(target, mapped,
                     annotation_mode='propagated_from_source', reference_flight=source,
                     reference_speed_factor=src_factor, target_speed_factor=dst_factor,
                     source_annotation_digest=digest)
            updated.append(target.name)
        except Exception as exc:
            errors.append({'flight': target.name, 'error': str(exc)})
    return {'source': source.name, 'digest': digest, 'copies_found': len(copies),
            'updated': updated, 'errors': errors}


def flight_complete(flight):
    flight = Path(flight)
    req = [flight / 'forward/video.mp4', flight / 'bottom/video.mp4',
           flight / 'forward/timestamps.csv', flight / 'bottom/timestamps.csv']
    missing = [str(p.relative_to(flight)) for p in req if not p.exists()]
    return not missing, missing


def source_annotation_info(source, raw_root):
    source = Path(source)
    payload = load_annotation_payload(source)
    segments = load_segments(source)
    digest = annotation_digest(segments) if payload and not payload.get('_error') else None
    copies = find_raw_copies(raw_root, source.name)
    synced = stale = other = unannotated = invalid = 0
    rows = []
    for c in copies:
        complete, missing = flight_complete(c)
        if not complete:
            status = 'invalid'; invalid += 1
        else:
            cp = load_annotation_payload(c)
            if cp is None:
                status = 'unannotated'; unannotated += 1
            elif cp.get('_error'):
                status = 'invalid_annotation'; invalid += 1
            elif digest is not None and cp.get('reference_flight') == source.name and cp.get('source_annotation_digest') == digest:
                status = 'synced'; synced += 1
            elif cp.get('reference_flight') == source.name:
                status = 'stale'; stale += 1
            else:
                status = 'other_annotation'; other += 1
        rows.append({'name': c.name, 'status': status, 'missing': missing})
    return {
        'annotation_exists': payload is not None and not payload.get('_error'),
        'annotation_error': payload.get('_error') if payload else None,
        'segments': len(segments), 'digest': digest,
        'copies_total': len(copies), 'copies_synced': synced, 'copies_stale': stale,
        'copies_other_annotation': other, 'copies_unannotated': unannotated,
        'copies_invalid': invalid, 'copies': rows,
    }


def validate_source_path(source, safe_root):
    p = Path(source).expanduser().resolve()
    safe_root = Path(safe_root).expanduser().resolve()
    if not is_relative_to(p, safe_root):
        raise ValueError('Исходный flight должен находиться внутри safe root')
    if not BASE_RE.fullmatch(p.name):
        raise ValueError('Ожидается исходный flight-YYYYMMDD-HHMMSS без -N')
    if not p.is_dir():
        raise ValueError(f'Flight not found: {p}')
    return p


def load_config():
    cfg = {'safe_root': DEFAULT_SAFE_ROOT, 'raw_root': DEFAULT_RAW_ROOT, 'parents': []}
    try:
        if CONFIG_PATH.exists():
            saved = json.loads(CONFIG_PATH.read_text(encoding='utf-8'))
            if isinstance(saved, dict):
                for k in cfg:
                    if k in saved:
                        cfg[k] = saved[k]
    except Exception as exc:
        print(f'[ANNOTATOR WARNING] cannot load config: {exc}', flush=True)
    return cfg


def save_config(cfg):
    CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
    tmp = CONFIG_PATH.with_suffix('.json.tmp')
    tmp.write_text(json.dumps(cfg, ensure_ascii=False, indent=2), encoding='utf-8')
    os.replace(tmp, CONFIG_PATH)


def scan_config(cfg):
    safe_root = Path(cfg['safe_root']).expanduser().resolve()
    raw_root = Path(cfg['raw_root']).expanduser().resolve()
    parents = [str(x).strip() for x in cfg.get('parents', []) if str(x).strip()]
    groups = []
    for idx, parent_text in enumerate(parents, 1):
        row = {'index': idx, 'input': parent_text, 'parent': None, 'error': None, 'flights': []}
        try:
            parent = resolve_parent_path(safe_root, parent_text)
            row['parent'] = str(parent)
            if not parent.is_dir():
                raise RuntimeError(f'Папка не существует: {parent}')
            sources = sorted((p.resolve() for p in parent.iterdir()
                              if p.is_dir() and BASE_RE.fullmatch(p.name)), key=lambda p: p.name)
            for source in sources:
                complete, missing = flight_complete(source)
                row['flights'].append({
                    'name': source.name, 'path': str(source), 'complete': complete,
                    'missing': missing, 'annotation': source_annotation_info(source, raw_root),
                })
        except Exception as exc:
            row['error'] = str(exc)
        groups.append(row)
    return {
        'safe_root': str(safe_root), 'raw_root': str(raw_root), 'parents': parents, 'groups': groups,
        'totals': {
            'parents': len(groups),
            'sources': sum(len(g['flights']) for g in groups),
            'annotated_sources': sum(1 for g in groups for f in g['flights'] if f['annotation']['annotation_exists']),
            'raw_copies': sum(f['annotation']['copies_total'] for g in groups for f in g['flights']),
            'synced_copies': sum(f['annotation']['copies_synced'] for g in groups for f in g['flights']),
        },
    }

HTML = r'''<!doctype html>
<html lang="ru"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Concept-VLA · массовая разметка</title>
<style>
:root{color-scheme:dark}*{box-sizing:border-box}body{margin:0;background:#121416;color:#e8eaed;font-family:system-ui,-apple-system,Segoe UI,sans-serif}header{position:sticky;top:0;z-index:8;padding:11px 15px;border-bottom:1px solid #30363d;background:#15191d;display:flex;gap:12px;align-items:center;flex-wrap:wrap}button,input{font:inherit}button{background:#252b31;color:#eee;border:1px solid #4b5560;border-radius:7px;padding:7px 10px;cursor:pointer}button:hover{background:#303840}button.primary{background:#174d7a;border-color:#2b78b7}button.green{background:#214f35;border-color:#3b8b5b}button.danger{background:#5d2727;border-color:#914141}button:disabled{opacity:.45;cursor:not-allowed}input{background:#0d1117;color:#eee;border:1px solid #3d444d;border-radius:6px;padding:7px}.small{font-size:12px;color:#9ca3af}.good{color:#86efac}.bad{color:#fca5a5}.warnText{color:#fde68a}.hidden{display:none!important}.page{padding:12px}.panel{border:1px solid #30363d;border-radius:9px;background:#171b1f;margin-bottom:12px;overflow:hidden}.panel h2{font-size:14px;margin:0;padding:9px 11px;background:#20252a}.panel .body{padding:10px}.rootgrid{display:grid;grid-template-columns:1fr 1fr auto;gap:8px;align-items:end}.field label{display:block;font-size:12px;color:#aab2bd;margin-bottom:4px}.field input{width:100%}.parentCard{border:1px solid #374151;border-radius:8px;padding:9px;margin:8px 0;background:#151a20}.parentRow{display:grid;grid-template-columns:1fr auto;gap:8px}.summary{display:flex;gap:12px;flex-wrap:wrap;font-size:13px}table{width:100%;border-collapse:collapse;font-size:12px}th,td{padding:7px;border-bottom:1px solid #2c333b;text-align:left;vertical-align:top}th{background:#20252a}tr:hover{background:#181e24}.actions{white-space:nowrap}.actions button{padding:5px 7px;margin:2px}.badge{display:inline-block;padding:2px 6px;border-radius:999px;border:1px solid #4b5563;margin:1px;font-size:11px}.badge.good{background:#163725}.badge.warn{background:#4c3e19;color:#fde68a}.badge.bad{background:#4c2020;color:#fecaca}.badge.dim{background:#24282d;color:#aaa}.details{font-size:11px;color:#9ca3af;max-width:460px;white-space:normal}.controls{display:flex;gap:6px;flex-wrap:wrap;margin:8px 0}.progressOverlay{position:fixed;inset:0;background:#0009;z-index:50;display:flex;align-items:center;justify-content:center}.progressBox{width:min(620px,90vw);background:#171b1f;border:1px solid #4b5563;border-radius:10px;padding:18px}progress{width:100%;height:18px}.editorGrid{display:grid;grid-template-columns:1.45fr .8fr;height:calc(100vh - 55px)}.left,.right{padding:10px;overflow:auto}.right{border-left:1px solid #333}.videos{display:grid;grid-template-columns:1fr 1fr;gap:8px}.box{background:#000;border:1px solid #333}.box h3{font-size:12px;margin:0;padding:5px;background:#222}video{width:100%;display:block;max-height:42vh}.programs{display:grid;grid-template-columns:1fr 1fr;gap:5px}.programs button{text-align:left;font-size:12px}.states{display:flex;gap:5px;flex-wrap:wrap}.section{border:1px solid #333;border-radius:7px;margin-bottom:10px;overflow:hidden}.section h2{font-size:14px;margin:0;background:#222;padding:7px}.section .body{padding:8px}.timebar{display:grid;grid-template-columns:100px 1fr 100px;gap:8px;align-items:center}.timebar input{width:100%}.current{background:#1f1f1f;padding:8px;border-radius:5px}button.active{outline:2px solid #62aaff;background:#20334b}button.stateActive{outline:2px solid #67df8c;background:#1e4930}.editing{background:#2b2618}.editBadge{display:none;margin-top:7px;padding:6px 8px;border:1px solid #8c7431;background:#302916;border-radius:5px;color:#ffe09a}.editBadge.on{display:block}.rowActions{white-space:nowrap}.rowActions button{padding:4px 7px;margin-right:4px}@media(max-width:1000px){.rootgrid{grid-template-columns:1fr}.editorGrid{grid-template-columns:1fr;height:auto}.right{border-left:0}.videos{grid-template-columns:1fr}}
</style></head><body>
<header><b>Concept-VLA · массовая разметка</b><span id="topStatus" class="small">загрузка…</span><button id="backBtn" class="hidden">← К списку</button><button id="prevBtn" class="hidden">← Предыдущий</button><button id="nextBtn" class="hidden">Следующий →</button></header>
<div id="dashboard" class="page">
<div class="panel"><h2>1. Общие папки</h2><div class="body"><div class="rootgrid"><div class="field"><label>Исходники safe</label><input id="safeRoot"></div><div class="field"><label>Replay-копии raw</label><input id="rawRoot"></div><button id="scanBtn" class="primary">Сохранить и сканировать</button></div><div class="small" style="margin-top:7px">Родительские пути ниже можно писать относительно safe, например <code>1along</code>, или полным путём внутри safe.</div></div></div>
<div class="panel"><h2>2. Родительские папки</h2><div class="body"><div id="parents"></div><div class="controls"><button id="addParent">+ Добавить родительскую папку</button><button id="propagateAll" class="green">Авторазметить копии всех размеченных исходников</button></div></div></div>
<div class="panel"><h2>3. Исходные flight</h2><div class="body"><div id="totals" class="summary"></div><div id="groups"></div></div></div>
</div>

<div id="editor" class="hidden"><div class="editorGrid"><div class="left">
<div class="panel"><div class="body"><b id="flightTitle"></b><div id="copySummary" class="small" style="margin-top:5px"></div><div class="controls"><button id="saveSourceTop" class="primary">Сохранить исходник</button><button id="savePropTop" class="green">Сохранить + авторазметить все копии</button></div></div></div>
<div class="videos"><div class="box"><h3>ПЕРЕДНЯЯ КАМЕРА — основной таймлайн</h3><video id="forward" muted preload="metadata"></video></div><div class="box"><h3>НИЖНЯЯ КАМЕРА — синхронизация</h3><video id="bottom" muted preload="metadata"></video></div></div><div id="videoStatus" class="small" style="margin:6px 0">Видео не загружено</div>
<div class="controls"><button id="play">▶ / ⏸ Пробел</button><button id="m1">−1 кадр</button><button id="p1">+1 кадр</button><button id="m10">−10</button><button id="p10">+10</button><button id="sync">Синхр. нижнюю</button></div>
<div class="timebar"><span id="frame">кадр 0</span><input id="seek" type="range" min="0" max="1" step="1"><span id="ftime">0.0000 с</span></div><div class="small">Сохранённая разметка исходника всегда загружается при повторном открытии. Авторазметка копий пересчитывает timestamps по speed factor и ближайшие номера кадров отдельно для каждой копии.</div>
<div class="section" style="margin-top:10px"><h2>Текущий сегмент</h2><div class="body"><div class="current" id="label">Выбери одну подпрограмму и одно или несколько состояний.</div><div id="editBadge" class="editBadge"></div><div class="controls"><button id="start">[ начало</button><button id="end">] конец</button><button id="saveSeg">Enter добавить/изменить сегмент</button><button id="cancelEdit" style="display:none">Отменить редактирование</button><button id="clear">Сбросить</button></div><div class="small" id="marks"></div><div id="msg"></div></div></div>
<div class="section"><h2>Сохранённые сегменты</h2><div class="body"><div class="small" style="margin-bottom:6px">Изменения сегментов сначала локальны в редакторе. Для записи на диск нажмите «Сохранить исходник».</div><table><thead><tr><th>#</th><th>Время</th><th>Подпрограмма</th><th>Состояния</th><th>Действия</th></tr></thead><tbody id="segments"></tbody></table></div></div>
</div><div class="right"><div class="section"><h2>Подпрограмма — одна из 18</h2><div class="body programs" id="programs"></div></div><div class="section"><h2>Состояния — можно несколько</h2><div class="body states" id="states"></div></div><div class="section"><h2>Клавиши</h2><div class="body small">Space — play/pause<br>←/→ — ±1 кадр<br>Shift+←/→ — ±10<br>[ — начало<br>] — конец<br>Enter — добавить/применить сегмент<br>Esc — отменить редактирование сегмента</div></div></div></div></div></div>
<div id="progressOverlay" class="progressOverlay hidden"><div class="progressBox"><b id="progressTitle">Обработка…</b><div id="progressText" class="small" style="margin:8px 0"></div><progress id="progressBar" max="100" value="0"></progress></div></div>
<script>
let CFG=null,SCAN=null,flatFlights=[],currentFlight=null,currentFlatIndex=-1;let D=null,A=[],P=null,S=new Set(),st=null,en=null,editIndex=null,dirty=false;let requestedFrame=null,queuedFrame=null,activeSeekFrame=null,seekWatchdog=null,bottomSeekTimer=null;const f=document.getElementById('forward'),b=document.getElementById('bottom'),seek=document.getElementById('seek');
async function api(url,opt){let r=await fetch(url,opt);let t=await r.text();let j;try{j=JSON.parse(t)}catch(e){throw new Error(t||r.statusText)}if(!r.ok||j.ok===false)throw new Error(j.error||t);return j}function esc(s){return String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]))}function q(s){return encodeURIComponent(s)}function toast(t,bad=false){let e=document.getElementById('topStatus');e.textContent=t;e.className=bad?'small bad':'small good'}
function parentValues(){return [...document.querySelectorAll('.parentInput')].map(x=>x.value.trim()).filter(Boolean)}function renumberParents(){[...document.querySelectorAll('.parentCard')].forEach((d,i)=>d.querySelector('.small').textContent='Родительская папка '+(i+1))}
function renderParentCards(parents){let root=document.getElementById('parents');root.innerHTML='';(parents.length?parents:['']).forEach((v,i)=>{let d=document.createElement('div');d.className='parentCard';d.innerHTML=`<div class="small">Родительская папка ${i+1}</div><div class="parentRow"><input class="parentInput" value="${esc(v)}" placeholder="например 1along"><button class="danger">Удалить</button></div>`;d.querySelector('button').onclick=()=>{d.remove();renumberParents()};root.appendChild(d)})}
document.getElementById('addParent').onclick=()=>{let d=document.createElement('div');d.className='parentCard';d.innerHTML='<div class="small"></div><div class="parentRow"><input class="parentInput" placeholder="например 1along"><button class="danger">Удалить</button></div>';d.querySelector('button').onclick=()=>{d.remove();renumberParents()};document.getElementById('parents').appendChild(d);renumberParents()};
async function saveConfigAndScan(){let payload={safe_root:document.getElementById('safeRoot').value.trim(),raw_root:document.getElementById('rawRoot').value.trim(),parents:parentValues()};await api('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});await scan()}document.getElementById('scanBtn').onclick=()=>saveConfigAndScan().catch(e=>toast(e.message,true));
function annBadges(a){let src=a.annotation_exists?`<span class="badge good">исходник: ${a.segments} сегм.</span>`:'<span class="badge dim">исходник: не размечен</span>';let c=`<span class="badge dim">копий: ${a.copies_total}</span>`;let s=a.copies_synced?`<span class="badge good">синхр.: ${a.copies_synced}</span>`:'';let st=a.copies_stale?`<span class="badge warn">устарели: ${a.copies_stale}</span>`:'';let un=a.copies_unannotated?`<span class="badge dim">без разметки: ${a.copies_unannotated}</span>`:'';let oth=a.copies_other_annotation?`<span class="badge warn">другая: ${a.copies_other_annotation}</span>`:'';let inv=a.copies_invalid?`<span class="badge bad">ошибка: ${a.copies_invalid}</span>`:'';return src+' '+c+' '+s+' '+st+' '+un+' '+oth+' '+inv}function copyDetail(a){if(!a.copies.length)return 'raw-копии не найдены';return a.copies.map(x=>x.name+': '+x.status).join(' · ')}
function renderScan(){flatFlights=[];let t=SCAN.totals;document.getElementById('totals').innerHTML=`<span>Родителей: <b>${t.parents}</b></span><span>Исходных flight: <b>${t.sources}</b></span><span>Размечено исходников: <b>${t.annotated_sources}</b></span><span>Raw-копий: <b>${t.raw_copies}</b></span><span>Синхронизировано: <b>${t.synced_copies}</b></span>`;let root=document.getElementById('groups');root.innerHTML='';SCAN.groups.forEach(g=>{let box=document.createElement('div');box.className='panel';let h=document.createElement('h2');h.textContent=`${g.index}. ${g.input} — ${g.flights.length} flight`;box.appendChild(h);let body=document.createElement('div');body.className='body';if(g.error){body.innerHTML=`<div class="bad">${esc(g.error)}</div>`;box.appendChild(body);root.appendChild(box);return}let tools=document.createElement('div');tools.className='controls';let pb=document.createElement('button');pb.className='green';pb.textContent='Авторазметить копии всех размеченных flight этой папки';pb.onclick=()=>propagateParent(g);tools.appendChild(pb);body.appendChild(tools);let table=document.createElement('table');table.innerHTML='<thead><tr><th>Исходный flight</th><th>Разметка / копии</th><th>Файлы</th><th>Действия</th></tr></thead><tbody></tbody>';let tb=table.querySelector('tbody');g.flights.forEach(row=>{let idx=flatFlights.length;flatFlights.push(row);let tr=document.createElement('tr');let fs=row.complete?'<span class="good">OK</span>':`<span class="bad">нет: ${esc(row.missing.join(', '))}</span>`;tr.innerHTML=`<td><b>${esc(row.name)}</b><div class="details">${esc(row.path)}</div></td><td>${annBadges(row.annotation)}<div class="details">${esc(copyDetail(row.annotation))}</div></td><td>${fs}</td><td class="actions"></td>`;let act=tr.querySelector('.actions');let edit=document.createElement('button');edit.className='primary';edit.textContent=row.annotation.annotation_exists?'✎ Редактировать исходник':'✎ Разметить исходник';edit.disabled=!row.complete;edit.onclick=()=>openEditor(row.path,idx);let prop=document.createElement('button');prop.className='green';prop.textContent=`Авторазметить ${row.annotation.copies_total} копий`;prop.disabled=!row.annotation.annotation_exists||row.annotation.copies_total===0;prop.onclick=()=>propagateOne(row.path);act.append(edit,prop);tb.appendChild(tr)});body.appendChild(table);box.appendChild(body);root.appendChild(box)})}
async function scan(){SCAN=await api('/api/scan');CFG={safe_root:SCAN.safe_root,raw_root:SCAN.raw_root,parents:SCAN.parents};document.getElementById('safeRoot').value=CFG.safe_root;document.getElementById('rawRoot').value=CFG.raw_root;renderParentCards(CFG.parents);renderScan();toast(`Найдено исходных flight: ${SCAN.totals.sources}`)}
function showProgress(title,text,pct){document.getElementById('progressOverlay').classList.remove('hidden');document.getElementById('progressTitle').textContent=title;document.getElementById('progressText').textContent=text||'';document.getElementById('progressBar').value=pct||0}function hideProgress(){document.getElementById('progressOverlay').classList.add('hidden')}
async function propagateOne(path){if(!confirm('Перезаписать разметку всех raw-копий текущей сохранённой разметкой исходника?'))return;showProgress('Авторазметка копий',path,25);try{let r=await api('/api/propagate',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({flight:path})});showProgress('Авторазметка копий',`Готово ${r.updated.length}/${r.copies_found}; ошибок ${r.errors.length}`,100);setTimeout(hideProgress,600);await scan();toast(`Обновлено копий: ${r.updated.length}${r.errors.length?', ошибок: '+r.errors.length:''}`,r.errors.length>0)}catch(e){hideProgress();toast(e.message,true)}}
async function propagateMany(paths,title){let done=0,errors=0,updated=0;showProgress(title,`0 / ${paths.length}`,0);for(let i=0;i<paths.length;i++){try{let r=await api('/api/propagate',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({flight:paths[i]})});updated+=r.updated.length;errors+=r.errors.length}catch(e){errors++}done++;showProgress(title,`${done}/${paths.length} исходников · копий ${updated} · ошибок ${errors}`,100*done/paths.length)}setTimeout(hideProgress,600);await scan();toast(`Авторазметка завершена: копий ${updated}, ошибок ${errors}`,errors>0)}
async function propagateParent(g){let paths=g.flights.filter(x=>x.annotation.annotation_exists&&x.annotation.copies_total>0).map(x=>x.path);if(!paths.length){toast('Нет размеченных исходников с копиями',true);return}if(confirm(`Авторазметить копии для ${paths.length} исходных flight?`))await propagateMany(paths,`Группа ${g.input}`)}document.getElementById('propagateAll').onclick=async()=>{if(!SCAN)return;let paths=flatFlights.filter(x=>x.annotation.annotation_exists&&x.annotation.copies_total>0).map(x=>x.path);if(!paths.length){toast('Нет размеченных исходников с копиями',true);return}if(confirm(`Авторазметить копии для всех ${paths.length} размеченных исходников?`))await propagateMany(paths,'Все выбранные папки')};
function near(arr,t){let l=0,h=arr.length;while(l<h){let m=(l+h)>>1;if(arr[m]<t)l=m+1;else h=m}if(l<=0)return 0;if(l>=arr.length)return arr.length-1;return Math.abs(arr[l]-t)<Math.abs(arr[l-1]-t)?l:l-1}
function fi(){if(!D||!D.forward_ts.length||!Number.isFinite(f.currentTime))return 0;return Math.max(0,Math.min(D.forward_ts.length-1,Math.round(f.currentTime*D.video_fps)))}
function navFi(){return requestedFrame==null?fi():requestedFrame}
function ft(i){return D.forward_ts[Math.max(0,Math.min(D.forward_ts.length-1,i))]}
function setVideoStatus(t,bad=false){let e=document.getElementById('videoStatus');e.textContent=t;e.className=bad?'small bad':'small'}
function waitMediaReady(v,label,timeout=8000){return new Promise((resolve,reject)=>{if(v.readyState>=1&&Number.isFinite(v.duration)){resolve();return}let done=false;let timer=setTimeout(()=>finish(new Error(label+': timeout загрузки metadata')),timeout);function cleanup(){clearTimeout(timer);v.removeEventListener('loadedmetadata',ok);v.removeEventListener('canplay',ok);v.removeEventListener('error',bad)}function finish(err){if(done)return;done=true;cleanup();err?reject(err):resolve()}function ok(){finish()}function bad(){let m=v.error?(' media error '+v.error.code):' неизвестная ошибка';finish(new Error(label+':'+m))}v.addEventListener('loadedmetadata',ok,{once:true});v.addEventListener('canplay',ok,{once:true});v.addEventListener('error',bad,{once:true})})}
function setMediaSource(v,url){try{v.pause()}catch(e){}v.removeAttribute('src');v.load();v.src=url;v.load()}
async function loadVideoPair(path,attempt=1){let token=Date.now()+'-'+attempt;setVideoStatus('Загрузка видео… попытка '+attempt);setMediaSource(f,'/media?flight='+q(path)+'&camera=forward&v='+token);setMediaSource(b,'/media?flight='+q(path)+'&camera=bottom&v='+token);try{await Promise.all([waitMediaReady(f,'Передняя камера'),waitMediaReady(b,'Нижняя камера')]);setVideoStatus('Видео готово');return true}catch(e){if(attempt<3){setVideoStatus(e.message+'; повтор…',true);await new Promise(r=>setTimeout(r,250*attempt));return loadVideoPair(path,attempt+1)}setVideoStatus('Не удалось загрузить видео: '+e.message,true);return false}}
function safeSeek(v,time){if(!Number.isFinite(time)||v.readyState<1)return false;try{let d=Number.isFinite(v.duration)&&v.duration>0?v.duration:time;v.currentTime=Math.max(0,Math.min(time,Math.max(0,d-0.0001)));return true}catch(e){return false}}
function showPosition(i){if(!D||!D.forward_ts.length)return;i=Math.max(0,Math.min(D.forward_ts.length-1,i));seek.value=i;document.getElementById('frame').textContent='кадр '+i;document.getElementById('ftime').textContent=ft(i).toFixed(4)+' с'}
function sync(i,immediate=false){if(!D||!D.bottom_ts.length)return;clearTimeout(bottomSeekTimer);let run=()=>{if(b.readyState<1)return;let j=near(D.bottom_ts,ft(i));safeSeek(b,j/D.video_fps)};if(immediate)run();else bottomSeekTimer=setTimeout(run,70)}
function pos(){if(requestedFrame==null&&!f.seeking)showPosition(fi())}
function finishForwardSeek(){if(activeSeekFrame==null)return;clearTimeout(seekWatchdog);let completed=activeSeekFrame;activeSeekFrame=null;if(queuedFrame!=null&&queuedFrame!==completed){pumpForwardSeek();return}queuedFrame=null;requestedFrame=null;let i=fi();showPosition(i);sync(i)}
function pumpForwardSeek(){if(activeSeekFrame!=null||queuedFrame==null||f.readyState<1)return;let i=queuedFrame;queuedFrame=null;activeSeekFrame=i;let target=i/D.video_fps;if(Math.abs(f.currentTime-target)<0.0005&&!f.seeking){finishForwardSeek();return}if(!safeSeek(f,target)){activeSeekFrame=null;queuedFrame=i;setVideoStatus('Видео ещё загружается — кадр будет показан после загрузки',true);return}seekWatchdog=setTimeout(finishForwardSeek,2000)}
function go(i){if(!D||!D.forward_ts.length)return;i=Math.max(0,Math.min(D.forward_ts.length-1,Math.round(i)));requestedFrame=i;queuedFrame=i;showPosition(i);pumpForwardSeek()}
function stepFrames(delta){go(navFi()+delta)}
function programs(){let r=document.getElementById('programs');r.innerHTML='';D.ontology.forEach(x=>{let z=document.createElement('button');z.textContent=x.id+'. '+x.name_ru;if(P&&P.id===x.id)z.classList.add('active');z.onclick=()=>selProg(x.id);r.appendChild(z)})}function selProg(id){P=D.ontology.find(x=>x.id===id);S.clear();programs();states();label()}function states(){let r=document.getElementById('states');r.innerHTML='';if(!P){r.textContent='Сначала выбери подпрограмму';return}P.states.forEach(x=>{let z=document.createElement('button');z.textContent=x.ru;if(S.has(x.en))z.classList.add('stateActive');z.onclick=()=>{S.has(x.en)?S.delete(x.en):S.add(x.en);states();label()};r.appendChild(z)})}function stateRu(id,en){let p=D.ontology.find(x=>x.id===id);if(!p)return en;let s=p.states.find(x=>x.en===en);return s?s.ru:en}function progRu(id){let p=D.ontology.find(x=>x.id===id);return p?p.name_ru:String(id)}function label(){if(!P){document.getElementById('label').textContent='Выбери одну подпрограмму и одно или несколько состояний.';return}let rr=P.states.filter(x=>S.has(x.en)).map(x=>x.ru);document.getElementById('label').innerHTML='<b>'+P.id+'. '+P.name_ru+'</b><br>Состояния: <b>'+(rr.length?rr.join(', '):'не выбраны')+'</b>'}function marks(){let x=v=>v==null?'—':'кадр '+v+', t='+ft(v).toFixed(4);document.getElementById('marks').textContent='Начало: '+x(st)+' | Конец: '+x(en)}function message(t,bad=false){let e=document.getElementById('msg');e.textContent=t;e.className=bad?'bad':'good'}
function updateEditUi(){let badge=document.getElementById('editBadge'),save=document.getElementById('saveSeg'),cancel=document.getElementById('cancelEdit');if(editIndex==null){badge.classList.remove('on');badge.textContent='';save.textContent='Enter добавить сегмент';cancel.style.display='none'}else{let x=A[editIndex];badge.classList.add('on');badge.textContent='РЕДАКТИРОВАНИЕ: сегмент #'+(x?x.segment_id:editIndex+1);save.textContent='Enter применить изменение';cancel.style.display='inline-block'}}function resetEditor(clearProgram=true){editIndex=null;st=null;en=null;if(clearProgram){P=null;S.clear();programs();states();label()}marks();updateEditUi();renderSeg()}function editSeg(i){let x=A[i];if(!x)return;editIndex=i;P=D.ontology.find(p=>p.id===x.subprogram_id)||null;S=new Set(x.states||[]);st=near(D.forward_ts,Number(x.start_timestamp));en=near(D.forward_ts,Number(x.end_timestamp));programs();states();label();marks();updateEditUi();renderSeg();go(st);message('Сегмент #'+x.segment_id+' загружен для редактирования')}
function saveSegLocal(){if(!P){message('Выбери подпрограмму',true);return}if(S.size===0){message('Выбери хотя бы одно состояние',true);return}if(st==null||en==null){message('Поставь начало и конец',true);return}let a=Math.min(st,en),z=Math.max(st,en),ta=ft(a),tz=ft(z),ss=P.states.filter(x=>S.has(x.en)).map(x=>x.en);let item={segment_id:editIndex==null?A.length+1:A[editIndex].segment_id,subprogram_id:P.id,subprogram_name:P.name_en,states:ss,start_timestamp:ta,end_timestamp:tz};if(editIndex!=null)A[editIndex]=item;else A.push(item);A.sort((x,y)=>x.start_timestamp-y.start_timestamp);A.forEach((x,i)=>x.segment_id=i+1);editIndex=null;dirty=true;renderSeg();updateEditUi();st=Math.min(z+1,D.forward_ts.length-1);en=null;marks();message('Изменение в редакторе. Нажмите «Сохранить исходник».')}function deleteSeg(i){let x=A[i];if(!x||!confirm('Удалить сегмент #'+x.segment_id+'?'))return;A.splice(i,1);A.forEach((y,j)=>y.segment_id=j+1);editIndex=null;dirty=true;resetEditor(false);message('Сегмент удалён в редакторе')}function renderSeg(){let t=document.getElementById('segments');t.innerHTML='';A.forEach((x,i)=>{let tr=document.createElement('tr');if(editIndex===i)tr.classList.add('editing');let sr=(x.states||[]).map(s=>stateRu(x.subprogram_id,s)).join(', ');tr.innerHTML='<td>'+x.segment_id+'</td><td>'+Number(x.start_timestamp).toFixed(3)+'–'+Number(x.end_timestamp).toFixed(3)+'</td><td>'+x.subprogram_id+'. '+esc(progRu(x.subprogram_id))+'</td><td>'+esc(sr)+'</td><td class="rowActions"><button>✎ Ред.</button><button class="danger">×</button></td>';tr.onclick=e=>{if(!e.target.closest('button'))go(near(D.forward_ts,x.start_timestamp))};let bs=tr.querySelectorAll('button');bs[0].onclick=e=>{e.stopPropagation();editSeg(i)};bs[1].onclick=e=>{e.stopPropagation();deleteSeg(i)};t.appendChild(tr)})}
async function openEditor(path,idx){if(dirty&&!confirm('Есть несохранённые изменения текущего flight. Перейти без сохранения?'))return;currentFlight=path;currentFlatIndex=idx;message('');setVideoStatus('Подготовка…');D=await api('/api/editor?flight='+q(path));A=D.annotations||[];dirty=false;P=null;S.clear();st=en=editIndex=null;requestedFrame=queuedFrame=activeSeekFrame=null;clearTimeout(seekWatchdog);clearTimeout(bottomSeekTimer);document.getElementById('dashboard').classList.add('hidden');document.getElementById('editor').classList.remove('hidden');['backBtn','prevBtn','nextBtn'].forEach(id=>document.getElementById(id).classList.remove('hidden'));document.getElementById('prevBtn').disabled=currentFlatIndex<=0;document.getElementById('nextBtn').disabled=currentFlatIndex>=flatFlights.length-1;document.getElementById('flightTitle').textContent='Исходный: '+D.flight_name;document.getElementById('copySummary').textContent=`Сохранено сегментов: ${A.length} · raw-копий: ${D.copy_info.copies_total} · синхр.: ${D.copy_info.copies_synced} · устарело: ${D.copy_info.copies_stale}`;seek.max=Math.max(0,D.forward_ts.length-1);programs();states();label();marks();renderSeg();updateEditUi();f.ontimeupdate=pos;f.onseeked=finishForwardSeek;f.onloadedmetadata=pumpForwardSeek;let ok=await loadVideoPair(path);if(ok)go(0);else message('Видео не загрузилось. Можно открыть этот flight повторно.',true);toast('Загружена сохранённая разметка: '+A.length+' сегм.')}
async function saveSource(propagate){try{let r=await api('/api/save_source',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({flight:currentFlight,segments:A,propagate:!!propagate})});dirty=false;let m=`Исходник сохранён: ${r.segments} сегм.`;if(propagate)m+=` · копий ${r.propagation.updated.length}/${r.propagation.copies_found}`;message(m,r.propagation&&r.propagation.errors.length>0);await scan();let idx=flatFlights.findIndex(x=>x.path===currentFlight);if(idx>=0)currentFlatIndex=idx;D=await api('/api/editor?flight='+q(currentFlight));document.getElementById('copySummary').textContent=`Сохранено сегментов: ${D.annotations.length} · raw-копий: ${D.copy_info.copies_total} · синхр.: ${D.copy_info.copies_synced} · устарело: ${D.copy_info.copies_stale}`}catch(e){message(e.message,true)}}
document.getElementById('saveSourceTop').onclick=()=>saveSource(false);document.getElementById('savePropTop').onclick=()=>saveSource(true);document.getElementById('backBtn').onclick=()=>{if(dirty&&!confirm('Есть несохранённые изменения. Вернуться без сохранения?'))return;document.getElementById('editor').classList.add('hidden');document.getElementById('dashboard').classList.remove('hidden');['backBtn','prevBtn','nextBtn'].forEach(id=>document.getElementById(id).classList.add('hidden'));try{f.pause();b.pause()}catch(e){}f.removeAttribute('src');b.removeAttribute('src');f.load();b.load();message('');setVideoStatus('Видео не загружено');currentFlight=null;dirty=false};document.getElementById('prevBtn').onclick=()=>{if(currentFlatIndex>0)openEditor(flatFlights[currentFlatIndex-1].path,currentFlatIndex-1)};document.getElementById('nextBtn').onclick=()=>{if(currentFlatIndex<flatFlights.length-1)openEditor(flatFlights[currentFlatIndex+1].path,currentFlatIndex+1)};
document.getElementById('play').onclick=()=>{if(f.paused){f.play();b.play().catch(()=>{})}else{f.pause();b.pause();sync(navFi())}};document.getElementById('m1').onclick=()=>stepFrames(-1);document.getElementById('p1').onclick=()=>stepFrames(1);document.getElementById('m10').onclick=()=>stepFrames(-10);document.getElementById('p10').onclick=()=>stepFrames(10);document.getElementById('sync').onclick=()=>sync(navFi(),true);document.getElementById('start').onclick=()=>{st=navFi();marks()};document.getElementById('end').onclick=()=>{en=navFi();if(st!=null&&en<st)[st,en]=[en,st];marks()};document.getElementById('saveSeg').onclick=saveSegLocal;document.getElementById('cancelEdit').onclick=()=>{resetEditor(false);message('Редактирование сегмента отменено')};document.getElementById('clear').onclick=()=>{st=en=null;marks()};seek.oninput=()=>go(Number(seek.value));window.onkeydown=e=>{let field=e.target.closest('input,textarea,select,[contenteditable="true"]');if(document.getElementById('editor').classList.contains('hidden')||(field&&field!==seek))return;if(e.code==='Space'){e.preventDefault();document.getElementById('play').click()}else if(e.key==='ArrowLeft'){e.preventDefault();stepFrames(e.shiftKey?-10:-1)}else if(e.key==='ArrowRight'){e.preventDefault();stepFrames(e.shiftKey?10:1)}else if(e.key==='['){st=navFi();marks()}else if(e.key===']'){en=navFi();if(st!=null&&en<st)[st,en]=[en,st];marks()}else if(e.key==='Enter'){saveSegLocal()}else if(e.key==='Escape'&&editIndex!=null){resetEditor(false);message('Редактирование сегмента отменено')}};
async function init(){let c=await api('/api/config');CFG=c.config;document.getElementById('safeRoot').value=CFG.safe_root;document.getElementById('rawRoot').value=CFG.raw_root;renderParentCards(CFG.parents||[]);await scan()}init().catch(e=>toast(e.message,true));
</script></body></html>'''


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def send_bytes(self, data, ctype, status=200):
        self.send_response(status)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def send_json(self, obj, status=200):
        self.send_bytes(json.dumps(obj, ensure_ascii=False).encode('utf-8'),
                        'application/json; charset=utf-8', status)

    def read_json(self):
        n = int(self.headers.get('Content-Length', '0'))
        return json.loads((self.rfile.read(n) if n else b'{}').decode('utf-8'))

    @property
    def cfg(self):
        return self.server.cfg

    def source_index(self):
        return make_source_index(Path(self.cfg['safe_root']).resolve(), self.cfg.get('parents', []))

    def do_GET(self):
        u = urllib.parse.urlparse(self.path)
        p = u.path
        query = urllib.parse.parse_qs(u.query)
        try:
            if p == '/':
                return self.send_bytes(HTML.encode('utf-8'), 'text/html; charset=utf-8')
            if p == '/api/config':
                return self.send_json({'ok': True, 'config': self.cfg})
            if p == '/api/scan':
                return self.send_json({'ok': True, **scan_config(self.cfg)})
            if p == '/api/editor':
                source = validate_source_path(query.get('flight', [''])[0], self.cfg['safe_root'])
                complete, missing = flight_complete(source)
                if not complete:
                    raise RuntimeError('Исходный flight неполный: ' + ', '.join(missing))
                return self.send_json({'ok': True, 'flight_name': source.name, 'flight_path': str(source),
                    'ontology': ONTOLOGY_JSON, 'forward_ts': read_ts(source / 'forward/timestamps.csv'),
                    'bottom_ts': read_ts(source / 'bottom/timestamps.csv'), 'video_fps': VIDEO_FPS,
                    'annotations': load_segments(source), 'copy_info': source_annotation_info(source, self.cfg['raw_root'])})
            if p == '/media':
                source = validate_source_path(query.get('flight', [''])[0], self.cfg['safe_root'])
                camera = query.get('camera', [''])[0]
                if camera not in ('forward', 'bottom'):
                    raise ValueError('camera must be forward or bottom')
                return self.serve_file(source / camera / 'video.mp4')
            return self.send_bytes(b'Not found', 'text/plain', 404)
        except Exception as exc:
            return self.send_json({'ok': False, 'error': str(exc)}, 400)

    def _video_headers(self, ctype, size, *, status=200, start=None, end=None):
        self.send_response(status)
        self.send_header('Content-Type', ctype)
        self.send_header('Accept-Ranges', 'bytes')
        self.send_header('Cache-Control', 'private, max-age=3600')
        if status == 206 and start is not None and end is not None:
            self.send_header('Content-Range', f'bytes {start}-{end}/{size}')
            self.send_header('Content-Length', str(end - start + 1))
        else:
            self.send_header('Content-Length', str(size))
        self.end_headers()

    @staticmethod
    def _parse_single_range(header, size):
        if not header or not header.startswith('bytes='):
            return None
        spec = header[6:].split(',', 1)[0].strip()
        if '-' not in spec:
            raise ValueError('Invalid Range')
        a, z = spec.split('-', 1)
        if not a:
            n = int(z)
            if n <= 0:
                raise ValueError('Invalid suffix range')
            n = min(n, size)
            return size - n, size - 1
        start = int(a)
        if start < 0 or start >= size:
            raise ValueError('Range start outside file')
        end = int(z) if z else size - 1
        end = min(end, size - 1)
        if end < start:
            raise ValueError('Invalid Range end')
        return start, end

    def serve_file(self, path, *, head_only=False):
        path = Path(path)
        if not path.exists():
            return self.send_bytes(b'Missing video', 'text/plain', 404)
        size = path.stat().st_size
        ctype = mimetypes.guess_type(str(path))[0] or 'application/octet-stream'
        try:
            byte_range = self._parse_single_range(self.headers.get('Range'), size)
        except Exception:
            self.send_response(416)
            self.send_header('Content-Range', f'bytes */{size}')
            self.send_header('Accept-Ranges', 'bytes')
            self.send_header('Content-Length', '0')
            self.end_headers()
            return
        if byte_range is None:
            self._video_headers(ctype, size, status=200)
            start, end = 0, size - 1
        else:
            start, end = byte_range
            self._video_headers(ctype, size, status=206, start=start, end=end)
        if head_only:
            return
        left = end - start + 1
        try:
            with open(path, 'rb') as fh:
                fh.seek(start)
                while left > 0:
                    chunk = fh.read(min(1024 * 1024, left))
                    if not chunk:
                        break
                    try:
                        self.wfile.write(chunk)
                    except (BrokenPipeError, ConnectionResetError):
                        break
                    left -= len(chunk)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_HEAD(self):
        u = urllib.parse.urlparse(self.path)
        query = urllib.parse.parse_qs(u.query)
        if u.path != '/media':
            self.send_response(404)
            self.send_header('Content-Length', '0')
            self.end_headers()
            return
        try:
            source = validate_source_path(query.get('flight', [''])[0], self.cfg['safe_root'])
            camera = query.get('camera', [''])[0]
            if camera not in ('forward', 'bottom'):
                raise ValueError('camera must be forward or bottom')
            return self.serve_file(source / camera / 'video.mp4', head_only=True)
        except Exception:
            self.send_response(400)
            self.send_header('Content-Length', '0')
            self.end_headers()


    def do_POST(self):
        p = urllib.parse.urlparse(self.path).path
        try:
            if p == '/api/config':
                body = self.read_json()
                safe_root = Path(body.get('safe_root') or DEFAULT_SAFE_ROOT).expanduser().resolve()
                raw_root = Path(body.get('raw_root') or DEFAULT_RAW_ROOT).expanduser().resolve()
                parents = [str(x).strip() for x in body.get('parents', []) if str(x).strip()]
                if not safe_root.is_dir(): raise RuntimeError(f'Safe root не существует: {safe_root}')
                if not raw_root.is_dir(): raise RuntimeError(f'Raw root не существует: {raw_root}')
                for x in parents: resolve_parent_path(safe_root, x)
                self.server.cfg = {'safe_root': str(safe_root), 'raw_root': str(raw_root), 'parents': parents}
                save_config(self.server.cfg)
                return self.send_json({'ok': True, 'config': self.server.cfg})
            if p == '/api/save_source':
                body = self.read_json()
                source = validate_source_path(body.get('flight', ''), self.cfg['safe_root'])
                idx = self.source_index()
                saved = save_source_annotation(source, body.get('segments', []), self.cfg['raw_root'], idx)
                result = {'ok': True, 'flight': source.name, 'segments': len(saved['segments']), 'digest': saved['source_annotation_digest']}
                if body.get('propagate'):
                    result['propagation'] = propagate_source(source, self.cfg['raw_root'], idx)
                return self.send_json(result)
            if p == '/api/propagate':
                body = self.read_json()
                source = validate_source_path(body.get('flight', ''), self.cfg['safe_root'])
                result = propagate_source(source, self.cfg['raw_root'], self.source_index())
                return self.send_json({'ok': True, **result})
            return self.send_bytes(b'Not found', 'text/plain', 404)
        except Exception as exc:
            return self.send_json({'ok': False, 'error': str(exc)}, 400)


def main():
    ap = argparse.ArgumentParser(description='Concept-VLA mass annotator for safe originals and raw replay copies.')
    ap.add_argument('--safe-root', default=None)
    ap.add_argument('--raw-root', default=None)
    ap.add_argument('--parent', action='append', default=None, help='Parent folder under safe root; repeat option')
    ap.add_argument('--host', default=HOST)
    ap.add_argument('--port', type=int, default=PORT)
    ap.add_argument('--no-browser', action='store_true')
    args = ap.parse_args()
    cfg = load_config()
    if args.safe_root: cfg['safe_root'] = str(Path(args.safe_root).expanduser().resolve())
    if args.raw_root: cfg['raw_root'] = str(Path(args.raw_root).expanduser().resolve())
    if args.parent is not None: cfg['parents'] = args.parent
    safe_root = Path(cfg['safe_root']).expanduser().resolve(); raw_root = Path(cfg['raw_root']).expanduser().resolve()
    if not safe_root.is_dir(): raise SystemExit(f'Safe root not found: {safe_root}')
    if not raw_root.is_dir(): raise SystemExit(f'Raw root not found: {raw_root}')
    cfg['safe_root'] = str(safe_root); cfg['raw_root'] = str(raw_root); save_config(cfg)
    server = ThreadingHTTPServer((args.host, args.port), Handler); server.cfg = cfg
    url = f'http://{args.host}:{args.port}/'
    print('[ANNOTATOR] mass annotation UI'); print('[ANNOTATOR] safe root:', cfg['safe_root']); print('[ANNOTATOR] raw root:', cfg['raw_root']); print('[ANNOTATOR] open:', url); print('[ANNOTATOR] Ctrl+C to stop')
    if not args.no_browser: threading.Timer(.4, lambda: webbrowser.open(url)).start()
    try: server.serve_forever()
    except KeyboardInterrupt: print('\n[ANNOTATOR] stopped')
    finally: server.server_close()


if __name__ == '__main__':
    main()
