#!/usr/bin/env python3
"""Local web UI + batch orchestrator for Concept-VLA trajectory augmentation.

The script intentionally does not reimplement the flight/replay mechanics.  It
starts aerobot_gz_sim with the requested map appearance and calls the existing
replay.py once per generated sample, with deterministic parameters sampled from
GUI ranges.

Each source flight is replayed with its own captured ROS2 launch command; only
wall_style/floor_style are overridden.

Default output is fixed to:
  /home/sed/Desktop/concept-vla/datasets/raw

No third-party Python packages are required for this web UI.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import random
import re
import shutil
import signal
import shlex
import subprocess
import sys
import threading
import time
import uuid
import webbrowser
import zipfile
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

PROJECT_DIR = Path("/home/sed/Desktop/concept-vla")
OUTPUT_ROOT = PROJECT_DIR / "datasets" / "raw"
DEFAULT_REPLAY = PROJECT_DIR / "scripts" / "replay.py"
CACHE_ROOT = Path("/tmp/concept_vla_batch_sources")

# Required 50/50 split from the task.
STYLE_A = {
    "label": "Картон + ArUco",
    "wall_style": 4,
    "floor_style": 8,
    "description": "wall_style:=4 — коробочная / картонная текстура; floor_style:=8 — аруко-поле",
}
STYLE_B = {
    "label": "Обои + дерево",
    "wall_style": 2,
    "floor_style": 3,
    "description": "wall_style:=2 — стены с обоями; floor_style:=3 — деревянный пол",
}

PARAM_SPECS = {
    "flight_speed": {"label": "Скорость replay", "unit": "×", "default": (0.90, 1.10), "positive": True},
    "coord_dx": {"label": "Смещение X", "unit": "m", "default": (-0.12, 0.12)},
    "coord_dy": {"label": "Смещение Y", "unit": "m", "default": (-0.12, 0.12)},
    "coord_dz": {"label": "Смещение Z", "unit": "m", "default": (-0.06, 0.06)},
    "coord_dyaw": {"label": "Смещение yaw", "unit": "deg", "default": (-4.0, 4.0)},
    "coord_x_amp": {"label": "Wobble X amp", "unit": "m", "default": (0.02, 0.08), "nonnegative": True},
    "coord_y_amp": {"label": "Wobble Y amp", "unit": "m", "default": (0.02, 0.08), "nonnegative": True},
    "coord_z_amp": {"label": "Wobble Z amp", "unit": "m", "default": (0.01, 0.04), "nonnegative": True},
    "coord_yaw_amp": {"label": "Wobble yaw amp", "unit": "deg", "default": (0.5, 2.5), "nonnegative": True},
    "coord_timescale": {"label": "Wobble timescale", "unit": "s", "default": (3.0, 6.0), "positive": True},
}

LIGHTING_PRESETS = {"off", "constant", "day_to_sunset", "day_to_night", "cloud_pass", "random"}
REQUIRED_TOPICS = {
    "/uav1/mavros/state",
    "/uav1/mavros/local_position/pose",
    "/uav1/camera",
    "/uav1/camera_down",
}
REQUIRED_SERVICES = {
    "/uav1/mavros/cmd/arming",
    "/uav1/mavros/set_mode",
}


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def finite_float(v: Any, name: str) -> float:
    try:
        x = float(v)
    except Exception as exc:
        raise ValueError(f"{name}: требуется число") from exc
    if not math.isfinite(x):
        raise ValueError(f"{name}: значение должно быть конечным")
    return x


def flight_base_and_index(name: str) -> tuple[str, int]:
    canonical = re.sub(r"(?:-(?:CB|SP)\d+)+$", "", name)
    m = re.fullmatch(r"(flight-\d{8}-\d{6})(?:-(\d+))?", canonical)
    if not m:
        raise ValueError(
            f"Имя источника {name!r} не соответствует flight-YYYYMMDD-HHMMSS[-N]"
        )
    return m.group(1), int(m.group(2) or 0)


def meaningful_flight(path: Path) -> bool:
    for rel in (
        "forward/video.mp4",
        "bottom/video.mp4",
        "odom/odom.csv",
        "trajectory/trajectory.csv",
    ):
        p = path / rel
        try:
            if p.is_file() and p.stat().st_size > 0:
                return True
        except OSError:
            pass
    return False


def safe_extract_zip(zip_path: Path) -> Path:
    """Extract one flight zip into a stable /tmp cache and return the flight dir."""
    stat = zip_path.stat()
    key = f"{zip_path.stem}-{stat.st_size}-{stat.st_mtime_ns}"
    cache = CACHE_ROOT / re.sub(r"[^A-Za-z0-9_.-]", "_", key)
    marker = cache / ".ready"
    if marker.is_file():
        target = Path(marker.read_text(encoding="utf-8").strip())
        if (target / "trajectory" / "trajectory.csv").is_file():
            return target

    shutil.rmtree(cache, ignore_errors=True)
    cache.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(zip_path) as zf:
        members = [n for n in zf.namelist() if not n.endswith("/")]
        traj = [n for n in members if n.endswith("/trajectory/trajectory.csv") or n == "trajectory/trajectory.csv"]
        if not traj:
            raise ValueError("В ZIP не найден trajectory/trajectory.csv")
        if len(traj) > 1:
            raise ValueError("В ZIP найдено несколько flight-каталогов; нужен один источник")
        traj_name = traj[0]
        prefix = traj_name[: -len("trajectory/trajectory.csv")].rstrip("/")
        source_name = Path(prefix).name if prefix else zip_path.stem
        flight_base_and_index(source_name)
        target = cache / source_name
        target.mkdir(parents=True, exist_ok=True)

        root_resolved = target.resolve()
        for member in members:
            if prefix:
                if not member.startswith(prefix + "/"):
                    continue
                rel = member[len(prefix) + 1 :]
            else:
                rel = member
            if not rel:
                continue
            dst = (target / rel).resolve()
            if root_resolved != dst and root_resolved not in dst.parents:
                raise ValueError(f"Небезопасный путь внутри ZIP: {member}")
            dst.parent.mkdir(parents=True, exist_ok=True)
            with zf.open(member) as src, open(dst, "wb") as out:
                shutil.copyfileobj(src, out)

    if not (target / "trajectory" / "trajectory.csv").is_file():
        raise ValueError("После распаковки источник не содержит trajectory/trajectory.csv")
    marker.write_text(str(target), encoding="utf-8")
    return target


def resolve_source(value: str) -> Path:
    if not str(value).strip():
        raise ValueError("Укажите исходный flight-каталог или ZIP")
    p = Path(value).expanduser()
    candidates: list[Path]
    if p.is_absolute():
        candidates = [p]
    else:
        candidates = [Path.cwd() / p, PROJECT_DIR / p, OUTPUT_ROOT / p]
    found: Path | None = None
    for c in candidates:
        if c.exists():
            found = c.resolve()
            break
    if found is None:
        raise FileNotFoundError("Источник не найден: " + ", ".join(str(x) for x in candidates))
    if found.is_file() and found.suffix.lower() == ".zip":
        return safe_extract_zip(found)
    if not found.is_dir():
        raise ValueError("Источник должен быть flight-каталогом или ZIP")
    if not (found / "trajectory" / "trajectory.csv").is_file():
        raise ValueError(f"Нет {found / 'trajectory' / 'trajectory.csv'}")
    flight_base_and_index(found.name)
    return found


def trajectory_duration(source: Path) -> tuple[float, int]:
    path = source / "trajectory" / "trajectory.csv"
    last_t: float | None = None
    count = 0
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            count += 1
            raw = row.get("timestamp_start", row.get("t"))
            if raw is None:
                raise ValueError("trajectory.csv: нет timestamp_start/t")
            last_t = float(raw)
    if last_t is None or count < 2:
        raise ValueError("trajectory.csv пуст или содержит меньше 2 точек")
    return float(last_t), count


def detect_source_launch(source: Path) -> dict[str, Any]:
    """Read the exact ROS2 launch invocation captured with the source flight.

    Batch replay must use the same package, launch file and every explicit launch
    argument as the source flight.  Only wall_style/floor_style are allowed to
    be replaced by the augmentation style.
    """
    p = source / "environment" / "launch_context.json"
    if not p.is_file():
        return {"valid": False, "error": f"Нет {p}"}
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
        sel = data.get("selected") or {}
        package = str(sel.get("package") or "").strip()
        launch_file = str(sel.get("launch_file") or "").strip()
        argv = sel.get("argv")
        if not isinstance(argv, list) or len(argv) < 4:
            command = str(sel.get("command") or "").strip()
            argv = shlex.split(command) if command else []
        argv = [str(x) for x in argv]
        if len(argv) < 4 or Path(argv[0]).name != "ros2" or argv[1] != "launch":
            return {"valid": False, "error": "launch_context.json не содержит корректный ros2 launch argv"}
        if not package:
            package = argv[2]
        if not launch_file:
            launch_file = argv[3]
        if argv[2] != package or argv[3] != launch_file:
            return {"valid": False, "error": "package/launch_file не совпадают с сохранённым argv"}

        explicit_args: dict[str, str] = {}
        passthrough: list[str] = []
        for token in argv[4:]:
            if ":=" in token:
                key, raw = token.split(":=", 1)
                if key:
                    explicit_args[key] = raw
                    continue
            passthrough.append(token)

        out: dict[str, Any] = {
            "valid": True,
            "package": package,
            "launch_file": launch_file,
            "command": shlex.join(argv),
            "argv": argv,
            "arguments": explicit_args,
            "passthrough_args": passthrough,
        }
        for key in ("wall_style", "floor_style", "start_position"):
            if key in explicit_args:
                raw = explicit_args[key]
                try:
                    out[key] = int(raw)
                except ValueError:
                    out[key] = raw
        return out
    except Exception as exc:
        return {"valid": False, "error": f"Не удалось прочитать launch_context.json: {exc}"}


def launch_argv_with_style(source_launch: dict[str, Any], wall_style: int, floor_style: int) -> list[str]:
    """Copy source launch argv verbatim, replacing only texture arguments."""
    if not source_launch.get("valid"):
        raise ValueError(source_launch.get("error") or "Некорректный source launch metadata")
    argv = [str(x) for x in source_launch.get("argv") or []]
    if len(argv) < 4:
        raise ValueError("Source launch argv отсутствует")
    result = argv[:4]
    seen_wall = False
    seen_floor = False
    for token in argv[4:]:
        if token.startswith("wall_style:="):
            result.append(f"wall_style:={int(wall_style)}")
            seen_wall = True
        elif token.startswith("floor_style:="):
            result.append(f"floor_style:={int(floor_style)}")
            seen_floor = True
        else:
            result.append(token)
    if not seen_wall:
        result.append(f"wall_style:={int(wall_style)}")
    if not seen_floor:
        result.append(f"floor_style:={int(floor_style)}")
    return result


def source_info(value: str) -> dict[str, Any]:
    source = resolve_source(value)
    duration, samples = trajectory_duration(source)
    base, source_idx = flight_base_and_index(source.name)
    return {
        "resolved": str(source),
        "name": source.name,
        "base": base,
        "source_index": source_idx,
        "duration_sec": duration,
        "trajectory_samples": samples,
        "detected_launch": detect_source_launch(source),
    }


def resolve_parent(value: str) -> Path:
    if not str(value).strip():
        raise ValueError("Укажите родительскую папку")
    p = Path(value).expanduser()
    candidates = [p] if p.is_absolute() else [Path.cwd()/p, PROJECT_DIR/p, OUTPUT_ROOT/p]
    for c in candidates:
        if c.is_dir():
            return c.resolve()
    raise FileNotFoundError("Родительская папка не найдена: " + ", ".join(str(x) for x in candidates))


def discover_parent(value: str) -> dict[str, Any]:
    parent = resolve_parent(value)
    items: list[dict[str, Any]] = []
    for child in sorted(parent.iterdir(), key=lambda x: x.name):
        if not child.is_dir():
            continue
        try:
            flight_base_and_index(child.name)
        except ValueError:
            continue
        if not (child / "trajectory" / "trajectory.csv").is_file():
            continue
        try:
            items.append(source_info(str(child)))
        except Exception as exc:
            items.append({"name": child.name, "resolved": str(child.resolve()), "error": str(exc)})
    valid = [x for x in items if not x.get("error")]
    if not valid:
        raise ValueError(f"В {parent} не найдено ни одной корректной папки flight-YYYYMMDD-HHMMSS[-N]")
    return {
        "parent": str(parent),
        "count": len(valid),
        "flights": valid,
        "errors": [x for x in items if x.get("error")],
    }


def normalize_range(raw: Any, key: str) -> tuple[float, float]:
    spec = PARAM_SPECS[key]
    if not isinstance(raw, dict):
        lo, hi = spec["default"]
    else:
        lo = finite_float(raw.get("min", spec["default"][0]), key + ".min")
        hi = finite_float(raw.get("max", spec["default"][1]), key + ".max")
    if hi < lo:
        lo, hi = hi, lo
    if spec.get("positive") and lo <= 0:
        raise ValueError(f"{key}: min должен быть > 0")
    if spec.get("nonnegative") and lo < 0:
        raise ValueError(f"{key}: min должен быть >= 0")
    return lo, hi


def _validate_group(raw: dict[str, Any], group_index: int) -> dict[str, Any]:
    parent_info = discover_parent(str(raw.get("parent_path", "")))
    copies = int(raw.get("copies", 2))
    if not 1 <= copies <= 10000:
        raise ValueError(f"Группа {group_index}: количество копий должно быть 1..10000")
    seed = int(raw.get("seed", 42003))
    bad_launch = [
        f for f in parent_info["flights"]
        if not (f.get("detected_launch") or {}).get("valid")
    ]
    if bad_launch:
        details = "; ".join(
            f"{f.get('name')}: {(f.get('detected_launch') or {}).get('error', 'нет launch metadata')}"
            for f in bad_launch[:8]
        )
        if len(bad_launch) > 8:
            details += f"; … ещё {len(bad_launch) - 8}"
        raise ValueError(
            f"Группа {group_index}: у каждого source flight нужен достоверный environment/launch_context.json. {details}"
        )
    sampling_mode = str(raw.get("sampling_mode", "stratified"))
    if sampling_mode not in {"stratified", "uniform"}:
        raise ValueError(f"Группа {group_index}: sampling_mode: stratified или uniform")
    coord_frame = str(raw.get("coord_frame", "body"))
    if coord_frame not in {"body", "world"}:
        raise ValueError(f"Группа {group_index}: coord_frame: body или world")
    lighting = str(raw.get("lighting", "off"))
    if lighting not in LIGHTING_PRESETS:
        raise ValueError(f"Группа {group_index}: неизвестный lighting preset")
    ranges_raw = raw.get("ranges") or {}
    ranges = {k: normalize_range(ranges_raw.get(k), k) for k in PARAM_SPECS}
    return {
        "group_index": group_index,
        "parent_path": parent_info["parent"],
        "parent_info": parent_info,
        "copies": copies,
        "seed": seed,
        "sampling_mode": sampling_mode,
        "coord_frame": coord_frame,
        "lighting": lighting,
        "ranges": {k: {"min": v[0], "max": v[1]} for k, v in ranges.items()},
    }


def validate_config(raw: dict[str, Any]) -> dict[str, Any]:
    replay = Path(str(raw.get("replay_script") or DEFAULT_REPLAY)).expanduser()
    replay = ((Path.cwd() / replay).resolve() if not replay.is_absolute() else replay.resolve())
    if not replay.is_file():
        raise FileNotFoundError(f"replay.py не найден: {replay}")
    groups_raw = raw.get("groups")
    if not isinstance(groups_raw, list) or not groups_raw:
        raise ValueError("Добавьте хотя бы одну родительскую папку")
    if len(groups_raw) > 100:
        raise ValueError("Слишком много родительских групп (максимум 100)")
    groups = [_validate_group(g if isinstance(g, dict) else {}, i+1) for i, g in enumerate(groups_raw)]
    return {
        "replay_script": str(replay),
        "groups": groups,
        "restart_simulator_each_run": bool(raw.get("restart_simulator_each_run", True)),
        "stop_on_error": bool(raw.get("stop_on_error", True)),
        "sim_ready_timeout_sec": clamp(finite_float(raw.get("sim_ready_timeout_sec", 60), "sim_ready_timeout_sec"), 10, 300),
        "between_runs_sec": clamp(finite_float(raw.get("between_runs_sec", 1.5), "between_runs_sec"), 0, 30),
        "output_root": str(OUTPUT_ROOT),
    }

def sample_parameter_matrix(n: int, ranges: dict[str, dict[str, float]], seed: int, mode: str) -> list[dict[str, float]]:
    rng = random.Random(seed)
    columns: dict[str, list[float]] = {}
    for pi, key in enumerate(PARAM_SPECS):
        lo = float(ranges[key]["min"])
        hi = float(ranges[key]["max"])
        if n == 1 or hi == lo:
            vals = [(lo + hi) * 0.5] * n
        elif mode == "uniform":
            vals = [rng.uniform(lo, hi) for _ in range(n)]
        else:
            # Latin-hypercube-like one-dimensional strata, independently
            # shuffled per parameter. This covers every configured range better
            # than pure random sampling while remaining deterministic.
            local = random.Random(seed + 7919 * (pi + 1))
            vals = [lo + (hi - lo) * ((i + local.random()) / n) for i in range(n)]
            local.shuffle(vals)
        columns[key] = vals
    rows = []
    for i in range(n):
        row = {k: float(columns[k][i]) for k in PARAM_SPECS}
        row["coord_seed"] = int(seed + 1009 * (i + 1))
        row["lighting_seed"] = int(seed + 2003 * (i + 1))
        rows.append(row)
    return rows


def next_output_names(source_name: str, n: int, reserved: set[str] | None = None) -> list[str]:
    base, source_idx = flight_base_and_index(source_name)
    reserved = reserved if reserved is not None else set()
    names: list[str] = []
    idx = source_idx + 1
    while len(names) < n:
        name = f"{base}-{idx}"
        p = OUTPUT_ROOT / name
        if name not in reserved and (not p.exists() or not meaningful_flight(p)):
            names.append(name)
            reserved.add(name)
        idx += 1
    return names


def build_plan(cfg: dict[str, Any]) -> list[dict[str, Any]]:
    runs: list[dict[str, Any]] = []
    reserved: set[str] = set()
    global_index = 0
    for group in cfg["groups"]:
        flights = group["parent_info"]["flights"]
        for source_pos, info in enumerate(flights, 1):
            n = group["copies"]
            # Stable but distinct sampling for every source in a parent group.
            source_seed = int(group["seed"] + 1000003 * source_pos)
            params = sample_parameter_matrix(n, group["ranges"], source_seed, group["sampling_mode"])
            names = next_output_names(info["name"], n, reserved)
            a_count = (n + 1) // 2
            for copy_i in range(n):
                global_index += 1
                style = STYLE_A if copy_i < a_count else STYLE_B
                p = params[copy_i]
                source_launch = info["detected_launch"]
                launch_argv = launch_argv_with_style(
                    source_launch, style["wall_style"], style["floor_style"]
                )
                runs.append({
                    "index": global_index,
                    "group_index": group["group_index"],
                    "parent_path": group["parent_path"],
                    "source_position": source_pos,
                    "source_name": info["name"],
                    "source_flight": info["resolved"],
                    "copy_index": copy_i + 1,
                    "copies_for_source": n,
                    "status": "queued",
                    "phase": "В очереди",
                    "progress": 0.0,
                    "style_label": style["label"],
                    "wall_style": style["wall_style"],
                    "floor_style": style["floor_style"],
                    "params": p,
                    "planned_output": str(OUTPUT_ROOT / names[copy_i]),
                    "actual_output": None,
                    "error": None,
                    "started_at": None,
                    "finished_at": None,
                    "expected_route_sec": info["duration_sec"] / p["flight_speed"],
                    "run_config": {
                        "coord_frame": group["coord_frame"],
                        "lighting": group["lighting"],
                        "source_launch": source_launch,
                        "launch_argv": launch_argv,
                        "launch_command": shlex.join(launch_argv),
                    },
                })
    return runs


@dataclass
class SharedState:
    lock: threading.RLock = field(default_factory=threading.RLock)
    status: str = "idle"
    message: str = "Готово к планированию"
    config: dict[str, Any] | None = None
    runs: list[dict[str, Any]] = field(default_factory=list)
    current_index: int | None = None
    batch_id: str | None = None
    batch_manifest: str | None = None
    logs: deque[str] = field(default_factory=lambda: deque(maxlen=400))
    stop_event: threading.Event = field(default_factory=threading.Event)
    worker: threading.Thread | None = None
    active_sim: subprocess.Popen[str] | None = None
    active_replay: subprocess.Popen[str] | None = None
    dry_run: bool = False

    def log(self, text: str) -> None:
        line = f"{datetime.now().strftime('%H:%M:%S')} {text.rstrip()}"
        with self.lock:
            self.logs.append(line)
        print(line, flush=True)

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            total = len(self.runs)
            overall = 0.0
            if total:
                overall = sum(float(r.get("progress", 0.0)) for r in self.runs) / total
            return {
                "status": self.status,
                "message": self.message,
                "config": self.config,
                "runs": json.loads(json.dumps(self.runs)),
                "current_index": self.current_index,
                "batch_id": self.batch_id,
                "batch_manifest": self.batch_manifest,
                "overall_progress": overall,
                "logs": list(self.logs)[-240:],
                "dry_run": self.dry_run,
                "output_root": str(OUTPUT_ROOT),
                "styles": [STYLE_A, STYLE_B],
                "param_specs": PARAM_SPECS,
            }

    def set_run(self, idx: int, **updates: Any) -> None:
        with self.lock:
            self.runs[idx].update(updates)


class BatchRunner:
    def __init__(self, state: SharedState):
        self.state = state
        self.sim_style: tuple[Any, ...] | None = None
        self._sim_reader: threading.Thread | None = None
        self._route_started_at: float | None = None
        self._parsed_output: str | None = None

    def start(self, cfg: dict[str, Any], runs: list[dict[str, Any]]) -> None:
        with self.state.lock:
            if self.state.worker and self.state.worker.is_alive():
                raise RuntimeError("Batch уже выполняется")
            self.state.config = cfg
            self.state.runs = runs
            self.state.status = "running"
            self.state.message = "Batch запущен"
            self.state.current_index = None
            self.state.stop_event.clear()
            self.state.batch_id = datetime.now().strftime("batch-%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:6]
            self.state.logs.clear()
            worker = threading.Thread(target=self._run, name="flight-batch", daemon=True)
            self.state.worker = worker
            worker.start()

    def request_stop(self) -> None:
        self.state.stop_event.set()
        with self.state.lock:
            self.state.status = "stopping"
            self.state.message = "Остановка..."
            replay = self.state.active_replay
            sim = self.state.active_sim
        if replay:
            self._terminate_group(replay, "replay", graceful=2.0)
        if sim:
            self._terminate_group(sim, "simulator", graceful=2.0)

    def _manifest_path(self) -> Path:
        OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
        d = OUTPUT_ROOT / "_batches"
        d.mkdir(parents=True, exist_ok=True)
        return d / f"{self.state.batch_id}.json"

    def _save_manifest(self) -> None:
        path = self._manifest_path()
        snap = self.state.snapshot()
        data = {
            "schema_version": 2,
            "batch_id": snap["batch_id"],
            "updated_at": now_iso(),
            "status": snap["status"],
            "message": snap["message"],
            "config": snap["config"],
            "runs": snap["runs"],
        }
        tmp = path.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        tmp.replace(path)
        with self.state.lock:
            self.state.batch_manifest = str(path)

    def _write_run_metadata(self, output: Path, run: dict[str, Any]) -> None:
        aug = output / "augmentation"
        aug.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema_version": 1,
            "batch_id": self.state.batch_id,
            "run_index": run["index"],
            "source": run["source_flight"],
            "parent_path": run["parent_path"],
            "group_index": run["group_index"],
            "source_name": run["source_name"],
            "copy_index": run["copy_index"],
            "style": {
                "wall_style": run["wall_style"],
                "floor_style": run["floor_style"],
                "label": run["style_label"],
            },
            "sampled_parameters": run["params"],
            "source_launch": run["run_config"]["source_launch"],
            "applied_launch": {
                "argv": run["run_config"]["launch_argv"],
                "command": run["run_config"]["launch_command"],
                "only_overrides": {
                    "wall_style": run["wall_style"],
                    "floor_style": run["floor_style"],
                },
            },
            "created_at": now_iso(),
        }
        (aug / "batch_run.json").write_text(
            json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )

    @staticmethod
    def _terminate_group(proc: subprocess.Popen[str], label: str, graceful: float = 8.0) -> None:
        if proc.poll() is not None:
            return
        try:
            pgid = os.getpgid(proc.pid)
        except ProcessLookupError:
            return
        for sig, wait_sec in ((signal.SIGINT, graceful), (signal.SIGTERM, 3.0), (signal.SIGKILL, 1.0)):
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                return
            try:
                proc.wait(timeout=wait_sec)
                return
            except subprocess.TimeoutExpired:
                continue

    def _spawn(self, cmd: list[str], label: str) -> subprocess.Popen[str]:
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        env.setdefault("RCUTILS_COLORIZED_OUTPUT", "0")
        self.state.log(f"[{label}] $ " + " ".join(cmd))
        return subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
            env=env,
        )

    def _sim_reader_loop(self, proc: subprocess.Popen[str]) -> None:
        if proc.stdout is None:
            return
        for line in proc.stdout:
            self.state.log("[SIM] " + line.rstrip())

    def _start_simulator(self, run_idx: int, run: dict[str, Any]) -> subprocess.Popen[str]:
        cfg = self.state.config
        assert cfg is not None
        rcfg = run["run_config"]
        style = (int(run["wall_style"]), int(run["floor_style"]))
        cmd = [str(x) for x in rcfg["launch_argv"]]
        signature = tuple(cmd)
        if self.state.dry_run:
            self.state.log(
                f"[DRY] source-exact simulator launch: {shlex.join(cmd)}"
            )
            self.sim_style = signature
            return None  # type: ignore[return-value]

        proc = self._spawn(cmd, "SIM")
        with self.state.lock:
            self.state.active_sim = proc
        self.sim_style = signature
        t = threading.Thread(target=self._sim_reader_loop, args=(proc,), daemon=True)
        self._sim_reader = t
        t.start()
        self.state.set_run(run_idx, phase="Запуск симулятора", progress=0.03)
        self._wait_ros_ready(proc, float(cfg["sim_ready_timeout_sec"]), run_idx)
        self.state.set_run(run_idx, phase="Симулятор готов", progress=0.10)
        return proc

    def _stop_simulator(self) -> None:
        with self.state.lock:
            proc = self.state.active_sim
        if proc is not None:
            self.state.log("[SIM] stopping simulator process group")
            self._terminate_group(proc, "simulator")
        with self.state.lock:
            self.state.active_sim = None
        self.sim_style = None
        if self._sim_reader is not None:
            self._sim_reader.join(timeout=1.0)
            self._sim_reader = None

    def _ros_list(self, kind: str) -> set[str]:
        try:
            cp = subprocess.run(
                ["ros2", kind, "list"], capture_output=True, text=True, timeout=5.0,
                env=os.environ.copy(), check=False,
            )
        except Exception:
            return set()
        if cp.returncode != 0:
            return set()
        return {x.strip() for x in cp.stdout.splitlines() if x.strip()}

    def _wait_ros_ready(self, sim: subprocess.Popen[str], timeout: float, run_idx: int) -> None:
        deadline = time.monotonic() + timeout
        last_missing: tuple[set[str], set[str]] | None = None
        while time.monotonic() < deadline:
            if self.state.stop_event.is_set():
                raise InterruptedError("Остановлено пользователем")
            rc = sim.poll()
            if rc is not None:
                raise RuntimeError(f"ros2 launch завершился до готовности, code={rc}")
            topics = self._ros_list("topic")
            services = self._ros_list("service")
            missing_t = REQUIRED_TOPICS - topics
            missing_s = REQUIRED_SERVICES - services
            last_missing = (missing_t, missing_s)
            if not missing_t and not missing_s:
                self.state.log("[SIM] ROS2 topics/services готовы")
                return
            self.state.set_run(run_idx, phase="Ожидание MAVROS/камер", progress=0.06)
            time.sleep(1.0)
        mt, ms = last_missing or (REQUIRED_TOPICS, REQUIRED_SERVICES)
        raise TimeoutError(
            "Симулятор не стал готов за %.0f s; missing topics=%s; services=%s"
            % (timeout, sorted(mt), sorted(ms))
        )

    def _replay_reader_loop(self, proc: subprocess.Popen[str], run_idx: int) -> None:
        if proc.stdout is None:
            return
        for raw in proc.stdout:
            line = raw.rstrip()
            self.state.log("[REPLAY] " + line)
            low = line.lower()
            if "pre-streaming mavros hold" in low:
                self.state.set_run(run_idx, phase="Setpoints / pre-stream", progress=0.14)
            elif "offboard request" in low or "arm request" in low:
                self.state.set_run(run_idx, phase="OFFBOARD / ARM", progress=0.17)
            elif "px4 confirmed offboard + armed" in low:
                self.state.set_run(run_idx, phase="ARM подтверждён", progress=0.20)
            elif "teleporting to recorded start" in low:
                self.state.set_run(run_idx, phase="Вариация стартовой позы", progress=0.22)
            elif "start pose fixed; beginning exact route" in low:
                self._route_started_at = time.monotonic()
                self.state.set_run(run_idx, phase="Полёт по вариативной траектории", progress=0.24)
            elif "manifest:" in low:
                self.state.set_run(run_idx, phase="Сохранение metadata", progress=0.97)
            m = re.search(r"\[REPLAY\]\s+completed:\s*(.+)$", line)
            if m:
                self._parsed_output = m.group(1).strip()

    def _replay_cmd(self, run: dict[str, Any]) -> list[str]:
        cfg = self.state.config
        assert cfg is not None
        rcfg = run["run_config"]
        p = run["params"]
        cmd = [
            "python3", cfg["replay_script"], run["source_flight"],
            "--coord-seed", str(int(p["coord_seed"])),
            "--coord-frame", rcfg["coord_frame"],
            "--coord-dx", f"{p['coord_dx']:.9g}",
            "--coord-dy", f"{p['coord_dy']:.9g}",
            "--coord-dz", f"{p['coord_dz']:.9g}",
            "--coord-dyaw", f"{p['coord_dyaw']:.9g}",
            "--coord-x-amp", f"{p['coord_x_amp']:.9g}",
            "--coord-y-amp", f"{p['coord_y_amp']:.9g}",
            "--coord-z-amp", f"{p['coord_z_amp']:.9g}",
            "--coord-yaw-amp", f"{p['coord_yaw_amp']:.9g}",
            "--coord-timescale", f"{p['coord_timescale']:.9g}",
            "--flight-speed", f"{p['flight_speed']:.9g}",
            "--lighting", rcfg["lighting"],
            "--lighting-seed", str(int(p["lighting_seed"])),
        ]
        return cmd

    def _run_replay(self, run_idx: int, run: dict[str, Any]) -> Path:
        if self.state.dry_run:
            total = 2.2
            self._route_started_at = time.monotonic()
            self.state.set_run(run_idx, phase="DRY: полёт", progress=0.25)
            while time.monotonic() - self._route_started_at < total:
                if self.state.stop_event.is_set():
                    raise InterruptedError("Остановлено пользователем")
                u = (time.monotonic() - self._route_started_at) / total
                self.state.set_run(run_idx, progress=0.25 + 0.70 * clamp(u, 0, 1))
                time.sleep(0.1)
            return Path(run["planned_output"])

        self._route_started_at = None
        self._parsed_output = None
        cmd = self._replay_cmd(run)
        proc = self._spawn(cmd, "REPLAY")
        with self.state.lock:
            self.state.active_replay = proc
        reader = threading.Thread(target=self._replay_reader_loop, args=(proc, run_idx), daemon=True)
        reader.start()
        expected = max(0.1, float(run["expected_route_sec"]))
        while proc.poll() is None:
            if self.state.stop_event.is_set():
                self._terminate_group(proc, "replay", graceful=2.0)
                raise InterruptedError("Остановлено пользователем")
            if self._route_started_at is not None:
                u = clamp((time.monotonic() - self._route_started_at) / expected, 0.0, 1.0)
                self.state.set_run(run_idx, progress=0.24 + 0.70 * u)
            time.sleep(0.15)
        reader.join(timeout=2.0)
        with self.state.lock:
            self.state.active_replay = None
        if proc.returncode != 0:
            raise RuntimeError(f"replay.py завершился с code={proc.returncode}")
        output = Path(self._parsed_output) if self._parsed_output else Path(run["planned_output"])
        if not output.is_absolute():
            output = (Path.cwd() / output).resolve()
        return output

    @staticmethod
    def _validate_output(output: Path, run: dict[str, Any]) -> None:
        required = [
            output / "forward" / "video.mp4",
            output / "bottom" / "video.mp4",
            output / "odom" / "odom.csv",
            output / "trajectory" / "trajectory.csv",
            output / "environment" / "launch_context.json",
        ]
        missing = []
        for p in required:
            try:
                if not p.is_file() or p.stat().st_size <= 0:
                    missing.append(str(p))
            except OSError:
                missing.append(str(p))
        if missing:
            raise RuntimeError("Неполный output; отсутствуют/пусты: " + ", ".join(missing))

        # Verify that replay.py captured exactly the simulator launch we planned.
        launch_path = output / "environment" / "launch_context.json"
        try:
            ctx = json.loads(launch_path.read_text(encoding="utf-8"))
            selected = ctx.get("selected") or {}
            actual = selected.get("argv")
            if not isinstance(actual, list) or len(actual) < 4:
                actual = shlex.split(str(selected.get("command") or ""))
            actual = [str(x) for x in actual]
        except Exception as exc:
            raise RuntimeError(f"Не удалось проверить output launch_context.json: {exc}") from exc
        expected = [str(x) for x in run["run_config"]["launch_argv"]]
        if actual != expected:
            raise RuntimeError(
                "Фактический launch output не совпадает с исходной картой/параметрами. "
                f"expected={shlex.join(expected)}; actual={shlex.join(actual)}"
            )

    def _run(self) -> None:
        cfg = self.state.config
        assert cfg is not None
        error_happened = False
        self.state.log(f"[BATCH] id={self.state.batch_id}")
        self.state.log(f"[BATCH] parent_groups={len(cfg['groups'])}")
        self.state.log(f"[BATCH] total_sources={sum(len(g['parent_info']['flights']) for g in cfg['groups'])}")
        self.state.log(f"[BATCH] output_root={OUTPUT_ROOT}")
        try:
            OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
            self._save_manifest()
            for idx, run in enumerate(self.state.runs):
                if self.state.stop_event.is_set():
                    raise InterruptedError("Остановлено пользователем")
                with self.state.lock:
                    self.state.current_index = idx
                    self.state.message = f"Полёт {idx + 1}/{len(self.state.runs)}"
                self.state.set_run(
                    idx, status="running", phase="Подготовка", progress=0.01,
                    started_at=now_iso(), error=None,
                )
                self._save_manifest()
                rcfg = run["run_config"]
                desired_style = tuple(str(x) for x in rcfg["launch_argv"])
                try:
                    need_restart = (
                        self.state.dry_run
                        or self.state.active_sim is None
                        or (self.state.active_sim is not None and self.state.active_sim.poll() is not None)
                        or self.sim_style != desired_style
                        or cfg["restart_simulator_each_run"]
                    )
                    if need_restart:
                        self._stop_simulator()
                        self._start_simulator(idx, run)
                    output = self._run_replay(idx, run)
                    self.state.set_run(idx, phase="Проверка output", progress=0.98)
                    if not self.state.dry_run:
                        self._validate_output(output, run)
                        self._write_run_metadata(output, run)
                    self.state.set_run(
                        idx, status="completed", phase="Готово", progress=1.0,
                        actual_output=str(output), finished_at=now_iso(),
                    )
                    self.state.log(f"[BATCH] run {idx + 1} completed -> {output}")
                except InterruptedError:
                    self.state.set_run(
                        idx, status="stopped", phase="Остановлено", finished_at=now_iso(),
                        error="Остановлено пользователем",
                    )
                    raise
                except Exception as exc:
                    error_happened = True
                    self.state.set_run(
                        idx, status="failed", phase="Ошибка", finished_at=now_iso(),
                        error=str(exc),
                    )
                    self.state.log(f"[ERROR] run {idx + 1}: {exc}")
                    self._stop_simulator()
                    if cfg["stop_on_error"]:
                        raise
                finally:
                    if cfg["restart_simulator_each_run"]:
                        self._stop_simulator()
                    self._save_manifest()
                if idx + 1 < len(self.state.runs) and cfg["between_runs_sec"] > 0:
                    end = time.monotonic() + float(cfg["between_runs_sec"])
                    while time.monotonic() < end:
                        if self.state.stop_event.wait(timeout=min(0.2, end - time.monotonic())):
                            raise InterruptedError("Остановлено пользователем")

            with self.state.lock:
                self.state.status = "completed" if not error_happened else "completed_with_errors"
                self.state.message = "Все полёты завершены" if not error_happened else "Batch завершён с ошибками"
        except InterruptedError:
            with self.state.lock:
                self.state.status = "stopped"
                self.state.message = "Остановлено пользователем"
            self.state.log("[BATCH] stopped")
        except Exception as exc:
            with self.state.lock:
                self.state.status = "failed"
                self.state.message = str(exc)
            self.state.log(f"[BATCH ERROR] {exc}")
        finally:
            self._stop_simulator()
            with self.state.lock:
                self.state.active_replay = None
                self.state.current_index = None
            try:
                self._save_manifest()
            except Exception as exc:
                self.state.log(f"[WARN] manifest save failed: {exc}")


HTML = r'''<!doctype html>
<html lang="ru"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Concept-VLA Multi Parent Flight Batch</title>
<style>
:root{--bg:#0b1020;--panel:#121a2e;--panel2:#18223a;--text:#e8eefb;--muted:#91a1bd;--line:#2a3857;--accent:#72a7ff;--good:#46d6a0;--warn:#ffc86b;--bad:#ff6b7a}
*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at 20% 0,#19284b 0,#0b1020 42%);color:var(--text);font:14px/1.45 Inter,system-ui,-apple-system,Segoe UI,sans-serif}.wrap{max-width:1600px;margin:auto;padding:24px}.hero{display:flex;justify-content:space-between;gap:20px;align-items:end;margin-bottom:18px}.hero h1{margin:0;font-size:28px}.hero p{margin:5px 0 0;color:var(--muted)}
.panel,.group{background:rgba(18,26,46,.94);border:1px solid var(--line);border-radius:16px;overflow:hidden;box-shadow:0 12px 34px #0005;margin-bottom:14px}.panel h2,.groupHead{font-size:15px;margin:0;padding:13px 16px;background:#ffffff05;border-bottom:1px solid var(--line)}.groupHead{display:flex;align-items:center;justify-content:space-between;gap:10px}.body,.groupBody{padding:15px}.group{border-color:#344767}.groupBody{display:grid;grid-template-columns:1fr 1fr;gap:14px}.full{grid-column:1/-1}
label{display:block;color:var(--muted);font-size:12px;margin:8px 0 4px}.row{display:flex;gap:9px;align-items:center;flex-wrap:wrap}.field{flex:1;min-width:180px}input,select,button{border:1px solid var(--line);background:#0d1529;color:var(--text);border-radius:9px;padding:9px 10px}input,select{width:100%}button{width:auto;cursor:pointer;font-weight:700}button:hover{border-color:var(--accent)}button.primary{background:#275fb8;border-color:#4c86e5}button.good{background:#176b51;border-color:#2daf80}button.bad{background:#702534;border-color:#b8495d}button.ghost{background:#121a2e}button:disabled{opacity:.45;cursor:not-allowed}.muted{color:var(--muted)}.tiny{font-size:12px}.chip{display:inline-flex;align-items:center;border:1px solid var(--line);border-radius:999px;padding:4px 9px;background:#0d1529}.styleA{border-color:#c89a51}.styleB{border-color:#8da7cf}.ok{color:var(--good)}.fail{color:var(--bad)}.running{color:var(--warn)}
.rangeGrid{display:grid;grid-template-columns:minmax(150px,1.2fr) repeat(2,minmax(90px,1fr)) 55px;gap:6px 8px;align-items:center}.rangeGrid .head{color:var(--muted);font-size:11px;text-transform:uppercase}.rangeGrid input{padding:7px 8px}.sourceInfo{padding:10px 11px;border-radius:9px;background:#0d1529;border:1px solid var(--line);margin-top:9px;max-height:150px;overflow:auto}.envs{display:grid;grid-template-columns:1fr 1fr;gap:8px}.env{padding:10px;border:1px solid var(--line);border-radius:10px;background:#0d1529}
.progress{height:12px;background:#08101f;border:1px solid var(--line);border-radius:99px;overflow:hidden}.bar{height:100%;width:0;background:linear-gradient(90deg,#4d8df7,#46d6a0);transition:width .25s}.metrics{display:grid;grid-template-columns:repeat(4,1fr);gap:9px;margin:10px 0}.metric{padding:10px;background:#0d1529;border:1px solid var(--line);border-radius:11px}.metric b{font-size:19px;display:block}.metric span{color:var(--muted);font-size:11px}.tableWrap{overflow:auto;max-height:600px;border:1px solid var(--line);border-radius:11px}table{border-collapse:collapse;width:100%;min-width:1450px;background:#0d1529}th,td{text-align:left;padding:8px;border-bottom:1px solid #1d2a45;white-space:nowrap}th{position:sticky;top:0;background:#172139;z-index:1;color:#aebbd1;font-size:11px;text-transform:uppercase}.pmini{width:90px;height:7px;background:#08101f;border-radius:99px;overflow:hidden;display:inline-block;vertical-align:middle}.pmini i{display:block;height:100%;background:#69a2ff}.log{height:330px;overflow:auto;background:#070c16;border:1px solid var(--line);border-radius:11px;padding:10px;font:12px/1.45 ui-monospace,SFMono-Regular,Consolas,monospace;white-space:pre-wrap;color:#cad5e8}.sectionTitle{font-weight:800;margin:0 0 8px}.topGrid{display:grid;grid-template-columns:1.2fr 1fr;gap:14px}
@media(max-width:1000px){.groupBody,.topGrid{grid-template-columns:1fr}.full{grid-column:auto}.metrics{grid-template-columns:1fr 1fr}.hero{align-items:start;flex-direction:column}}
</style></head>
<body><div class="wrap">
<div class="hero"><div><h1>Multi-Parent Flight Batch Control</h1><p>Каждый replay запускается на исходной карте с исходными launch-параметрами; меняются только wall_style/floor_style.</p></div><div class="row"><span id="statusChip" class="chip">idle</span><span class="chip">output: /home/sed/Desktop/concept-vla/datasets/raw</span></div></div>
<section class="panel"><h2>Общие настройки выполнения</h2><div class="body topGrid"><div><div class="row"><div class="field"><label>replay.py</label><input id="replay" value="/home/sed/Desktop/concept-vla/scripts/replay.py"></div><div style="width:170px"><label>Пауза между полётами, s</label><input id="between" type="number" min="0" step="0.1" value="1.5"></div><div style="width:190px"><label>Timeout симулятора, s</label><input id="readyTimeout" type="number" min="10" value="60"></div></div><div class="row" style="margin-top:8px"><label class="row"><input id="restart" type="checkbox" checked style="width:auto"> Перезапускать симулятор перед каждым полётом</label><label class="row"><input id="stopErr" type="checkbox" checked style="width:auto"> Остановиться при первой ошибке</label></div></div><div><div class="envs"><div class="env"><b>Первая половина каждого N</b><br><span class="chip styleA">wall 4 · floor 8</span><div class="muted tiny">картон + ArUco</div></div><div class="env"><b>Вторая половина каждого N</b><br><span class="chip styleB">wall 2 · floor 3</span><div class="muted tiny">обои + дерево</div></div></div><div class="muted tiny" style="margin-top:7px">При нечётном N первая конфигурация получает один дополнительный прогон.</div></div></div></section>
<section class="panel"><h2>Родительские папки и параметры вариаций</h2><div class="body"><div class="row" style="margin-bottom:12px"><div style="width:210px"><label>Количество родительских папок</label><input id="parentCount" type="number" min="1" max="100" value="1"></div><button id="applyCount" class="primary">Применить количество</button><button id="addGroup" class="ghost">+ Добавить папку</button><span class="muted tiny">N задаётся отдельно для группы. Карта/package/launch/start_position и прочие launch args берутся отдельно из каждого source flight.</span></div><div id="groups"></div></div></section>
<section class="panel"><h2>Управление и прогресс</h2><div class="body"><div class="row"><button id="plan" class="primary">Сформировать план</button><button id="start" class="good">▶ Запустить весь batch</button><button id="stop" class="bad">■ Остановить</button></div><div style="margin-top:14px"><div class="row" style="justify-content:space-between"><b id="msg">Готово</b><span id="pct">0%</span></div><div class="progress"><div id="overallBar" class="bar"></div></div></div><div class="metrics"><div class="metric"><b id="mDone">0</b><span>готово</span></div><div class="metric"><b id="mRun">0</b><span>в работе</span></div><div class="metric"><b id="mFail">0</b><span>ошибки</span></div><div class="metric"><b id="mTotal">0</b><span>всего запусков</span></div></div><div id="manifest" class="muted tiny"></div></div></section>
<section class="panel"><h2>Очередь полётов</h2><div class="body"><div class="tableWrap"><table><thead><tr><th>#</th><th>Группа</th><th>Источник</th><th>Исходная карта / launch</th><th>Копия</th><th>Статус / фаза</th><th>Окружение</th><th>Прогресс</th><th>output</th><th>speed</th><th>dx</th><th>dy</th><th>dz</th><th>dyaw</th><th>x/y/z amp</th><th>yaw amp</th><th>timescale</th></tr></thead><tbody id="tbody"></tbody></table></div></div></section>
<section class="panel"><h2>Live log</h2><div class="body"><div id="log" class="log"></div></div></section>
</div>
<script>
const specs={flight_speed:{label:'Скорость replay',unit:'×',d:[.90,1.10]},coord_dx:{label:'Смещение X',unit:'m',d:[-.12,.12]},coord_dy:{label:'Смещение Y',unit:'m',d:[-.12,.12]},coord_dz:{label:'Смещение Z',unit:'m',d:[-.06,.06]},coord_dyaw:{label:'Смещение yaw',unit:'deg',d:[-4,4]},coord_x_amp:{label:'Wobble X amp',unit:'m',d:[.02,.08]},coord_y_amp:{label:'Wobble Y amp',unit:'m',d:[.02,.08]},coord_z_amp:{label:'Wobble Z amp',unit:'m',d:[.01,.04]},coord_yaw_amp:{label:'Wobble yaw amp',unit:'deg',d:[.5,2.5]},coord_timescale:{label:'Wobble timescale',unit:'s',d:[3,6]}};
const $=id=>document.getElementById(id);let nextGid=1;
function escapeHtml(x){return String(x??'').replace(/[&<>'"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;',"'":'&#39;','"':'&quot;'}[c]))}
function rangeHtml(gid){return '<div class="head">Параметр</div><div class="head">min</div><div class="head">max</div><div class="head">unit</div>'+Object.entries(specs).map(([k,s])=>`<div>${s.label}</div><input data-rkey="${k}" data-edge="min" type="number" step="any" value="${s.d[0]}"><input data-rkey="${k}" data-edge="max" type="number" step="any" value="${s.d[1]}"><div class="muted">${s.unit}</div>`).join('')}
function addGroup(prefill={}){let gid=nextGid++;let d=document.createElement('div');d.className='group';d.dataset.gid=gid;d.innerHTML=`<div class="groupHead"><b>Родительская группа <span class="groupNum"></span></b><div class="row"><button class="clone ghost">Дублировать настройки</button><button class="remove bad">Удалить</button></div></div><div class="groupBody"><div><div class="sectionTitle">Источники и исходные карты</div><label>Путь к родительской папке</label><div class="row"><div class="field"><input class="parentPath" placeholder="/home/sed/.../parent_folder" value="${escapeHtml(prefill.parent_path||'')}"></div><button class="inspect">Проверить папку</button></div><div class="parentInfo sourceInfo muted">Папка ещё не проверена. Карта и все launch-параметры будут автоматически прочитаны отдельно из environment/launch_context.json каждого flight.</div></div><div><div class="sectionTitle">Batch-параметры этой папки</div><div class="row"><div class="field"><label>Копий N на каждый flight</label><input class="copies" type="number" min="1" max="10000" value="${prefill.copies??10}"></div><div class="field"><label>Seed</label><input class="seed" type="number" value="${prefill.seed??42003}"></div><div class="field"><label>Sampling</label><select class="sampling"><option value="stratified">Stratified</option><option value="uniform">Uniform random</option></select></div></div><div class="row"><div class="field"><label>Coordinate frame</label><select class="coordFrame"><option value="body">body</option><option value="world">world</option></select></div><div class="field"><label>Lighting</label><select class="lighting"><option>off</option><option>constant</option><option>day_to_sunset</option><option>day_to_night</option><option>cloud_pass</option><option>random</option></select></div></div></div><div class="full"><div class="sectionTitle">Диапазоны вариаций этой папки</div><div class="ranges rangeGrid">${rangeHtml(gid)}</div></div></div>`;document.getElementById('groups').appendChild(d);d.querySelector('.sampling').value=prefill.sampling_mode||'stratified';d.querySelector('.coordFrame').value=prefill.coord_frame||'body';d.querySelector('.lighting').value=prefill.lighting||'random';if(prefill.ranges){for(const [k,v] of Object.entries(prefill.ranges)){let a=d.querySelector(`[data-rkey="${k}"][data-edge="min"]`),b=d.querySelector(`[data-rkey="${k}"][data-edge="max"]`);if(a)a.value=v.min;if(b)b.value=v.max}}d.querySelector('.remove').onclick=()=>{if(document.querySelectorAll('.group').length<=1){alert('Нужна хотя бы одна родительская группа');return}d.remove();renumber()};d.querySelector('.clone').onclick=()=>addGroup(groupFromCard(d));d.querySelector('.inspect').onclick=()=>inspectGroup(d);renumber();return d}
function renumber(){document.querySelectorAll('.group').forEach((g,i)=>g.querySelector('.groupNum').textContent=i+1);$('parentCount').value=document.querySelectorAll('.group').length}
function groupFromCard(g){let rr={};for(const k of Object.keys(specs))rr[k]={min:+g.querySelector(`[data-rkey="${k}"][data-edge="min"]`).value,max:+g.querySelector(`[data-rkey="${k}"][data-edge="max"]`).value};return {parent_path:g.querySelector('.parentPath').value.trim(),copies:+g.querySelector('.copies').value,seed:+g.querySelector('.seed').value,sampling_mode:g.querySelector('.sampling').value,coord_frame:g.querySelector('.coordFrame').value,lighting:g.querySelector('.lighting').value,ranges:rr}}
async function inspectGroup(g){let info=g.querySelector('.parentInfo');try{info.textContent='Сканирование…';let d=await api('/api/parent-info',{parent_path:g.querySelector('.parentPath').value.trim()});let names=d.flights.map((x,i)=>{let l=x.detected_launch||{};let launch=l.valid?`${escapeHtml(l.launch_file)} · ${escapeHtml(l.command||'')}`:`<span class="fail">ОШИБКА launch metadata: ${escapeHtml(l.error||'неизвестно')}</span>`;return `${i+1}. <b>${escapeHtml(x.name)}</b> · ${Number(x.duration_sec).toFixed(2)} s<br><span class="tiny muted">${launch}</span>`}).join('<br>');info.innerHTML=`<b>${d.count} flight найдено</b><br><span class="tiny">${escapeHtml(d.parent)}</span><hr style="border:0;border-top:1px solid #263653">${names}`}catch(e){info.textContent='Ошибка: '+e.message}}
function cfg(){return {replay_script:$('replay').value.trim(),restart_simulator_each_run:$('restart').checked,stop_on_error:$('stopErr').checked,sim_ready_timeout_sec:+$('readyTimeout').value,between_runs_sec:+$('between').value,groups:[...document.querySelectorAll('.group')].map(groupFromCard)}}
async function api(path,body){let r=await fetch(path,{method:body?'POST':'GET',headers:{'Content-Type':'application/json'},body:body?JSON.stringify(body):undefined});let d=await r.json();if(!r.ok)throw new Error(d.error||r.statusText);return d}
function f(v,n=3){return Number(v).toFixed(n)}function statusClass(s){return s==='completed'?'ok':s==='failed'?'fail':s==='running'?'running':''}
function render(s){$('statusChip').textContent=s.status+(s.dry_run?' · DRY':'');$('msg').textContent=s.message||'';let pc=Math.round((s.overall_progress||0)*100);$('pct').textContent=pc+'%';$('overallBar').style.width=pc+'%';let runs=s.runs||[];$('mDone').textContent=runs.filter(x=>x.status==='completed').length;$('mRun').textContent=runs.filter(x=>x.status==='running').length;$('mFail').textContent=runs.filter(x=>x.status==='failed').length;$('mTotal').textContent=runs.length;$('manifest').textContent=s.batch_manifest?'batch manifest: '+s.batch_manifest:'';$('tbody').innerHTML=runs.map(r=>{let p=r.params||{},out=r.actual_output||r.planned_output||'';return `<tr><td>${r.index}</td><td><b>G${r.group_index}</b><br><span class="muted tiny">${escapeHtml((r.parent_path||'').split('/').pop())}</span></td><td><b>${escapeHtml(r.source_name||'')}</b><br><span class="muted tiny">#${r.source_position||''}</span></td><td title="${escapeHtml(((r.run_config||{}).launch_command)||'')}"><b>${escapeHtml((((r.run_config||{}).source_launch||{}).launch_file)||'—')}</b><br><span class="muted tiny">${escapeHtml(((((r.run_config||{}).source_launch||{}).arguments||{}).start_position!==undefined)?'start_position='+(((r.run_config||{}).source_launch||{}).arguments||{}).start_position:'source launch params preserved')}</span></td><td>${r.copy_index}/${r.copies_for_source}</td><td class="${statusClass(r.status)}"><b>${r.status}</b><br><span class="muted tiny">${escapeHtml(r.phase||'')}${r.error?'<br>'+escapeHtml(r.error):''}</span></td><td><span class="chip ${r.wall_style===4?'styleA':'styleB'}">w${r.wall_style}/f${r.floor_style}</span></td><td><span class="pmini"><i style="width:${Math.round((r.progress||0)*100)}%"></i></span> ${Math.round((r.progress||0)*100)}%</td><td title="${escapeHtml(out)}">${escapeHtml(out.split('/').pop()||out)}</td><td>${f(p.flight_speed)}</td><td>${f(p.coord_dx)}</td><td>${f(p.coord_dy)}</td><td>${f(p.coord_dz)}</td><td>${f(p.coord_dyaw,2)}</td><td>${f(p.coord_x_amp)}/${f(p.coord_y_amp)}/${f(p.coord_z_amp)}</td><td>${f(p.coord_yaw_amp,2)}</td><td>${f(p.coord_timescale,2)}</td></tr>`}).join('');let log=$('log'),near=log.scrollTop+log.clientHeight>log.scrollHeight-80;log.textContent=(s.logs||[]).join('\n');if(near)log.scrollTop=log.scrollHeight;let busy=['running','stopping'].includes(s.status);$('start').disabled=busy;$('plan').disabled=busy;$('stop').disabled=!busy;document.querySelectorAll('.remove,.clone,.inspect,#addGroup,#applyCount').forEach(x=>x.disabled=busy)}
$('addGroup').onclick=()=>addGroup();$('applyCount').onclick=()=>{let want=Math.max(1,Math.min(100,+$('parentCount').value||1)),cards=[...document.querySelectorAll('.group')];while(cards.length<want){cards.push(addGroup())}while(cards.length>want){cards.pop().remove()}renumber()};$('plan').onclick=async()=>{try{render(await api('/api/plan',cfg()))}catch(e){alert(e.message)}};$('start').onclick=async()=>{try{render(await api('/api/start',cfg()))}catch(e){alert(e.message)}};$('stop').onclick=async()=>{try{render(await api('/api/stop',{}))}catch(e){alert(e.message)}};async function poll(){try{render(await api('/api/state'))}catch(e){}finally{setTimeout(poll,650)}}addGroup();poll();
</script></body></html>'''


class Handler(BaseHTTPRequestHandler):
    server_version = "ConceptVLAFlightBatch/3.0"

    def log_message(self, fmt: str, *args: Any) -> None:
        return

    @property
    def app(self) -> "AppHTTPServer":
        return self.server  # type: ignore[return-value]

    def _json_body(self) -> dict[str, Any]:
        try:
            n = int(self.headers.get("Content-Length", "0"))
            if n > 2_000_000:
                raise ValueError("request too large")
            raw = self.rfile.read(n) if n else b"{}"
            data = json.loads(raw.decode("utf-8"))
            if not isinstance(data, dict):
                raise ValueError("JSON object expected")
            return data
        except Exception as exc:
            raise ValueError(f"Некорректный JSON: {exc}") from exc

    def _send_json(self, data: Any, status: int = 200) -> None:
        raw = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/":
            raw = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)
            return
        if path == "/api/state":
            self._send_json(self.app.state.snapshot())
            return
        self._send_json({"error": "not found"}, 404)

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        try:
            body = self._json_body()
            if path == "/api/parent-info":
                self._send_json(discover_parent(str(body.get("parent_path", ""))))
                return
            if path == "/api/source-info":
                self._send_json(source_info(str(body.get("source_flight", ""))))
                return
            if path == "/api/plan":
                cfg = validate_config(body)
                runs = build_plan(cfg)
                with self.app.state.lock:
                    if self.app.state.worker and self.app.state.worker.is_alive():
                        raise RuntimeError("Нельзя перепланировать во время выполнения")
                    self.app.state.config = cfg
                    self.app.state.runs = runs
                    self.app.state.status = "planned"
                    self.app.state.message = f"План готов: {len(runs)} полётов"
                    self.app.state.current_index = None
                    self.app.state.batch_id = None
                    self.app.state.batch_manifest = None
                self._send_json(self.app.state.snapshot())
                return
            if path == "/api/start":
                cfg = validate_config(body)
                runs = build_plan(cfg)
                self.app.runner.start(cfg, runs)
                self._send_json(self.app.state.snapshot())
                return
            if path == "/api/stop":
                self.app.runner.request_stop()
                self._send_json(self.app.state.snapshot())
                return
            self._send_json({"error": "not found"}, 404)
        except Exception as exc:
            self._send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)


class AppHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, addr: tuple[str, int], state: SharedState, runner: BatchRunner):
        super().__init__(addr, Handler)
        self.state = state
        self.runner = runner


def main() -> None:
    ap = argparse.ArgumentParser(description="Concept-VLA batch replay web UI")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8787)
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--dry-run", action="store_true", help="UI/demo mode without ros2/replay execution")
    args = ap.parse_args()

    state = SharedState(dry_run=bool(args.dry_run))
    runner = BatchRunner(state)
    server = AppHTTPServer((args.host, args.port), state, runner)
    url = f"http://{args.host}:{args.port}/"
    print(f"Flight Batch Control: {url}", flush=True)
    print(f"Output root: {OUTPUT_ROOT}", flush=True)
    if args.dry_run:
        print("DRY RUN mode enabled", flush=True)
    if not args.no_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever(poll_interval=0.4)
    except KeyboardInterrupt:
        pass
    finally:
        runner.request_stop()
        server.server_close()


if __name__ == "__main__":
    main()
