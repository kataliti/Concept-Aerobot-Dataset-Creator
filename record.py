#!/usr/bin/env python3

import csv
import json
import math
import re
import select
import shlex
import subprocess
import sys
import termios
import threading
import time
import tty
from copy import deepcopy
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


# ============================================================
# USER PARAMETERS
# ============================================================

CONCEPT_VLA_DIR = "/home/sed/Desktop/concept-vla"

FORWARD_CAMERA_TOPIC = "/uav1/camera"
BOTTOM_CAMERA_TOPIC = "/uav1/camera_down"
ODOM_TOPIC = "/uav1/mavros/local_position/pose"

GZ_MODEL_NAME = "uav1"

FORWARD_VIDEO_FPS = 30.0
BOTTOM_VIDEO_FPS = 30.0

# World trajectory stored for replay.
TRAJECTORY_HZ = 20.0

FFMPEG_CODEC = "libx264"
FFMPEG_PRESET = "veryfast"
FFMPEG_CRF = "18"

SPACE_DEBOUNCE_SEC = 0.20
REQUIRE_ODOM_BEFORE_START = True


# ============================================================
# ROS 2 LAUNCH METADATA
# ============================================================

WALL_STYLE_DESCRIPTIONS = {
    0: "нейтральные стены",
    1: "каменные стены",
    2: "стены с обоями",
    3: "деревянные стены",
    4: "коробочная / картонная текстура",
    5: "жёлтые стены",
    6: "разные текстуры по помещениям",
    7: "коллаж",
    8: "тёплые бежевые обои",
    9: "красно-коричневый кирпич",
    10: "коричневые деревянные панели",
    11: "светлая тёплая штукатурка",
    12: "серо-голубые обои",
}

FLOOR_STYLE_DESCRIPTIONS = {
    0: "нейтральный пол",
    1: "каменный пол",
    2: "пол с текстурой обоев",
    3: "деревянный пол",
    4: "коробочная / картонная текстура",
    5: "жёлтый пол",
    6: "разные текстуры по комнатам и коридору",
    7: "коллаж",
    8: "аруко-поле",
}


def _coerce_launch_value(value):
    """Keep launch values readable without losing the original text."""
    s = str(value)
    low = s.lower()

    if low == "true":
        return True
    if low == "false":
        return False

    try:
        if re.fullmatch(r"[+-]?\d+", s):
            return int(s)

        if re.fullmatch(
            r"[+-]?(?:\d+\.\d*|\d*\.\d+)(?:[eE][+-]?\d+)?",
            s,
        ):
            return float(s)

    except Exception:
        pass

    return s


def _launch_parameter_info(name, raw_value=None):
    explicit = raw_value is not None
    value = _coerce_launch_value(raw_value) if explicit else None

    info = {
        "explicit": explicit,
        "value": value,
        "raw_value": str(raw_value) if explicit else None,
    }

    if name == "wall_style":
        desc = (
            WALL_STYLE_DESCRIPTIONS.get(value)
            if isinstance(value, int)
            else None
        )

        info["description"] = (
            desc
            if desc is not None
            else (
                "launch default (argument was not explicitly passed)"
                if not explicit
                else "unknown wall_style value"
            )
        )

    elif name == "floor_style":
        desc = (
            FLOOR_STYLE_DESCRIPTIONS.get(value)
            if isinstance(value, int)
            else None
        )

        info["description"] = (
            desc
            if desc is not None
            else (
                "launch default (argument was not explicitly passed)"
                if not explicit
                else "unknown floor_style value"
            )
        )

    elif name == "start_position":
        info["description"] = (
            f"start_position {value}"
            if explicit
            else "launch default (argument was not explicitly passed)"
        )

    return info


def _parse_ros2_launch_argv(argv, pid=None, start_ticks=None):
    """Parse one /proc/<pid>/cmdline containing `ros2 launch ...`."""
    ros2_index = None

    for i, token in enumerate(argv):
        if Path(token).name == "ros2":
            if i + 1 < len(argv) and argv[i + 1] == "launch":
                ros2_index = i
                break

    if ros2_index is None:
        return None

    args = argv[ros2_index:]

    if len(args) < 4:
        return None

    package = args[2]
    launch_file = args[3]

    launch_args = {}
    passthrough = []

    for token in args[4:]:
        if ":=" in token:
            key, value = token.split(":=", 1)

            if key:
                launch_args[key] = value
                continue

        passthrough.append(token)

    normalized_argv = ["ros2"] + args[1:]

    parsed = {
        "pid": int(pid) if pid is not None else None,
        "process_start_ticks": (
            int(start_ticks)
            if start_ticks is not None
            else None
        ),
        "package": package,
        "launch_file": launch_file,
        "command": shlex.join(normalized_argv),
        "argv": normalized_argv,
        "arguments": {
            key: {
                "value": _coerce_launch_value(value),
                "raw_value": value,
            }
            for key, value in launch_args.items()
        },
        "passthrough_args": passthrough,
    }

    parsed["dataset_parameters"] = {
        "wall_style": _launch_parameter_info(
            "wall_style",
            launch_args.get("wall_style"),
        ),
        "floor_style": _launch_parameter_info(
            "floor_style",
            launch_args.get("floor_style"),
        ),
        "start_position": _launch_parameter_info(
            "start_position",
            launch_args.get("start_position"),
        ),
    }

    return parsed


def detect_running_ros2_launch():
    """Capture active `ros2 launch` command(s) from /proc at script startup.

    Prefer an aerobot_gz_sim launch if several launch processes are active;
    otherwise use the newest detected launch process. All detected launch
    processes are retained in the JSON for diagnostics.
    """
    found = []
    proc_root = Path("/proc")

    try:
        proc_entries = list(proc_root.iterdir())
    except Exception:
        proc_entries = []

    for proc in proc_entries:
        if not proc.name.isdigit():
            continue

        try:
            raw = (proc / "cmdline").read_bytes()

            if not raw:
                continue

            argv = [
                x.decode("utf-8", errors="replace")
                for x in raw.split(b"\0")
                if x
            ]

            stat_fields = (
                proc / "stat"
            ).read_text(errors="replace").split()

            start_ticks = (
                int(stat_fields[21])
                if len(stat_fields) > 21
                else 0
            )

            parsed = _parse_ros2_launch_argv(
                argv,
                pid=int(proc.name),
                start_ticks=start_ticks,
            )

            if parsed is not None:
                found.append(parsed)

        except (
            FileNotFoundError,
            ProcessLookupError,
            PermissionError,
            OSError,
            ValueError,
        ):
            continue

    # De-duplicate identical command lines.
    unique = {}

    for item in found:
        key = item["command"]
        old = unique.get(key)

        if (
            old is None
            or (item.get("process_start_ticks") or 0)
            > (old.get("process_start_ticks") or 0)
        ):
            unique[key] = item

    found = list(unique.values())

    preferred = [
        x
        for x in found
        if x.get("package") == "aerobot_gz_sim"
    ]

    pool = preferred if preferred else found

    selected = (
        max(
            pool,
            key=lambda x: x.get("process_start_ticks") or 0,
        )
        if pool
        else None
    )

    captured_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")

    return {
        "schema_version": 1,
        "captured_at_script_start": captured_at,
        "detected": selected is not None,
        "selected": selected,
        "all_detected_launches": found,
        "selection_rule": (
            "newest aerobot_gz_sim ros2 launch process; "
            "otherwise newest ros2 launch process"
        ),
        "note": (
            "Only launch arguments explicitly present in the running "
            "process can be known reliably. Missing arguments are marked "
            "as launch defaults rather than guessed."
        ),
    }


def save_launch_metadata(output, launch_context):
    """Write launch metadata into one recorded dataset directory."""
    env_dir = Path(output) / "environment"
    env_dir.mkdir(parents=True, exist_ok=True)

    context = launch_context or {
        "schema_version": 1,
        "captured_at_script_start": None,
        "detected": False,
        "selected": None,
        "all_detected_launches": [],
        "note": "No launch context was supplied to the recorder.",
    }

    json_path = env_dir / "launch_context.json"

    json_path.write_text(
        json.dumps(
            context,
            indent=2,
            ensure_ascii=False,
        )
        + "\n",
        encoding="utf-8",
    )

    txt_path = env_dir / "launch_command.txt"

    selected = (
        context.get("selected")
        if isinstance(context, dict)
        else None
    )

    if selected:
        params = selected.get("dataset_parameters", {})

        lines = [
            selected.get("command", ""),
            "",
            f"package: {selected.get('package')}",
            f"launch_file: {selected.get('launch_file')}",
            "",
            "dataset parameters:",
        ]

        for name in (
            "wall_style",
            "floor_style",
            "start_position",
        ):
            info = params.get(name, {})
            value = info.get("value")
            desc = info.get("description", "")

            if info.get("explicit"):
                lines.append(
                    f"  {name}:={value}  # {desc}"
                )
            else:
                lines.append(
                    f"  {name}: <launch default>  # {desc}"
                )

        extra = selected.get("arguments", {})
        known = {
            "wall_style",
            "floor_style",
            "start_position",
        }

        extra_names = [
            k
            for k in extra
            if k not in known
        ]

        if extra_names:
            lines.extend([
                "",
                "other explicit launch arguments:",
            ])

            for name in sorted(extra_names):
                lines.append(
                    f"  {name}:="
                    f"{extra[name].get('raw_value')}"
                )

        txt = "\n".join(lines).rstrip() + "\n"

    else:
        txt = (
            "No active `ros2 launch ...` process was detected "
            "when record.py started.\n"
            "See launch_context.json for details.\n"
        )

    txt_path.write_text(
        txt,
        encoding="utf-8",
    )

    print(
        f"[ENV] launch metadata: {json_path}",
        flush=True,
    )

    return json_path


# ============================================================


def f4(value: float) -> str:
    return f"{float(value):.4f}"


def f6(value: float) -> str:
    return f"{float(value):.6f}"


def normalize_angle_deg(value: float) -> float:
    return (float(value) + 180.0) % 360.0 - 180.0


def quaternion_to_rpy_deg(x: float, y: float, z: float, w: float):
    # roll
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # yaw
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (
        math.degrees(roll),
        math.degrees(pitch),
        math.degrees(yaw),
    )


def mavros_pose_dict(msg: PoseStamped):
    p = msg.pose.position
    q = msg.pose.orientation
    roll, pitch, yaw = quaternion_to_rpy_deg(
        q.x, q.y, q.z, q.w
    )

    return {
        "x": float(p.x),
        "y": float(p.y),
        "z": float(p.z),
        "roll_deg": roll,
        "pitch_deg": pitch,
        "yaw_deg": yaw,
    }


def get_gazebo_model_pose(model_name: str):
    cmd = [
        "gz",
        "model",
        "-m",
        model_name,
        "--pose",
    ]

    try:
        result = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=5.0,
        )
    except FileNotFoundError:
        raise RuntimeError("`gz` command not found")
    except subprocess.TimeoutExpired:
        raise RuntimeError("Timed out while reading Gazebo model pose")

    if result.returncode != 0:
        raise RuntimeError(
            "Could not read Gazebo model pose:\n"
            + result.stdout
        )

    text = result.stdout
    pose_pos = text.find("Pose")

    if pose_pos >= 0:
        text = text[pose_pos:]

    number = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"

    # Accept both:
    # [1.0 2.0 3.0]
    # [1.0 | 2.0 | 3.0]
    triples = re.findall(
        rf"\[\s*({number})\s*(?:\|\s*)?"
        rf"({number})\s*(?:\|\s*)?"
        rf"({number})\s*\]",
        text,
    )

    if len(triples) < 2:
        raise RuntimeError(
            "Could not parse `gz model --pose` output:\n"
            + result.stdout
        )

    xyz = [float(v) for v in triples[0]]
    rpy = [float(v) for v in triples[1]]

    return {
        "x": xyz[0],
        "y": xyz[1],
        "z": xyz[2],
        "roll_deg": math.degrees(rpy[0]),
        "pitch_deg": math.degrees(rpy[1]),
        "yaw_deg": math.degrees(rpy[2]),
    }


def local_to_world_pose(
    current_local: dict,
    initial_local: dict,
    initial_world: dict,
):
    """
    Reconstruct Gazebo-world trajectory from MAVROS local motion.

    At the recording start we know both frames exactly. We assume their
    relationship is a rigid yaw + translation transform for the duration
    of one flight.
    """
    theta_deg = normalize_angle_deg(
        initial_world["yaw_deg"]
        - initial_local["yaw_deg"]
    )
    theta = math.radians(theta_deg)

    dlx = current_local["x"] - initial_local["x"]
    dly = current_local["y"] - initial_local["y"]
    dlz = current_local["z"] - initial_local["z"]

    dwx = math.cos(theta) * dlx - math.sin(theta) * dly
    dwy = math.sin(theta) * dlx + math.cos(theta) * dly

    return {
        "x": initial_world["x"] + dwx,
        "y": initial_world["y"] + dwy,
        "z": initial_world["z"] + dlz,
        "roll_deg": current_local["roll_deg"],
        "pitch_deg": current_local["pitch_deg"],
        "yaw_deg": normalize_angle_deg(
            current_local["yaw_deg"] + theta_deg
        ),
    }


def image_encoding_to_ffmpeg(encoding: str):
    table = {
        "rgb8": ("rgb24", 3),
        "bgr8": ("bgr24", 3),
        "rgba8": ("rgba", 4),
        "bgra8": ("bgra", 4),
        "mono8": ("gray", 1),
        "8UC1": ("gray", 1),
        "8UC3": ("bgr24", 3),
        "8UC4": ("bgra", 4),
    }

    if encoding not in table:
        raise RuntimeError(
            f"Unsupported Image encoding: {encoding}"
        )

    return table[encoding]


def image_bytes_without_padding(msg: Image, bytes_per_pixel: int):
    row_size = int(msg.width) * bytes_per_pixel
    step = int(msg.step)
    raw = bytes(msg.data)

    if step == row_size:
        return raw

    if step < row_size:
        raise RuntimeError(
            f"Invalid Image.step={step}, expected >= {row_size}"
        )

    out = bytearray(row_size * int(msg.height))
    dst = 0

    for row in range(int(msg.height)):
        src = row * step
        out[dst:dst + row_size] = raw[src:src + row_size]
        dst += row_size

    return bytes(out)


class FFmpegCameraRecorder:
    def __init__(
        self,
        name,
        output_dir,
        fps,
        start_monotonic_ns,
    ):
        self.name = name
        self.output_dir = output_dir
        self.fps = fps
        self.start_monotonic_ns = start_monotonic_ns

        output_dir.mkdir(parents=True, exist_ok=True)

        self.video_path = output_dir / "video.mp4"
        self.timestamps_path = output_dir / "timestamps.csv"
        self.log_path = output_dir / "ffmpeg.log"

        self.proc = None
        self.log_file = None
        self.frame_index = 0

        self.width = None
        self.height = None
        self.encoding = None
        self.pix_fmt = None
        self.bytes_per_pixel = None

        self.csv_file = open(
            self.timestamps_path,
            "w",
            newline="",
            buffering=1,
        )
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "frame_index",
            "timestamp_start",
        ])

    def start_ffmpeg(self, msg):
        self.width = int(msg.width)
        self.height = int(msg.height)
        self.encoding = msg.encoding

        self.pix_fmt, self.bytes_per_pixel = image_encoding_to_ffmpeg(
            self.encoding
        )

        self.log_file = open(self.log_path, "w")

        cmd = [
            "ffmpeg",
            "-y",
            "-loglevel", "warning",
            "-f", "rawvideo",
            "-pixel_format", self.pix_fmt,
            "-video_size", f"{self.width}x{self.height}",
            "-framerate", str(self.fps),
            "-i", "-",
            "-an",
            "-c:v", FFMPEG_CODEC,
            "-preset", FFMPEG_PRESET,
            "-crf", FFMPEG_CRF,
            "-pix_fmt", "yuv420p",
            "-movflags", "+faststart",
            str(self.video_path),
        ]

        self.proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=self.log_file,
            stderr=subprocess.STDOUT,
            bufsize=0,
        )

    def process(self, msg, receipt_ns):
        if self.proc is None:
            self.start_ffmpeg(msg)

        if (
            int(msg.width) != self.width
            or int(msg.height) != self.height
            or msg.encoding != self.encoding
        ):
            return

        t = max(
            0.0,
            (receipt_ns - self.start_monotonic_ns) / 1e9,
        )

        try:
            self.proc.stdin.write(
                image_bytes_without_padding(
                    msg,
                    self.bytes_per_pixel,
                )
            )
        except Exception:
            return

        self.csv_writer.writerow([
            self.frame_index,
            f4(t),
        ])
        self.frame_index += 1

    def close(self):
        if self.proc is not None:
            try:
                self.proc.stdin.close()
            except Exception:
                pass

            try:
                self.proc.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                try:
                    self.proc.terminate()
                    self.proc.wait(timeout=3.0)
                except Exception:
                    try:
                        self.proc.kill()
                    except Exception:
                        pass

            self.proc = None

        if self.log_file is not None:
            self.log_file.close()
            self.log_file = None

        if not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()


class FlightSession:
    def __init__(
        self,
        flight_dir,
        initial_local_msg,
        initial_world,
        start_ns,
    ):
        self.flight_dir = flight_dir
        self.start_ns = start_ns

        self.lock = threading.RLock()
        self.closed = False

        self.forward_dir = flight_dir / "forward"
        self.bottom_dir = flight_dir / "bottom"
        self.odom_dir = flight_dir / "odom"
        self.trajectory_dir = flight_dir / "trajectory"

        for p in (
            self.forward_dir,
            self.bottom_dir,
            self.odom_dir,
            self.trajectory_dir,
        ):
            p.mkdir(parents=True, exist_ok=True)

        self.forward = FFmpegCameraRecorder(
            "forward",
            self.forward_dir,
            FORWARD_VIDEO_FPS,
            start_ns,
        )

        self.bottom = FFmpegCameraRecorder(
            "bottom",
            self.bottom_dir,
            BOTTOM_VIDEO_FPS,
            start_ns,
        )

        self.initial_local = mavros_pose_dict(
            initial_local_msg
        )
        self.initial_world = dict(initial_world)

        self.odom_file = open(
            self.odom_dir / "odom.csv",
            "w",
            newline="",
            buffering=1,
        )
        self.odom_writer = csv.writer(self.odom_file)
        self.odom_writer.writerow([
            "sample_index",
            "timestamp_start",
            "x",
            "y",
            "z",
            "yaw_deg",
        ])
        self.odom_index = 0

        self.trajectory_file = open(
            self.trajectory_dir / "trajectory.csv",
            "w",
            newline="",
            buffering=1,
        )
        self.trajectory_writer = csv.writer(
            self.trajectory_file
        )
        self.trajectory_writer.writerow([
            "sample_index",
            "timestamp_start",
            "world_x",
            "world_y",
            "world_z",
            "world_roll_deg",
            "world_pitch_deg",
            "world_yaw_deg",
        ])

        self.trajectory_index = 0
        self.last_trajectory_t = 0.0
        self.trajectory_period = 1.0 / TRAJECTORY_HZ

        # Explicit t=0 world pose.
        self.trajectory_writer.writerow([
            0,
            "0.0000",
            f6(self.initial_world["x"]),
            f6(self.initial_world["y"]),
            f6(self.initial_world["z"]),
            f4(self.initial_world["roll_deg"]),
            f4(self.initial_world["pitch_deg"]),
            f4(self.initial_world["yaw_deg"]),
        ])
        self.trajectory_index = 1

        initial_json = {
            "mavros_local": {
                k: round(float(v), 6)
                for k, v in self.initial_local.items()
            },
            "gazebo_world": {
                k: round(float(v), 6)
                for k, v in self.initial_world.items()
            },
            "gazebo_model_name": GZ_MODEL_NAME,
            "trajectory_frame": "gazebo_world",
        }

        with open(
            self.trajectory_dir / "initial_state.json",
            "w",
        ) as f:
            json.dump(initial_json, f, indent=2)

    def process_forward(self, msg, receipt_ns):
        with self.lock:
            if not self.closed:
                self.forward.process(msg, receipt_ns)

    def process_bottom(self, msg, receipt_ns):
        with self.lock:
            if not self.closed:
                self.bottom.process(msg, receipt_ns)

    def process_odom(self, msg, receipt_ns):
        with self.lock:
            if self.closed:
                return

            t = max(
                0.0,
                (receipt_ns - self.start_ns) / 1e9,
            )

            local = mavros_pose_dict(msg)

            self.odom_writer.writerow([
                self.odom_index,
                f4(t),
                f4(local["x"]),
                f4(local["y"]),
                f4(local["z"]),
                f4(local["yaw_deg"]),
            ])
            self.odom_index += 1

            if (
                t - self.last_trajectory_t
                >= self.trajectory_period
            ):
                world = local_to_world_pose(
                    local,
                    self.initial_local,
                    self.initial_world,
                )

                self.trajectory_writer.writerow([
                    self.trajectory_index,
                    f4(t),
                    f6(world["x"]),
                    f6(world["y"]),
                    f6(world["z"]),
                    f4(world["roll_deg"]),
                    f4(world["pitch_deg"]),
                    f4(world["yaw_deg"]),
                ])

                self.trajectory_index += 1
                self.last_trajectory_t = t

    def close(self):
        with self.lock:
            if self.closed:
                return

            self.closed = True

            self.forward.close()
            self.bottom.close()

            for f in (
                self.odom_file,
                self.trajectory_file,
            ):
                if not f.closed:
                    f.flush()
                    f.close()

            summary = {
                "trajectory_hz": TRAJECTORY_HZ,
                "trajectory_frame": "gazebo_world",
                "trajectory_samples": self.trajectory_index,
                "odom_samples": self.odom_index,
                "forward_frames": self.forward.frame_index,
                "bottom_frames": self.bottom.frame_index,
            }

            with open(
                self.trajectory_dir / "summary.json",
                "w",
            ) as f:
                json.dump(summary, f, indent=2)


class KeyboardThread(threading.Thread):
    def __init__(self, toggle_event, quit_event):
        super().__init__(daemon=True)
        self.toggle_event = toggle_event
        self.quit_event = quit_event
        self.stop_event = threading.Event()
        self.last_space = -1e9

    def run(self):
        if not sys.stdin.isatty():
            print("ERROR: stdin is not a terminal")
            return

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)

        try:
            tty.setcbreak(fd)

            while not self.stop_event.is_set():
                ready, _, _ = select.select(
                    [sys.stdin], [], [], 0.05
                )

                if not ready:
                    continue

                ch = sys.stdin.read(1)

                if ch == " ":
                    now = time.monotonic()

                    if now - self.last_space >= SPACE_DEBOUNCE_SEC:
                        self.last_space = now
                        print("[KEY] SPACE", flush=True)
                        self.toggle_event.set()

                elif ch.lower() == "q":
                    print("[KEY] q", flush=True)
                    self.quit_event.set()

        finally:
            termios.tcsetattr(
                fd,
                termios.TCSADRAIN,
                old,
            )

    def stop(self):
        self.stop_event.set()


class FlightRecorder(Node):
    def __init__(self, launch_context=None):
        super().__init__("concept_vla_raw_recorder")

        self.launch_context = launch_context

        self.state_lock = threading.RLock()

        self.latest_odom = None
        self.session = None

        self.toggle_event = threading.Event()
        self.quit_event = threading.Event()
        self.finalizers = []

        self.keyboard = KeyboardThread(
            self.toggle_event,
            self.quit_event,
        )
        self.keyboard.start()

        self.control_thread = threading.Thread(
            target=self.control_loop,
            daemon=True,
        )
        self.control_thread.start()

        self.forward_sub = self.create_subscription(
            Image,
            FORWARD_CAMERA_TOPIC,
            self.forward_callback,
            qos_profile_sensor_data,
        )

        self.bottom_sub = self.create_subscription(
            Image,
            BOTTOM_CAMERA_TOPIC,
            self.bottom_callback,
            qos_profile_sensor_data,
        )

        self.odom_sub = self.create_subscription(
            PoseStamped,
            ODOM_TOPIC,
            self.odom_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "READY. SPACE = start/stop, q = quit"
        )

    @property
    def is_recording(self):
        with self.state_lock:
            return self.session is not None

    def create_flight_dir(self):
        raw = (
            Path(CONCEPT_VLA_DIR)
            / "datasets"
            / "raw"
        )
        raw.mkdir(parents=True, exist_ok=True)

        while True:
            candidate = raw / datetime.now().strftime(
                "flight-%Y%m%d-%H%M%S"
            )

            if not candidate.exists():
                return candidate

            time.sleep(0.02)

    def start_recording(self):
        with self.state_lock:
            if self.session is not None:
                return

            if (
                REQUIRE_ODOM_BEFORE_START
                and self.latest_odom is None
            ):
                self.get_logger().warning(
                    "Cannot start: no odometry yet"
                )
                return

        print(
            f"[START] reading Gazebo world pose for {GZ_MODEL_NAME}...",
            flush=True,
        )

        try:
            world_pose = get_gazebo_model_pose(
                GZ_MODEL_NAME
            )
        except Exception as exc:
            print(
                f"[START ERROR] {exc}",
                flush=True,
            )
            return

        with self.state_lock:
            if self.session is not None:
                return

            if self.latest_odom is None:
                return

            initial_local = deepcopy(
                self.latest_odom
            )

            flight_dir = self.create_flight_dir()
            start_ns = time.monotonic_ns()

            self.session = FlightSession(
                flight_dir,
                initial_local,
                world_pose,
                start_ns,
            )

            try:
                save_launch_metadata(
                    flight_dir,
                    self.launch_context,
                )
            except Exception as exc:
                print(
                    f"[ENV WARNING] could not write launch metadata: {exc}",
                    flush=True,
                )

        print(
            "[START] Gazebo world: "
            f"x={world_pose['x']:.3f} "
            f"y={world_pose['y']:.3f} "
            f"z={world_pose['z']:.3f} "
            f"yaw={world_pose['yaw_deg']:.2f}",
            flush=True,
        )

        self.get_logger().info(
            f"RECORDING STARTED: {flight_dir}"
        )

    def stop_recording(self):
        with self.state_lock:
            session = self.session

            if session is None:
                return

            self.session = None

        self.get_logger().info(
            f"RECORDING STOPPED: {session.flight_dir}"
        )

        def finalize():
            try:
                session.close()
                print(
                    f"[FINALIZED] {session.flight_dir}",
                    flush=True,
                )
            except Exception as exc:
                print(
                    f"[FINALIZE ERROR] {exc}",
                    flush=True,
                )

        thread = threading.Thread(
            target=finalize,
            daemon=False,
        )
        self.finalizers.append(thread)
        thread.start()

    def control_loop(self):
        while rclpy.ok():
            if self.quit_event.is_set():
                self.quit_event.clear()

                if self.is_recording:
                    self.stop_recording()

                if rclpy.ok():
                    rclpy.shutdown()

                return

            if self.toggle_event.wait(timeout=0.02):
                self.toggle_event.clear()

                if self.is_recording:
                    self.stop_recording()
                else:
                    self.start_recording()

    def forward_callback(self, msg):
        receipt_ns = time.monotonic_ns()

        with self.state_lock:
            session = self.session

        if session is not None:
            session.process_forward(
                msg,
                receipt_ns,
            )

    def bottom_callback(self, msg):
        receipt_ns = time.monotonic_ns()

        with self.state_lock:
            session = self.session

        if session is not None:
            session.process_bottom(
                msg,
                receipt_ns,
            )

    def odom_callback(self, msg):
        receipt_ns = time.monotonic_ns()

        with self.state_lock:
            self.latest_odom = msg
            session = self.session

        if session is not None:
            session.process_odom(
                msg,
                receipt_ns,
            )

    def close(self):
        if self.is_recording:
            self.stop_recording()

        self.keyboard.stop()

        for thread in self.finalizers:
            if thread.is_alive():
                thread.join()


def main(args=None):
    # Capture simulator/map launch context once when the recorder starts.
    # The same snapshot is stored in every flight recorded during this run.
    launch_context = detect_running_ros2_launch()

    if launch_context.get("selected"):
        selected_launch = launch_context["selected"]

        print(
            f"[ENV] detected launch: "
            f"{selected_launch['package']} "
            f"{selected_launch['launch_file']}",
            flush=True,
        )

        print(
            f"[ENV] command: "
            f"{selected_launch['command']}",
            flush=True,
        )

    else:
        print(
            "[ENV WARNING] no active `ros2 launch ...` "
            "process detected at script startup",
            flush=True,
        )

    rclpy.init(args=args)
    node = FlightRecorder(
        launch_context=launch_context,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()


