#!/usr/bin/env python3

import argparse
import csv
import json
import math
import os
import random
import re
import shlex
import sys
import shutil
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
try:
    from mavros_msgs.srv import CommandLong
except ImportError:
    CommandLong = None
from rclpy.executors import MultiThreadedExecutor
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
MAVROS_STATE_TOPIC = "/uav1/mavros/state"

SETPOINT_TOPIC = "/uav1/mavros/setpoint_position/local"
ARM_SERVICE = "/uav1/mavros/cmd/arming"
SET_MODE_SERVICE = "/uav1/mavros/set_mode"
COMMAND_LONG_SERVICE = "/uav1/mavros/cmd/command"

GZ_WORLD_NAME = "default"
GZ_MODEL_NAME = "uav1"

FORWARD_VIDEO_FPS = 30.0
BOTTOM_VIDEO_FPS = 30.0

# Exact Gazebo pose replay rate.
POSE_REPLAY_HZ = 100.0

# PX4 hold setpoint stream. PX4 remains armed/offboard, but Gazebo pose
# is the authority for the replay trajectory.
SETPOINT_HZ = 30.0

PRE_OFFBOARD_STREAM_SEC = 2.0
ARM_OFFBOARD_TIMEOUT_SEC = 12.0

# Keep the start world pose fixed before route playback.
START_PIN_SEC = 1.5

# Keep the final world pose fixed before finishing.
FINAL_PIN_SEC = 1.0

FFMPEG_CODEC = "libx264"
FFMPEG_PRESET = "veryfast"
FFMPEG_CRF = "18"

# ============================================================


def f4(value):
    return f"{float(value):.4f}"


def f6(value):
    return f"{float(value):.6f}"


def normalize_angle_deg(value):
    return (float(value) + 180.0) % 360.0 - 180.0


def quaternion_to_yaw_deg(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(
        math.atan2(siny_cosp, cosy_cosp)
    )


def rpy_deg_to_quaternion(
    roll_deg,
    pitch_deg,
    yaw_deg,
):
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def pose_dict(msg):
    p = msg.pose.position
    q = msg.pose.orientation

    return {
        "x": float(p.x),
        "y": float(p.y),
        "z": float(p.z),
        "yaw_deg": quaternion_to_yaw_deg(
            q.x, q.y, q.z, q.w
        ),
    }


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
    if low == 'true':
        return True
    if low == 'false':
        return False
    try:
        if re.fullmatch(r'[+-]?\d+', s):
            return int(s)
        if re.fullmatch(r'[+-]?(?:\d+\.\d*|\d*\.\d+)(?:[eE][+-]?\d+)?', s):
            return float(s)
    except Exception:
        pass
    return s


def _launch_parameter_info(name, raw_value=None):
    explicit = raw_value is not None
    value = _coerce_launch_value(raw_value) if explicit else None
    info = {
        'explicit': explicit,
        'value': value,
        'raw_value': str(raw_value) if explicit else None,
    }

    if name == 'wall_style':
        desc = WALL_STYLE_DESCRIPTIONS.get(value) if isinstance(value, int) else None
        info['description'] = desc if desc is not None else (
            'launch default (argument was not explicitly passed)' if not explicit
            else 'unknown wall_style value'
        )
    elif name == 'floor_style':
        desc = FLOOR_STYLE_DESCRIPTIONS.get(value) if isinstance(value, int) else None
        info['description'] = desc if desc is not None else (
            'launch default (argument was not explicitly passed)' if not explicit
            else 'unknown floor_style value'
        )
    elif name == 'start_position':
        info['description'] = (
            f'start_position {value}' if explicit
            else 'launch default (argument was not explicitly passed)'
        )
    return info


def _parse_ros2_launch_argv(argv, pid=None, start_ticks=None):
    """Parse one /proc/<pid>/cmdline containing `ros2 launch ...`."""
    ros2_index = None
    for i, token in enumerate(argv):
        if Path(token).name == 'ros2':
            if i + 1 < len(argv) and argv[i + 1] == 'launch':
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
        if ':=' in token:
            key, value = token.split(':=', 1)
            if key:
                launch_args[key] = value
                continue
        passthrough.append(token)

    normalized_argv = ['ros2'] + args[1:]
    parsed = {
        'pid': int(pid) if pid is not None else None,
        'process_start_ticks': int(start_ticks) if start_ticks is not None else None,
        'package': package,
        'launch_file': launch_file,
        'command': shlex.join(normalized_argv),
        'argv': normalized_argv,
        'arguments': {
            key: {
                'value': _coerce_launch_value(value),
                'raw_value': value,
            }
            for key, value in launch_args.items()
        },
        'passthrough_args': passthrough,
    }

    # Always expose the three dataset-relevant launch args.  If they were not
    # passed explicitly we say so rather than guessing the launch-file default.
    parsed['dataset_parameters'] = {
        'wall_style': _launch_parameter_info('wall_style', launch_args.get('wall_style')),
        'floor_style': _launch_parameter_info('floor_style', launch_args.get('floor_style')),
        'start_position': _launch_parameter_info('start_position', launch_args.get('start_position')),
    }
    return parsed


def detect_running_ros2_launch():
    """Capture active `ros2 launch` command(s) from /proc at script startup.

    Prefer an aerobot_gz_sim launch if several launch processes are active;
    otherwise use the newest detected launch process.  All detected launch
    processes are retained in the JSON for diagnostics.
    """
    found = []
    proc_root = Path('/proc')
    try:
        proc_entries = list(proc_root.iterdir())
    except Exception:
        proc_entries = []

    for proc in proc_entries:
        if not proc.name.isdigit():
            continue
        try:
            raw = (proc / 'cmdline').read_bytes()
            if not raw:
                continue
            argv = [x.decode('utf-8', errors='replace') for x in raw.split(b'\0') if x]
            stat_fields = (proc / 'stat').read_text(errors='replace').split()
            start_ticks = int(stat_fields[21]) if len(stat_fields) > 21 else 0
            parsed = _parse_ros2_launch_argv(argv, pid=int(proc.name), start_ticks=start_ticks)
            if parsed is not None:
                found.append(parsed)
        except (FileNotFoundError, ProcessLookupError, PermissionError, OSError, ValueError):
            continue

    # De-duplicate identical command lines (rare wrappers can expose the same command).
    unique = {}
    for item in found:
        key = item['command']
        old = unique.get(key)
        if old is None or (item.get('process_start_ticks') or 0) > (old.get('process_start_ticks') or 0):
            unique[key] = item
    found = list(unique.values())

    preferred = [x for x in found if x.get('package') == 'aerobot_gz_sim']
    pool = preferred if preferred else found
    selected = max(pool, key=lambda x: x.get('process_start_ticks') or 0) if pool else None

    captured_at = time.strftime('%Y-%m-%dT%H:%M:%S%z')
    result = {
        'schema_version': 1,
        'captured_at_script_start': captured_at,
        'detected': selected is not None,
        'selected': selected,
        'all_detected_launches': found,
        'selection_rule': (
            'newest aerobot_gz_sim ros2 launch process; otherwise newest ros2 launch process'
        ),
        'note': (
            'Only launch arguments explicitly present in the running process can be known reliably. '
            'Missing arguments are marked as launch defaults rather than guessed.'
        ),
    }
    return result


def save_launch_metadata(output, launch_context):
    """Write launch metadata into one recorded dataset directory."""
    env_dir = Path(output) / 'environment'
    env_dir.mkdir(parents=True, exist_ok=True)

    context = launch_context or {
        'schema_version': 1,
        'captured_at_script_start': None,
        'detected': False,
        'selected': None,
        'all_detected_launches': [],
        'note': 'No launch context was supplied to the recorder.',
    }

    json_path = env_dir / 'launch_context.json'
    json_path.write_text(
        json.dumps(context, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8',
    )

    txt_path = env_dir / 'launch_command.txt'
    selected = context.get('selected') if isinstance(context, dict) else None
    if selected:
        params = selected.get('dataset_parameters', {})
        lines = [
            selected.get('command', ''),
            '',
            f"package: {selected.get('package')}",
            f"launch_file: {selected.get('launch_file')}",
            '',
            'dataset parameters:',
        ]
        for name in ('wall_style', 'floor_style', 'start_position'):
            info = params.get(name, {})
            value = info.get('value')
            desc = info.get('description', '')
            if info.get('explicit'):
                lines.append(f'  {name}:={value}  # {desc}')
            else:
                lines.append(f'  {name}: <launch default>  # {desc}')
        extra = selected.get('arguments', {})
        known = {'wall_style', 'floor_style', 'start_position'}
        extra_names = [k for k in extra if k not in known]
        if extra_names:
            lines.extend(['', 'other explicit launch arguments:'])
            for name in sorted(extra_names):
                lines.append(f"  {name}:={extra[name].get('raw_value')}")
        txt = '\n'.join(lines).rstrip() + '\n'
    else:
        txt = (
            'No active `ros2 launch ...` process was detected when replay_speed.py started.\n'
            'See launch_context.json for details.\n'
        )
    txt_path.write_text(txt, encoding='utf-8')

    print(f'[ENV] launch metadata: {json_path}', flush=True)
    return json_path



class GazeboPoseClient:
    """
    Direct Gazebo Transport client for:
        /world/<world>/set_pose
        request:  gz.msgs.Pose
        response: gz.msgs.Boolean

    We try several Gazebo Python ABI combinations because ROS / Gazebo
    distributions ship different transport / msgs version numbers.
    """

    def __init__(self, world_name: str, model_name: str):
        import importlib

        self.service = f"/world/{world_name}/set_pose"
        self.model_name = model_name

        candidates = [
            # Gazebo Harmonic-style packages
            ("gz.transport13", "gz.msgs10.pose_pb2", "gz.msgs10.boolean_pb2"),
            # Nearby releases
            ("gz.transport14", "gz.msgs11.pose_pb2", "gz.msgs11.boolean_pb2"),
            ("gz.transport12", "gz.msgs9.pose_pb2", "gz.msgs9.boolean_pb2"),
            ("gz.transport11", "gz.msgs8.pose_pb2", "gz.msgs8.boolean_pb2"),
            ("gz.transport10", "gz.msgs7.pose_pb2", "gz.msgs7.boolean_pb2"),
            # Some installations expose unversioned compatibility modules.
            ("gz.transport", "gz.msgs.pose_pb2", "gz.msgs.boolean_pb2"),
        ]

        errors = []

        for transport_mod, pose_mod, bool_mod in candidates:
            try:
                transport = importlib.import_module(transport_mod)
                pose_module = importlib.import_module(pose_mod)
                bool_module = importlib.import_module(bool_mod)

                self.Node = getattr(transport, "Node")
                self.Pose = getattr(pose_module, "Pose")
                self.Boolean = getattr(bool_module, "Boolean")

                self.node = self.Node()
                self.backend = (
                    f"{transport_mod} + "
                    f"{pose_mod.rsplit('.', 1)[0]}"
                )

                print(
                    f"[REPLAY] direct Gazebo Transport: {self.backend}",
                    flush=True,
                )
                return

            except Exception as exc:
                errors.append(
                    f"{transport_mod}: {type(exc).__name__}: {exc}"
                )

        raise RuntimeError(
            "Gazebo Python Transport bindings are not available.\n"
            "Tried:\n  - "
            + "\n  - ".join(errors)
            + "\nInstall the Python Gazebo Transport package matching "
              "your Gazebo version (for Harmonic this is usually "
              "`sudo apt install python3-gz-transport13`)."
        )

    def set_pose(self, pose: dict, timeout_ms: int = 200):
        request = self.Pose()
        request.name = self.model_name

        request.position.x = float(pose["x"])
        request.position.y = float(pose["y"])
        request.position.z = float(pose["z"])

        qx, qy, qz, qw = rpy_deg_to_quaternion(
            pose["roll"],
            pose["pitch"],
            pose["yaw"],
        )

        request.orientation.x = qx
        request.orientation.y = qy
        request.orientation.z = qz
        request.orientation.w = qw

        try:
            executed, response = self.node.request(
                self.service,
                request,
                self.Pose,
                self.Boolean,
                timeout_ms,
            )
        except TypeError as exc:
            raise RuntimeError(
                "Gazebo Transport Python request() API does not match "
                "the expected service requester signature. "
                f"Backend: {self.backend}. Error: {exc}"
            )

        if not executed:
            raise RuntimeError(
                f"Gazebo service timed out: {self.service}"
            )

        # gz.msgs.Boolean has a `data` field.
        if hasattr(response, "data") and not bool(response.data):
            raise RuntimeError(
                f"Gazebo set_pose rejected pose for {self.model_name}"
            )

# ============================================================
# DYNAMIC LIGHTING / SKY-SPHERE AUGMENTATION
# ============================================================


def clamp(value, lo=0.0, hi=1.0):
    return max(lo, min(hi, float(value)))


def normalize_vec3(v):
    x, y, z = (float(v[0]), float(v[1]), float(v[2]))
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-9:
        return (-0.5, 0.1, -0.9)
    return (x / n, y / n, z / n)


def lerp(a, b, u):
    return float(a) + (float(b) - float(a)) * float(u)


def lerp3(a, b, u):
    return tuple(lerp(a[i], b[i], u) for i in range(3))


def color3(value):
    if isinstance(value, str):
        parts = [p.strip() for p in value.replace(';', ',').split(',')]
        if len(parts) != 3:
            raise ValueError('RGB must be r,g,b')
        value = [float(x) for x in parts]
    if len(value) != 3:
        raise ValueError('RGB must contain 3 components')
    return tuple(clamp(x) for x in value)


def trajectory_sky_sphere(trajectory):
    """Pick a sphere that encloses this recorded route with a safe margin."""
    xs = [float(p['x']) for p in trajectory]
    ys = [float(p['y']) for p in trajectory]
    zs = [float(p['z']) for p in trajectory]
    cx = 0.5 * (min(xs) + max(xs))
    cy = 0.5 * (min(ys) + max(ys))
    cz = 0.5 * (min(zs) + max(zs))
    farthest = max(
        math.sqrt((x-cx)**2 + (y-cy)**2 + (z-cz)**2)
        for x, y, z in zip(xs, ys, zs)
    )
    radius = max(35.0, min(75.0, farthest + 25.0))
    return (cx, cy, cz), radius


def ensure_inverted_sky_mesh(rgb=(0.5, 0.7, 1.0)):
    """Generate an inward-facing OBJ sphere with colour baked into MTL.

    This avoids runtime /visual_config material updates, which were accepted
    by Gazebo on the target machine but did not recolour the Ogre2 render.
    """
    rgb = color3(rgb)
    key = ''.join(f'{max(0, min(255, round(c * 255))):02x}' for c in rgb)
    path = Path(f'/tmp/concept_vla_inverted_sky_{key}.obj')
    mtl_path = Path(f'/tmp/concept_vla_inverted_sky_{key}.mtl')
    marker = f'# concept-vla inward sky sphere v2 {key}\n'
    if path.is_file() and mtl_path.is_file():
        try:
            if path.read_text(errors='ignore').startswith(marker):
                return path
        except Exception:
            pass

    lat_steps = 18
    lon_steps = 36
    vertices = [(0.0, 0.0, 1.0)]
    rings = []
    for ilat in range(1, lat_steps):
        theta = math.pi * ilat / lat_steps
        st = math.sin(theta); ct = math.cos(theta)
        ring = []
        for ilon in range(lon_steps):
            phi = 2.0 * math.pi * ilon / lon_steps
            ring.append(len(vertices) + 1)
            vertices.append((st * math.cos(phi), st * math.sin(phi), ct))
        rings.append(ring)
    south = len(vertices) + 1
    vertices.append((0.0, 0.0, -1.0))

    triangles = []
    first = rings[0]
    for j in range(lon_steps):
        triangles.append((1, first[(j + 1) % lon_steps], first[j]))
    for r0, r1 in zip(rings[:-1], rings[1:]):
        for j in range(lon_steps):
            j1 = (j + 1) % lon_steps
            triangles.append((r0[j], r1[j1], r0[j1]))
            triangles.append((r0[j], r1[j], r1[j1]))
    last = rings[-1]
    for j in range(lon_steps):
        triangles.append((last[j], last[(j + 1) % lon_steps], south))

    inward = []
    for ia, ib, ic in triangles:
        a = vertices[ia - 1]; b = vertices[ib - 1]; c = vertices[ic - 1]
        ab = (b[0]-a[0], b[1]-a[1], b[2]-a[2])
        ac = (c[0]-a[0], c[1]-a[1], c[2]-a[2])
        cross = (ab[1]*ac[2]-ab[2]*ac[1], ab[2]*ac[0]-ab[0]*ac[2], ab[0]*ac[1]-ab[1]*ac[0])
        centroid = ((a[0]+b[0]+c[0])/3.0, (a[1]+b[1]+c[1])/3.0, (a[2]+b[2]+c[2])/3.0)
        outward = (cross[0]*centroid[0] + cross[1]*centroid[1] + cross[2]*centroid[2]) > 0.0
        inward.append((ia, ic, ib) if outward else (ia, ib, ic))

    r, g, b = rgb
    mtl_path.write_text(
        'newmtl dataset_sky\n'
        f'Ka {r:.6f} {g:.6f} {b:.6f}\n'
        f'Kd {r:.6f} {g:.6f} {b:.6f}\n'
        f'Ke {r:.6f} {g:.6f} {b:.6f}\n'
        'Ks 0 0 0\nNs 1\nd 1.0\nillum 1\n'
    )
    lines = [marker.rstrip('\n'), f'mtllib {mtl_path.name}', 'usemtl dataset_sky']
    lines += [f'v {x:.9f} {y:.9f} {z:.9f}' for x, y, z in vertices]
    lines += [f'f {a} {b} {c}' for a, b, c in inward]
    path.write_text('\n'.join(lines) + '\n')
    return path


class GazeboEnvironmentClient:
    """
    Runtime Gazebo environment controller.

    Existing world lights are preferred.  In the supplied aerobot world the
    resource model://sun defines a directional light named ``sunUTC`` (not
    ``sun``).  We therefore probe known names through /light_config and only
    create a dedicated top-level light as a fallback.
    """

    SUN_NAME = '__dataset_sun'
    SUN_CANDIDATES = ('sunUTC', 'sun')
    SKY_MODEL_NAME = '__dataset_sky_shell'
    SKY_LINK_NAME = '__dataset_sky_link'
    SKY_VISUAL_NAME = '__dataset_sky_visual'

    def __init__(self, world_name):
        import importlib
        self.world_name = world_name
        self.light_service = f'/world/{world_name}/light_config'
        self.create_service = f'/world/{world_name}/create'
        self.remove_service = f'/world/{world_name}/remove'
        self.visual_service = f'/world/{world_name}/visual_config'
        self.visual_service_blocking = f'/world/{world_name}/visual_config/blocking'
        self.material_topic = f'/world/{world_name}/material_color'
        self.scene_service = f'/world/{world_name}/scene/info'
        self.controlled_sun_name = None
        self.dataset_sun_created = False
        self.sky_created = False
        self.sky_visual_id = None
        self.sky_parent_id = None
        self.sky_center = None
        self.sky_radius = None
        self.sky_rgb = None
        self.sky_model_name = None
        self.sky_generation = 0
        candidates = [
            ('gz.transport13', 'gz.msgs10'),
            ('gz.transport14', 'gz.msgs11'),
            ('gz.transport12', 'gz.msgs9'),
            ('gz.transport11', 'gz.msgs8'),
            ('gz.transport10', 'gz.msgs7'),
            ('gz.transport', 'gz.msgs'),
        ]
        errors = []
        for transport_mod, msgs_prefix in candidates:
            try:
                transport = importlib.import_module(transport_mod)
                self.Node = getattr(transport, 'Node')
                self.Light = getattr(importlib.import_module(f'{msgs_prefix}.light_pb2'), 'Light')
                self.Boolean = getattr(importlib.import_module(f'{msgs_prefix}.boolean_pb2'), 'Boolean')
                self.EntityFactory = getattr(importlib.import_module(f'{msgs_prefix}.entity_factory_pb2'), 'EntityFactory')
                self.Entity = getattr(importlib.import_module(f'{msgs_prefix}.entity_pb2'), 'Entity')
                self.Visual = getattr(importlib.import_module(f'{msgs_prefix}.visual_pb2'), 'Visual')
                try:
                    self.MaterialColor = getattr(importlib.import_module(f'{msgs_prefix}.material_color_pb2'), 'MaterialColor')
                except Exception:
                    self.MaterialColor = None
                self.Empty = getattr(importlib.import_module(f'{msgs_prefix}.empty_pb2'), 'Empty')
                self.Scene = getattr(importlib.import_module(f'{msgs_prefix}.scene_pb2'), 'Scene')
                self.node = self.Node()
                self.material_pub = None
                if self.MaterialColor is not None:
                    try:
                        self.material_pub = self.node.advertise(self.material_topic, self.MaterialColor)
                    except Exception:
                        self.material_pub = None
                self._visual_service_selected = None
                self.backend = f'{transport_mod} + {msgs_prefix}'
                print(f'[LIGHT] Gazebo Transport: {self.backend}', flush=True)
                return
            except Exception as exc:
                errors.append(f'{transport_mod}: {type(exc).__name__}: {exc}')
        raise RuntimeError('Gazebo Python Transport bindings required for lighting are not available. Tried:\n  - ' + '\n  - '.join(errors))

    def _request(self, service, request, request_type, timeout_ms=500):
        try:
            executed, response = self.node.request(service, request, request_type, self.Boolean, timeout_ms)
        except TypeError as exc:
            raise RuntimeError(f'Gazebo Transport request() signature mismatch for {service}: {exc}')
        if not executed:
            raise RuntimeError(f'Gazebo service timed out: {service}')
        if hasattr(response, 'data') and not bool(response.data):
            raise RuntimeError(f'Gazebo service rejected request: {service}')
        return True

    @staticmethod
    def _set_msg_color(msg_color, rgb, alpha=1.0):
        msg_color.r = float(rgb[0]); msg_color.g = float(rgb[1]); msg_color.b = float(rgb[2]); msg_color.a = float(alpha)

    def _remove_entity(self, name, type_name=None, quiet=True):
        req = self.Entity(); req.name = str(name)
        if type_name and hasattr(self.Entity, type_name):
            req.type = getattr(self.Entity, type_name)
        try:
            self._request(self.remove_service, req, self.Entity, timeout_ms=500)
            time.sleep(0.10)
            return True
        except Exception:
            if not quiet: raise
            return False

    def _make_light(self, rgb, intensity, direction, name=None):
        rgb = color3(rgb); direction = normalize_vec3(direction)
        light = self.Light(); light.name = str(name or self.SUN_NAME)
        light.type = getattr(self.Light, 'DIRECTIONAL', 2)
        self._set_msg_color(light.diffuse, rgb)
        self._set_msg_color(light.specular, tuple(0.35 * x for x in rgb))
        if hasattr(light, 'intensity'): light.intensity = max(0.0, float(intensity))
        light.direction.x, light.direction.y, light.direction.z = direction
        light.cast_shadows = True
        return light

    def create_dataset_sun(self, rgb, intensity, direction):
        # Prefer the light that already belongs to the world.  Calling
        # light_config is also a cheap existence check because UserCommands
        # returns Boolean(false) for an unknown light.  The aerobot resource
        # uses the name ``sunUTC``.
        errors = []
        for name in self.SUN_CANDIDATES:
            try:
                req = self._make_light(rgb, intensity, direction, name)
                self._request(self.light_service, req, self.Light, timeout_ms=700)
                self.controlled_sun_name = name
                self.dataset_sun_created = False
                print(f'[LIGHT] controlling existing directional light: {name}', flush=True)
                return name
            except Exception as exc:
                errors.append(f'{name}: {exc}')

        # Generic fallback for worlds whose light has another name or no sun at
        # all.  Creating a top-level light is supported by UserCommands and
        # makes subsequent light_config calls deterministic.
        self._remove_entity(self.SUN_NAME, 'LIGHT', quiet=True)
        light = self._make_light(rgb, intensity, direction, self.SUN_NAME)
        req = self.EntityFactory()
        try:
            req.light.CopyFrom(light)
        except Exception:
            r, g, b = color3(rgb); dx, dy, dz = normalize_vec3(direction)
            req.sdf = f"""<?xml version="1.0"?>
<sdf version="1.7"><light type="directional" name="{self.SUN_NAME}">
<cast_shadows>true</cast_shadows><intensity>{float(intensity)}</intensity>
<diffuse>{r} {g} {b} 1</diffuse><specular>{0.35*r} {0.35*g} {0.35*b} 1</specular>
<direction>{dx} {dy} {dz}</direction></light></sdf>"""
        if hasattr(req, 'allow_renaming'):
            req.allow_renaming = False
        self._request(self.create_service, req, self.EntityFactory, timeout_ms=1000)
        time.sleep(0.20)
        self.controlled_sun_name = self.SUN_NAME
        self.dataset_sun_created = True
        print('[LIGHT WARNING] existing sun was not configurable: ' + '; '.join(errors), flush=True)
        print(f'[LIGHT] fallback directional light ready: {self.SUN_NAME}', flush=True)
        return self.SUN_NAME

    def set_sun(self, rgb, intensity, direction):
        name = self.controlled_sun_name or self.SUN_CANDIDATES[0]
        req = self._make_light(rgb, intensity, direction, name)
        self._request(self.light_service, req, self.Light)

    def remove_dataset_sun(self):
        # Never remove an original world light.  Only the fallback entity is
        # owned by replay_raw.py.
        if self.dataset_sun_created:
            self._remove_entity(self.SUN_NAME, 'LIGHT', quiet=True)
            self.dataset_sun_created = False
        self.controlled_sun_name = None

    def restore_stock_sun(self):
        # Kept as a no-op for cleanup compatibility with previous UI versions.
        return

    def remove_sky_sphere(self, quiet=True):
        """Remove the currently active visual sky shell.

        The public method name is kept for compatibility with earlier replay
        versions, but v9 intentionally uses six primitive box visuals instead
        of an OBJ sphere. Primitive materials are handled directly by Gazebo /
        Ogre2 and avoid the stale-white mesh material seen on the target host.
        """
        name = self.sky_model_name
        if not name:
            self.sky_created = False
            return True
        ok = self._remove_entity(name, 'MODEL', quiet=quiet)
        if ok:
            self.sky_created = False
            self.sky_model_name = None
        self.sky_visual_id = None
        self.sky_parent_id = None
        return ok

    def _sky_shell_sdf(self, name, rgb):
        """Build a visual-only six-sided shell around the trajectory."""
        if self.sky_center is None or self.sky_radius is None:
            raise RuntimeError('sky geometry is not initialized')
        r, g, b = color3(rgb)
        cx, cy, cz = (float(v) for v in self.sky_center)
        h = float(self.sky_radius)
        side = 2.0 * h
        thickness = max(0.25, min(1.0, h * 0.015))
        mat = (
            f'<material><lighting>false</lighting>'
            f'<ambient>{r} {g} {b} 1</ambient>'
            f'<diffuse>{r} {g} {b} 1</diffuse>'
            f'<emissive>{r} {g} {b} 1</emissive></material>'
        )
        specs = [
            ('west',  -h, 0.0, 0.0, thickness, side,      side),
            ('east',   h, 0.0, 0.0, thickness, side,      side),
            ('south', 0.0, -h, 0.0, side,      thickness, side),
            ('north', 0.0,  h, 0.0, side,      thickness, side),
            ('floor', 0.0, 0.0, -h, side,      side,      thickness),
            ('roof',  0.0, 0.0,  h, side,      side,      thickness),
        ]
        visuals = []
        for label, x, y, z, sx, sy, sz in specs:
            visuals.append(
                f'<visual name="{self.SKY_VISUAL_NAME}_{label}">'
                f'<pose>{x} {y} {z} 0 0 0</pose>'
                '<cast_shadows>false</cast_shadows>'
                f'<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
                f'{mat}</visual>'
            )
        return (
            '<?xml version="1.0"?><sdf version="1.7">'
            f'<model name="{name}"><static>true</static>'
            f'<pose>{cx} {cy} {cz} 0 0 0</pose>'
            f'<link name="{self.SKY_LINK_NAME}">' + ''.join(visuals) +
            '</link></model></sdf>'
        )

    def _spawn_sky_sphere(self, rgb):
        # Fresh entity names force Ogre2 to build a new material instead of
        # reusing the cached white material from the previous mesh version.
        rgb = color3(rgb)
        self.sky_generation += 1
        new_name = f'{self.SKY_MODEL_NAME}_{self.sky_generation:04d}'
        req = self.EntityFactory()
        req.sdf = self._sky_shell_sdf(new_name, rgb)
        if hasattr(req, 'allow_renaming'):
            req.allow_renaming = False
        self._request(self.create_service, req, self.EntityFactory, timeout_ms=1200)

        old_name = self.sky_model_name
        self.sky_model_name = new_name
        self.sky_created = True
        self.sky_rgb = rgb
        self.sky_visual_id = None
        self.sky_parent_id = None
        if old_name and old_name != new_name:
            self._remove_entity(old_name, 'MODEL', quiet=True)
        print(
            f'[LIGHT] sky shell applied: rgb={rgb[0]:.3f},{rgb[1]:.3f},{rgb[2]:.3f} '
            f'entity={new_name}',
            flush=True,
        )
        return True

    def create_sky_sphere(self, rgb, center, radius):
        self.remove_sky_sphere(quiet=True)
        self.sky_center = tuple(float(v) for v in center)
        self.sky_radius = float(radius)
        return self._spawn_sky_sphere(rgb)

    def set_sky_color(self, rgb):
        rgb = color3(rgb)
        if self.sky_rgb is not None and max(abs(a-b) for a, b in zip(rgb, self.sky_rgb)) < 0.004:
            return True
        if self.sky_center is None or self.sky_radius is None:
            raise RuntimeError('sky shell has not been created')
        return self._spawn_sky_sphere(rgb)


LIGHTING_PRESETS = (
    'off', 'constant', 'day_to_sunset', 'day_to_night', 'cloud_pass', 'random'
)


def _kf(u, sun_i, sun_rgb, sky_rgb, direction=(-0.5, 0.1, -0.9)):
    return {
        'u': float(u),
        'sun_intensity': float(sun_i),
        'sun_rgb': color3(sun_rgb),
        'sky_rgb': color3(sky_rgb),
        'sun_direction': normalize_vec3(direction),
    }


def preset_keyframes(name, seed=0):
    if name == 'constant':
        return [
            _kf(0.0, 0.95, (1.0, 0.98, 0.94), (0.52, 0.67, 0.88)),
            _kf(1.0, 0.95, (1.0, 0.98, 0.94), (0.52, 0.67, 0.88)),
        ]
    if name == 'day_to_sunset':
        return [
            _kf(0.0, 1.00, (1.0, 0.98, 0.94), (0.48, 0.68, 0.95), (-0.45, 0.1, -0.89)),
            _kf(0.55, 0.82, (1.0, 0.88, 0.68), (0.75, 0.48, 0.40), (-0.60, 0.1, -0.79)),
            _kf(1.0, 0.48, (1.0, 0.58, 0.32), (0.34, 0.16, 0.28), (-0.78, 0.1, -0.62)),
        ]
    if name == 'day_to_night':
        return [
            _kf(0.0, 1.00, (1.0, 0.98, 0.94), (0.50, 0.70, 0.98), (-0.45, 0.1, -0.89)),
            _kf(0.60, 0.45, (0.72, 0.78, 1.00), (0.18, 0.22, 0.42), (-0.67, 0.1, -0.74)),
            _kf(1.0, 0.18, (0.46, 0.56, 0.92), (0.035, 0.045, 0.12), (-0.82, 0.1, -0.56)),
        ]
    if name == 'cloud_pass':
        return [
            _kf(0.0, 0.95, (1.0, 0.98, 0.94), (0.52, 0.68, 0.88)),
            _kf(0.30, 0.85, (0.92, 0.95, 1.00), (0.46, 0.56, 0.70)),
            _kf(0.50, 0.28, (0.70, 0.76, 0.86), (0.25, 0.30, 0.38)),
            _kf(0.70, 0.78, (0.90, 0.93, 1.00), (0.43, 0.54, 0.70)),
            _kf(1.0, 0.98, (1.0, 0.98, 0.94), (0.52, 0.68, 0.88)),
        ]
    if name == 'random':
        rng = random.Random(int(seed))
        palette_sun = [
            (1.0, 0.98, 0.94), (1.0, 0.82, 0.60),
            (0.72, 0.82, 1.0), (1.0, 0.62, 0.38),
        ]
        palette_sky = [
            (0.50, 0.70, 0.98), (0.32, 0.42, 0.62),
            (0.78, 0.48, 0.38), (0.10, 0.13, 0.28),
            (0.60, 0.64, 0.70),
        ]
        out = []
        count = 5
        for i in range(count):
            u = i / (count - 1)
            az = rng.uniform(-math.pi, math.pi)
            z = -rng.uniform(0.55, 0.98)
            horizontal = math.sqrt(max(0.0, 1.0 - z*z))
            direction = (horizontal * math.cos(az), horizontal * math.sin(az), z)
            out.append(_kf(
                u, rng.uniform(0.25, 1.10), rng.choice(palette_sun),
                rng.choice(palette_sky), direction,
            ))
        return out
    if name == 'off':
        return []
    raise ValueError(f'Unknown lighting preset: {name}')


def load_lighting_json(path):
    data = json.loads(Path(path).read_text())
    frames = data.get('keyframes', []) if isinstance(data, dict) else data
    if not isinstance(frames, list) or not frames:
        raise ValueError('lighting JSON needs a non-empty keyframes list')
    out = []
    for item in frames:
        row = {
            'sun_intensity': float(item.get('sun_intensity', 1.0)),
            'sun_rgb': color3(item.get('sun_rgb', [1.0, 1.0, 1.0])),
            'sky_rgb': color3(item.get('sky_rgb', [0.5, 0.7, 1.0])),
            'sun_direction': normalize_vec3(item.get(
                'sun_direction', [-0.5, 0.1, -0.9])),
        }
        if 't' in item:
            row['t'] = float(item['t'])
        elif 'u' in item:
            row['u'] = float(item['u'])
        else:
            raise ValueError('each lighting keyframe needs t or u')
        out.append(row)
    return out


def resolve_keyframe_times(frames, duration):
    out = []
    for row in frames:
        r = dict(row)
        if 't' not in r:
            r['t'] = clamp(r.pop('u'), 0.0, 1.0) * float(duration)
        r.pop('u', None)
        out.append(r)
    out.sort(key=lambda x: x['t'])
    return out


def sample_lighting(frames, t):
    if not frames:
        return None
    if t <= frames[0]['t']:
        return dict(frames[0])
    if t >= frames[-1]['t']:
        return dict(frames[-1])
    for a, b in zip(frames, frames[1:]):
        if a['t'] <= t <= b['t']:
            dt = max(1e-9, b['t'] - a['t'])
            u = (t - a['t']) / dt
            return {
                't': float(t),
                'sun_intensity': lerp(a['sun_intensity'], b['sun_intensity'], u),
                'sun_rgb': lerp3(a['sun_rgb'], b['sun_rgb'], u),
                'sky_rgb': lerp3(a['sky_rgb'], b['sky_rgb'], u),
                'sun_direction': normalize_vec3(lerp3(
                    a['sun_direction'], b['sun_direction'], u)),
            }
    return dict(frames[-1])


class LightingController:
    def __init__(self, env, keyframes, duration, update_hz=5.0, sky_enabled=True):
        self.env = env
        self.duration = float(duration)
        self.keyframes = resolve_keyframe_times(keyframes, duration)
        self.update_hz = max(0.2, float(update_hz))
        self.sky_enabled = bool(sky_enabled)
        self.stop_event = threading.Event()
        self.thread = None
        self.route_start = None
        self.lock = threading.RLock()
        self.auto_enabled = True
        self.manual = {}
        self.last_state = None
        self.events = []
        self.sun_error = None
        self.sky_error = None
        self._last_sun_error_print = 0.0
        self._last_sky_error_print = 0.0

        # Avoid hammering Gazebo UserCommands with identical light / material
        # requests.  The old UI sent light_config + visual_config every 200 ms
        # even for a constant scene, contending with the 100 Hz set_pose replay.
        self._applied_sun = None
        self._applied_sky = None

    def initial_state(self):
        return sample_lighting(self.keyframes, 0.0)

    def current_time(self):
        if self.route_start is None:
            return 0.0
        return max(0.0, time.monotonic() - self.route_start)

    def set_manual(self, **kwargs):
        with self.lock:
            self.manual.update(kwargs)
            self.events.append({
                't': round(self.current_time(), 4),
                'command': 'manual',
                **{k: list(v) if isinstance(v, tuple) else v
                   for k, v in kwargs.items()},
            })

    def clear_manual(self):
        with self.lock:
            self.manual.clear()
            self.events.append({
                't': round(self.current_time(), 4),
                'command': 'clear_manual',
            })

    def set_auto(self, enabled):
        with self.lock:
            self.auto_enabled = bool(enabled)
            self.events.append({
                't': round(self.current_time(), 4),
                'command': 'auto',
                'enabled': bool(enabled),
            })

    def state(self):
        with self.lock:
            if self.auto_enabled:
                state = sample_lighting(self.keyframes, self.current_time())
            else:
                state = dict(self.last_state or self.initial_state())
            if state is None:
                return None
            state.update(self.manual)
            return state

    @staticmethod
    def _vec_changed(a, b, eps):
        if a is None or b is None:
            return True
        return max(abs(float(a[i]) - float(b[i])) for i in range(len(a))) >= float(eps)

    def apply_once(self):
        state = self.state()
        if state is None:
            return

        sun_signature = (
            float(state['sun_intensity']),
            tuple(state['sun_rgb']),
            tuple(state['sun_direction']),
        )
        old_sun = self._applied_sun
        sun_changed = (
            old_sun is None
            or abs(sun_signature[0] - old_sun[0]) >= 0.005
            or self._vec_changed(sun_signature[1], old_sun[1], 0.005)
            or self._vec_changed(sun_signature[2], old_sun[2], 0.004)
        )

        if sun_changed:
            try:
                self.env.set_sun(state['sun_rgb'], state['sun_intensity'], state['sun_direction'])
                self._applied_sun = sun_signature
                with self.lock: self.sun_error = None
            except Exception as exc:
                now = time.monotonic()
                with self.lock: self.sun_error = str(exc)
                if now - self._last_sun_error_print > 3.0:
                    print(f'[LIGHT WARNING] sun update failed: {exc}', flush=True)
                    self._last_sun_error_print = now

        sky_signature = tuple(state['sky_rgb'])
        sky_changed = self._vec_changed(sky_signature, self._applied_sky, 0.008)
        if self.sky_enabled and sky_changed:
            try:
                self.env.set_sky_color(state['sky_rgb'])
                self._applied_sky = sky_signature
                with self.lock: self.sky_error = None
            except Exception as exc:
                now = time.monotonic()
                with self.lock: self.sky_error = str(exc)
                if now - self._last_sky_error_print > 3.0:
                    print(f'[LIGHT WARNING] sky update failed: {exc}', flush=True)
                    self._last_sky_error_print = now

        with self.lock:
            self.last_state = dict(state)

    def health(self):
        with self.lock:
            return {'sun_error': self.sun_error, 'sky_error': self.sky_error}

    def start(self, route_start):
        self.route_start = float(route_start)
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        period = 1.0 / self.update_hz
        next_tick = time.monotonic()
        while not self.stop_event.is_set():
            try:
                self.apply_once()
            except Exception as exc:
                print(f'[LIGHT] update failed: {exc}', flush=True)
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                self.stop_event.wait(delay)
            else:
                next_tick = time.monotonic()

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
            self.thread = None

    def manifest(self):
        with self.lock:
            return {
                'update_hz': self.update_hz,
                'sky_sphere': self.sky_enabled,
                'keyframes': [
                    {
                        **r,
                        'sun_rgb': list(r['sun_rgb']),
                        'sky_rgb': list(r['sky_rgb']),
                        'sun_direction': list(r['sun_direction']),
                    }
                    for r in self.keyframes
                ],
                'manual_events': list(self.events),
            }


class LightingConsole(threading.Thread):
    """Optional line-based live control while replay is running."""

    def __init__(self, controller):
        super().__init__(daemon=True)
        self.controller = controller

    @staticmethod
    def help_text():
        return (
            '\n[LIGHT UI] commands:\n'
            '  status                     show current values\n'
            '  sun <intensity>            override sun intensity\n'
            '  sunrgb <r> <g> <b>         override sun RGB (0..1)\n'
            '  sky <r> <g> <b>            override sky-sphere RGB (0..1)\n'
            '  dir <x> <y> <z>            override sun direction\n'
            '  auto on|off                resume / freeze timeline\n'
            '  clear                      remove manual overrides\n'
            '  help                       show this text\n'
        )

    def run(self):
        if not sys.stdin.isatty():
            print('[LIGHT UI] stdin is not a TTY; console disabled', flush=True)
            return
        print(self.help_text(), flush=True)
        while not self.controller.stop_event.is_set():
            try:
                line = input('light> ').strip()
            except (EOFError, KeyboardInterrupt):
                return
            if not line:
                continue
            parts = line.split()
            cmd = parts[0].lower()
            try:
                if cmd == 'help':
                    print(self.help_text(), flush=True)
                elif cmd == 'status':
                    print('[LIGHT UI]', self.controller.state(), flush=True)
                elif cmd == 'sun' and len(parts) == 2:
                    self.controller.set_manual(
                        sun_intensity=max(0.0, float(parts[1])))
                elif cmd == 'sunrgb' and len(parts) == 4:
                    self.controller.set_manual(sun_rgb=color3(
                        [float(x) for x in parts[1:4]]))
                elif cmd == 'sky' and len(parts) == 4:
                    self.controller.set_manual(sky_rgb=color3(
                        [float(x) for x in parts[1:4]]))
                elif cmd == 'dir' and len(parts) == 4:
                    self.controller.set_manual(sun_direction=normalize_vec3(
                        [float(x) for x in parts[1:4]]))
                elif cmd == 'auto' and len(parts) == 2:
                    self.controller.set_auto(
                        parts[1].lower() in ('1', 'on', 'true', 'yes'))
                elif cmd == 'clear':
                    self.controller.clear_manual()
                else:
                    print('[LIGHT UI] unknown command; type help', flush=True)
            except Exception as exc:
                print(f'[LIGHT UI] error: {exc}', flush=True)


def image_encoding_to_ffmpeg(encoding):
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


def image_bytes_without_padding(msg, bpp):
    row_size = int(msg.width) * bpp
    step = int(msg.step)
    raw = bytes(msg.data)

    if step == row_size:
        return raw

    out = bytearray(
        row_size * int(msg.height)
    )
    dst = 0

    for row in range(int(msg.height)):
        src = row * step
        out[dst:dst + row_size] = raw[
            src:src + row_size
        ]
        dst += row_size

    return bytes(out)


class FFmpegCameraRecorder:
    def __init__(
        self,
        output_dir,
        fps,
        start_ns,
    ):
        self.output_dir = output_dir
        self.fps = fps
        self.start_ns = start_ns

        output_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.video_path = output_dir / "video.mp4"
        self.timestamps_path = (
            output_dir / "timestamps.csv"
        )
        self.log_path = output_dir / "ffmpeg.log"

        self.proc = None
        self.log_file = None

        self.width = None
        self.height = None
        self.encoding = None
        self.pix_fmt = None
        self.bpp = None

        self.frame_index = 0

        self.csv_file = open(
            self.timestamps_path,
            "w",
            newline="",
            buffering=1,
        )
        self.writer = csv.writer(
            self.csv_file
        )
        self.writer.writerow([
            "frame_index",
            "timestamp_start",
        ])

    def start_ffmpeg(self, msg):
        self.width = int(msg.width)
        self.height = int(msg.height)
        self.encoding = msg.encoding

        self.pix_fmt, self.bpp = (
            image_encoding_to_ffmpeg(
                self.encoding
            )
        )

        self.log_file = open(
            self.log_path,
            "w",
        )

        cmd = [
            "ffmpeg",
            "-y",
            "-loglevel", "warning",
            "-f", "rawvideo",
            "-pixel_format", self.pix_fmt,
            "-video_size",
            f"{self.width}x{self.height}",
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
            (receipt_ns - self.start_ns) / 1e9,
        )

        try:
            self.proc.stdin.write(
                image_bytes_without_padding(
                    msg,
                    self.bpp,
                )
            )
        except Exception:
            return

        self.writer.writerow([
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


class ReplayRecorder:
    def __init__(
        self,
        flight_dir,
        start_ns,
        source_trajectory,
    ):
        self.flight_dir = flight_dir
        self.start_ns = start_ns

        self.forward_dir = flight_dir / "forward"
        self.bottom_dir = flight_dir / "bottom"
        self.trajectory_dir = flight_dir / "trajectory"

        for p in (
            self.forward_dir,
            self.bottom_dir,
            self.trajectory_dir,
        ):
            p.mkdir(
                parents=True,
                exist_ok=True,
            )

        self.forward = FFmpegCameraRecorder(
            self.forward_dir,
            FORWARD_VIDEO_FPS,
            start_ns,
        )
        self.bottom = FFmpegCameraRecorder(
            self.bottom_dir,
            BOTTOM_VIDEO_FPS,
            start_ns,
        )

        # Save the exact world trajectory used for this replay.
        with open(
            self.trajectory_dir / "trajectory.csv",
            "w",
            newline="",
        ) as f:
            writer = csv.writer(f)
            writer.writerow([
                "sample_index",
                "timestamp_start",
                "world_x",
                "world_y",
                "world_z",
                "world_roll_deg",
                "world_pitch_deg",
                "world_yaw_deg",
            ])

            for i, row in enumerate(
                source_trajectory
            ):
                writer.writerow([
                    i,
                    f4(row["t"]),
                    f6(row["x"]),
                    f6(row["y"]),
                    f6(row["z"]),
                    f4(row["roll"]),
                    f4(row["pitch"]),
                    f4(row["yaw"]),
                ])

    def process_forward(self, msg, receipt_ns):
        self.forward.process(
            msg,
            receipt_ns,
        )

    def process_bottom(self, msg, receipt_ns):
        self.bottom.process(
            msg,
            receipt_ns,
        )

    def close(self):
        self.forward.close()
        self.bottom.close()


class ReplayNode(Node):
    def __init__(self):
        super().__init__(
            "concept_vla_exact_replay"
        )

        self.lock = threading.RLock()

        self.latest_odom = None
        self.latest_state = None
        self.recorder = None

        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            SETPOINT_TOPIC,
            10,
        )

        self.arm_client = self.create_client(
            CommandBool,
            ARM_SERVICE,
        )

        self.mode_client = self.create_client(
            SetMode,
            SET_MODE_SERVICE,
        )

        self.command_long_client = None
        if CommandLong is not None:
            self.command_long_client = self.create_client(
                CommandLong,
                COMMAND_LONG_SERVICE,
            )

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

        self.state_sub = self.create_subscription(
            State,
            MAVROS_STATE_TOPIC,
            self.state_callback,
            10,
        )

    def forward_callback(self, msg):
        now = time.monotonic_ns()

        with self.lock:
            recorder = self.recorder

        if recorder is not None:
            recorder.process_forward(
                msg,
                now,
            )

    def bottom_callback(self, msg):
        now = time.monotonic_ns()

        with self.lock:
            recorder = self.recorder

        if recorder is not None:
            recorder.process_bottom(
                msg,
                now,
            )

    def odom_callback(self, msg):
        with self.lock:
            self.latest_odom = msg

    def state_callback(self, msg):
        with self.lock:
            self.latest_state = msg

    def get_local_pose(self):
        with self.lock:
            if self.latest_odom is None:
                return None
            return pose_dict(
                self.latest_odom
            )

    def get_state(self):
        with self.lock:
            return self.latest_state

    def publish_hold(self, pose):
        msg = PoseStamped()
        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )
        msg.header.frame_id = "map"

        msg.pose.position.x = float(
            pose["x"]
        )
        msg.pose.position.y = float(
            pose["y"]
        )
        msg.pose.position.z = float(
            pose["z"]
        )

        yaw = math.radians(
            pose["yaw_deg"]
        )
        msg.pose.orientation.z = math.sin(
            yaw * 0.5
        )
        msg.pose.orientation.w = math.cos(
            yaw * 0.5
        )

        self.setpoint_pub.publish(msg)

    def request_mode(self, mode):
        if not self.mode_client.wait_for_service(
            timeout_sec=2.0
        ):
            return False

        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode

        future = self.mode_client.call_async(
            req
        )

        deadline = time.monotonic() + 2.0

        while not future.done():
            if time.monotonic() > deadline:
                return False
            time.sleep(0.01)

        result = future.result()

        return bool(
            result is not None
            and result.mode_sent
        )

    def request_arm(self, value=True):
        if not self.arm_client.wait_for_service(
            timeout_sec=2.0
        ):
            return False

        req = CommandBool.Request()
        req.value = bool(value)

        future = self.arm_client.call_async(
            req
        )

        deadline = time.monotonic() + 2.0

        while not future.done():
            if time.monotonic() > deadline:
                return False
            time.sleep(0.01)

        result = future.result()

        return bool(
            result is not None
            and result.success
        )

    def request_force_arm(self):
        """PX4 SITL fallback: MAV_CMD_COMPONENT_ARM_DISARM with force magic.

        This is only used by the interactive simulator replay after normal
        MAVROS arming was rejected.  It is intentionally not used in the
        non-UI/original replay path.
        """
        if CommandLong is None or self.command_long_client is None:
            return False
        if not self.command_long_client.wait_for_service(timeout_sec=1.5):
            return False

        req = CommandLong.Request()
        req.broadcast = False
        req.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
        req.confirmation = 0
        req.param1 = 1.0
        req.param2 = 21196.0  # PX4 force-arm magic
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0

        future = self.command_long_client.call_async(req)
        deadline = time.monotonic() + 2.0
        while not future.done():
            if time.monotonic() > deadline:
                return False
            time.sleep(0.01)
        result = future.result()
        return bool(result is not None and getattr(result, 'success', False))



class HoldStreamer:
    """30 Hz MAVROS setpoint publisher with a fixed arming target mode."""
    def __init__(self, node):
        self.node = node
        self.stop_event = threading.Event()
        self.lock = threading.RLock()
        self.fixed_pose = None
        self.sent_count = 0
        self.thread = threading.Thread(target=self.run, daemon=True)

    def set_fixed_target(self, pose):
        with self.lock:
            self.fixed_pose = dict(pose)

    def set_follow_current(self):
        with self.lock:
            self.fixed_pose = None

    def target_snapshot(self):
        with self.lock:
            return None if self.fixed_pose is None else dict(self.fixed_pose)

    def start(self):
        self.thread.start()

    def run(self):
        period = 1.0 / SETPOINT_HZ
        next_tick = time.monotonic()
        while not self.stop_event.is_set():
            if not rclpy.ok():
                break
            try:
                target = self.target_snapshot()
                if target is None:
                    target = self.node.get_local_pose()
                if target is not None:
                    self.node.publish_hold(target)
                    self.sent_count += 1
            except Exception as exc:
                if not rclpy.ok():
                    break
                print(f'[REPLAY WARNING] setpoint streamer: {exc}', flush=True)
                self.stop_event.wait(0.1)
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                self.stop_event.wait(delay)
            else:
                next_tick = time.monotonic()

    def stop(self):
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)


def ensure_mavros(node, timeout=15.0):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        state = node.get_state()
        pose = node.get_local_pose()

        if (
            state is not None
            and state.connected
            and pose is not None
        ):
            return

        time.sleep(0.05)

    raise RuntimeError(
        "MAVROS not ready"
    )


def ensure_offboard_and_armed(
    node,
    timeout,
    allow_force_arm=False,
):
    """Enter OFFBOARD and arm using the original replay sequence.

    For UI/SITL only, allow_force_arm adds one fallback after normal arming
    has been rejected for a few seconds.  The 30 Hz hold stream must already
    be running before this function is called.
    """
    deadline = time.monotonic() + timeout
    last_mode = -1e9
    last_arm = -1e9
    last_force = -1e9
    first_arm_attempt = None
    normal_arm_failures = 0

    while time.monotonic() < deadline:
        state = node.get_state()

        if state is None:
            time.sleep(0.05)
            continue

        now = time.monotonic()

        if state.mode != "OFFBOARD":
            if now - last_mode >= 0.8:
                last_mode = now
                ok = node.request_mode("OFFBOARD")
                print(f"[REPLAY] OFFBOARD request: {'accepted' if ok else 'not accepted'}", flush=True)
            time.sleep(0.05)
            continue

        if not state.armed:
            if first_arm_attempt is None:
                first_arm_attempt = now
            if now - last_arm >= 0.8:
                last_arm = now
                ok = node.request_arm(True)
                if not ok:
                    normal_arm_failures += 1
                print(f"[REPLAY] ARM request: {'accepted' if ok else 'rejected'}", flush=True)

            # In PX4 SITL, use the documented MAV_CMD_COMPONENT_ARM_DISARM
            # force value only after ordinary arming has repeatedly failed.
            if (allow_force_arm and first_arm_attempt is not None
                    and now - first_arm_attempt >= 2.5
                    and normal_arm_failures >= 2
                    and now - last_force >= 1.5):
                last_force = now
                ok = node.request_force_arm()
                print(f"[REPLAY] FORCE ARM request: {'accepted' if ok else 'rejected/unavailable'}", flush=True)

            time.sleep(0.05)
            continue

        print(
            "[REPLAY] PX4 confirmed OFFBOARD + ARMED",
            flush=True,
        )
        return

    state = node.get_state()

    if state is None:
        text = "no MAVROS state"
    else:
        text = (
            f"mode={state.mode}, "
            f"armed={state.armed}, "
            f"connected={state.connected}"
        )

    raise RuntimeError(
        f"Could not ARM/OFFBOARD: {text}"
    )


def load_trajectory(source):
    path = (
        source
        / "trajectory"
        / "trajectory.csv"
    )

    if not path.exists():
        raise FileNotFoundError(
            f"Missing: {path}"
        )

    rows = []

    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        fields = set(
            reader.fieldnames or []
        )

        required = {
            "timestamp_start",
            "world_x",
            "world_y",
            "world_z",
            "world_roll_deg",
            "world_pitch_deg",
            "world_yaw_deg",
        }

        if not required.issubset(fields):
            raise RuntimeError(
                "This flight has an old trajectory format. "
                "Record a new source flight with the new record_raw.py."
            )

        for row in reader:
            rows.append({
                "t": float(
                    row["timestamp_start"]
                ),
                "x": float(row["world_x"]),
                "y": float(row["world_y"]),
                "z": float(row["world_z"]),
                "roll": float(
                    row["world_roll_deg"]
                ),
                "pitch": float(
                    row["world_pitch_deg"]
                ),
                "yaw": float(
                    row["world_yaw_deg"]
                ),
            })

    if len(rows) < 2:
        raise RuntimeError(
            "Trajectory must contain at least two samples"
        )

    t0 = rows[0]["t"]

    for row in rows:
        row["t"] -= t0

    # Unwrap Euler angles once so cubic interpolation never jumps
    # through +/-180 degrees.
    for key in ("roll", "pitch", "yaw"):
        prev = rows[0][key]

        for i in range(1, len(rows)):
            raw = rows[i][key]
            delta = normalize_angle_deg(raw - prev)
            rows[i][key] = prev + delta
            prev = rows[i][key]

    return rows


def catmull_rom(p0, p1, p2, p3, u):
    """
    Uniform Catmull-Rom spline.
    Continuous first derivative across trajectory samples, which removes
    the visible piecewise-linear jerks from pose replay.
    """
    u2 = u * u
    u3 = u2 * u

    return 0.5 * (
        (2.0 * p1)
        + (-p0 + p2) * u
        + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * u2
        + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * u3
    )


def interpolate_trajectory(
    rows,
    t,
    cursor,
):
    # Cursor avoids searching from the start at every 100 Hz tick.
    while (
        cursor + 1 < len(rows)
        and rows[cursor + 1]["t"] <= t
    ):
        cursor += 1

    if cursor + 1 >= len(rows):
        return dict(rows[-1]), cursor

    i1 = cursor
    i2 = cursor + 1
    i0 = max(0, i1 - 1)
    i3 = min(len(rows) - 1, i2 + 1)

    p0 = rows[i0]
    p1 = rows[i1]
    p2 = rows[i2]
    p3 = rows[i3]

    dt = p2["t"] - p1["t"]

    if dt <= 0.0:
        u = 0.0
    else:
        u = max(
            0.0,
            min(1.0, (t - p1["t"]) / dt),
        )

    pose = {
        "t": t,
        "x": catmull_rom(
            p0["x"], p1["x"], p2["x"], p3["x"], u
        ),
        "y": catmull_rom(
            p0["y"], p1["y"], p2["y"], p3["y"], u
        ),
        "z": catmull_rom(
            p0["z"], p1["z"], p2["z"], p3["z"], u
        ),
        "roll": normalize_angle_deg(
            catmull_rom(
                p0["roll"], p1["roll"], p2["roll"], p3["roll"], u
            )
        ),
        "pitch": normalize_angle_deg(
            catmull_rom(
                p0["pitch"], p1["pitch"], p2["pitch"], p3["pitch"], u
            )
        ),
        "yaw": normalize_angle_deg(
            catmull_rom(
                p0["yaw"], p1["yaw"], p2["yaw"], p3["yaw"], u
            )
        ),
    }

    return pose, cursor



class CoordinateAugmenter:
    """Deterministic smooth coordinate / yaw augmentation.

    Constant offsets plus a bounded low-frequency wobble.  The same function is
    used for Gazebo pose replay, saved world trajectory and synthetic local odom.
    """
    def __init__(self, duration, seed=42003):
        self.duration = max(1e-6, float(duration))
        self.lock = threading.RLock()
        self.enabled = True
        self.frame = 'body'
        self.dx = 0.0
        self.dy = 0.0
        self.dz = 0.0
        self.dyaw = 0.0
        self.x_amp = 0.05
        self.y_amp = 0.05
        self.z_amp = 0.025
        self.yaw_amp = 1.5
        self.timescale = 4.0
        self.seed = int(seed)
        self._reseed()

    def _reseed(self):
        rng = random.Random(int(self.seed))
        self._phases = {
            key: [rng.uniform(0.0, 2.0 * math.pi) for _ in range(3)]
            for key in ('x','y','z','yaw')
        }
        self._freq_jitter = {
            key: [rng.uniform(0.90, 1.10) for _ in range(3)]
            for key in ('x','y','z','yaw')
        }

    def snapshot(self):
        with self.lock:
            return {
                'enabled': bool(self.enabled), 'frame': self.frame,
                'dx': self.dx, 'dy': self.dy, 'dz': self.dz,
                'dyaw_deg': self.dyaw,
                'x_amp': self.x_amp, 'y_amp': self.y_amp,
                'z_amp': self.z_amp, 'yaw_amp_deg': self.yaw_amp,
                'timescale_sec': self.timescale, 'seed': self.seed,
                'envelope_duration_sec': self.duration,
                'wobble_time_basis': 'replay_wall_time',
            }

    def set_duration(self, duration):
        # Only the start/end envelope follows the replay duration.  The actual
        # wobble sinusoids use replay seconds directly, so a 4 s wobble stays
        # 4 s at 0.5x, 1x and 2x flight speed.
        with self.lock:
            self.duration = max(1e-6, float(duration))

    def update(self, data):
        with self.lock:
            if 'enabled' in data: self.enabled = bool(data['enabled'])
            if 'frame' in data:
                f = str(data['frame']).lower()
                if f not in ('world','body'): raise ValueError('frame must be world or body')
                self.frame = f
            for attr, key in (
                ('dx','dx'),('dy','dy'),('dz','dz'),('dyaw','dyaw_deg'),
                ('x_amp','x_amp'),('y_amp','y_amp'),('z_amp','z_amp'),
                ('yaw_amp','yaw_amp_deg'),('timescale','timescale_sec')):
                if key in data: setattr(self, attr, float(data[key]))
            self.timescale = max(0.5, float(self.timescale))
            if 'seed' in data:
                self.seed = int(data['seed'])
                self._reseed()

    def reset(self):
        with self.lock:
            self.dx=self.dy=self.dz=self.dyaw=0.0
            self.x_amp=self.y_amp=0.05
            self.z_amp=0.025
            self.yaw_amp=1.5
            self.timescale=4.0
            self.frame='body'
            self.enabled=True
            self.seed=42003
            self._reseed()

    def randomize_safe(self, seed=None):
        with self.lock:
            if seed is not None: self.seed=int(seed)
            rng=random.Random(self.seed)
            self.dx=rng.uniform(-0.15,0.15)
            self.dy=rng.uniform(-0.15,0.15)
            self.dz=rng.uniform(-0.07,0.07)
            self.dyaw=rng.uniform(-3.0,3.0)
            self.x_amp=rng.uniform(0.025,0.07)
            self.y_amp=rng.uniform(0.025,0.07)
            self.z_amp=rng.uniform(0.01,0.04)
            self.yaw_amp=rng.uniform(0.7,2.5)
            self.timescale=rng.uniform(3.0,6.0)
            self.frame='body'
            self.enabled=True
            self._reseed()

    def _wobble_component(self, key, amp, t):
        if amp == 0.0: return 0.0
        # Envelope guarantees zero wobble at the beginning / end while constant
        # offsets remain active for the whole replay.
        u=max(0.0,min(1.0,float(t)/self.duration))
        envelope=math.sin(math.pi*u)**2
        base=1.0/max(0.5,self.timescale)
        multipliers=(0.55,1.0,1.75)
        weights=(0.55,0.30,0.15)
        value=0.0
        for i,(m,w) in enumerate(zip(multipliers,weights)):
            f=base*m*self._freq_jitter[key][i]
            p=self._phases[key][i]
            value += w*math.sin(2.0*math.pi*f*t+p)
        return float(amp)*envelope*value

    def offsets(self, t):
        with self.lock:
            if not self.enabled:
                return {'dx':0.0,'dy':0.0,'dz':0.0,'dyaw':0.0,'frame':self.frame}
            return {
                'dx': self.dx + self._wobble_component('x', self.x_amp, t),
                'dy': self.dy + self._wobble_component('y', self.y_amp, t),
                'dz': self.dz + self._wobble_component('z', self.z_amp, t),
                'dyaw': self.dyaw + self._wobble_component('yaw', self.yaw_amp, t),
                'frame': self.frame,
            }

    def apply_pose(self, pose, t=None):
        t=float(pose.get('t',0.0) if t is None else t)
        off=self.offsets(t)
        out=dict(pose)
        dx,dy=off['dx'],off['dy']
        if off['frame']=='body':
            yaw=math.radians(float(pose['yaw']))
            dx,dy=(dx*math.cos(yaw)-dy*math.sin(yaw),
                   dx*math.sin(yaw)+dy*math.cos(yaw))
        out['x']=float(pose['x'])+dx
        out['y']=float(pose['y'])+dy
        out['z']=float(pose['z'])+off['dz']
        out['yaw']=normalize_angle_deg(float(pose['yaw'])+off['dyaw'])
        out['t']=t
        return out

    def generate_trajectory(self, rows):
        return [self.apply_pose(r, r['t']) for r in rows]

    def generate_scaled_trajectory(self, rows, speed_scale):
        speed = max(1e-6, float(speed_scale))
        out = []
        for row in rows:
            replay_t = float(row['t']) / speed
            aug = self.apply_pose(row, replay_t)
            aug['t'] = replay_t
            out.append(aug)
        return out


def speed_output_dir(raw_dir, source):
    """Create the next ordinary numeric flight copy.

    Examples:
        flight-20260921-061133     -> flight-20260921-061133-1
        flight-20260921-061133-4   -> flight-20260921-061133-5

    Legacy augmentation suffixes are accepted as source names and stripped:
        flight-20260921-061133-4-CB2     -> flight-20260921-061133-5
        flight-20260921-061133-4-SP3     -> flight-20260921-061133-5
        flight-20260921-061133-4-CB2-SP1 -> flight-20260921-061133-5

    If the preferred target already exists and contains data, the next free
    numeric index is selected. Empty/incomplete target directories may be
    reused after removal, matching the previous replay behavior.
    """
    name = source.name

    # Accept old CB/SP generated folders as inputs, but do not propagate those
    # suffixes into new dataset names.
    canonical = re.sub(r'(?:-(?:CB|SP)\d+)+$', '', name)

    m = re.fullmatch(
        r'(flight-\d{8}-\d{6})(?:-(\d+))?',
        canonical,
    )
    if m is None:
        raise RuntimeError(
            f'Invalid source flight name: {name!r}. '
            'Expected flight-YYYYMMDD-HHMMSS[-N] '
            '(legacy trailing -CBN/-SPN is also accepted)'
        )

    base = m.group(1)
    source_idx = int(m.group(2)) if m.group(2) is not None else 0
    idx = source_idx + 1

    while True:
        candidate = raw_dir / f'{base}-{idx}'

        if not candidate.exists():
            return candidate

        meaningful = any(
            (candidate / p).exists() and (candidate / p).stat().st_size > 0
            for p in (
                'forward/video.mp4',
                'bottom/video.mp4',
                'odom/odom.csv',
                'trajectory/trajectory.csv',
            )
        )

        if not meaningful:
            shutil.rmtree(candidate, ignore_errors=True)
            return candidate

        idx += 1

def _first_odom_row(source):
    path=source/'odom'/'odom.csv'
    if not path.exists(): raise RuntimeError(f'Missing source odom: {path}')
    with open(path,newline='') as f:
        r=csv.DictReader(f)
        row=next(r,None)
    if row is None: raise RuntimeError('Source odom.csv is empty')
    return {k:float(row[k]) for k in ('x','y','z','yaw_deg')}


def load_frame_reference(source, trajectory):
    p=source/'trajectory'/'initial_state.json'
    if p.exists():
        try:
            data=json.loads(p.read_text())
            ml=data['mavros_local']; gw=data['gazebo_world']
            return ({'x':float(ml['x']),'y':float(ml['y']),'z':float(ml['z']),'yaw_deg':float(ml['yaw_deg'])},
                    {'x':float(gw['x']),'y':float(gw['y']),'z':float(gw['z']),'yaw_deg':float(gw['yaw_deg'])})
        except Exception:
            pass
    local=_first_odom_row(source)
    world={'x':trajectory[0]['x'],'y':trajectory[0]['y'],'z':trajectory[0]['z'],'yaw_deg':trajectory[0]['yaw']}
    return local,world


def world_to_source_local(world_pose, ref_local, ref_world):
    theta_deg=normalize_angle_deg(float(ref_world['yaw_deg'])-float(ref_local['yaw_deg']))
    th=math.radians(theta_deg)
    dx=float(world_pose['x'])-float(ref_world['x'])
    dy=float(world_pose['y'])-float(ref_world['y'])
    dz=float(world_pose['z'])-float(ref_world['z'])
    dlx=math.cos(th)*dx+math.sin(th)*dy
    dly=-math.sin(th)*dx+math.cos(th)*dy
    return {
        'x':float(ref_local['x'])+dlx,
        'y':float(ref_local['y'])+dly,
        'z':float(ref_local['z'])+dz,
        'yaw_deg':normalize_angle_deg(float(world_pose['yaw'])-theta_deg),
    }


def write_synthetic_odom(source, output, base_trajectory, augmenter, speed_scale):
    """Write local odometry coherent with the speed-scaled augmented world path.

    Source odom timestamps live on the source-flight clock.  Output timestamps
    are divided by speed_scale, while coordinate wobble is evaluated on the
    *output / replay* clock, per the chosen semantics.
    """
    speed = max(1e-6, float(speed_scale))
    src = source / 'odom' / 'odom.csv'
    dst = output / 'odom' / 'odom.csv'
    dst.parent.mkdir(parents=True, exist_ok=True)
    ref_local, ref_world = load_frame_reference(source, base_trajectory)

    with open(src, newline='') as f:
        rows = list(csv.DictReader(f))

    source_duration = float(base_trajectory[-1]['t'])
    cursor = 0
    with open(dst, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['sample_index', 'timestamp_start', 'x', 'y', 'z', 'yaw_deg'])

        for i, row in enumerate(rows):
            source_t = float(row['timestamp_start'])
            source_t = max(0.0, min(source_duration, source_t))
            replay_t = source_t / speed

            base, cursor = interpolate_trajectory(
                base_trajectory, source_t, cursor
            )
            aug = augmenter.apply_pose(base, replay_t)
            loc = world_to_source_local(aug, ref_local, ref_world)

            w.writerow([
                row.get('sample_index', i),
                f4(replay_t),
                f4(loc['x']),
                f4(loc['y']),
                f4(loc['z']),
                f4(loc['yaw_deg']),
            ])

    # Make initial_state coherent with the augmented start and synthetic local pose.
    aug0 = augmenter.apply_pose(base_trajectory[0], 0.0)
    loc0 = world_to_source_local(aug0, ref_local, ref_world)
    init = {
        'mavros_local': {
            **loc0,
            'roll_deg': float(base_trajectory[0]['roll']),
            'pitch_deg': float(base_trajectory[0]['pitch']),
        },
        'gazebo_world': {
            'x': aug0['x'],
            'y': aug0['y'],
            'z': aug0['z'],
            'roll_deg': aug0['roll'],
            'pitch_deg': aug0['pitch'],
            'yaw_deg': aug0['yaw'],
        },
        'gazebo_model_name': GZ_MODEL_NAME,
        'trajectory_frame': 'gazebo_world',
        'augmentation': 'coordinate_bias+time_scale',
        'speed_scale': speed,
    }
    tdir = output / 'trajectory'
    tdir.mkdir(parents=True, exist_ok=True)
    with open(tdir / 'initial_state.json', 'w') as f:
        json.dump(init, f, indent=2)


def scale_annotations(source, output, speed_scale):
    """Copy known timestamp-based annotations onto the scaled replay clock."""
    speed = max(1e-6, float(speed_scale))
    src = source / 'annotations' / 'subprograms.json'
    if not src.exists():
        return None

    try:
        data = json.loads(src.read_text())
    except Exception as exc:
        print(f'[SPEED WARNING] cannot read annotations: {exc}', flush=True)
        return None

    segments = data.get('segments', [])
    for seg in segments:
        if 'start_timestamp' in seg:
            seg['start_timestamp'] = float(seg['start_timestamp']) / speed
        if 'end_timestamp' in seg:
            seg['end_timestamp'] = float(seg['end_timestamp']) / speed

    data['flight'] = output.name
    data['time_basis'] = (
        'timestamp_start scaled by flight speed; '
        f'source={source.name}; speed_scale={speed:.6f}'
    )
    data['speed_scale'] = speed
    data['source_flight'] = source.name

    dst_dir = output / 'annotations'
    dst_dir.mkdir(parents=True, exist_ok=True)
    dst = dst_dir / 'subprograms.json'
    with open(dst, 'w') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

    print(f'[SPEED] scaled annotations: {dst}', flush=True)
    return dst


def save_coord_manifest(output, augmenter, source):
    aug = output / 'augmentation'
    aug.mkdir(parents=True, exist_ok=True)
    data = {
        'type': 'coordinate_bias',
        'source_flight': source.name,
        **augmenter.snapshot(),
    }
    with open(aug / 'coord_bias.json', 'w') as f:
        json.dump(data, f, indent=2)
    return aug / 'coord_bias.json'


def save_speed_manifest(output, source, speed_scale, source_duration):
    speed = max(1e-6, float(speed_scale))
    aug = output / 'augmentation'
    aug.mkdir(parents=True, exist_ok=True)
    data = {
        'type': 'time_scale',
        'source_flight': source.name,
        'speed_scale': speed,
        'source_duration_sec': float(source_duration),
        'output_duration_sec': float(source_duration) / speed,
        'wobble_time_basis': 'replay_wall_time',
        'note': (
            'Coordinate wobble periods are measured in output replay seconds. '
            'A 4 s wobble remains 4 s regardless of speed_scale.'
        ),
    }
    path = aug / 'speed.json'
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)
    return path


def replay_output_dir(raw_dir, source):
    return speed_output_dir(raw_dir, source)


def pin_pose(
    gazebo,
    pose,
    duration,
):
    period = 1.0 / POSE_REPLAY_HZ
    end = time.monotonic() + duration
    next_tick = time.monotonic()

    while time.monotonic() < end:
        gazebo.set_pose(pose)

        next_tick += period
        delay = next_tick - time.monotonic()

        if delay > 0:
            time.sleep(delay)
        else:
            next_tick = time.monotonic()



# ============================================================
# INTERACTIVE WEB REPLAY CONTROL CENTER
# ============================================================


def direction_to_azimuth_elevation(direction):
    x, y, z = normalize_vec3(direction)
    azimuth = math.degrees(math.atan2(y, x))
    elevation = math.degrees(math.asin(clamp(-z, 0.0, 1.0)))
    return azimuth, elevation


def azimuth_elevation_to_direction(azimuth_deg, elevation_deg):
    az = math.radians(float(azimuth_deg))
    el = math.radians(clamp(float(elevation_deg), 1.0, 89.0))
    horizontal = math.cos(el)
    return normalize_vec3((
        horizontal * math.cos(az),
        horizontal * math.sin(az),
        -math.sin(el),
    ))


def rgb_to_hex(rgb):
    rgb = color3(rgb)
    return '#%02x%02x%02x' % tuple(int(round(v * 255.0)) for v in rgb)


def image_msg_to_bmp(msg):
    """Convert ROS Image to browser-displayable 24-bit BMP."""
    import struct
    width = int(msg.width); height = int(msg.height); enc = str(msg.encoding)
    table = {'rgb8':3,'bgr8':3,'rgba8':4,'bgra8':4,'mono8':1,'8UC1':1,'8UC3':3,'8UC4':4}
    if enc not in table or width <= 0 or height <= 0:
        raise ValueError(f'Unsupported preview encoding: {enc}')
    bpp = table[enc]; raw = image_bytes_without_padding(msg, bpp)
    row_bytes = width*3; pad=(4-(row_bytes%4))%4
    try:
        import numpy as np
        a=np.frombuffer(raw,dtype=np.uint8).reshape(height,width,bpp)
        if enc == 'rgb8': bgr=a[:,:,[2,1,0]]
        elif enc in ('bgr8','8UC3'): bgr=a[:,:,:3]
        elif enc == 'rgba8': bgr=a[:,:,[2,1,0]]
        elif enc in ('bgra8','8UC4'): bgr=a[:,:,:3]
        else:
            mono=a[:,:,0]; bgr=np.repeat(mono[:,:,None],3,axis=2)
        bgr=bgr[::-1,:,:]
        if pad:
            out=np.zeros((height,row_bytes+pad),dtype=np.uint8); out[:,:row_bytes]=bgr.reshape(height,row_bytes); pixels=out.tobytes()
        else: pixels=bgr.tobytes()
    except Exception:
        pixels=bytearray((row_bytes+pad)*height); dst=0
        for y in range(height-1,-1,-1):
            row=raw[y*width*bpp:(y+1)*width*bpp]
            if enc in ('bgr8','8UC3'):
                pixels[dst:dst+row_bytes]=row; dst+=row_bytes
            elif enc == 'rgb8':
                for i in range(0,len(row),3): pixels[dst:dst+3]=bytes((row[i+2],row[i+1],row[i])); dst+=3
            elif enc in ('bgra8','8UC4'):
                for i in range(0,len(row),4): pixels[dst:dst+3]=row[i:i+3]; dst+=3
            elif enc == 'rgba8':
                for i in range(0,len(row),4): pixels[dst:dst+3]=bytes((row[i+2],row[i+1],row[i])); dst+=3
            else:
                for v in row: pixels[dst:dst+3]=bytes((v,v,v)); dst+=3
            dst+=pad
        pixels=bytes(pixels)
    image_size=len(pixels); file_size=54+image_size
    return struct.pack('<2sIHHI',b'BM',file_size,0,0,54)+struct.pack('<IiiHHIIiiII',40,width,height,1,24,0,image_size,2835,2835,0,0)+pixels


class InteractiveReplayNode(ReplayNode):
    def __init__(self):
        self.latest_forward = None
        self.latest_bottom = None
        self.forward_seq = 0
        self.bottom_seq = 0
        self.odom_seq = 0
        self.state_seq = 0
        self.preview_cache = {
            'forward': (-1, None),
            'bottom': (-1, None),
        }
        super().__init__()

    def forward_callback(self, msg):
        with self.lock:
            self.latest_forward = msg
            self.forward_seq += 1
        super().forward_callback(msg)

    def bottom_callback(self, msg):
        with self.lock:
            self.latest_bottom = msg
            self.bottom_seq += 1
        super().bottom_callback(msg)

    def odom_callback(self, msg):
        with self.lock: self.odom_seq += 1
        super().odom_callback(msg)

    def state_callback(self, msg):
        with self.lock: self.state_seq += 1
        super().state_callback(msg)

    def topic_counters(self):
        with self.lock:
            return {'forward': int(self.forward_seq), 'bottom': int(self.bottom_seq), 'odom': int(self.odom_seq), 'state': int(self.state_seq)}

    def camera_bmp(self, which):
        with self.lock:
            if which == 'forward':
                msg = self.latest_forward
                seq = self.forward_seq
            else:
                msg = self.latest_bottom
                seq = self.bottom_seq
            cached_seq, cached = self.preview_cache[which]

        if msg is None:
            return None
        if cached_seq == seq and cached is not None:
            return cached

        bmp = image_msg_to_bmp(msg)
        with self.lock:
            self.preview_cache[which] = (seq, bmp)
        return bmp


class ReplayControl:
    def __init__(self, source_duration, speed_scale=1.0):
        self.source_duration = max(1e-6, float(source_duration))
        self.lock = threading.RLock()
        self.t = 0.0
        self.paused = True
        self.speed = max(1e-6, float(speed_scale))
        self.duration = self.source_duration / self.speed
        self.phase = 'preview'
        self.record_requested = False
        self.quit_requested = False
        self.last_output = None
        self.message = 'Preview ready'
        self.current_pose = None
        self.record_started_at = None
        self.motion_ready = False

    def current_time(self):
        with self.lock:
            return float(self.t)

    def source_time(self):
        with self.lock:
            return min(self.source_duration, float(self.t) * float(self.speed))

    def snapshot(self):
        with self.lock:
            return {
                'time': float(self.t),
                'duration': float(self.duration),
                'source_duration': float(self.source_duration),
                'paused': bool(self.paused),
                'speed': float(self.speed),
                'phase': self.phase,
                'last_output': self.last_output,
                'message': self.message,
                'current_pose': dict(self.current_pose) if self.current_pose else None,
                'motion_ready': bool(self.motion_ready),
            }

    def play(self):
        with self.lock:
            if not self.motion_ready:
                self.message = 'ARM/OFFBOARD is required before trajectory motion'
                return False
            if self.phase != 'recording':
                if self.t >= self.duration:
                    self.t = 0.0
                self.paused = False
                self.phase = 'preview'
                self.message = f'Preview playing at {self.speed:.2f}× flight speed'
                return True
            return False

    def pause(self):
        with self.lock:
            if self.phase != 'recording':
                self.paused = True
                self.message = 'Preview paused'

    def seek(self, value):
        with self.lock:
            if self.phase == 'recording' or not self.motion_ready:
                if not self.motion_ready:
                    self.message = 'ARM/OFFBOARD is required before trajectory motion'
                return False
            self.t = max(0.0, min(self.duration, float(value)))
            self.phase = 'preview'
            self.message = f'Preview seek: {self.t:.2f}s'
            return True

    def set_speed(self, value):
        """Set physical flight time scale while preserving current route position."""
        with self.lock:
            if self.phase == 'recording':
                return False
            new_speed = float(value)
            if not math.isfinite(new_speed) or new_speed <= 0.0:
                raise ValueError('Flight speed must be a finite value > 0')
            source_t = min(self.source_duration, self.t * self.speed)
            self.speed = new_speed
            self.duration = self.source_duration / self.speed
            self.t = min(self.duration, source_t / self.speed)
            self.message = (
                f'Flight speed: {self.speed:.2f}×; '
                f'duration {self.duration:.2f}s'
            )
            return True

    def request_record(self):
        with self.lock:
            if self.phase == 'recording':
                return False
            self.record_requested = True
            self.paused = True
            self.message = 'Record requested...'
            return True

    def consume_record_request(self):
        with self.lock:
            value = self.record_requested
            self.record_requested = False
            return value

    def start_recording(self, output):
        with self.lock:
            self.t = 0.0
            self.paused = False
            self.phase = 'recording'
            self.last_output = str(output)
            self.record_started_at = time.monotonic()
            self.message = (
                f'Recording to {output.name} at {self.speed:.2f}×'
            )

    def finish_recording(self):
        with self.lock:
            self.t = self.duration
            self.paused = True
            self.phase = 'preview'
            self.message = 'Recording complete; preview unlocked'

    def request_quit(self):
        with self.lock:
            self.quit_requested = True
            self.message = 'Stopping...'


class InteractiveLightingController(LightingController):
    def __init__(self, env, keyframes, duration, time_provider,
                 update_hz=5.0, sky_enabled=True):
        super().__init__(env, keyframes, duration, update_hz, sky_enabled)
        self.time_provider = time_provider

    def current_time(self):
        return max(0.0, min(self.duration, float(self.time_provider())))

    def start(self, route_start=None):
        self.route_start = time.monotonic()
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def set_duration(self, duration, rescale_keyframes=True):
        new_duration = max(1e-6, float(duration))
        with self.lock:
            old_duration = max(1e-6, float(self.duration))
            if rescale_keyframes and self.keyframes:
                ratio = new_duration / old_duration
                for row in self.keyframes:
                    row['t'] = clamp(float(row['t']) * ratio, 0.0, new_duration)
                self.keyframes.sort(key=lambda x: x['t'])
            self.duration = new_duration
            self.events.append({
                't': round(self.current_time(), 4),
                'command': 'duration_rescale',
                'duration': new_duration,
            })

    def keyframes_snapshot(self):
        with self.lock:
            return [
                {
                    **row,
                    'sun_rgb': list(row['sun_rgb']),
                    'sky_rgb': list(row['sky_rgb']),
                    'sun_direction': list(row['sun_direction']),
                }
                for row in self.keyframes
            ]

    def replace_keyframes(self, frames, event_name='replace_keyframes'):
        resolved = resolve_keyframe_times(frames, self.duration)
        if not resolved:
            raise ValueError('At least one lighting keyframe is required')
        with self.lock:
            self.keyframes = resolved
            self.auto_enabled = True
            self.manual.clear()
            self.events.append({
                't': round(self.current_time(), 4),
                'command': event_name,
                'count': len(resolved),
            })

    def add_keyframe_from_current(self):
        state = self.state()
        if state is None:
            return
        row = {
            't': self.current_time(),
            'sun_intensity': float(state['sun_intensity']),
            'sun_rgb': color3(state['sun_rgb']),
            'sky_rgb': color3(state['sky_rgb']),
            'sun_direction': normalize_vec3(state['sun_direction']),
        }
        with self.lock:
            # Replace a keyframe within 50 ms, otherwise insert a new one.
            nearest = None
            nearest_dist = None
            for i, old in enumerate(self.keyframes):
                d = abs(float(old['t']) - row['t'])
                if nearest_dist is None or d < nearest_dist:
                    nearest, nearest_dist = i, d
            if nearest is not None and nearest_dist <= 0.05:
                self.keyframes[nearest] = row
            else:
                self.keyframes.append(row)
                self.keyframes.sort(key=lambda x: x['t'])
            self.manual.clear()
            self.auto_enabled = True
            self.events.append({
                't': round(row['t'], 4),
                'command': 'add_keyframe',
            })

    def create_keyframe(self, row):
        row = dict(row)
        row['t'] = clamp(float(row['t']), 0.0, self.duration)
        row['sun_intensity'] = max(0.0, float(row['sun_intensity']))
        row['sun_rgb'] = color3(row['sun_rgb'])
        row['sky_rgb'] = color3(row['sky_rgb'])
        row['sun_direction'] = normalize_vec3(row['sun_direction'])

        with self.lock:
            # Keep the time axis well-defined: an explicitly created point
            # within 10 ms of an existing point replaces that point.
            replace_i = None
            for i, old in enumerate(self.keyframes):
                if abs(float(old['t']) - row['t']) <= 0.01:
                    replace_i = i
                    break
            if replace_i is None:
                self.keyframes.append(row)
                command = 'create_keyframe'
            else:
                self.keyframes[replace_i] = row
                command = 'replace_keyframe_at_same_time'
            self.keyframes.sort(key=lambda x: x['t'])
            new_index = min(
                range(len(self.keyframes)),
                key=lambda i: abs(float(self.keyframes[i]['t']) - row['t']),
            )
            self.manual.clear()
            self.auto_enabled = True
            self.events.append({
                't': round(self.current_time(), 4),
                'command': command,
                'keyframe_t': round(row['t'], 4),
            })
            return new_index

    def update_keyframe(self, index, row):
        row = dict(row)
        row['t'] = clamp(float(row['t']), 0.0, self.duration)
        row['sun_intensity'] = max(0.0, float(row['sun_intensity']))
        row['sun_rgb'] = color3(row['sun_rgb'])
        row['sky_rgb'] = color3(row['sky_rgb'])
        row['sun_direction'] = normalize_vec3(row['sun_direction'])

        with self.lock:
            index = int(index)
            if index < 0 or index >= len(self.keyframes):
                raise IndexError('Keyframe index out of range')
            old_t = float(self.keyframes[index]['t'])
            self.keyframes[index] = row
            self.keyframes.sort(key=lambda x: x['t'])
            new_index = min(
                range(len(self.keyframes)),
                key=lambda i: abs(float(self.keyframes[i]['t']) - row['t']),
            )
            self.manual.clear()
            self.auto_enabled = True
            self.events.append({
                't': round(self.current_time(), 4),
                'command': 'update_keyframe',
                'old_t': round(old_t, 4),
                'new_t': round(row['t'], 4),
            })
            return new_index

    def delete_keyframe(self, index):
        with self.lock:
            index = int(index)
            if len(self.keyframes) <= 1:
                raise ValueError('Cannot delete the last keyframe')
            if index < 0 or index >= len(self.keyframes):
                raise IndexError('Keyframe index out of range')
            old = self.keyframes.pop(index)
            self.events.append({
                't': round(self.current_time(), 4),
                'command': 'delete_keyframe',
                'deleted_t': float(old['t']),
            })

    def set_preset(self, name, seed=0):
        frames = preset_keyframes(name, seed)
        if not frames:
            frames = preset_keyframes('constant', seed)
        self.replace_keyframes(frames, event_name=f'preset:{name}')


REPLAY_UI_HTML = r'''<!doctype html>
<html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Concept-VLA · Replay Speed</title>
<style>
:root{color-scheme:dark;--bg:#0d1117;--panel:#161b22;--panel2:#0f141b;--line:#30363d;--text:#e6edf3;--muted:#8b949e;--blue:#1f6feb;--green:#238636;--red:#da3633;--amber:#9e6a03}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:14px system-ui,-apple-system,sans-serif}header{position:sticky;top:0;z-index:5;background:#0d1117ee;backdrop-filter:blur(8px);border-bottom:1px solid var(--line);padding:12px 18px;display:flex;gap:9px;align-items:center;flex-wrap:wrap}.title{font-size:17px;font-weight:750;margin-right:auto}.badge{background:#21262d;border:1px solid #30363d;border-radius:999px;padding:4px 9px;font-size:12px}.ok{background:#12361f;border-color:#238636}.warn{background:#4a3510;border-color:#9e6a03}.bad{background:#4b1718;border-color:#da3633}.wrap{max-width:1250px;margin:0 auto;padding:16px}.grid{display:grid;grid-template-columns:minmax(0,1.15fr) minmax(360px,.85fr);gap:14px}.panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;overflow:hidden;margin-bottom:14px}.panel h2{font-size:14px;margin:0;padding:10px 12px;background:#1c2128;border-bottom:1px solid var(--line)}.body{padding:12px}.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin:8px 0}.stack{display:grid;gap:9px}.grid2{display:grid;grid-template-columns:1fr 1fr;gap:10px}.control{background:var(--panel2);border:1px solid var(--line);border-radius:8px;padding:9px}.control label{display:flex;justify-content:space-between;gap:8px;margin-bottom:6px}.muted{color:var(--muted);font-size:12px}.msg{background:#0f141b;border:1px solid var(--line);padding:8px 10px;border-radius:7px;min-height:36px}.pose{font-family:ui-monospace,monospace;background:#0f141b;border:1px solid var(--line);padding:8px;border-radius:7px;white-space:pre-wrap}.timeline{display:grid;grid-template-columns:62px minmax(0,1fr) 62px;gap:8px;align-items:center}.timeline input{width:100%}button{background:#24292f;color:var(--text);border:1px solid #484f58;border-radius:7px;padding:8px 11px;cursor:pointer;font-weight:600}button:hover{border-color:#8b949e}button.primary{background:var(--blue);border-color:#388bfd}button.good{background:var(--green);border-color:#2ea043}button.danger{background:#8e1519;border-color:var(--red)}button.active{outline:2px solid #58a6ff;background:#1f3d61}button:disabled{opacity:.42;cursor:not-allowed}.big{padding:10px 16px;font-size:14px}.speedGroup{display:flex;gap:5px;flex-wrap:wrap}.speedGroup button{padding:6px 9px;font-weight:500}.colorControl{display:grid;grid-template-columns:auto 70px auto auto;gap:8px;align-items:center}.colorControl input[type=color]{width:70px;height:38px;background:none;border:0;cursor:pointer}.colorState{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-top:7px}.colorState>div{background:#0b1016;border:1px solid #252b33;border-radius:6px;padding:6px 8px}.colorState b{font-family:ui-monospace,monospace}.dirty{color:#f2cc60}.presetGrid{display:grid;grid-template-columns:repeat(5,1fr);gap:6px}.statusGrid{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}.statusCard{background:#0f141b;border:1px solid var(--line);border-radius:8px;padding:8px;text-align:center}.statusCard b{display:block;font-size:17px}.statusCard span{font-size:11px;color:var(--muted)}.keyframes{max-height:330px;overflow:auto}.kf{display:grid;grid-template-columns:62px minmax(0,1fr) auto auto auto;gap:7px;align-items:center;padding:7px;border-bottom:1px solid #272c33;border-radius:6px}.kf.selected{background:#16263a;outline:1px solid #388bfd}.swatch{height:8px;border-radius:4px;margin-bottom:3px}.smallBtn{padding:4px 7px;font-size:11px}.recordBox{border:1px solid #2ea043;background:#102719;border-radius:9px;padding:12px}.advanced summary{cursor:pointer;color:#c9d1d9;font-weight:650;padding:4px 0}.kfStrip{position:relative;height:34px;margin:10px 0 14px;background:#0b1016;border:1px solid var(--line);border-radius:7px}.kfStrip:before{content:"";position:absolute;left:8px;right:8px;top:16px;height:2px;background:#30363d}.kfMark{position:absolute;top:7px;width:18px;height:18px;border-radius:50%;border:2px solid #8b949e;background:#1f6feb;transform:translateX(-50%);padding:0;z-index:2}.kfMark.selected{background:#f2cc60;border-color:#fff;outline:2px solid #795e00}.editor{border:1px solid #388bfd55;background:#0e1825;border-radius:9px;padding:10px;margin-top:10px}.editorGrid{display:grid;grid-template-columns:1fr 1fr;gap:9px}.editorField{background:#0b1016;border:1px solid #252b33;border-radius:7px;padding:8px}.editorField label{display:block;color:var(--muted);font-size:12px;margin-bottom:5px}.editorField input[type=number]{width:100%;background:#0d1117;color:var(--text);border:1px solid #484f58;border-radius:6px;padding:7px}.editorField input[type=color]{width:100%;height:38px;background:none;border:0;cursor:pointer}.editorActions{display:flex;gap:7px;flex-wrap:wrap;margin-top:10px}.editorTitle{display:flex;justify-content:space-between;gap:8px;align-items:center}.editorTitle b{font-size:14px}.editorTitle span{font-size:12px;color:var(--muted)}@media(max-width:900px){.grid{grid-template-columns:1fr}.grid2{grid-template-columns:1fr}.presetGrid{grid-template-columns:repeat(2,1fr)}.statusGrid{grid-template-columns:repeat(2,1fr)}}
</style></head><body>
<header><span class="title">Concept‑VLA · Replay Speed + Coord</span><span class="badge" id="phase">loading</span><span class="badge" id="mavros">MAVROS: —</span><span class="badge" id="clock">0.00 / 0.00 s</span></header>
<div class="wrap">
<div class="panel"><h2>Состояние</h2><div class="body"><div class="statusGrid"><div class="statusCard"><b id="fCount">0</b><span>forward frames</span></div><div class="statusCard"><b id="bCount">0</b><span>bottom frames</span></div><div class="statusCard"><b id="oCount">0</b><span>odom msgs</span></div><div class="statusCard"><b id="sCount">0</b><span>state msgs</span></div></div><div class="row"><button id="arm">ARM + OFFBOARD retry</button><span class="muted">Перед ARM публикуется фиксированный setpoint на подъём; после ARM — hold-current, как в рабочей логике replay.</span></div></div></div>
<div class="grid"><main>
<div class="panel"><h2>1. Предпросмотр траектории</h2><div class="body">
<div class="row"><button id="play" class="primary">▶ Просмотр</button><button id="pause">⏸ Пауза</button><button id="restart">↶ В начало</button></div>
<div class="timeline"><span id="t0">0.00</span><input id="timeline" type="range" min="0" max="1" step="0.01"><span id="tdur">0.00</span></div>
<div class="control" style="margin-top:10px"><label><span>Скорость физического пролёта</span><b id="speedNow">1.00×</b></label>
<div class="row"><input id="flightSpeed" type="number" min="0.01" step="0.05" value="1.00" style="width:105px"><button id="speedApply" class="primary">Применить</button><button id="speedReset">↺ 1×</button></div>
<div class="speedGroup"><button class="flightSpeedPreset" data-v="0.5">0.5×</button><button class="flightSpeedPreset" data-v="0.75">0.75×</button><button class="flightSpeedPreset" data-v="1">1×</button><button class="flightSpeedPreset" data-v="1.25">1.25×</button><button class="flightSpeedPreset" data-v="1.5">1.5×</button><button class="flightSpeedPreset" data-v="2">2×</button><button class="flightSpeedPreset" data-v="4">4×</button><button class="flightSpeedPreset" data-v="8">8×</button><button class="flightSpeedPreset" data-v="16">16×</button></div>
<div class="editorGrid" style="margin-top:9px"><div class="editorField"><label>Random min</label><input id="speedMin" type="number" min="0.01" step="0.05" value="0.75"></div><div class="editorField"><label>Random max</label><input id="speedMax" type="number" min="0.01" step="0.05" value="1.50"></div><div class="editorField"><label>Random seed</label><input id="speedSeed" type="number" value="42003"></div><div class="editorField"><label>Длительность</label><div><b id="speedDuration">—</b></div></div></div>
<div class="row"><button id="speedRandom">🎲 Random speed</button></div>
<div class="muted" style="margin-top:6px">Верхнего ограничения скорости нет: можно ввести 5×, 10×, 20× и т.д. Preview и финальная запись используют выбранный коэффициент. Wobble остаётся привязан к реальному времени: T≈4 с остаётся 4 с при любой скорости. <b>Важно:</b> очень быстрый preview подходит для визуальной проверки маршрута, но не является строгой проверкой столкновений: pose задаётся дискретно с частотой 100 Гц, поэтому тонкое препятствие на большой скорости можно перескочить между двумя pose.</div></div>
<div id="message" class="msg" style="margin-top:10px">...</div><div id="lightHealth" class="muted" style="margin-top:5px"></div><div class="pose" id="pose" style="margin-top:8px">pose: —</div>
</div></div>
<div class="panel"><h2>2. Аугментация координат / yaw</h2><div class="body">
<div class="row"><label><input id="coordEnabled" type="checkbox" checked> Включить coordinate bias</label><label>Frame <select id="coordFrame"><option value="body">body</option><option value="world">world</option></select></label><label>Seed <input id="coordSeed" type="number" value="42003" style="width:110px"></label></div>
<div class="grid2">
<div class="control"><b>Постоянное смещение</b><div class="editorGrid" style="margin-top:8px"><div class="editorField"><label>DX, м</label><input id="coordDx" type="number" step="0.01"></div><div class="editorField"><label>DY, м</label><input id="coordDy" type="number" step="0.01"></div><div class="editorField"><label>DZ, м</label><input id="coordDz" type="number" step="0.01"></div><div class="editorField"><label>DYAW, °</label><input id="coordDyaw" type="number" step="0.1"></div></div></div>
<div class="control"><b>Плавное биение</b><div class="editorGrid" style="margin-top:8px"><div class="editorField"><label>X amp, м</label><input id="coordXamp" type="number" min="0" step="0.005"></div><div class="editorField"><label>Y amp, м</label><input id="coordYamp" type="number" min="0" step="0.005"></div><div class="editorField"><label>Z amp, м</label><input id="coordZamp" type="number" min="0" step="0.005"></div><div class="editorField"><label>Yaw amp, °</label><input id="coordYawAmp" type="number" min="0" step="0.1"></div><div class="editorField"><label>Характерное время, с</label><input id="coordTimescale" type="number" min="0.5" step="0.1"></div></div></div>
</div>
<div class="row"><button id="coordApply" class="primary">Применить</button><button id="coordRandom">🎲 Safe random</button><button id="coordReset">Сбросить</button><span class="muted">Preview и финальная запись используют один и тот же профиль. Биение плавное, с нулём на старте/финише.</span></div>
<div id="coordSummary" class="pose">coordinate bias: —</div>
</div></div>
<div class="panel"><h2>3. Финальная запись</h2><div class="body"><div class="recordBox"><div class="row"><button id="record" class="good big">● Record from start</button><button id="quit" class="danger">Остановить replay</button></div><div class="muted">Возвращает траекторию в t=0 и запускает exact replay на выбранной скорости / 100 Гц. Во время записи скорость, coordinate bias, seek и pause блокируются.</div></div></div></div>
</main><aside>
<div class="panel"><h2>Быстрые сценарии освещения</h2><div class="body">
<div class="muted" style="margin-bottom:8px">Одним нажатием заменяют текущий сценарий освещения. Детальная настройка выполняется ниже в редакторе динамического освещения.</div>
<div class="presetGrid"><button class="preset editLight" data-p="constant">☀ День</button><button class="preset editLight" data-p="cloud_pass">☁ Облака</button><button class="preset editLight" data-p="day_to_sunset">🌇 Закат</button><button class="preset editLight" data-p="day_to_night">🌙 Ночь</button><button id="random" class="editLight">🎲 Random</button></div>

<!-- Служебные legacy-контролы сохранены скрытыми, чтобы backend/UI state
     оставались полностью совместимыми с редактором сценариев. -->
<div style="display:none" aria-hidden="true">
  <input id="sunI" type="range" min="0" max="1.5" step="0.01"><span id="sunIVal"></span>
  <span id="sunAppliedHex"></span><input id="sunColor" type="color"><button id="applySun"></button><button id="resetSunColor"></button><span id="sunCurrent"></span><span id="sunDraft"></span>
  <span id="skyAppliedHex"></span><input id="skyColor" type="color"><button id="applySky"></button><button id="resetSkyColor"></button><span id="skyCurrent"></span><span id="skyDraft"></span>
  <input id="az" type="range" min="-180" max="180" step="1"><span id="azVal"></span>
  <input id="el" type="range" min="1" max="89" step="1"><span id="elVal"></span>
  <input id="auto" type="checkbox" checked><button id="clear"></button>
</div>
</div></div>
<div class="panel"><h2>Редактор динамического освещения</h2><div class="body">
<div class="muted">Точки ниже задают состояние света во времени. Между соседними точками значения интерполируются автоматически.</div>
<div id="kfStrip" class="kfStrip" title="Точки освещения на временной шкале"></div>
<div class="row"><button id="newKf" class="primary editLight">+ Новая точка здесь</button><button id="captureKf" class="editLight">Взять текущее освещение</button><button id="addKf" class="editLight">Быстро сохранить текущую сцену</button></div>
<div class="editor">
  <div class="editorTitle"><b id="kfEditorTitle">Новая точка</b><span id="kfEditorHint">не сохранена</span></div>
  <div class="editorGrid" style="margin-top:9px">
    <div class="editorField"><label>Время, с</label><input id="kfTime" class="editLight" type="number" min="0" step="0.01"></div>
    <div class="editorField"><label>Яркость солнца</label><input id="kfIntensity" class="editLight" type="number" min="0" max="3" step="0.01"></div>
    <div class="editorField"><label>Цвет солнца</label><input id="kfSunColor" class="editLight" type="color"></div>
    <div class="editorField"><label>Цвет неба / фона</label><input id="kfSkyColor" class="editLight" type="color"></div>
    <div class="editorField"><label>Азимут солнца, °</label><input id="kfAz" class="editLight" type="number" min="-180" max="180" step="1"></div>
    <div class="editorField"><label>Высота солнца, °</label><input id="kfEl" class="editLight" type="number" min="1" max="89" step="1"></div>
  </div>
  <div class="editorActions"><button id="saveKf" class="good editLight">Сохранить точку</button><button id="previewKf" class="editLight">Показать эти значения сейчас</button><button id="seekKf" class="editLight">Перейти ко времени точки</button><button id="duplicateKf" class="editLight">Дублировать</button><button id="deleteSelectedKf" class="danger editLight">Удалить</button><button id="cancelKf" class="smallBtn">Снять выбор</button></div>
</div>
<details class="advanced" open><summary>Все точки</summary><div id="keyframes" class="keyframes"></div></details>
</div></div>
</aside></div></div>
<script>
let S=null,dragging=false,lightTimer=null;let colorInit=false;let sunColorDirty=false,skyColorDirty=false;let appliedSunHex='#ffffff',appliedSkyHex='#ffffff';let selectedKfIndex=null,kfEditorDirty=false;const $=id=>document.getElementById(id);
async function post(action,payload={}){let r=await fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action,...payload})});let j=await r.json();if(!r.ok)throw new Error(j.error||'request failed');return j}
function hexToRgb(h){h=h.replace('#','');return [parseInt(h.slice(0,2),16)/255,parseInt(h.slice(2,4),16)/255,parseInt(h.slice(4,6),16)/255]}
function rgbToHex(a){return '#'+a.map(x=>Math.max(0,Math.min(255,Math.round(x*255))).toString(16).padStart(2,'0')).join('')}
function sendPartialLight(payload,delay=80){clearTimeout(lightTimer);lightTimer=setTimeout(()=>post('light_set',payload).catch(e=>$('lightHealth').textContent='LIGHT ERR: '+e),delay)}
function sendSunScalar(delay=80){sendPartialLight({sun_intensity:+$('sunI').value},delay)}
function sendDirection(delay=80){sendPartialLight({azimuth:+$('az').value,elevation:+$('el').value},delay)}
function applySunColor(){let h=$('sunColor').value;return post('light_set',{sun_rgb:hexToRgb(h)}).then(()=>{sunColorDirty=false;$('sunDraft').classList.remove('dirty')}).catch(e=>$('lightHealth').textContent='LIGHT ERR: '+e)}
function applySkyColor(){let h=$('skyColor').value;return post('light_set',{sky_rgb:hexToRgb(h)}).then(()=>{skyColorDirty=false;$('skyDraft').classList.remove('dirty')}).catch(e=>$('lightHealth').textContent='LIGHT ERR: '+e)}
function syncSunDraft(){sunColorDirty=false;$('sunColor').value=appliedSunHex;$('sunDraft').textContent=appliedSunHex;$('sunDraft').classList.remove('dirty')}
function syncSkyDraft(){skyColorDirty=false;$('skyColor').value=appliedSkyHex;$('skyDraft').textContent=appliedSkyHex;$('skyDraft').classList.remove('dirty')}
function currentLightingForEditor(){let l=S&&S.lighting||{},st=l.state||null;if(!st)return null;return {t:(S.control||{}).time||0,sun_intensity:+st.sun_intensity,sun_rgb:st.sun_rgb,sky_rgb:st.sky_rgb,azimuth:+(l.azimuth||0),elevation:+(l.elevation||45)}}
function setKfEditor(v,index=null,dirty=false){if(!v)return;selectedKfIndex=index;$('kfTime').value=(+v.t).toFixed(2);$('kfIntensity').value=(+v.sun_intensity).toFixed(2);$('kfSunColor').value=rgbToHex(v.sun_rgb);$('kfSkyColor').value=rgbToHex(v.sky_rgb);let az=v.azimuth,el=v.elevation;if((az===undefined||el===undefined)&&v.sun_direction){let x=v.sun_direction[0],y=v.sun_direction[1],z=v.sun_direction[2];az=Math.atan2(y,x)*180/Math.PI;el=Math.asin(Math.max(-1,Math.min(1,-z)))*180/Math.PI} $('kfAz').value=Math.round(az||0);$('kfEl').value=Math.round(el||45);kfEditorDirty=dirty;$('kfEditorTitle').textContent=index===null?'Новая точка':`Точка #${index+1}`;$('kfEditorHint').textContent=dirty?'есть несохранённые изменения':(index===null?'не сохранена':'сохранена');renderKeyframes((S&&S.lighting&&S.lighting.keyframes)||[])}
function editorPayload(){return {t:+$('kfTime').value,sun_intensity:+$('kfIntensity').value,sun_rgb:hexToRgb($('kfSunColor').value),sky_rgb:hexToRgb($('kfSkyColor').value),azimuth:+$('kfAz').value,elevation:+$('kfEl').value}}
function markEditorDirty(){kfEditorDirty=true;$('kfEditorHint').textContent='есть несохранённые изменения'}
function selectKf(i,seek=false){let k=S&&S.lighting&&S.lighting.keyframes&&S.lighting.keyframes[i];if(!k)return;let l=S.lighting;let x=k.sun_direction[0],y=k.sun_direction[1],z=k.sun_direction[2];let v={...k,azimuth:Math.atan2(y,x)*180/Math.PI,elevation:Math.asin(Math.max(-1,Math.min(1,-z)))*180/Math.PI};setKfEditor(v,i,false);if(seek)post('seek',{time:k.t}).catch(e=>$('message').textContent=e)}
function renderKfStrip(kfs){let root=$('kfStrip');root.innerHTML='';let dur=(S&&S.control&&S.control.duration)||1;kfs.forEach((k,i)=>{let b=document.createElement('button');b.className='kfMark'+(i===selectedKfIndex?' selected':'');b.style.left=(Math.max(0,Math.min(100,100*k.t/dur)))+'%';b.title=`#${i+1} · ${k.t.toFixed(2)} s`;b.onclick=()=>selectKf(i,true);root.appendChild(b)})}
function renderKeyframes(kfs){let root=$('keyframes');root.innerHTML='';kfs.forEach((k,i)=>{let d=document.createElement('div');d.className='kf'+(i===selectedKfIndex?' selected':'');d.innerHTML=`<b>${k.t.toFixed(2)}s</b><div><div class="swatch" style="background:${rgbToHex(k.sky_rgb)}"></div><span class="muted">sun ${k.sun_intensity.toFixed(2)} · ${rgbToHex(k.sun_rgb)} · sky ${rgbToHex(k.sky_rgb)}</span></div><button class="smallBtn">Редактировать</button><button class="smallBtn">Перейти</button><button class="smallBtn editLight">×</button>`;let bs=d.querySelectorAll('button');bs[0].onclick=()=>selectKf(i,false);bs[1].onclick=()=>selectKf(i,true);bs[2].onclick=async()=>{if(!confirm(`Удалить точку #${i+1} (${k.t.toFixed(2)}s)?`))return;await post('delete_keyframe',{index:i});if(selectedKfIndex===i)selectedKfIndex=null;else if(selectedKfIndex!==null&&selectedKfIndex>i)selectedKfIndex--;};root.appendChild(d)});renderKfStrip(kfs)}

function coordPayload(){return {enabled:$('coordEnabled').checked,frame:$('coordFrame').value,seed:+$('coordSeed').value,dx:+$('coordDx').value,dy:+$('coordDy').value,dz:+$('coordDz').value,dyaw_deg:+$('coordDyaw').value,x_amp:+$('coordXamp').value,y_amp:+$('coordYamp').value,z_amp:+$('coordZamp').value,yaw_amp_deg:+$('coordYawAmp').value,timescale_sec:+$('coordTimescale').value}}
function applyCoordToUi(c){if(!c)return;$('coordEnabled').checked=!!c.enabled;$('coordFrame').value=c.frame;$('coordSeed').value=c.seed;$('coordDx').value=(+c.dx).toFixed(3);$('coordDy').value=(+c.dy).toFixed(3);$('coordDz').value=(+c.dz).toFixed(3);$('coordDyaw').value=(+c.dyaw_deg).toFixed(2);$('coordXamp').value=(+c.x_amp).toFixed(3);$('coordYamp').value=(+c.y_amp).toFixed(3);$('coordZamp').value=(+c.z_amp).toFixed(3);$('coordYawAmp').value=(+c.yaw_amp_deg).toFixed(2);$('coordTimescale').value=(+c.timescale_sec).toFixed(2);$('coordSummary').textContent=`${c.enabled?'ON':'OFF'} · ${c.frame} · offset [${(+c.dx).toFixed(2)}, ${(+c.dy).toFixed(2)}, ${(+c.dz).toFixed(2)}] m · yaw ${(+c.dyaw_deg).toFixed(1)}°\nwobble [${(+c.x_amp).toFixed(3)}, ${(+c.y_amp).toFixed(3)}, ${(+c.z_amp).toFixed(3)}] m · yaw ${(+c.yaw_amp_deg).toFixed(1)}° · T≈${(+c.timescale_sec).toFixed(1)}s · seed ${c.seed}`}
function applyState(j){S=j;let c=j.control,l=j.lighting||{},m=j.mavros||{},tc=j.topics||{};if(j.coord&&!window.coordEditing)applyCoordToUi(j.coord);$('phase').textContent=c.phase;$('phase').className='badge '+(c.phase==='recording'?'bad':'');let mt=m.connected?`${m.mode||'?'} · ${m.armed?'ARMED':'DISARMED'}`:'MAVROS disconnected';$('mavros').textContent=mt;$('mavros').className='badge '+(m.armed?'ok':(m.connected?'warn':'bad'));$('clock').textContent=`${c.time.toFixed(2)} / ${c.duration.toFixed(2)} s`;$('fCount').textContent=tc.forward||0;$('bCount').textContent=tc.bottom||0;$('oCount').textContent=tc.odom||0;$('sCount').textContent=tc.state||0;$('tdur').textContent=c.duration.toFixed(2);$('t0').textContent=c.time.toFixed(2);if(!dragging){$('timeline').max=c.duration;$('timeline').value=c.time}$('speedNow').textContent=c.speed.toFixed(2)+'×';if(document.activeElement!==$('flightSpeed'))$('flightSpeed').value=c.speed.toFixed(2);$('speedDuration').textContent=`${c.source_duration.toFixed(2)} s → ${c.duration.toFixed(2)} s`;$('message').textContent=c.message;let h=l.health||{};$('lightHealth').textContent=(h.sun_error?'SUN ERR: '+h.sun_error+'  ':'')+(h.sky_error?'SKY ERR: '+h.sky_error:'');let p=c.current_pose;$('pose').textContent=p?`x ${p.x.toFixed(3)}   y ${p.y.toFixed(3)}   z ${p.z.toFixed(3)}
roll ${p.roll.toFixed(2)}°   pitch ${p.pitch.toFixed(2)}°   yaw ${p.yaw.toFixed(2)}°`:'pose: —';let rec=c.phase==='recording',ready=!!c.motion_ready;[$('play'),$('pause'),$('restart'),$('timeline'),$('record')].forEach(x=>x.disabled=rec||(!ready&&x!==$('pause')));document.querySelectorAll('.flightSpeedPreset').forEach(x=>{x.disabled=rec;x.classList.toggle('active',Math.abs(+x.dataset.v-c.speed)<0.001)});[$('flightSpeed'),$('speedApply'),$('speedReset'),$('speedRandom'),$('speedMin'),$('speedMax'),$('speedSeed')].forEach(x=>x.disabled=rec);document.querySelectorAll('.editLight').forEach(x=>x.disabled=rec);if(l.state){$('sunI').value=l.state.sun_intensity;$('sunIVal').textContent=l.state.sun_intensity.toFixed(2);appliedSunHex=rgbToHex(l.state.sun_rgb);appliedSkyHex=rgbToHex(l.state.sky_rgb);$('sunAppliedHex').textContent=appliedSunHex;$('skyAppliedHex').textContent=appliedSkyHex;$('sunCurrent').textContent=appliedSunHex;$('skyCurrent').textContent=appliedSkyHex;if(!colorInit){$('sunColor').value=appliedSunHex;$('skyColor').value=appliedSkyHex;$('sunDraft').textContent=appliedSunHex;$('skyDraft').textContent=appliedSkyHex;colorInit=true}else{if(!sunColorDirty){$('sunColor').value=appliedSunHex;$('sunDraft').textContent=appliedSunHex}if(!skyColorDirty){$('skyColor').value=appliedSkyHex;$('skyDraft').textContent=appliedSkyHex}}$('az').value=l.azimuth;$('azVal').textContent=Math.round(l.azimuth)+'°';$('el').value=l.elevation;$('elVal').textContent=Math.round(l.elevation)+'°'}$('auto').checked=!!l.auto_enabled;let kfs=l.keyframes||[];if(selectedKfIndex!==null&&selectedKfIndex>=kfs.length)selectedKfIndex=null;renderKeyframes(kfs);if(!kfEditorDirty&&selectedKfIndex===null&&!$('kfTime').value){let cur=currentLightingForEditor();if(cur)setKfEditor(cur,null,false)}}
async function poll(){try{let j=await(await fetch('/api/state',{cache:'no-store'})).json();applyState(j)}catch(e){$('message').textContent='UI disconnected: '+e}setTimeout(poll,250)}
$('play').onclick=()=>post('play');$('pause').onclick=()=>post('pause');$('restart').onclick=()=>post('seek',{time:0});$('arm').onclick=()=>post('arm_offboard');$('record').onclick=()=>confirm(`Начать финальную exact-запись с t=0 на скорости ${S&&S.control?S.control.speed.toFixed(2):'1.00'}×?`)&&post('record_from_start');$('quit').onclick=()=>confirm('Остановить replay?')&&post('quit');document.querySelectorAll('.flightSpeedPreset').forEach(b=>b.onclick=()=>post('speed',{value:+b.dataset.v}));$('speedApply').onclick=()=>post('speed',{value:+$('flightSpeed').value});$('speedReset').onclick=()=>post('speed',{value:1.0});$('speedRandom').onclick=()=>post('speed_random',{min:+$('speedMin').value,max:+$('speedMax').value,seed:+$('speedSeed').value});$('timeline').onpointerdown=()=>dragging=true;$('timeline').onpointerup=()=>{dragging=false;post('seek',{time:+$('timeline').value})};$('timeline').oninput=()=>{$('t0').textContent=(+$('timeline').value).toFixed(2)};
document.querySelectorAll('.preset').forEach(b=>b.onclick=async()=>{await post('preset',{name:b.dataset.p});sunColorDirty=false;skyColorDirty=false;selectedKfIndex=null;kfEditorDirty=false;$('kfTime').value=''});$('random').onclick=async()=>{await post('randomize',{seed:Math.floor(Math.random()*1000000000)});sunColorDirty=false;skyColorDirty=false;selectedKfIndex=null;kfEditorDirty=false;$('kfTime').value=''};$('auto').onchange=()=>post('auto',{enabled:$('auto').checked});$('clear').onclick=async()=>{await post('clear_manual');sunColorDirty=false;skyColorDirty=false};$('addKf').onclick=async()=>{let j=await post('add_keyframe');applyState(j);let t=j.control.time,k=j.lighting.keyframes||[];let i=k.reduce((best,x,n)=>best<0||Math.abs(x.t-t)<Math.abs(k[best].t-t)?n:best,-1);if(i>=0)selectKf(i,false)};$('newKf').onclick=()=>{let cur=currentLightingForEditor();if(cur){cur.t=S.control.time;setKfEditor(cur,null,true)}};$('captureKf').onclick=()=>{let cur=currentLightingForEditor();if(cur){cur.t=+$('kfTime').value||S.control.time;setKfEditor(cur,selectedKfIndex,true)}};$('saveKf').onclick=async()=>{try{let payload=editorPayload();let action=selectedKfIndex===null?'create_keyframe':'update_keyframe';if(selectedKfIndex!==null)payload.index=selectedKfIndex;let j=await post(action,payload);applyState(j);let k=j.lighting.keyframes||[],t=payload.t;let i=k.reduce((best,x,n)=>best<0||Math.abs(x.t-t)<Math.abs(k[best].t-t)?n:best,-1);if(i>=0)selectKf(i,false);kfEditorDirty=false;$('kfEditorHint').textContent='сохранена'}catch(e){$('message').textContent='KEYFRAME ERR: '+e}};$('previewKf').onclick=async()=>{let p=editorPayload();await post('light_set',{sun_intensity:p.sun_intensity,sun_rgb:p.sun_rgb,sky_rgb:p.sky_rgb,azimuth:p.azimuth,elevation:p.elevation})};$('seekKf').onclick=()=>post('seek',{time:+$('kfTime').value});$('duplicateKf').onclick=()=>{let p=editorPayload();p.t=Math.min((S&&S.control&&S.control.duration)||p.t,p.t+0.5);setKfEditor(p,null,true)};$('deleteSelectedKf').onclick=async()=>{if(selectedKfIndex===null)return;if(!confirm(`Удалить выбранную точку #${selectedKfIndex+1}?`))return;let j=await post('delete_keyframe',{index:selectedKfIndex});selectedKfIndex=null;kfEditorDirty=false;applyState(j);let cur=currentLightingForEditor();if(cur)setKfEditor(cur,null,false)};$('cancelKf').onclick=()=>{selectedKfIndex=null;kfEditorDirty=false;let cur=currentLightingForEditor();if(cur)setKfEditor(cur,null,false)};['kfTime','kfIntensity','kfSunColor','kfSkyColor','kfAz','kfEl'].forEach(id=>$(id).oninput=markEditorDirty);
$('applySun').onclick=()=>applySunColor();$('applySky').onclick=()=>applySkyColor();$('resetSunColor').onclick=()=>syncSunDraft();$('resetSkyColor').onclick=()=>syncSkyDraft();
$('sunColor').oninput=()=>{sunColorDirty=true;$('sunDraft').textContent=$('sunColor').value;$('sunDraft').classList.add('dirty')};$('skyColor').oninput=()=>{skyColorDirty=true;$('skyDraft').textContent=$('skyColor').value;$('skyDraft').classList.add('dirty')};
$('sunI').oninput=()=>{$('sunIVal').textContent=(+$('sunI').value).toFixed(2);sendSunScalar(120)};$('sunI').onchange=()=>sendSunScalar(0);$('az').oninput=()=>{$('azVal').textContent=$('az').value+'°';sendDirection(120)};$('az').onchange=()=>sendDirection(0);$('el').oninput=()=>{$('elVal').textContent=$('el').value+'°';sendDirection(120)};$('el').onchange=()=>sendDirection(0);
$('coordApply').onclick=async()=>{try{window.coordEditing=false;let j=await post('coord_set',coordPayload());applyState(j)}catch(e){$('message').textContent='COORD ERR: '+e}};$('coordRandom').onclick=async()=>{try{window.coordEditing=false;let j=await post('coord_random',{seed:+$('coordSeed').value||Math.floor(Math.random()*1e9)});applyState(j)}catch(e){$('message').textContent='COORD ERR: '+e}};$('coordReset').onclick=async()=>{window.coordEditing=false;let j=await post('coord_reset');applyState(j)};['coordEnabled','coordFrame','coordSeed','coordDx','coordDy','coordDz','coordDyaw','coordXamp','coordYamp','coordZamp','coordYawAmp','coordTimescale'].forEach(id=>$(id).oninput=()=>window.coordEditing=true);
poll();
</script></body></html>'''



class ReplayWebUI:
    def __init__(self, control, node, lighting, coord, host='127.0.0.1', port=8766,
                 open_browser=True):
        from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
        import webbrowser

        self.control = control
        self.node = node
        self.lighting = lighting
        self.coord = coord
        self.host = host
        self.port = int(port)
        self.open_browser = bool(open_browser)
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                return

            def _safe_write(self, data):
                try:
                    self.wfile.write(data)
                except (BrokenPipeError, ConnectionResetError):
                    return False
                return True

            def _send_json(self, obj, status=200):
                data = json.dumps(obj).encode('utf-8')
                self.send_response(status)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self._safe_write(data)

            def do_GET(self):
                path = self.path.split('?', 1)[0]
                if path == '/':
                    data = REPLAY_UI_HTML.encode('utf-8')
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.send_header('Content-Length', str(len(data)))
                    self.end_headers()
                    self._safe_write(data)
                    return
                if path == '/api/state':
                    self._send_json(owner.state_payload())
                    return
                if path in ('/camera/forward.bmp', '/camera/bottom.bmp'):
                    # Do not spend Python CPU / GIL on BMP conversion during
                    # the timing-critical exact replay. Recording still gets
                    # the native ROS Image stream directly.
                    if owner.control.snapshot()['phase'] == 'recording':
                        self.send_response(204)
                        self.end_headers()
                        return
                    which = 'forward' if 'forward' in path else 'bottom'
                    try:
                        data = owner.node.camera_bmp(which)
                    except Exception as exc:
                        self._send_json({'error': str(exc)}, 500)
                        return
                    if data is None:
                        self.send_response(204)
                        self.end_headers()
                        return
                    self.send_response(200)
                    self.send_header('Content-Type', 'image/bmp')
                    self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
                    self.send_header('Content-Length', str(len(data)))
                    self.end_headers()
                    self._safe_write(data)
                    return
                self.send_response(404)
                self.end_headers()

            def do_POST(self):
                if self.path.split('?', 1)[0] != '/api/control':
                    self.send_response(404)
                    self.end_headers()
                    return
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                    body = json.loads(self.rfile.read(length) or b'{}')
                    result = owner.handle_action(body)
                    self._send_json({'ok': True, **(result or {})})
                except Exception as exc:
                    self._send_json({'ok': False, 'error': str(exc)}, 400)

        self.server = ThreadingHTTPServer((host, self.port), Handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.url = f'http://{host}:{self.port}/'
        self._webbrowser = webbrowser
        self._arm_lock = threading.Lock()

    def _mavros_payload(self):
        state = self.node.get_state()
        if state is None:
            return {'connected': False, 'armed': False, 'mode': None}
        return {
            'connected': bool(getattr(state, 'connected', False)),
            'armed': bool(getattr(state, 'armed', False)),
            'mode': str(getattr(state, 'mode', '')),
        }

    def _arm_worker(self):
        if not self._arm_lock.acquire(blocking=False):
            with self.control.lock:
                self.control.message = 'ARM/OFFBOARD attempt is already running'
            return
        try:
            with self.control.lock:
                self.control.message = 'Streaming fixed takeoff setpoint; trying OFFBOARD + ARM...'
            try:
                ensure_mavros(self.node, timeout=5.0)
                ensure_offboard_and_armed(self.node, ARM_OFFBOARD_TIMEOUT_SEC, allow_force_arm=True)
            except Exception as exc:
                with self.control.lock:
                    self.control.message = f'ARM/OFFBOARD failed: {exc}'
                print(f'[REPLAY WARNING] ARM/OFFBOARD failed: {exc}', flush=True)
            else:
                with self.control.lock:
                    self.control.message = 'PX4 OFFBOARD + ARMED confirmed; preparing exact replay start'
        finally:
            self._arm_lock.release()

    def request_arm_retry(self):
        threading.Thread(target=self._arm_worker, daemon=True).start()

    def start(self):
        self.thread.start()
        print(f'[UI] Replay Control Center: {self.url}', flush=True)
        if self.open_browser:
            try:
                self._webbrowser.open(self.url)
            except Exception:
                pass

    def stop(self):
        try:
            self.server.shutdown()
            self.server.server_close()
        finally:
            if self.thread.is_alive():
                self.thread.join(timeout=2.0)

    def state_payload(self):
        light_state = self.lighting.state()
        if light_state is None:
            light = None
            az = 0.0
            el = 45.0
        else:
            az, el = direction_to_azimuth_elevation(light_state['sun_direction'])
            light = {
                'sun_intensity': float(light_state['sun_intensity']),
                'sun_rgb': list(light_state['sun_rgb']),
                'sky_rgb': list(light_state['sky_rgb']),
                'sun_direction': list(light_state['sun_direction']),
            }
        return {
            'control': self.control.snapshot(),
            'mavros': self._mavros_payload(),
            'topics': self.node.topic_counters(),
            'coord': self.coord.snapshot(),
            'lighting': {
                'state': light,
                'azimuth': az,
                'elevation': el,
                'auto_enabled': bool(self.lighting.auto_enabled),
                'keyframes': self.lighting.keyframes_snapshot(),
                'health': self.lighting.health(),
            },
        }

    def handle_action(self, body):
        action = str(body.get('action', ''))
        if action == 'play':
            if not self.control.play():
                raise RuntimeError('ARM/OFFBOARD is required before trajectory motion')
        elif action == 'pause':
            self.control.pause()
        elif action == 'seek':
            if not self.control.seek(body['time']):
                raise RuntimeError('Seek is locked until PX4 is OFFBOARD + ARMED, and while recording')
        elif action == 'speed':
            if not self.control.set_speed(body['value']):
                raise RuntimeError('Flight speed is locked while recording')
            new_duration = self.control.snapshot()['duration']
            self.coord.set_duration(new_duration)
            self.lighting.set_duration(new_duration, rescale_keyframes=True)
        elif action == 'speed_random':
            if self.control.snapshot()['phase'] == 'recording':
                raise RuntimeError('Flight speed is locked while recording')
            lo = float(body.get('min', 0.75))
            hi = float(body.get('max', 1.50))
            if not math.isfinite(lo) or not math.isfinite(hi) or lo <= 0.0 or hi <= 0.0:
                raise RuntimeError('Random speed min/max must be finite values > 0')
            if hi < lo:
                lo, hi = hi, lo
            seed = int(body.get('seed', 0))
            rng = random.Random(seed)
            value = rng.uniform(lo, hi)
            self.control.set_speed(value)
            new_duration = self.control.snapshot()['duration']
            self.coord.set_duration(new_duration)
            self.lighting.set_duration(new_duration, rescale_keyframes=True)
        elif action == 'record_from_start':
            if not self.control.request_record():
                raise RuntimeError('Already recording')
        elif action == 'arm_offboard':
            self.request_arm_retry()
        elif action == 'quit':
            self.control.request_quit()
        elif action == 'auto':
            self.lighting.set_auto(bool(body.get('enabled', True)))
        elif action == 'clear_manual':
            self.lighting.clear_manual()
        elif action == 'light_set':
            kwargs = {}
            if 'sun_intensity' in body:
                kwargs['sun_intensity'] = max(0.0, float(body['sun_intensity']))
            if 'sun_rgb' in body:
                kwargs['sun_rgb'] = color3(body['sun_rgb'])
            if 'sky_rgb' in body:
                kwargs['sky_rgb'] = color3(body['sky_rgb'])
            if 'azimuth' in body or 'elevation' in body:
                state = self.lighting.state() or self.lighting.initial_state()
                az, el = direction_to_azimuth_elevation(state['sun_direction'])
                az = float(body.get('azimuth', az))
                el = float(body.get('elevation', el))
                kwargs['sun_direction'] = azimuth_elevation_to_direction(az, el)
            self.lighting.set_manual(**kwargs)
        elif action in ('create_keyframe', 'update_keyframe'):
            if self.control.snapshot()['phase'] == 'recording':
                raise RuntimeError('Keyframe editing is locked while recording')
            t = clamp(float(body['t']), 0.0, self.control.duration)
            az = float(body.get('azimuth', 0.0))
            el = float(body.get('elevation', 45.0))
            row = {
                't': t,
                'sun_intensity': max(0.0, float(body.get('sun_intensity', 1.0))),
                'sun_rgb': color3(body.get('sun_rgb', [1.0, 1.0, 1.0])),
                'sky_rgb': color3(body.get('sky_rgb', [0.5, 0.7, 1.0])),
                'sun_direction': azimuth_elevation_to_direction(az, el),
            }
            if action == 'create_keyframe':
                self.lighting.create_keyframe(row)
            else:
                self.lighting.update_keyframe(body['index'], row)
        elif action == 'add_keyframe':
            if self.control.snapshot()['phase'] == 'recording':
                raise RuntimeError('Keyframe editing is locked while recording')
            self.lighting.add_keyframe_from_current()
        elif action == 'delete_keyframe':
            if self.control.snapshot()['phase'] == 'recording':
                raise RuntimeError('Keyframe editing is locked while recording')
            self.lighting.delete_keyframe(body['index'])
        elif action == 'preset':
            if self.control.snapshot()['phase'] == 'recording':
                raise RuntimeError('Preset changes are locked while recording')
            self.lighting.set_preset(str(body['name']), int(body.get('seed', 0)))
        elif action == 'randomize':
            if self.control.snapshot()['phase'] == 'recording':
                raise RuntimeError('Randomize is locked while recording')
            self.lighting.set_preset('random', int(body.get('seed', 0)))
        elif action == 'coord_set':
            if self.control.snapshot()['phase'] == 'recording':
                raise RuntimeError('Coordinate augmentation is locked while recording')
            self.coord.update(body)
        elif action == 'coord_random':
            if self.control.snapshot()['phase'] == 'recording':
                raise RuntimeError('Coordinate augmentation is locked while recording')
            self.coord.randomize_safe(int(body.get('seed', self.coord.seed)))
        elif action == 'coord_reset':
            if self.control.snapshot()['phase'] == 'recording':
                raise RuntimeError('Coordinate augmentation is locked while recording')
            self.coord.reset()
        else:
            raise ValueError(f'Unknown action: {action}')
        return self.state_payload()


def prepare_replay_output(
    raw_dir, source, base_trajectory, augmented_trajectory, augmenter,
    speed_scale, launch_context=None,
):
    output = speed_output_dir(raw_dir, source)
    for name in ('bottom', 'forward', 'trajectory', 'odom'):
        (output / name).mkdir(parents=True, exist_ok=True)

    write_synthetic_odom(
        source, output, base_trajectory, augmenter, speed_scale
    )
    scale_annotations(source, output, speed_scale)
    save_launch_metadata(output, launch_context)
    return output

def save_lighting_manifest(output, lighting_controller, lighting_source,
                           preset, seed, ui=True):
    aug_dir = output / 'augmentation'
    aug_dir.mkdir(parents=True, exist_ok=True)
    manifest = lighting_controller.manifest()
    manifest.update({
        'source': lighting_source,
        'preset': preset,
        'seed': int(seed),
        'interactive_ui': bool(ui),
    })
    path = aug_dir / 'lighting.json'
    with open(path, 'w') as f:
        json.dump(manifest, f, indent=2)
    return path


def run_interactive_replay(args, concept_dir, source, trajectory, raw_dir, launch_context):
    """Interactive UI around the ORIGINAL exact replay core.

    v8 performs fixed takeoff-setpoint pre-stream + OFFBOARD + ARM *before* the
    browser, lighting thread or sky material controller starts.  This removes
    all Python/Gazebo UI work from the arming window and restores the startup
    ordering of replay_raw.py.bak.
    """
    source_duration = float(trajectory[-1]['t'])
    initial_speed = max(1e-6, float(getattr(args, 'flight_speed', 1.0)))
    output_duration = source_duration / initial_speed
    if args.lighting_json:
        lighting_keyframes = load_lighting_json(args.lighting_json)
        lighting_source = str(Path(args.lighting_json).resolve())
    else:
        preset = args.lighting if args.lighting != 'off' else 'constant'
        lighting_keyframes = preset_keyframes(preset, args.lighting_seed)
        lighting_source = preset

    rclpy.init()
    node = InteractiveReplayNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    gazebo = None
    environment = None
    lighting_controller = None
    hold_streamer = None
    web_ui = None
    recorder = None
    current_output = None
    control = ReplayControl(source_duration, initial_speed)
    coord_augmenter = CoordinateAugmenter(output_duration, seed=getattr(args, 'coord_seed', 42003))

    coord_augmenter.update({'frame':getattr(args,'coord_frame','body'),'dx':getattr(args,'coord_dx',0.0),'dy':getattr(args,'coord_dy',0.0),'dz':getattr(args,'coord_dz',0.0),'dyaw_deg':getattr(args,'coord_dyaw',0.0),'x_amp':getattr(args,'coord_x_amp',0.05),'y_amp':getattr(args,'coord_y_amp',0.05),'z_amp':getattr(args,'coord_z_amp',0.025),'yaw_amp_deg':getattr(args,'coord_yaw_amp',1.5),'timescale_sec':getattr(args,'coord_timescale',4.0)})

    def mavros_is_ready_for_motion():
        state = node.get_state()
        return bool(
            state is not None
            and getattr(state, 'connected', False)
            and getattr(state, 'armed', False)
            and str(getattr(state, 'mode', '')) == 'OFFBOARD'
        )

    def pin_start_and_unlock(message='Ready'):
        with control.lock:
            control.phase = 'preparing_preview'
            control.paused = True
            control.t = 0.0
            control.message = 'Pinning recorded start pose...'
        aug_start = coord_augmenter.apply_pose(trajectory[0], 0.0)
        pin_pose(gazebo, aug_start, START_PIN_SEC)
        with control.lock:
            control.current_pose = dict(aug_start)
            control.motion_ready = True
            control.phase = 'preview'
            control.paused = True
            control.t = 0.0
            control.message = message

    def run_exact_recording():
        """Byte-for-byte timing strategy of the original exact replay loop."""
        nonlocal recorder, current_output

        # If PX4 dropped state while the UI was open, restore it with the hold
        # stream still running. No lighting/UI heavy work occurs in this call.
        if not mavros_is_ready_for_motion():
            with control.lock:
                control.message = 'Restoring PX4 OFFBOARD + ARMED with takeoff setpoint...'
            ensure_mavros(node, timeout=8.0)
            local_now = node.get_local_pose()
            if local_now is not None and hold_streamer is not None:
                rearm_target = dict(local_now)
                rearm_target['z'] = float(local_now['z']) + float(args.arm_takeoff_dz)
                hold_streamer.set_fixed_target(rearm_target)
                print(
                    f'[SETPOINT] re-arm target: x={rearm_target["x"]:.3f} '
                    f'y={rearm_target["y"]:.3f} z={rearm_target["z"]:.3f}',
                    flush=True,
                )
                time.sleep(PRE_OFFBOARD_STREAM_SEC)
            ensure_offboard_and_armed(
                node, ARM_OFFBOARD_TIMEOUT_SEC, allow_force_arm=True
            )
            if hold_streamer is not None:
                hold_streamer.set_follow_current()

        with control.lock:
            control.phase = 'preparing_record'
            control.paused = True
            control.t = 0.0
            control.message = 'Pinning start pose...'

        # Freeze the selected time scale and coordinate profile for this recording.
        record_speed = float(control.snapshot()['speed'])
        record_duration = source_duration / record_speed
        coord_augmenter.set_duration(record_duration)
        lighting_controller.set_duration(record_duration, rescale_keyframes=False)
        augmented_trajectory = coord_augmenter.generate_scaled_trajectory(
            trajectory, record_speed
        )

        planned_output = speed_output_dir(raw_dir, source)
        print(
            f'[SPEED] record requested -> {planned_output.name}',
            flush=True,
        )

        # Same order as replay_raw.py.bak: armed first, then teleport/pin.
        pin_pose(gazebo, augmented_trajectory[0], START_PIN_SEC)
        print(
            '[REPLAY] start pose pinned; preparing SP output...',
            flush=True,
        )

        current_output = prepare_replay_output(
            raw_dir,
            source,
            trajectory,
            augmented_trajectory,
            coord_augmenter,
            record_speed,
            launch_context,
        )
        print(
            f'[SPEED] output prepared: {current_output}',
            flush=True,
        )

        start_ns = time.monotonic_ns()
        recorder = ReplayRecorder(current_output, start_ns, augmented_trajectory)
        with node.lock:
            node.recorder = recorder

        with control.lock:
            control.t = 0.0
            control.duration = record_duration
            control.paused = False
            control.phase = 'recording'
            control.last_output = str(current_output)
            control.message = (
                f'Exact recording to {current_output.name} '
                f'at {record_speed:.2f}×'
            )

        route_start = time.monotonic()
        period = 1.0 / POSE_REPLAY_HZ
        next_tick = route_start
        cursor = 0

        while True:
            now = time.monotonic()
            t = now - route_start
            if t >= record_duration:
                break

            source_t = min(source_duration, t * record_speed)
            base_target, cursor = interpolate_trajectory(
                trajectory, source_t, cursor
            )
            # Wobble uses real replay time t, not source_t.
            target = coord_augmenter.apply_pose(base_target, t)
            gazebo.set_pose(target)

            # Keep UI bookkeeping minimal: once per 100 ms, not every 10 ms.
            if int(t * 10.0) != int(max(0.0, t - period) * 10.0):
                with control.lock:
                    control.t = t
                    control.current_pose = dict(target)
                    quit_requested = control.quit_requested
                if quit_requested:
                    raise KeyboardInterrupt

            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()

        final_pose = coord_augmenter.apply_pose(trajectory[-1], record_duration)
        pin_pose(gazebo, final_pose, FINAL_PIN_SEC)

        with node.lock:
            node.recorder = None
        recorder.close()
        recorder = None

        with control.lock:
            control.t = record_duration
            control.current_pose = dict(final_pose)

        manifest_path = save_lighting_manifest(
            current_output,
            lighting_controller,
            lighting_source,
            args.lighting,
            args.lighting_seed,
            ui=True,
        )
        coord_manifest = save_coord_manifest(current_output, coord_augmenter, source)
        speed_manifest = save_speed_manifest(
            current_output, source, record_speed, source_duration
        )
        print(f'[LIGHT] manifest: {manifest_path}', flush=True)
        print(f'[COORD] manifest: {coord_manifest}', flush=True)
        print(f'[SPEED] manifest: {speed_manifest}', flush=True)
        print(f'[REPLAY] completed: {current_output}', flush=True)
        control.finish_recording()

    try:
        # ------------------------------------------------------------------
        # CRITICAL STARTUP SECTION: identical ordering to the original replay.
        # Do NOT start browser, sky, lighting thread or image conversion here.
        # ------------------------------------------------------------------
        ensure_mavros(node)
        gazebo = GazeboPoseClient(GZ_WORLD_NAME, GZ_MODEL_NAME)

        arm_pose = node.get_local_pose()
        if arm_pose is None:
            raise RuntimeError('No MAVROS local pose available for arming setpoint')
        arm_target = dict(arm_pose)
        arm_target['z'] = float(arm_pose['z']) + float(args.arm_takeoff_dz)

        hold_streamer = HoldStreamer(node)
        hold_streamer.set_fixed_target(arm_target)
        hold_streamer.start()
        print(
            f'[SETPOINT] FIXED takeoff target @ {SETPOINT_HZ:.0f} Hz: '
            f'x={arm_target["x"]:.3f} y={arm_target["y"]:.3f} '
            f'z={arm_target["z"]:.3f} (dz=+{args.arm_takeoff_dz:.2f} m)',
            flush=True,
        )
        print(
            f'[REPLAY] pre-streaming takeoff setpoint for '
            f'{PRE_OFFBOARD_STREAM_SEC:.1f}s',
            flush=True,
        )
        time.sleep(PRE_OFFBOARD_STREAM_SEC)
        print(f'[SETPOINT] pre-arm setpoints published: {hold_streamer.sent_count}', flush=True)

        arm_error = None
        try:
            ensure_offboard_and_armed(
                node,
                ARM_OFFBOARD_TIMEOUT_SEC,
                allow_force_arm=True,
            )
            hold_streamer.set_follow_current()
            print('[SETPOINT] ARMED: switched to follow-current hold', flush=True)
            pin_start_and_unlock(
                'Ready: PX4 OFFBOARD + ARMED; original replay mechanics active'
            )
        except Exception as exc:
            arm_error = exc
            print(f'[REPLAY WARNING] startup ARM/OFFBOARD failed: {exc}', flush=True)
            with control.lock:
                control.motion_ready = False
                control.phase = 'waiting_arm'
                control.paused = True
                control.message = (
                    f'ARM/OFFBOARD failed: {exc}. '
                    'The UI is now available for retry, but recording stays locked.'
                )

        # Only after the original arming window is finished do we start the
        # augmentation/UI side of the process.
        environment = GazeboEnvironmentClient(GZ_WORLD_NAME)
        lighting_controller = InteractiveLightingController(
            environment,
            lighting_keyframes,
            control.duration,
            time_provider=control.current_time,
            update_hz=args.lighting_hz,
            sky_enabled=not args.no_sky_sphere,
        )

        initial_light = lighting_controller.initial_state()
        if initial_light is not None:
            try:
                environment.create_dataset_sun(
                    initial_light['sun_rgb'],
                    initial_light['sun_intensity'],
                    initial_light['sun_direction'],
                )
            except Exception as exc:
                print(f'[LIGHT WARNING] runtime sun setup failed: {exc}', flush=True)
                with lighting_controller.lock:
                    lighting_controller.sun_error = str(exc)
            if not args.no_sky_sphere:
                try:
                    sky_center, sky_radius = trajectory_sky_sphere(coord_augmenter.generate_scaled_trajectory(trajectory, control.speed))
                    environment.create_sky_sphere(
                        initial_light['sky_rgb'], sky_center, sky_radius
                    )
                    print(
                        f'[LIGHT] sky shell center={sky_center} '
                        f'radius={sky_radius:.1f}m visual_id={environment.sky_visual_id}',
                        flush=True,
                    )
                except Exception as exc:
                    print(f'[LIGHT WARNING] sky shell creation failed: {exc}', flush=True)
                    with lighting_controller.lock:
                        lighting_controller.sky_error = str(exc)

        lighting_controller.start()

        web_ui = ReplayWebUI(
            control, node, lighting_controller, coord_augmenter,
            host=args.ui_host,
            port=args.ui_port,
            open_browser=not args.no_browser,
        )
        web_ui.start()

        # If startup arming failed, UI is available now and the retry button
        # uses the same force-capable routine while hold-streaming continues.
        period = 1.0 / POSE_REPLAY_HZ
        next_tick = time.monotonic()
        last_wall = next_tick
        cursor = 0

        while True:
            now = time.monotonic()
            dt = max(0.0, now - last_wall)
            last_wall = now

            with control.lock:
                if control.quit_requested:
                    break
                record_request = control.record_requested
                motion_ready = control.motion_ready

            # Retry worker only performs ARM/OFFBOARD. Once state is confirmed,
            # pin exactly once before unlocking motion.
            if not motion_ready and mavros_is_ready_for_motion():
                try:
                    if hold_streamer is not None:
                        hold_streamer.set_follow_current()
                        print('[SETPOINT] ARM retry succeeded: switched to follow-current hold', flush=True)
                    pin_start_and_unlock(
                        'Ready after ARM retry: OFFBOARD + ARMED confirmed'
                    )
                    cursor = 0
                    next_tick = time.monotonic()
                    last_wall = next_tick
                except Exception as exc:
                    with control.lock:
                        control.message = f'ARM succeeded but start pin failed: {exc}'
                continue

            if record_request and control.consume_record_request():
                if not control.motion_ready:
                    with control.lock:
                        control.message = 'Record blocked: PX4 is not OFFBOARD + ARMED'
                    continue
                try:
                    run_exact_recording()
                except Exception as exc:
                    print(f'[REPLAY ERROR] exact recording failed: {exc}', flush=True)
                    with node.lock:
                        node.recorder = None
                    if recorder is not None:
                        try:
                            recorder.close()
                        except Exception:
                            pass
                        recorder = None
                    with control.lock:
                        control.phase = 'preview'
                        control.paused = True
                        control.message = f'Exact recording failed: {exc}'
                cursor = 0
                next_tick = time.monotonic()
                last_wall = next_tick
                continue

            if control.motion_ready and not mavros_is_ready_for_motion():
                with control.lock:
                    control.motion_ready = False
                    control.paused = True
                    control.phase = 'waiting_arm'
                    control.message = 'PX4 left OFFBOARD/ARMED; trajectory frozen'
                continue

            # Preview motion uses the same 100 Hz interpolation but is not used
            # for dataset recording. With live BMP conversion disabled it is
            # light enough to remain smooth.
            with control.lock:
                if not control.motion_ready:
                    t = control.t
                else:
                    if not control.paused:
                        control.t = min(
                            control.duration,
                            control.t + dt,
                        )
                        if control.t >= control.duration:
                            control.paused = True
                            control.message = 'Preview reached the end'
                    t = control.t

            if control.motion_ready:
                with control.lock:
                    speed_now = float(control.speed)
                source_t = min(source_duration, t * speed_now)
                if (
                    cursor >= len(trajectory) - 1
                    or trajectory[cursor]['t'] > source_t
                ):
                    cursor = 0
                base_target, cursor = interpolate_trajectory(
                    trajectory, source_t, cursor
                )
                # Chosen semantics: wobble time is real replay time.
                target = coord_augmenter.apply_pose(base_target, t)
                gazebo.set_pose(target)
                with control.lock:
                    control.current_pose = dict(target)

            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()

    finally:
        # Exact original cleanup priority: stop ROS publishers first.
        if hold_streamer is not None:
            try:
                hold_streamer.stop()
            except Exception:
                pass
            hold_streamer = None
        if lighting_controller is not None:
            try:
                lighting_controller.stop()
            except Exception:
                pass
        if web_ui is not None:
            try:
                web_ui.stop()
            except Exception:
                pass
        if environment is not None and not args.no_sky_sphere:
            try:
                environment.remove_sky_sphere(quiet=True)
            except Exception:
                pass
        if environment is not None:
            try:
                environment.remove_dataset_sun()
                environment.restore_stock_sun()
            except Exception:
                pass
        with node.lock:
            active = node.recorder
            node.recorder = None
        if active is not None:
            try:
                active.close()
            except Exception:
                pass
        if recorder is not None:
            try:
                recorder.close()
            except Exception:
                pass
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        executor_thread.join(timeout=2.0)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "source_flight",
        help=(
            "Example: "
            "datasets/raw/flight-20260920-054140"
        ),
    )

    parser.add_argument(
        "--lighting",
        choices=LIGHTING_PRESETS,
        default="off",
        help=(
            "Dynamic lighting preset. Default: off. "
            "Try day_to_sunset, day_to_night, cloud_pass, random."
        ),
    )
    parser.add_argument(
        "--lighting-json",
        default=None,
        help=(
            "Optional JSON with keyframes. Overrides --lighting. "
            "Use t(seconds) or u(0..1), sun_intensity, sun_rgb, "
            "sky_rgb, sun_direction."
        ),
    )
    parser.add_argument(
        "--lighting-seed",
        type=int,
        default=0,
        help="Seed used by --lighting random.",
    )
    parser.add_argument(
        "--lighting-hz",
        type=float,
        default=2.0,
        help="Gazebo lighting update rate. Default 2 Hz to minimize interference with the 100 Hz exact pose loop.",
    )
    parser.add_argument(
        "--lighting-console",
        action="store_true",
        help="Live terminal commands: sun, sunrgb, sky, dir, auto, clear.",
    )
    parser.add_argument(
        "--no-sky-sphere",
        action="store_true",
        help="Change the sun only; do not create/recolor the visual sky shell.",
    )

    parser.add_argument(
        "--ui",
        action="store_true",
        help="Open the browser Replay Control Center (preview + coordinate bias + lighting scenarios + record).",
    )
    parser.add_argument(
        "--ui-host",
        default="127.0.0.1",
        help="Replay UI bind host. Default: 127.0.0.1",
    )
    parser.add_argument(
        "--ui-port",
        type=int,
        default=8766,
        help="Replay UI TCP port. Default: 8766",
    )
    parser.add_argument(
        "--no-browser",
        action="store_true",
        help="With --ui, do not automatically open the browser.",
    )
    parser.add_argument(
        "--arm-takeoff-dz",
        type=float,
        default=1.0,
        help=(
            "Fixed +Z MAVROS local-position setpoint used before OFFBOARD/ARM. "
            "Default: 1.0 m."
        ),
    )
    parser.add_argument(
        "--require-arm-for-record",
        action="store_true",
        help="Compatibility flag retained; v8 UI publishes a fixed +Z takeoff setpoint before OFFBOARD/ARM; force-arm remains a SITL fallback.",
    )

    parser.add_argument('--coord-seed', type=int, default=42003)
    parser.add_argument('--coord-frame', choices=['world','body'], default='body')
    parser.add_argument('--coord-dx', type=float, default=0.0)
    parser.add_argument('--coord-dy', type=float, default=0.0)
    parser.add_argument('--coord-dz', type=float, default=0.0)
    parser.add_argument('--coord-dyaw', type=float, default=0.0)
    parser.add_argument('--coord-x-amp', type=float, default=0.05)
    parser.add_argument('--coord-y-amp', type=float, default=0.05)
    parser.add_argument('--coord-z-amp', type=float, default=0.025)
    parser.add_argument('--coord-yaw-amp', type=float, default=1.5)
    parser.add_argument('--coord-timescale', type=float, default=4.0)

    parser.add_argument(
        '--flight-speed',
        type=float,
        default=1.0,
        help=(
            'Physical replay speed multiplier. 0.5 = twice as long, '
            '1.0 = source timing, 2.0 = twice as fast. No upper limit. Default: 1.0.'
        ),
    )
    parser.add_argument('--speed-random-min', type=float, default=0.75)
    parser.add_argument('--speed-random-max', type=float, default=1.50)
    parser.add_argument('--speed-seed', type=int, default=42003)
    parser.add_argument(
        '--random-speed',
        action='store_true',
        help='Choose --flight-speed uniformly from the configured random range.',
    )

    args = parser.parse_args()

    if not math.isfinite(args.flight_speed) or args.flight_speed <= 0:
        parser.error('--flight-speed must be a finite value > 0')

    if args.random_speed:
        lo = float(args.speed_random_min)
        hi = float(args.speed_random_max)
        if not math.isfinite(lo) or not math.isfinite(hi) or lo <= 0.0 or hi <= 0.0:
            parser.error('--speed-random-min/max must be finite values > 0')
        if hi < lo:
            lo, hi = hi, lo
        args.flight_speed = random.Random(int(args.speed_seed)).uniform(lo, hi)
        print(
            f'[SPEED] random seed={args.speed_seed} range=[{lo:.2f},{hi:.2f}] '
            f'-> {args.flight_speed:.3f}x',
            flush=True,
        )

    # Capture the simulator launch context once, at replay script startup.
    # It may no longer be discoverable later when Record is pressed.
    launch_context = detect_running_ros2_launch()
    if launch_context.get('selected'):
        selected_launch = launch_context['selected']
        print(
            f"[ENV] detected launch: {selected_launch['package']} "
            f"{selected_launch['launch_file']}",
            flush=True,
        )
        print(f"[ENV] command: {selected_launch['command']}", flush=True)
    else:
        print(
            '[ENV WARNING] no active `ros2 launch ...` process detected at script startup',
            flush=True,
        )

    concept_dir = Path(
        CONCEPT_VLA_DIR
    ).resolve()

    source_arg = Path(args.source_flight).expanduser()

    if source_arg.is_absolute():
        candidates = [source_arg]
    else:
        # 1) Normal shell semantics: relative to the current working directory.
        #    Example from ~/Desktop/concept-vla/scripts:
        #      ../datasets/raw/flight-...
        # 2) Project-root relative path:
        #      datasets/raw/flight-...
        # 3) Bare flight directory name:
        #      flight-20260921-061133
        candidates = [
            Path.cwd() / source_arg,
            concept_dir / source_arg,
            concept_dir / "datasets" / "raw" / source_arg,
        ]

    source = None
    checked = []
    for candidate in candidates:
        resolved = candidate.resolve()
        checked.append(resolved)
        if (resolved / "trajectory" / "trajectory.csv").is_file():
            source = resolved
            break

    if source is None:
        checked_text = "\n  - ".join(str(x) for x in checked)
        raise FileNotFoundError(
            "Could not find a replayable source flight. Checked:\n  - "
            + checked_text
            + "\nExpected trajectory/trajectory.csv inside the flight directory."
        )

    trajectory = load_trajectory(source)

    raw_dir = (
        concept_dir
        / "datasets"
        / "raw"
    )

    if args.ui:
        source_duration = float(trajectory[-1]['t'])
        speed0 = max(1e-6, float(args.flight_speed))
        print(f"[REPLAY] source: {source}")
        print(f"[REPLAY] source duration: {source_duration:.2f} s")
        print(
            f"[SPEED] initial scale: {speed0:.2f}x -> "
            f"{source_duration / speed0:.2f} s"
        )
        print("[REPLAY] interactive speed + coordinate replay", flush=True)
        run_interactive_replay(args, concept_dir, source, trajectory, raw_dir, launch_context)
        return

    output = replay_output_dir(
        raw_dir,
        source,
    )

    print(f"[REPLAY] source: {source}")
    print(f"[REPLAY] output: {output}")
    print(
        f"[REPLAY] route duration: "
        f"{trajectory[-1]['t']:.2f} s"
    )
    print(
        "[REPLAY] mode: time-scaled exact Gazebo-world trajectory",
        flush=True,
    )
    print(
        f"[REPLAY] pose replay rate: {POSE_REPLAY_HZ:.0f} Hz "
        "(Catmull-Rom interpolation)",
        flush=True,
    )

    source_duration = float(trajectory[-1]["t"])
    flight_speed = max(1e-6, float(args.flight_speed))
    output_duration = source_duration / flight_speed

    coord_augmenter = CoordinateAugmenter(
        output_duration, seed=args.coord_seed
    )
    coord_augmenter.update({
        'frame': args.coord_frame, 'dx': args.coord_dx, 'dy': args.coord_dy,
        'dz': args.coord_dz, 'dyaw_deg': args.coord_dyaw,
        'x_amp': args.coord_x_amp, 'y_amp': args.coord_y_amp,
        'z_amp': args.coord_z_amp, 'yaw_amp_deg': args.coord_yaw_amp,
        'timescale_sec': args.coord_timescale,
    })
    augmented_trajectory = coord_augmenter.generate_scaled_trajectory(
        trajectory, flight_speed
    )
    print(
        f"[SPEED] scale={flight_speed:.3f}x "
        f"source={source_duration:.2f}s output={output_duration:.2f}s",
        flush=True,
    )
    lighting_enabled = bool(args.lighting_json) or args.lighting != "off"
    if args.lighting_json:
        lighting_keyframes = load_lighting_json(args.lighting_json)
        lighting_source = str(Path(args.lighting_json).resolve())
    else:
        lighting_keyframes = preset_keyframes(args.lighting, args.lighting_seed)
        lighting_source = args.lighting

    rclpy.init()
    node = ReplayNode()

    executor = MultiThreadedExecutor(
        num_threads=4
    )
    executor.add_node(node)

    executor_thread = threading.Thread(
        target=executor.spin,
        daemon=True,
    )
    executor_thread.start()

    gazebo = None
    environment = None
    lighting_controller = None
    lighting_console = None
    hold_streamer = None
    recorder = None

    try:
        ensure_mavros(node)

        # Direct Gazebo Transport: no ros_gz_bridge service is required.
        gazebo = GazeboPoseClient(
            GZ_WORLD_NAME,
            GZ_MODEL_NAME,
        )

        if lighting_enabled:
            environment = GazeboEnvironmentClient(GZ_WORLD_NAME)

        # Time-scaled output uses the normal numeric flight-copy naming.
        for name in ("bottom", "forward", "trajectory", "odom"):
            (output / name).mkdir(parents=True, exist_ok=True)
        save_launch_metadata(output, launch_context)
        write_synthetic_odom(
            source, output, trajectory, coord_augmenter, flight_speed
        )
        scale_annotations(source, output, flight_speed)
        print(
            f"[COORD] synthetic odom written: {output / 'odom' / 'odom.csv'}",
            flush=True,
        )

        if lighting_enabled:
            lighting_controller = LightingController(
                environment,
                lighting_keyframes,
                output_duration,
                update_hz=args.lighting_hz,
                sky_enabled=not args.no_sky_sphere,
            )
            initial_light = lighting_controller.initial_state()
            if initial_light is not None:
                environment.create_dataset_sun(
                    initial_light["sun_rgb"],
                    initial_light["sun_intensity"],
                    initial_light["sun_direction"],
                )
                if not args.no_sky_sphere:
                    sky_center, sky_radius = trajectory_sky_sphere(augmented_trajectory)
                    environment.create_sky_sphere(
                        initial_light["sky_rgb"],
                        sky_center,
                        sky_radius,
                    )
                    print(
                        f"[LIGHT] sky sphere center={sky_center} "
                        f"radius={sky_radius:.1f}m",
                        flush=True,
                    )
            print(
                f"[LIGHT] source={lighting_source} seed={args.lighting_seed} "
                f"update={args.lighting_hz:.1f}Hz",
                flush=True,
            )

        hold_streamer = HoldStreamer(node)
        hold_streamer.start()

        print(
            f"[REPLAY] pre-streaming MAVROS hold for "
            f"{PRE_OFFBOARD_STREAM_SEC:.1f}s",
            flush=True,
        )
        time.sleep(
            PRE_OFFBOARD_STREAM_SEC
        )

        # Arm/offboard first while drone is still at its normal spawn.
        ensure_offboard_and_armed(
            node,
            ARM_OFFBOARD_TIMEOUT_SEC,
        )

        start_pose = augmented_trajectory[0]

        print(
            "[REPLAY] teleporting to recorded start and pinning...",
            flush=True,
        )

        # The model cannot fall here: Gazebo pose is repeatedly enforced.
        pin_pose(
            gazebo,
            start_pose,
            START_PIN_SEC,
        )

        print(
            "[REPLAY] start pose fixed; beginning exact route",
            flush=True,
        )

        start_ns = time.monotonic_ns()

        recorder = ReplayRecorder(
            output,
            start_ns,
            augmented_trajectory,
        )

        with node.lock:
            node.recorder = recorder

        route_start = time.monotonic()
        if lighting_controller is not None:
            lighting_controller.start(route_start)
            if args.lighting_console:
                lighting_console = LightingConsole(lighting_controller)
                lighting_console.start()

        period = 1.0 / POSE_REPLAY_HZ
        next_tick = route_start
        cursor = 0

        while True:
            now = time.monotonic()
            t = now - route_start

            if t >= output_duration:
                break

            source_t = min(source_duration, t * flight_speed)
            base_target, cursor = interpolate_trajectory(
                trajectory,
                source_t,
                cursor,
            )
            # Wobble uses real replay time t.
            target = coord_augmenter.apply_pose(base_target, t)

            # Gazebo world pose is authoritative.
            gazebo.set_pose(target)

            next_tick += period
            delay = next_tick - time.monotonic()

            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()

        final_pose = augmented_trajectory[-1]

        pin_pose(
            gazebo,
            final_pose,
            FINAL_PIN_SEC,
        )

        if lighting_controller is not None:
            lighting_controller.stop()
            aug_dir = output / "augmentation"
            aug_dir.mkdir(parents=True, exist_ok=True)
            manifest = lighting_controller.manifest()
            manifest.update({
                "source": lighting_source,
                "preset": args.lighting,
                "seed": int(args.lighting_seed),
            })
            with open(aug_dir / "lighting.json", "w") as f:
                json.dump(manifest, f, indent=2)
            print(
                f"[LIGHT] manifest: {aug_dir / 'lighting.json'}",
                flush=True,
            )

        with node.lock:
            node.recorder = None

        recorder.close()
        recorder = None

        coord_manifest = save_coord_manifest(output, coord_augmenter, source)
        speed_manifest = save_speed_manifest(
            output, source, flight_speed, source_duration
        )
        print(f"[COORD] manifest: {coord_manifest}", flush=True)
        print(f"[SPEED] manifest: {speed_manifest}", flush=True)
        print(
            f"[REPLAY] completed: {output}",
            flush=True,
        )

    finally:
        if lighting_controller is not None:
            try:
                lighting_controller.stop()
            except Exception:
                pass

        if environment is not None and not args.no_sky_sphere:
            try:
                environment.remove_sky_sphere(quiet=True)
            except Exception:
                pass
        if environment is not None:
            try:
                environment.remove_dataset_sun()
                environment.restore_stock_sun()
            except Exception:
                pass

        with node.lock:
            active = node.recorder
            node.recorder = None

        if active is not None:
            try:
                active.close()
            except Exception:
                pass

        if recorder is not None:
            try:
                recorder.close()
            except Exception:
                pass

        if hold_streamer is not None:
            hold_streamer.stop()

        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        executor_thread.join(
            timeout=2.0
        )


if __name__ == "__main__":
    main()


