#!/usr/bin/env python3
import argparse
import math
import threading
import time
from copy import deepcopy

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def normalize_angle_rad(v):
    return (float(v) + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw):
    half = 0.5 * float(yaw)
    return 0.0, 0.0, math.sin(half), math.cos(half)


class ControlNode(Node):
    def __init__(
        self,
        namespace="/uav1",
        xy_speed=2.0,
        z_speed=1.0,
        yaw_speed_deg=60.0,
        setpoint_hz=30.0,
        prestream_sec=2.0,
        activation_timeout=20.0,
    ):
        super().__init__("concept_vla_keyboard_control")

        self.ns = namespace.rstrip("/")
        self.xy_speed = float(xy_speed)
        self.z_speed = float(z_speed)
        self.yaw_speed = math.radians(float(yaw_speed_deg))
        self.setpoint_hz = float(setpoint_hz)
        self.prestream_sec = float(prestream_sec)
        self.activation_timeout = float(activation_timeout)

        self.lock = threading.RLock()

        self.state = State()
        self.latest_pose = None

        self.target_x = None
        self.target_y = None
        self.target_z = None
        self.target_yaw = None

        self.pressed = set()
        self.active = False
        self.activation_in_progress = False
        self.activation_error = ""
        self.activation_message = "Ожидание Enter"

        self.first_setpoint_monotonic = None
        self.setpoint_count = 0
        self.last_timer_monotonic = time.monotonic()

        self.pose_sub = self.create_subscription(
            PoseStamped,
            f"{self.ns}/mavros/local_position/pose",
            self.pose_callback,
            qos_profile_sensor_data,
        )

        self.state_sub = self.create_subscription(
            State,
            f"{self.ns}/mavros/state",
            self.state_callback,
            10,
        )

        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            f"{self.ns}/mavros/setpoint_position/local",
            10,
        )

        self.mode_client = self.create_client(
            SetMode,
            f"{self.ns}/mavros/set_mode",
        )

        self.arm_client = self.create_client(
            CommandBool,
            f"{self.ns}/mavros/cmd/arming",
        )

        self.timer = self.create_timer(
            1.0 / self.setpoint_hz,
            self.publish_tick,
        )

        self.get_logger().info(
            "control ready; waiting for MAVROS pose/state. "
            "Press Enter in the control window for OFFBOARD + ARM."
        )

    def state_callback(self, msg):
        with self.lock:
            self.state = msg

    def pose_callback(self, msg):
        with self.lock:
            self.latest_pose = msg

            # Before activation the hold target follows the current UAV pose.
            # This gives PX4 a continuous valid setpoint stream without
            # commanding an unexpected motion.
            if not self.active and not self.activation_in_progress:
                self._target_from_pose_locked(msg)

    def _target_from_pose_locked(self, pose_msg):
        self.target_x = float(pose_msg.pose.position.x)
        self.target_y = float(pose_msg.pose.position.y)
        self.target_z = float(pose_msg.pose.position.z)
        self.target_yaw = quaternion_to_yaw(pose_msg.pose.orientation)

    def set_key(self, key, pressed):
        with self.lock:
            if pressed:
                self.pressed.add(key)
            else:
                self.pressed.discard(key)

    def clear_keys(self):
        with self.lock:
            self.pressed.clear()

    def hold_current(self):
        with self.lock:
            if self.latest_pose is not None:
                self._target_from_pose_locked(self.latest_pose)
            self.pressed.clear()

    def publish_tick(self):
        now = time.monotonic()

        with self.lock:
            dt = max(0.0, min(0.2, now - self.last_timer_monotonic))
            self.last_timer_monotonic = now

            pose = self.latest_pose
            active = self.active
            keys = set(self.pressed)

            if pose is None:
                return

            if self.target_x is None:
                self._target_from_pose_locked(pose)

            if active:
                forward = 0.0
                lateral = 0.0
                vertical = 0.0
                yaw_axis = 0.0

                if "up" in keys:
                    forward += 1.0
                if "down" in keys:
                    forward -= 1.0
                if "left" in keys:
                    lateral += 1.0
                if "right" in keys:
                    lateral -= 1.0
                if "shift" in keys:
                    vertical += 1.0
                if "ctrl" in keys:
                    vertical -= 1.0
                if "z" in keys:
                    yaw_axis += 1.0
                if "x" in keys:
                    yaw_axis -= 1.0

                # Keep diagonal translational speed <= xy_speed.
                planar_norm = math.hypot(forward, lateral)
                if planar_norm > 1.0:
                    forward /= planar_norm
                    lateral /= planar_norm

                yaw = self.target_yaw

                # Body-frame -> MAVROS local ENU.
                vx = (
                    forward * math.cos(yaw)
                    - lateral * math.sin(yaw)
                ) * self.xy_speed

                vy = (
                    forward * math.sin(yaw)
                    + lateral * math.cos(yaw)
                ) * self.xy_speed

                self.target_x += vx * dt
                self.target_y += vy * dt
                self.target_z += vertical * self.z_speed * dt
                self.target_yaw = normalize_angle_rad(
                    self.target_yaw + yaw_axis * self.yaw_speed * dt
                )

                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = pose.header.frame_id

                msg.pose.position.x = float(self.target_x)
                msg.pose.position.y = float(self.target_y)
                msg.pose.position.z = float(self.target_z)

                qx, qy, qz, qw = yaw_to_quaternion(self.target_yaw)
                msg.pose.orientation.x = qx
                msg.pose.orientation.y = qy
                msg.pose.orientation.z = qz
                msg.pose.orientation.w = qw

            else:
                # Pre-OFFBOARD hold stream: publish actual current pose.
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = pose.header.frame_id
                msg.pose = deepcopy(pose.pose)

        self.setpoint_pub.publish(msg)

        with self.lock:
            if self.first_setpoint_monotonic is None:
                self.first_setpoint_monotonic = now
            self.setpoint_count += 1

    def request_activation(self):
        with self.lock:
            if self.active:
                self.activation_message = "Уже OFFBOARD + ARMED"
                return False

            if self.activation_in_progress:
                return False

            self.activation_in_progress = True
            self.activation_error = ""
            self.activation_message = "Подготовка OFFBOARD..."

        threading.Thread(
            target=self._activation_worker,
            daemon=True,
        ).start()

        return True

    def _wait_future(self, future, timeout=2.0):
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline and rclpy.ok():
            if future.done():
                return future.result()
            time.sleep(0.02)

        return None

    def _activation_worker(self):
        try:
            deadline = time.monotonic() + self.activation_timeout

            # Wait for MAVROS connection and local pose.
            while rclpy.ok() and time.monotonic() < deadline:
                with self.lock:
                    connected = bool(self.state.connected)
                    have_pose = self.latest_pose is not None

                if connected and have_pose:
                    break

                with self.lock:
                    self.activation_message = (
                        "Ожидание MAVROS connection / local pose..."
                    )

                time.sleep(0.1)
            else:
                raise RuntimeError(
                    "Timeout waiting for MAVROS connection/local pose"
                )

            # Ensure enough valid setpoints were streamed before requesting
            # OFFBOARD. This mirrors the working replay startup sequence.
            while rclpy.ok() and time.monotonic() < deadline:
                with self.lock:
                    first = self.first_setpoint_monotonic
                    count = self.setpoint_count

                elapsed = (
                    0.0
                    if first is None
                    else time.monotonic() - first
                )

                remaining = self.prestream_sec - elapsed

                if remaining <= 0.0 and count >= 10:
                    break

                with self.lock:
                    self.activation_message = (
                        f"Pre-stream setpoints: "
                        f"{max(0.0, remaining):.1f}s"
                    )

                time.sleep(0.05)
            else:
                raise RuntimeError(
                    "Timeout while pre-streaming OFFBOARD setpoints"
                )

            # Wait for services.
            while rclpy.ok() and time.monotonic() < deadline:
                if (
                    self.mode_client.wait_for_service(timeout_sec=0.2)
                    and self.arm_client.wait_for_service(timeout_sec=0.2)
                ):
                    break
            else:
                raise RuntimeError(
                    "MAVROS mode/arming services are unavailable"
                )

            # OFFBOARD first, same order as the replay script that works
            # with this PX4/MAVROS setup.
            while rclpy.ok() and time.monotonic() < deadline:
                with self.lock:
                    current_mode = str(self.state.mode)

                if current_mode == "OFFBOARD":
                    break

                with self.lock:
                    self.activation_message = "Запрос OFFBOARD..."

                req = SetMode.Request()
                req.base_mode = 0
                req.custom_mode = "OFFBOARD"

                result = self._wait_future(
                    self.mode_client.call_async(req),
                    timeout=2.0,
                )

                if result is None:
                    time.sleep(0.2)
                else:
                    time.sleep(0.4)
            else:
                raise RuntimeError("PX4 did not enter OFFBOARD")

            # ARM after OFFBOARD.
            while rclpy.ok() and time.monotonic() < deadline:
                with self.lock:
                    armed = bool(self.state.armed)

                if armed:
                    break

                with self.lock:
                    self.activation_message = "Запрос ARM..."

                req = CommandBool.Request()
                req.value = True

                result = self._wait_future(
                    self.arm_client.call_async(req),
                    timeout=2.0,
                )

                if result is None:
                    time.sleep(0.2)
                else:
                    time.sleep(0.4)
            else:
                raise RuntimeError("PX4 did not ARM")

            # Start manual integration exactly from the actual pose at the
            # moment OFFBOARD + ARM are confirmed.
            with self.lock:
                if self.latest_pose is not None:
                    self._target_from_pose_locked(self.latest_pose)

                self.pressed.clear()
                self.active = True
                self.activation_message = "OFFBOARD + ARMED"
                self.activation_error = ""

            self.get_logger().info(
                "PX4 confirmed OFFBOARD + ARMED; keyboard control enabled."
            )

        except Exception as exc:
            with self.lock:
                self.activation_error = str(exc)
                self.activation_message = "Ошибка активации"

            self.get_logger().error(
                f"OFFBOARD/ARM failed: {exc}"
            )

        finally:
            with self.lock:
                self.activation_in_progress = False

    def snapshot(self):
        with self.lock:
            pose = self.latest_pose

            actual = None
            if pose is not None:
                actual = {
                    "x": float(pose.pose.position.x),
                    "y": float(pose.pose.position.y),
                    "z": float(pose.pose.position.z),
                    "yaw_deg": math.degrees(
                        quaternion_to_yaw(pose.pose.orientation)
                    ),
                }

            target = None
            if self.target_x is not None:
                target = {
                    "x": self.target_x,
                    "y": self.target_y,
                    "z": self.target_z,
                    "yaw_deg": math.degrees(self.target_yaw),
                }

            return {
                "connected": bool(self.state.connected),
                "armed": bool(self.state.armed),
                "mode": str(self.state.mode),
                "active": self.active,
                "activation_in_progress": self.activation_in_progress,
                "activation_message": self.activation_message,
                "activation_error": self.activation_error,
                "setpoint_count": self.setpoint_count,
                "actual": actual,
                "target": target,
                "pressed": sorted(self.pressed),
            }


class ControlWindow:
    KEY_MAP = {
        "Up": "up",
        "Down": "down",
        "Left": "left",
        "Right": "right",
        "z": "z",
        "Z": "z",
        "x": "x",
        "X": "x",
        "Shift_L": "shift",
        "Control_L": "ctrl",

        # Also accept the right-side modifiers as a convenience, although the
        # requested bindings are Left Shift / Left Ctrl.
        "Shift_R": "shift",
        "Control_R": "ctrl",
    }

    def __init__(self, node):
        try:
            import tkinter as tk
            from tkinter import ttk
        except Exception as exc:
            raise RuntimeError(
                "Tkinter is required for control.py because standalone "
                "Shift/Ctrl key press/release events cannot be read from "
                f"a normal terminal. Import failed: {exc}"
            )

        self.tk = tk
        self.ttk = ttk
        self.node = node

        self.root = tk.Tk()
        self.root.title("control")
        self.root.geometry("650x470")
        self.root.minsize(590, 430)

        self.root.protocol("WM_DELETE_WINDOW", self.close)

        outer = ttk.Frame(self.root, padding=14)
        outer.pack(fill="both", expand=True)

        title = ttk.Label(
            outer,
            text="UAV CONTROL · MAVROS OFFBOARD",
            font=("TkDefaultFont", 16, "bold"),
        )
        title.pack(anchor="w")

        ttk.Label(
            outer,
            text=(
                "Кликни по этому окну, затем удерживай клавиши движения. "
                "Enter = OFFBOARD + ARM."
            ),
        ).pack(anchor="w", pady=(3, 12))

        status_box = ttk.LabelFrame(
            outer,
            text="Состояние",
            padding=10,
        )
        status_box.pack(fill="x")

        self.status_var = tk.StringVar(value="MAVROS: ожидание")
        self.pose_var = tk.StringVar(value="Pose: —")
        self.target_var = tk.StringVar(value="Target: —")
        self.keys_var = tk.StringVar(value="Keys: —")
        self.error_var = tk.StringVar(value="")

        ttk.Label(
            status_box,
            textvariable=self.status_var,
            font=("TkDefaultFont", 11, "bold"),
        ).pack(anchor="w")
        ttk.Label(status_box, textvariable=self.pose_var).pack(anchor="w")
        ttk.Label(status_box, textvariable=self.target_var).pack(anchor="w")
        ttk.Label(status_box, textvariable=self.keys_var).pack(anchor="w")
        self.error_label = ttk.Label(
            status_box,
            textvariable=self.error_var,
        )
        self.error_label.pack(anchor="w")

        controls = ttk.LabelFrame(
            outer,
            text="Клавиши",
            padding=10,
        )
        controls.pack(fill="both", expand=True, pady=12)

        lines = [
            ("↑", "вперёд"),
            ("↓", "назад"),
            ("←", "влево"),
            ("→", "вправо"),
            ("Z / X", "yaw влево / вправо"),
            ("Left Shift", "вверх"),
            ("Left Ctrl", "вниз"),
            ("Enter", "OFFBOARD + ARM"),
        ]

        for key, action in lines:
            row = ttk.Frame(controls)
            row.pack(fill="x", pady=2)

            ttk.Label(
                row,
                text=key,
                width=18,
                font=("TkDefaultFont", 11, "bold"),
            ).pack(side="left")

            ttk.Label(
                row,
                text=action,
            ).pack(side="left")

        actions = ttk.Frame(outer)
        actions.pack(fill="x")

        self.arm_button = ttk.Button(
            actions,
            text="Enter · OFFBOARD + ARM",
            command=self.node.request_activation,
        )
        self.arm_button.pack(side="left")

        ttk.Button(
            actions,
            text="HOLD current",
            command=self.node.hold_current,
        ).pack(side="left", padx=8)

        ttk.Button(
            actions,
            text="Quit",
            command=self.close,
        ).pack(side="right")

        # Press/release bindings are why this window is used instead of a
        # terminal raw-mode reader: Shift/Ctrl by themselves have real events.
        self.root.bind_all("<KeyPress>", self.on_key_press)
        self.root.bind_all("<KeyRelease>", self.on_key_release)
        self.root.bind("<FocusOut>", self.on_focus_out)

        self.closed = False
        self.root.after(100, self.refresh)

        # Make key focus explicit after window creation.
        self.root.after(250, self.root.focus_force)

    def on_key_press(self, event):
        if event.keysym == "Return":
            self.node.request_activation()
            return "break"

        key = self.KEY_MAP.get(event.keysym)
        if key is not None:
            self.node.set_key(key, True)
            return "break"

        return None

    def on_key_release(self, event):
        key = self.KEY_MAP.get(event.keysym)

        if key is not None:
            self.node.set_key(key, False)
            return "break"

        return None

    def on_focus_out(self, _event):
        # Avoid a "stuck" command if the user Alt-Tabs while holding a key.
        self.node.clear_keys()

    def refresh(self):
        if self.closed:
            return

        s = self.node.snapshot()

        self.status_var.set(
            "MAVROS: "
            f"{'CONNECTED' if s['connected'] else 'disconnected'} · "
            f"mode={s['mode'] or '—'} · "
            f"{'ARMED' if s['armed'] else 'disarmed'} · "
            f"{s['activation_message']} · "
            f"setpoints={s['setpoint_count']}"
        )

        if s["actual"]:
            p = s["actual"]
            self.pose_var.set(
                "Pose: "
                f"x={p['x']:.2f}  "
                f"y={p['y']:.2f}  "
                f"z={p['z']:.2f}  "
                f"yaw={p['yaw_deg']:.1f}°"
            )
        else:
            self.pose_var.set("Pose: —")

        if s["target"]:
            p = s["target"]
            self.target_var.set(
                "Target: "
                f"x={p['x']:.2f}  "
                f"y={p['y']:.2f}  "
                f"z={p['z']:.2f}  "
                f"yaw={p['yaw_deg']:.1f}°"
            )
        else:
            self.target_var.set("Target: —")

        self.keys_var.set(
            "Keys: "
            + (", ".join(s["pressed"]) if s["pressed"] else "—")
        )

        self.error_var.set(
            f"ERROR: {s['activation_error']}"
            if s["activation_error"]
            else ""
        )

        if s["active"] or s["activation_in_progress"]:
            self.arm_button.state(["disabled"])
        else:
            self.arm_button.state(["!disabled"])

        self.root.after(100, self.refresh)

    def close(self):
        if self.closed:
            return

        self.closed = True
        self.node.clear_keys()
        self.node.hold_current()

        try:
            self.root.destroy()
        except Exception:
            pass

    def run(self):
        self.root.mainloop()


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Keyboard MAVROS OFFBOARD controller for Concept-VLA recording."
        )
    )
    parser.add_argument(
        "--ns",
        default="/uav1",
        help="MAVROS vehicle namespace (default: /uav1)",
    )
    parser.add_argument(
        "--xy-speed",
        type=float,
        default=0.8,
        help="Forward/lateral target speed in m/s (default: 0.8)",
    )
    parser.add_argument(
        "--z-speed",
        type=float,
        default=0.5,
        help="Vertical target speed in m/s (default: 0.5)",
    )
    parser.add_argument(
        "--yaw-speed",
        type=float,
        default=45.0,
        help="Yaw rate in deg/s (default: 45)",
    )
    parser.add_argument(
        "--setpoint-hz",
        type=float,
        default=30.0,
        help="Pose setpoint publish rate (default: 30 Hz)",
    )
    parser.add_argument(
        "--prestream",
        type=float,
        default=2.0,
        help="Minimum setpoint stream before OFFBOARD (default: 2 s)",
    )
    parser.add_argument(
        "--activation-timeout",
        type=float,
        default=20.0,
        help="OFFBOARD + ARM timeout in seconds (default: 20)",
    )

    args, ros_args = parser.parse_known_args()

    for name, value in (
        ("--xy-speed", args.xy_speed),
        ("--z-speed", args.z_speed),
        ("--yaw-speed", args.yaw_speed),
        ("--setpoint-hz", args.setpoint_hz),
        ("--prestream", args.prestream),
        ("--activation-timeout", args.activation_timeout),
    ):
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f"{name} must be a finite value > 0")

    rclpy.init(args=ros_args)

    node = ControlNode(
        namespace=args.ns,
        xy_speed=args.xy_speed,
        z_speed=args.z_speed,
        yaw_speed_deg=args.yaw_speed,
        setpoint_hz=args.setpoint_hz,
        prestream_sec=args.prestream,
        activation_timeout=args.activation_timeout,
    )

    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True,
    )
    spin_thread.start()

    try:
        window = ControlWindow(node)
        window.run()

    except KeyboardInterrupt:
        pass

    finally:
        node.clear_keys()
        node.hold_current()

        # Give the last HOLD target a few publish cycles before shutdown.
        time.sleep(0.15)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
