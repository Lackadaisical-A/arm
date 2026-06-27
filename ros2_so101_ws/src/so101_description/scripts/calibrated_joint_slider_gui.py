#!/usr/bin/env python3
"""LeRobot motor-space SO-101 slider GUI.

RViz still displays the URDF state on /joint_states. This GUI controls the real
arm using LeRobot's calibrated motor values directly, so visual mapping issues
cannot swallow a joint command.
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
DEFAULT_LIMITS = {
    "shoulder_pan": [-180.0, 180.0],
    "shoulder_lift": [-180.0, 180.0],
    "elbow_flex": [-180.0, 180.0],
    "wrist_flex": [-180.0, 180.0],
    "wrist_roll": [-180.0, 180.0],
    "gripper": [0.0, 100.0],
}


def clamp(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))


class SliderNode(Node):
    def __init__(self):
        super().__init__("so101_lerobot_slider_gui")
        self.declare_parameter("state_topic", "/so101_lerobot_state")
        self.declare_parameter("limits_topic", "/so101_lerobot_limits")
        self.declare_parameter("command_topic", "/lerobot_slider_positions")

        state_topic = self.get_parameter("state_topic").get_parameter_value().string_value
        limits_topic = self.get_parameter("limits_topic").get_parameter_value().string_value
        command_topic = self.get_parameter("command_topic").get_parameter_value().string_value

        self.positions = {joint: 0.0 for joint in JOINTS}
        self.limits = {joint: DEFAULT_LIMITS[joint][:] for joint in JOINTS}
        self.have_state = False
        self.have_limits = False

        self.publisher = self.create_publisher(JointState, command_topic, 10)
        self.create_subscription(JointState, state_topic, self.on_state, 10)
        self.create_subscription(JointState, limits_topic, self.on_limits, 10)
        self.get_logger().info(f"Sliders reading LeRobot state {state_topic}, limits {limits_topic}")
        self.get_logger().info(f"Sliders publishing LeRobot commands on {command_topic}")

    def on_state(self, msg: JointState) -> None:
        by_name = dict(zip(msg.name, msg.position, strict=False))
        if not all(joint in by_name for joint in JOINTS):
            return
        for joint in JOINTS:
            low, high = self.limits[joint]
            self.positions[joint] = clamp(float(by_name[joint]), low, high)
        self.have_state = True

    def on_limits(self, msg: JointState) -> None:
        if len(msg.velocity) < len(msg.name):
            return
        lower_by_name = dict(zip(msg.name, msg.position, strict=False))
        upper_by_name = dict(zip(msg.name, msg.velocity, strict=False))
        if not all(joint in lower_by_name and joint in upper_by_name for joint in JOINTS):
            return
        for joint in JOINTS:
            low = float(lower_by_name[joint])
            high = float(upper_by_name[joint])
            self.limits[joint] = [min(low, high), max(low, high)]
        self.have_limits = True

    def publish_command(self, positions: dict[str, float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = [float(positions[joint]) for joint in JOINTS]
        self.publisher.publish(msg)


class SliderGui:
    def __init__(self, node: SliderNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title("SO-101 LeRobot control")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.vars: dict[str, tk.DoubleVar] = {}
        self.value_labels: dict[str, ttk.Label] = {}
        self.scales: dict[str, tk.Scale] = {}
        self.dragging: set[str] = set()
        self.follow_real_state = True
        self.syncing = False
        self.closed = False
        self.command_publish_period_ms = 20

        self._build()
        self.root.after(20, self.spin_ros)
        self.root.after(50, self.sync_from_robot)
        self.root.after(self.command_publish_period_ms, self.publish_command_loop)

    def _build(self) -> None:
        frame = ttk.Frame(self.root, padding=10)
        frame.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)

        self.status = ttk.Label(frame, text="Waiting for real arm state...")
        self.status.grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))

        controls = ttk.Frame(frame)
        controls.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(0, 8))
        ttk.Button(controls, text="Resync to arm", command=self.resync_to_arm).grid(row=0, column=0, sticky="w")
        ttk.Button(controls, text="Hold sliders", command=self.hold_sliders).grid(row=0, column=1, sticky="w", padx=(8, 0))

        for row, joint in enumerate(JOINTS, start=2):
            unit = "%" if joint == "gripper" else "deg"
            ttk.Label(frame, text=joint).grid(row=row, column=0, sticky="w", padx=(0, 8), pady=3)
            variable = tk.DoubleVar(value=0.0)
            scale = tk.Scale(
                frame,
                from_=DEFAULT_LIMITS[joint][0],
                to=DEFAULT_LIMITS[joint][1],
                orient=tk.HORIZONTAL,
                resolution=0.1,
                showvalue=False,
                variable=variable,
                length=460,
                command=lambda _value, joint=joint: self.on_slider_changed(joint),
            )
            scale.grid(row=row, column=1, sticky="ew", pady=3)
            scale.bind("<ButtonPress-1>", lambda _event, joint=joint: self.dragging.add(joint))
            scale.bind("<ButtonRelease-1>", lambda _event, joint=joint: self.on_slider_released(joint))
            label = ttk.Label(frame, text=f"0.0 {unit}", width=13, anchor="e")
            label.grid(row=row, column=2, sticky="e", padx=(8, 0), pady=3)

            self.vars[joint] = variable
            self.scales[joint] = scale
            self.value_labels[joint] = label

    def close(self) -> None:
        self.closed = True
        self.root.destroy()

    def hold_sliders(self) -> None:
        self.follow_real_state = False
        self.status.configure(text="Command mode: sliders held.")

    def resync_to_arm(self) -> None:
        self.follow_real_state = True
        self.syncing = True
        try:
            self.sync_sliders_to_robot(force=True)
        finally:
            self.syncing = False
        self.status.configure(text="Following real arm pose.")

    def spin_ros(self) -> None:
        if self.closed:
            return
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.root.after(20, self.spin_ros)

    def slider_positions(self) -> dict[str, float]:
        values = {}
        for joint in JOINTS:
            low, high = self.node.limits[joint]
            values[joint] = clamp(float(self.vars[joint].get()), low, high)
        return values

    def publish_slider_command(self) -> None:
        self.node.publish_command(self.slider_positions())

    def publish_command_loop(self) -> None:
        if self.closed:
            return
        if not self.follow_real_state and self.node.have_state:
            self.publish_slider_command()
        self.root.after(self.command_publish_period_ms, self.publish_command_loop)

    def update_value_label(self, joint: str) -> None:
        unit = "%" if joint == "gripper" else "deg"
        self.value_labels[joint].configure(text=f"{self.vars[joint].get():+.1f} {unit}")

    def on_slider_changed(self, joint: str) -> None:
        self.update_value_label(joint)
        if self.syncing:
            return
        self.follow_real_state = False
        self.status.configure(text=f"Command mode: {joint} -> {self.vars[joint].get():+.1f}")
        self.publish_slider_command()

    def on_slider_released(self, joint: str) -> None:
        self.dragging.discard(joint)
        self.follow_real_state = False
        self.publish_slider_command()

    def sync_sliders_to_robot(self, force: bool = False) -> None:
        if not self.node.have_state:
            return
        for joint in JOINTS:
            if joint in self.dragging:
                continue
            low, high = self.node.limits[joint]
            self.scales[joint].configure(from_=low, to=high)
            value = clamp(self.node.positions[joint], low, high)
            if force or abs(value - self.vars[joint].get()) >= 0.25:
                self.vars[joint].set(value)
                self.update_value_label(joint)

    def sync_from_robot(self) -> None:
        if self.closed:
            return

        self.syncing = True
        try:
            for joint in JOINTS:
                low, high = self.node.limits[joint]
                self.scales[joint].configure(from_=low, to=high)
            if self.follow_real_state:
                self.sync_sliders_to_robot()
        finally:
            self.syncing = False

        if self.follow_real_state and self.node.have_state and self.node.have_limits:
            self.status.configure(text="Live arm pose synced; sliders use LeRobot calibrated limits.")
        elif self.follow_real_state and self.node.have_state:
            self.status.configure(text="Live arm pose synced; waiting for calibrated limits.")

        self.root.after(50, self.sync_from_robot)

    def run(self) -> None:
        self.root.mainloop()


def main() -> int:
    rclpy.init()
    node = SliderNode()
    gui = SliderGui(node)
    try:
        gui.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
