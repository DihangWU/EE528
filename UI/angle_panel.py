#!/usr/bin/env python3
"""Degree-based manual joint pose panel for the NexArm RViz model."""
import argparse
import math
import signal
import time
import tkinter as tk
from tkinter import ttk
import xml.etree.ElementTree as ET

import rclpy
from sensor_msgs.msg import JointState
import xacro


class AnglePanel:
    """Publish interpolated joint poses from validated degree inputs."""

    def __init__(self, root, model):
        self.root = root
        self.node = rclpy.create_node('manual_angle_panel')
        self.publisher = self.node.create_publisher(JointState, '/joint_states', 10)
        self.model = ET.fromstring(xacro.process_file(str(model)).toxml())
        self.names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5',
                      'gripper_base_joint']
        self.limits = []
        self.entries = []
        self.sliders = []
        self.current = [0.] * len(self.names)
        self.target = self.current.copy()
        self.origin = self.current.copy()
        self.started = time.monotonic()
        self.next_check = 0.
        root.title('NexArm · 关节角度控制')
        root.minsize(760, 470)
        frame = ttk.Frame(root, padding=22)
        frame.pack(fill='both', expand=True)
        ttk.Label(frame, text='指定关节姿态', font=('', 20, 'bold')).grid(
            row=0, column=0, columnspan=4, sticky='w', pady=(0, 8))
        ttk.Label(frame, text='输入模型关节角度（°），点击“移动到指定角度”或按 Enter。').grid(
            row=1, column=0, columnspan=4, sticky='w', pady=(0, 18))
        for col, label in enumerate(['关节', '目标角度 / °', '滑动调整', '允许范围 / °']):
            ttk.Label(frame, text=label).grid(row=2, column=col, sticky='w', padx=8)
        for i, name in enumerate(self.names):
            limit = self.model.find(f"joint[@name='{name}']/limit")
            lower, upper = [math.degrees(float(limit.get(key))) for key in ['lower', 'upper']]
            lower = math.ceil(lower * 100) / 100
            upper = math.floor(upper * 100) / 100
            self.limits.append((lower, upper))
            label = f'J{i + 1}' if i < 5 else '夹爪开合'
            ttk.Label(frame, text=label).grid(row=i + 3, column=0, sticky='w', padx=8)
            value = tk.StringVar(value='0.00')
            entry = ttk.Entry(frame, textvariable=value, width=13)
            entry.grid(row=i + 3, column=1, padx=8, pady=8)
            entry.bind('<Return>', lambda event: self.apply())
            slider = ttk.Scale(frame, from_=lower, to=upper,
                               command=lambda v, field=value: field.set(f'{float(v):.2f}'))
            slider.set(0)
            slider.grid(row=i + 3, column=2, sticky='ew', padx=10)
            slider.bind('<ButtonRelease-1>', lambda event: self.apply())
            ttk.Label(frame, text=f'{lower:.2f} ～ {upper:.2f}').grid(
                row=i + 3, column=3, sticky='w', padx=8)
            self.entries.append(value)
            self.sliders.append(slider)
        frame.columnconfigure(2, weight=1)
        actions = ttk.Frame(frame)
        actions.grid(row=9, column=0, columnspan=4, sticky='w', pady=(16, 10))
        ttk.Button(actions, text='移动到指定角度', command=self.apply).pack(side='left', padx=(0, 10))
        ttk.Button(actions, text='全部归零', command=self.zero).pack(side='left', padx=(0, 10))
        ttk.Button(actions, text='停止移动', command=self.stop).pack(side='left')
        self.status = tk.StringVar(value='就绪 · 角度以 URDF 零位为基准')
        self.connection = tk.StringVar()
        ttk.Label(frame, textvariable=self.status, wraplength=700).grid(
            row=10, column=0, columnspan=4, sticky='w', pady=6)
        ttk.Label(frame, textvariable=self.connection, wraplength=700).grid(
            row=11, column=0, columnspan=4, sticky='w')
        ttk.Label(frame, text='仅控制 RViz 模型。使用前请关闭写字演示、关节滑块及 Gazebo 演示。',
                  wraplength=700).grid(row=12, column=0, columnspan=4, sticky='w', pady=(12, 0))
        self.closed = False
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.tick()

    def apply(self):
        """Reject invalid poses atomically, then move all joints together."""
        values = []
        for i, (entry, (lower, upper)) in enumerate(zip(self.entries, self.limits)):
            try:
                value = float(entry.get())
            except ValueError:
                self.status.set(f'{self.names[i]}：请输入有效数字，未执行移动。')
                return False
            if not math.isfinite(value) or not lower <= value <= upper:
                self.status.set(f'{self.names[i]}：角度须在 {lower:.2f} ～ {upper:.2f}° 内，未执行移动。')
                return False
            values.append(value)
        self.origin = self.current.copy()
        self.target = [math.radians(v) for v in values]
        self.started = time.monotonic()
        for slider, value in zip(self.sliders, values):
            slider.set(value)
        self.status.set('正在移动到指定角度…')
        return True

    def zero(self):
        for entry in self.entries:
            entry.set('0')
        self.apply()

    def stop(self):
        self.target = self.current.copy()
        self.origin = self.current.copy()
        for entry, slider, angle in zip(self.entries, self.sliders, self.current):
            slider.set(math.degrees(angle))
            entry.set(f'{math.degrees(angle):.2f}')
        self.status.set('已停止，保持当前姿态。')

    def tick(self):
        if self.closed:
            return
        now = time.monotonic()
        t = min(1., (now - self.started) / 1.5)
        u = t * t * (3 - 2 * t)
        self.current = [a + u * (b - a) for a, b in zip(self.origin, self.target)]
        state = JointState()
        state.header.stamp = self.node.get_clock().now().to_msg()
        state.name = self.names.copy()
        state.position = self.current.copy()
        # Jaw translation follows the model's mimic relationship.
        for name, multiplier in [('left_jaw_joint', -.05), ('right_jaw_joint', .05)]:
            state.name.append(name)
            state.position.append(multiplier * self.current[-1])
        self.publisher.publish(state)
        rclpy.spin_once(self.node, timeout_sec=0)
        if t == 1 and self.status.get().startswith('正在移动'):
            self.status.set('已到达指定角度。')
        if now >= self.next_check:
            self.next_check = now + 2
            others = [p.node_name for p in self.node.get_publishers_info_by_topic('/joint_states')
                      if p.node_name != self.node.get_name()]
            self.connection.set('存在其他关节状态发布者，请关闭：' + ', '.join(others) if others
                                else 'ROS 2 已连接 · /joint_states · 约 30 Hz')
        self.root.after(33, self.tick)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.node.destroy_node()
        self.root.destroy()


def main():
    from ament_index_python.packages import get_package_share_directory
    from pathlib import Path
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default=str(Path(get_package_share_directory(
        'nexarm_description')) / 'urdf/nexarm.urdf.xacro'))
    args = parser.parse_args()
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    root = tk.Tk()
    panel = AnglePanel(root, args.model)
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: root.after(0, panel.close))
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if not panel.closed:
            panel.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
