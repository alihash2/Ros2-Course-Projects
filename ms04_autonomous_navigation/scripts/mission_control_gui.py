#!/usr/bin/env python3
"""PyQt5 Mission Control GUI for the ms04_autonomous_navigation package.

Sections:
  - Environment controls: simulation launcher for Office / Warehouse
  - Map loader + Nav2 activation button (reuses the AMCL navigation launches)
  - Initial pose trigger (predefined spawn pose published on /initialpose)
  - Goal & waypoint dispatcher: single pose (X, Y, Theta), predefined POIs,
    multi-waypoint list builder
  - Execution controls: start / cancel / pause / resume / replace
  - Real-time color-coded log viewer subscribed to /navigation/events and
    /navigation/status
The ROS 2 node spins in a background QThread; callbacks update the widget
tree through thread-safe Qt signals.
"""
import os
import subprocess
import sys
import time
import math
from collections import OrderedDict

if sys.platform.startswith('linux') and 'QT_QPA_PLATFORM' not in os.environ:
    os.environ['QT_QPA_PLATFORM'] = 'xcb'

from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QColor, QFont
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QFormLayout, QGroupBox,
    QHBoxLayout, QLabel, QListWidget, QMainWindow, QPlainTextEdit,
    QPushButton, QVBoxLayout, QWidget)

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from ms04_autonomous_navigation.msg import (
    NavigationEvent, NavigationMission, NavigationStatus)

PKG = 'ms04_autonomous_navigation'

ENVS = OrderedDict({
    'office': {
        'label': 'Office',
        'sim_launch': 'office_simulation.launch.py',
        'nav_launch': 'office_navigation.launch.py',
        'spawn': (0.0, 0.0, 0.0),
        'pois': OrderedDict([
            ('Spawn', (0.0, 0.0, 0.0)),
            ('Corridor Center', (0.0, -4.0, 0.0)),
            ('North Room', (4.0, 5.0, 0.0)),
            ('South Room', (-3.0, -5.0, 0.0)),
            ('Far East', (6.0, 0.0, 0.0)),
        ]),
    },
    'warehouse': {
        'label': 'Warehouse',
        'sim_launch': 'warehouse_simulation.launch.py',
        'nav_launch': 'warehouse_navigation.launch.py',
        'spawn': (-6.0, 0.0, 0.0),
        'pois': OrderedDict([
            ('Spawn', (-6.0, 0.0, 0.0)),
            ('Aisle Center', (0.0, 0.0, 0.0)),
            ('Loading Zone', (6.0, 0.0, 0.0)),
            ('Rack A', (-3.0, 4.0, 0.0)),
            ('Rack B', (3.0, -4.0, 0.0)),
        ]),
    },
})

EVENT_NAMES = {
    NavigationEvent.EVENT_GOAL_SUBMITTED: 'SUBMITTED',
    NavigationEvent.EVENT_GOAL_ACCEPTED: 'ACCEPTED',
    NavigationEvent.EVENT_GOAL_REJECTED: 'REJECTED',
    NavigationEvent.EVENT_FEEDBACK: 'FEEDBACK',
    NavigationEvent.EVENT_GOAL_COMPLETED: 'COMPLETED',
    NavigationEvent.EVENT_GOAL_CANCELED: 'CANCELED',
    NavigationEvent.EVENT_GOAL_ABORTED: 'ABORTED',
    NavigationEvent.EVENT_GOAL_PAUSED: 'PAUSED',
    NavigationEvent.EVENT_GOAL_RESUMED: 'RESUMED',
    NavigationEvent.EVENT_GOAL_REPLACED: 'REPLACED',
}

STATE_NAMES = {
    NavigationStatus.STATE_IDLE: 'IDLE',
    NavigationStatus.STATE_NAVIGATING: 'NAVIGATING',
    NavigationStatus.STATE_PAUSED: 'PAUSED',
    NavigationStatus.STATE_COMPLETED: 'COMPLETED',
    NavigationStatus.STATE_CANCELED: 'CANCELED',
    NavigationStatus.STATE_ABORTED: 'ABORTED',
    NavigationStatus.STATE_REJECTED: 'REJECTED',
}

EVENT_COLORS = {
    NavigationEvent.EVENT_GOAL_SUBMITTED: QColor(160, 160, 160),
    NavigationEvent.EVENT_GOAL_ACCEPTED: QColor(76, 175, 80),
    NavigationEvent.EVENT_GOAL_REJECTED: QColor(244, 67, 54),
    NavigationEvent.EVENT_FEEDBACK: QColor(33, 150, 243),
    NavigationEvent.EVENT_GOAL_COMPLETED: QColor(0, 200, 83),
    NavigationEvent.EVENT_GOAL_CANCELED: QColor(255, 152, 0),
    NavigationEvent.EVENT_GOAL_ABORTED: QColor(255, 23, 68),
    NavigationEvent.EVENT_GOAL_PAUSED: QColor(255, 193, 7),
    NavigationEvent.EVENT_GOAL_RESUMED: QColor(0, 188, 212),
    NavigationEvent.EVENT_GOAL_REPLACED: QColor(156, 39, 176),
}


def _yaw_to_quat(yaw):
    q = PoseStamped().pose.orientation
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class RosSpinnerThread(QThread):
    """Spins the shared ROS 2 node in a background QThread.

    Callbacks are executed in this thread; UI state changes are delivered
    through the signals below (thread-safe queued connections).
    """

    event_received = pyqtSignal(object)
    status_received = pyqtSignal(object)

    def __init__(self, node, parent=None):
        super().__init__(parent)
        self._node = node
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(node)

    def run(self):
        while not self.isInterruptionRequested() and rclpy.ok():
            self._executor.spin_once(timeout_sec=0.05)
        self._executor.shutdown()

    def shutdown(self):
        self.requestInterruption()
        self.wait(3000)


class MissionControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Mission Control - ms04_autonomous_navigation')
        self.resize(1150, 780)

        self._env_key = 'office'
        self._mission_seq = 0
        self._spawned_procs = []

        self._build_ros()
        self._build_ui()

        self._ros_thread = RosSpinnerThread(self._node)
        self._ros_thread.event_received.connect(self._on_event, Qt.QueuedConnection)
        self._ros_thread.status_received.connect(self._on_status, Qt.QueuedConnection)
        self._ros_thread.start()

        self._populate_env('office')
        self._log('system', 'GUI ready. Select an environment and launch the simulation.')

    # ------------------------------------------------------------------ ROS
    def _build_ros(self):
        rclpy.init()
        self._node = Node('mission_control_gui')
        self._mission_pub = self._node.create_publisher(
            NavigationMission, '/navigation/mission', 10)
        self._initialpose_pub = self._node.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self._events_sub = self._node.create_subscription(
            NavigationEvent, '/navigation/events', self._events_cb, 10)
        self._status_sub = self._node.create_subscription(
            NavigationStatus, '/navigation/status', self._status_cb, 10)

    def _events_cb(self, msg):
        self._ros_thread.event_received.emit(msg)

    def _status_cb(self, msg):
        self._ros_thread.status_received.emit(msg)

    def shutdown_ros(self):
        if self._ros_thread.isRunning():
            self._ros_thread.shutdown()
        self._node.destroy_node()
        rclpy.shutdown()

    def _publish_mission(self, command, mode, mission_id, pose=None, waypoints=None):
        msg = NavigationMission()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.mission_id = mission_id
        msg.command = command
        msg.mode = mode
        if pose is not None:
            msg.target_pose = pose
        if waypoints is not None:
            msg.waypoints = waypoints
        self._mission_pub.publish(msg)

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)

        left = QVBoxLayout()
        left.addWidget(self._build_env_group())
        left.addWidget(self._build_mission_group())
        left.addWidget(self._build_control_group())
        left.addStretch(1)

        right = QVBoxLayout()
        right.addWidget(self._build_status_group(), 0)
        right.addWidget(self._build_log_group(), 1)

        root.addLayout(left, 3)
        root.addLayout(right, 5)

    def _build_env_group(self):
        group = QGroupBox('1. Environment & Navigation')
        layout = QVBoxLayout()

        env_row = QHBoxLayout()
        env_row.addWidget(QLabel('Environment:'))
        self.env_combo = QComboBox()
        for key, cfg in ENVS.items():
            self.env_combo.addItem(cfg['label'], key)
        self.env_combo.currentIndexChanged.connect(self._on_env_changed)
        env_row.addWidget(self.env_combo, 1)
        layout.addLayout(env_row)

        btn_row = QHBoxLayout()
        self.btn_launch_sim = QPushButton('Launch Simulation')
        self.btn_launch_sim.clicked.connect(self._on_launch_sim)
        self.btn_activate_nav = QPushButton('Load Map + Activate Nav2')
        self.btn_activate_nav.clicked.connect(self._on_activate_nav)
        btn_row.addWidget(self.btn_launch_sim)
        btn_row.addWidget(self.btn_activate_nav)
        layout.addLayout(btn_row)

        layout.addWidget(QLabel(
            'Loader launches the AMCL navigation stack using the saved map.'))

        pose_row = QHBoxLayout()
        self.btn_set_initial_pose = QPushButton('Set Initial Pose (Spawn)')
        self.btn_set_initial_pose.clicked.connect(self._on_set_initial_pose)
        self.spawn_label = QLabel('')
        pose_row.addWidget(self.btn_set_initial_pose)
        pose_row.addWidget(self.spawn_label, 1)
        layout.addLayout(pose_row)

        group.setLayout(layout)
        return group

    def _build_mission_group(self):
        group = QGroupBox('2. Goal & Waypoint Dispatcher')
        layout = QVBoxLayout()

        form = QFormLayout()
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(['Single Goal', 'Waypoints'])
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        form.addRow('Mode:', self.mode_combo)

        self.spin_x = QDoubleSpinBox()
        self.spin_x.setRange(-30.0, 30.0)
        self.spin_x.setDecimals(2)
        self.spin_x.setSingleStep(0.5)
        self.spin_y = QDoubleSpinBox()
        self.spin_y.setRange(-30.0, 30.0)
        self.spin_y.setDecimals(2)
        self.spin_y.setSingleStep(0.5)
        self.spin_yaw = QDoubleSpinBox()
        self.spin_yaw.setRange(-math.pi, math.pi)
        self.spin_yaw.setDecimals(2)
        self.spin_yaw.setSingleStep(0.1)
        pose_form = QHBoxLayout()
        pose_form.addWidget(QLabel('X'))
        pose_form.addWidget(self.spin_x, 1)
        pose_form.addWidget(QLabel('Y'))
        pose_form.addWidget(self.spin_y, 1)
        pose_form.addWidget(QLabel('Theta'))
        pose_form.addWidget(self.spin_yaw, 1)
        form.addRow('Pose (map):', pose_form)
        layout.addLayout(form)

        poi_row = QHBoxLayout()
        poi_row.addWidget(QLabel('Predefined POIs:'))
        self.poi_combo = QComboBox()
        self.poi_combo.currentIndexChanged.connect(self._on_poi_changed)
        poi_row.addWidget(self.poi_combo, 1)
        layout.addLayout(poi_row)

        wp_row = QHBoxLayout()
        self.btn_add_wp = QPushButton('Add as Waypoint')
        self.btn_add_wp.clicked.connect(self._on_add_waypoint)
        self.btn_remove_wp = QPushButton('Remove Selected')
        self.btn_remove_wp.clicked.connect(self._on_remove_waypoint)
        self.btn_clear_wp = QPushButton('Clear All')
        self.btn_clear_wp.clicked.connect(self._on_clear_waypoints)
        wp_row.addWidget(self.btn_add_wp)
        wp_row.addWidget(self.btn_remove_wp)
        wp_row.addWidget(self.btn_clear_wp)
        layout.addLayout(wp_row)

        self.wp_list = QListWidget()
        self.wp_list.setMaximumHeight(130)
        layout.addWidget(self.wp_list)

        group.setLayout(layout)
        return group

    def _build_control_group(self):
        group = QGroupBox('3. Execution Control')
        layout = QHBoxLayout()

        self.btn_start = QPushButton('Start Mission')
        self.btn_start.setStyleSheet('background:#2e7d32;color:white;font-weight:bold;padding:8px;')
        self.btn_start.clicked.connect(self._on_start)

        self.btn_pause = QPushButton('Pause')
        self.btn_pause.setStyleSheet('background:#f9a825;padding:8px;')
        self.btn_pause.clicked.connect(self._on_pause)

        self.btn_resume = QPushButton('Resume')
        self.btn_resume.setStyleSheet('background:#0277bd;color:white;padding:8px;')
        self.btn_resume.clicked.connect(self._on_resume)

        self.btn_cancel = QPushButton('Cancel Goal')
        self.btn_cancel.setStyleSheet('background:#c62828;color:white;padding:8px;')
        self.btn_cancel.clicked.connect(self._on_cancel)

        self.btn_replace = QPushButton('Replace Goal')
        self.btn_replace.setStyleSheet('background:#6a1b9a;color:white;padding:8px;')
        self.btn_replace.clicked.connect(self._on_replace)

        for btn in (self.btn_start, self.btn_pause, self.btn_resume,
                    self.btn_cancel, self.btn_replace):
            layout.addWidget(btn)

        group.setLayout(layout)
        return group

    def _build_status_group(self):
        group = QGroupBox('Mission Status')
        layout = QHBoxLayout()

        col1 = QVBoxLayout()
        self.status_state = QLabel('IDLE')
        self.status_state.setStyleSheet('font-weight:bold;font-size:14px;')
        self.status_mission = QLabel('-')
        col1.addWidget(self.status_state)
        col1.addWidget(self.status_mission)

        col2 = QVBoxLayout()
        self.status_pose = QLabel('pose: -')
        self.status_dist = QLabel('-')
        col2.addWidget(self.status_pose)
        col2.addWidget(self.status_dist)

        col3 = QVBoxLayout()
        self.status_eta = QLabel('-')
        self.status_wp = QLabel('-')
        col3.addWidget(self.status_eta)
        col3.addWidget(self.status_wp)

        layout.addLayout(col1, 1)
        layout.addLayout(col2, 1)
        layout.addLayout(col3, 1)
        group.setLayout(layout)
        return group

    def _build_log_group(self):
        group = QGroupBox('Real-Time Navigation Log')
        layout = QVBoxLayout()

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(5000)
        self.log_view.setFont(QFont('Monospace', 9))
        self.log_view.setStyleSheet(
            'QPlainTextEdit{background:#101418;color:#d0d4d8;}')
        layout.addWidget(self.log_view)

        clear_row = QHBoxLayout()
        btn_clear = QPushButton('Clear Log')
        btn_clear.clicked.connect(self.log_view.clear)
        self.lbl_src = QLabel('sources: /navigation/events /navigation/status')
        self.lbl_src.setStyleSheet('color:#888;')
        clear_row.addWidget(btn_clear)
        clear_row.addWidget(self.lbl_src, 1)
        layout.addLayout(clear_row)

        group.setLayout(layout)
        return group

    # ------------------------------------------------------- event handlers
    def _on_env_changed(self, _index):
        self._env_key = self.env_combo.currentData()
        self._populate_env(self._env_key)

    def _populate_env(self, key):
        cfg = ENVS[key]
        self.poi_combo.blockSignals(True)
        self.poi_combo.clear()
        for name in cfg['pois']:
            self.poi_combo.addItem(name)
        self.poi_combo.blockSignals(False)
        sx, sy, syaw = cfg['spawn']
        self.spawn_label.setText(f'({sx:.1f}, {sy:.1f}, {syaw:.2f} rad)')
        self._apply_poi(0)

    def _on_poi_changed(self, index):
        self._apply_poi(index)

    def _apply_poi(self, index):
        cfg = ENVS[self._env_key]
        name = list(cfg['pois'].keys())[index]
        x, y, yaw = cfg['pois'][name]
        self.spin_x.setValue(x)
        self.spin_y.setValue(y)
        self.spin_yaw.setValue(yaw)

    def _on_mode_changed(self, index):
        waypoint_mode = (index == 1)
        self.btn_add_wp.setEnabled(waypoint_mode)
        self.btn_remove_wp.setEnabled(waypoint_mode)
        self.btn_clear_wp.setEnabled(waypoint_mode)

    def _on_launch_sim(self):
        cfg = ENVS[self._env_key]
        self._run_launch(cfg['sim_launch'], args={'gui': 'true'})
        self._log('system', f"Launched simulation: {cfg['label']} ({cfg['sim_launch']})")

    def _on_activate_nav(self):
        cfg = ENVS[self._env_key]
        self._run_launch(cfg['nav_launch'], args={'launch_sim': 'false', 'gui': 'true'})
        self._log('system',
                  f"Activated Nav2 + map load: {cfg['label']} ({cfg['nav_launch']}) "
                  f'with launch_sim:=false')

    def _run_launch(self, launch_file, args=None):
        cmd = ['ros2', 'launch', PKG, launch_file]
        for key, value in (args or {}).items():
            cmd.append(f'{key}:={value}')
        self._log('system', 'Running: ' + ' '.join(cmd))
        try:
            proc = subprocess.Popen(cmd)
        except OSError as exc:
            self._log('error', f'Failed to launch: {exc}')
            return
        self._spawned_procs.append(proc)

    def _on_set_initial_pose(self):
        cfg = ENVS[self._env_key]
        x, y, yaw = cfg['spawn']
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        q = _yaw_to_quat(yaw)
        msg.pose.pose.orientation = q
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06
        self._initialpose_pub.publish(msg)
        self._log('pose', f'Set initial pose -> ({x:.2f}, {y:.2f}, {yaw:.2f})')

    def _next_mission_id(self):
        self._mission_seq += 1
        return f'gui-{self._env_key}-{self._mission_seq:03d}'

    def _build_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation = _yaw_to_quat(float(yaw))
        return pose

    def _current_plan(self):
        x = self.spin_x.value()
        y = self.spin_y.value()
        yaw = self.spin_yaw.value()
        waypoint_mode = self.mode_combo.currentIndex() == 1
        if not waypoint_mode:
            return (NavigationMission.MODE_GO_TO_POSE, self._build_pose(x, y, yaw), None)
        if self.wp_list.count() == 0:
            self._log('warn', 'Waypoint mode selected but no waypoints in list')
            return None
        poses = []
        for i in range(self.wp_list.count()):
            wx, wy, wyaw = self.wp_list.item(i).data(Qt.UserRole)
            poses.append(self._build_pose(wx, wy, wyaw))
        return (NavigationMission.MODE_WAYPOINTS, None, poses)

    def _on_start(self):
        plan = self._current_plan()
        if plan is None:
            return
        mode, pose, waypoints = plan
        mission_id = self._next_mission_id()
        self._publish_mission(NavigationMission.COMMAND_START, mode, mission_id,
                              pose=pose, waypoints=waypoints)
        desc = ('POI/goal' if waypoints is None
                else f'{len(waypoints)} waypoints')
        self._log('command', f'START mission {mission_id} ({desc})')

    def _on_pause(self):
        self._publish_mission(NavigationMission.COMMAND_PAUSE,
                              NavigationMission.MODE_GO_TO_POSE, self._next_mission_id())
        self._log('command', 'PAUSE sent')

    def _on_resume(self):
        self._publish_mission(NavigationMission.COMMAND_RESUME,
                              NavigationMission.MODE_GO_TO_POSE, self._next_mission_id())
        self._log('command', 'RESUME sent')

    def _on_cancel(self):
        self._publish_mission(NavigationMission.COMMAND_CANCEL,
                              NavigationMission.MODE_GO_TO_POSE, self._next_mission_id())
        self._log('command', 'CANCEL sent')

    def _on_replace(self):
        plan = self._current_plan()
        if plan is None:
            return
        mode, pose, waypoints = plan
        mission_id = self._next_mission_id()
        self._publish_mission(NavigationMission.COMMAND_REPLACE, mode, mission_id,
                              pose=pose, waypoints=waypoints)
        desc = ('new goal' if waypoints is None else f'{len(waypoints)} waypoints')
        self._log('command', f'REPLACE mission {mission_id} ({desc})')

    def _on_add_waypoint(self):
        x = self.spin_x.value()
        y = self.spin_y.value()
        yaw = self.spin_yaw.value()
        item = self.wp_list.addItem(f'({x:.2f}, {y:.2f}, {yaw:.2f})')
        self.wp_list.item(self.wp_list.count() - 1).setData(Qt.UserRole, (x, y, yaw))

    def _on_remove_waypoint(self):
        row = self.wp_list.currentRow()
        if row >= 0:
            self.wp_list.takeItem(row)

    def _on_clear_waypoints(self):
        self.wp_list.clear()

    # ------------------------------------------------------------ Qt slots
    def _on_event(self, msg):
        name = EVENT_NAMES.get(msg.event_type, f'EVENT_{msg.event_type}')
        color = EVENT_COLORS.get(msg.event_type, QColor(255, 255, 255))
        extra = ''
        if msg.event_type == NavigationEvent.EVENT_FEEDBACK:
            extra = (f'  [dist {msg.distance_remaining:.2f} m, '
                     f'wp {msg.current_waypoint}/{msg.total_waypoints}, '
                     f'eta {msg.estimated_time_remaining.sec}s, '
                     f'recoveries {msg.number_of_recoveries}]')
        elif msg.message:
            extra = f'  ({msg.message})'
        self._log(name, f'{msg.mission_id}{extra}', color)

    def _on_status(self, msg):
        state = STATE_NAMES.get(msg.state, f'STATE_{msg.state}')
        self.status_state.setText(state)
        color = {
            NavigationStatus.STATE_IDLE: '#ffffff',
            NavigationStatus.STATE_NAVIGATING: '#4fc3f7',
            NavigationStatus.STATE_PAUSED: '#ffd54f',
            NavigationStatus.STATE_COMPLETED: '#81c784',
            NavigationStatus.STATE_CANCELED: '#ff8a65',
            NavigationStatus.STATE_ABORTED: '#e57373',
            NavigationStatus.STATE_REJECTED: '#f06292',
        }.get(msg.state, '#ffffff')
        self.status_state.setStyleSheet(f'font-weight:bold;font-size:14px;color:{color};')
        self.status_mission.setText(f'mission: {msg.mission_id}')
        p = msg.current_pose.pose
        self.status_pose.setText(f'x {p.position.x:.2f} y {p.position.y:.2f} '
                                 f'yaw {msg.current_pose.pose.orientation.w:.2f}')
        self.status_dist.setText(f'remaining: {msg.distance_remaining:.2f} m')
        self.status_eta.setText(f'ETA: {msg.estimated_time_remaining.sec}s')
        self.status_wp.setText(f'waypoint: {msg.current_waypoint}/{msg.total_waypoints}')

    _STYLE = {
        'system': ('#9e9e9e', 'SYS'),
        'command': ('#cddc39', 'CMD'),
        'pose': ('#64b5f6', 'POSE'),
        'warn': ('#ffb74d', 'WARN'),
        'error': ('#e57373', 'ERROR'),
    }

    def _log(self, kind, text, color=None):
        if color is None:
            color, tag = self._STYLE.get(kind, ('#ffffff', kind.upper()))
        else:
            tag = kind
        html = (f'<span style="color:#616161;">{time.strftime("%H:%M:%S")}</span> '
                f'<span style="color:{color};">[{tag}]</span> '
                f'<span style="color:{color};">{text}</span>')
        self.log_view.appendHtml(html)

    # ------------------------------------------------------------- cleanup
    def closeEvent(self, event):
        for proc in self._spawned_procs:
            if proc.poll() is None:
                proc.terminate()
        self.shutdown_ros()
        super().closeEvent(event)


def main(argv=None):
    app = QApplication(argv or sys.argv)
    gui = MissionControlGUI()
    gui.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())