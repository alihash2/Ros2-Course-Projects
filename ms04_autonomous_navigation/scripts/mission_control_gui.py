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
import signal
import subprocess
import sys
import time
import math
import random
from collections import OrderedDict

if sys.platform.startswith('linux') and 'QT_QPA_PLATFORM' not in os.environ:
    os.environ['QT_QPA_PLATFORM'] = 'xcb'

from PyQt5.QtCore import QThread, QPointF, QRectF, QTimer, pyqtSignal, Qt
from PyQt5.QtGui import (
    QBrush, QColor, QFont, QPainter, QPolygonF, QRadialGradient)
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QFormLayout,
    QGraphicsBlurEffect, QGraphicsDropShadowEffect, QGroupBox, QHBoxLayout,
    QLabel, QListWidget, QMainWindow, QMessageBox, QPlainTextEdit,
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
        'world': 'office.world',
        'sim_launch': 'office_simulation.launch.py',
        'nav_launch': 'office_navigation.launch.py',
        'spawn': (0.0, 0.0, 0.0),
        'initial_pose': (0.0, 0.0, 0.0),
        'pois': OrderedDict([
            ('Spawn', (0.0, 0.0, 0.0)),
            ('Corridor Center', (0.0, -4.0, 0.0)),
            ('NW Conference', (-2.5, 5.5, math.pi)),
            ('SW Reception', (-3.2, -2.2, math.pi)),
            ('NE Cubicles', (3.0, 3.2, 0.0)),
            ('SE Breakroom', (3.0, -1.8, 0.0)),
        ]),
    },
    'warehouse': {
        'label': 'Warehouse',
        'world': 'warehouse.world',
        'sim_launch': 'warehouse_simulation.launch.py',
        'nav_launch': 'warehouse_navigation.launch.py',
        'spawn': (-6.0, 0.0, 0.0),
        'initial_pose': (0.0, 0.0, 0.0),
        'pois': OrderedDict([
            ('Spawn', (0.0, 0.0, 0.0)),
            ('Aisle Center', (6.5, 0.0, 0.0)),
            ('Loading Dock (West)', (-3.0, 1.5, math.pi)),
            ('East Storage', (9.5, 0.0, 0.0)),
            ('Rack A (North)', (5.0, 6.0, 0.0)),
            ('Rack B (South)', (5.0, -6.0, 0.0)),
        ]),
    },
})

# How close (in metres) a typed/add pose must be to a predefined POI before the
# waypoint list labels it with the POI name instead of raw coordinates.
POI_MATCH_TOLERANCE = 0.5

# Accent tints used by the status banner. Turquoise night - a fresh, vibrant
# teal-to-cyan family over a deep sea backdrop.
STATE_GLOW = {
    'IDLE': ('#1f2e36', '#141d23'),
    'NAVIGATING': ('#29c6ff', '#0e4a66'),
    'PAUSED': ('#2fe3c4', '#0c5a52'),
    'COMPLETED': ('#3ecf8e', '#0c4a34'),
    'CANCELED': ('#5a8a94', '#2b4852'),
    'ABORTED': ('#ff6b6b', '#7a1e1e'),
    'REJECTED': ('#ff7a5c', '#7a2b1e'),
}

# Chroma per mission button - hue of the "lit lamp" glow in the dark.
# Mirrors the chosen STATE_GLOW (top-bar) tint for each execution status.
_BTN_GLOW = {
    'btnStart': (41, 198, 255, 110),
    'btnPause': (47, 227, 196, 110),
    'btnResume': (62, 207, 142, 110),
    'btnCancel': (110, 160, 170, 100),
    'btnReplace': (255, 122, 92, 105),
}


def _hex_rgb(h):
    """'#rrggbb' -> (r, g, b) tuple."""
    h = h.lstrip('#')
    return tuple(int(h[i:i + 2], 16) for i in (0, 2, 4))

DARK_QSS = '''
QMainWindow, QDialog {
    background: qlineargradient(x1:0.2, y1:0, x2:0.8, y2:1,
        stop:0 #1a4a50, stop:0.3 #13383f,
        stop:0.7 #0e2830, stop:1 #06161c);
    color: #eaf6f5;
}
QWidget {
    color: #eaf6f5;
    font-size: 15px;
    font-family: 'Lato', 'Cantarell', 'Noto Sans',
        'DejaVu Sans', sans-serif;
}
QGroupBox {
    border: 1px solid rgba(130,225,200,0.28);
    border-top-color: rgba(200,255,235,0.45);
    border-radius: 14px;
    margin-top: 26px;
    padding-top: 8px;
    padding-bottom: 6px;
    background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
        stop:0 rgba(45,150,140,0.34), stop:0.45 rgba(25,110,115,0.26),
        stop:0.8 rgba(12,70,80,0.20), stop:1 rgba(7,26,32,0.72));
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 14px;
    padding: 2px 10px 2px 10px;
    color: #a2f2dd;
    font-size: 14px;
    font-weight: 700;
    letter-spacing: 2px;
    background: transparent;
}
QLabel { background: transparent; }
QLabel#envHint { color: #7faeae; font-size: 13px; }
QLabel#monoVal {
    color: #bdeee8;
    font-family: 'DejaVu Sans Mono', monospace;
    font-size: 14px;
    background: transparent;
}
QLabel#monoVal[stateIdle="true"] { color: #4d7678; }

QComboBox, QDoubleSpinBox, QSpinBox {
    background: rgba(13,32,40,0.82);
    border: 1px solid rgba(120,200,210,0.22);
    border-radius: 9px;
    padding: 6px 10px;
    selection-background-color: #3fe0cf;
    selection-color: #05201d;
}
QComboBox:hover, QDoubleSpinBox:hover, QSpinBox:hover {
    border-color: rgba(110,240,220,0.55);
    background: rgba(16,38,48,0.88);
}
QComboBox:focus, QDoubleSpinBox:focus, QSpinBox:focus {
    border: 1px solid #47e8e0;
}
QComboBox::drop-down {
    border: none; width: 24px;
}
QComboBox::down-arrow {
    image: none;
    width: 0; height: 0;
    border-left: 5px solid transparent;
    border-right: 5px solid transparent;
    border-top: 6px solid #6ff2e0;
    margin-right: 6px;
}
QComboBox QAbstractItemView {
    background: #0f2a32;
    border: 1px solid rgba(120,200,210,0.26);
    border-radius: 10px;
    padding: 4px;
    selection-background-color: rgba(70,230,210,0.24);
    selection-color: #eaf6f5;
    outline: none;
}
QComboBox QAbstractItemView::item {
    border-radius: 6px;
    padding: 4px 8px;
}
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button,
QSpinBox::up-button, QSpinBox::down-button {
    background: transparent; border: none; width: 18px;
}
QDoubleSpinBox::up-arrow, QSpinBox::up-arrow {
    image: none; width: 0; height: 0;
    border-left: 4px solid transparent; border-right: 4px solid transparent;
    border-bottom: 5px solid #7fc9c0;
}
QDoubleSpinBox::down-arrow, QSpinBox::down-arrow {
    image: none; width: 0; height: 0;
    border-left: 4px solid transparent; border-right: 4px solid transparent;
    border-top: 5px solid #7fc9c0;
}

QPushButton {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #24464e, stop:0.08 #1b3a42,
        stop:0.5 #152e36, stop:1 #0d1f25);
    border: 1px solid rgba(140,220,225,0.25);
    border-top-color: rgba(200,255,255,0.38);
    border-radius: 12px;
    padding: 7px 14px;
    color: #e9faf8;
    font-size: 14px;
    font-weight: 600;
}
QPushButton:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #2a525c, stop:0.08 #20424c,
        stop:0.5 #183640, stop:1 #0f252c);
    border-color: rgba(110,240,220,0.55);
    color: #ffffff;
}
QPushButton:pressed {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #0a1a1f, stop:1 #12262d);
    padding-top: 11px; padding-bottom: 9px;
}
QPushButton:disabled { color: #4c7173; border-color: rgba(200,255,255,0.09); }
QPushButton:focus {
    border: 1px solid #47e8e0;
}

QPushButton#btnStart {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #bdeeff, stop:0.12 #29c6ff,
        stop:0.6 #1189c9, stop:1 #0e4a66);
    border: 1px solid #7fd4ff;
    border-top-color: #e8f8ff;
    color: #041a26;
}
QPushButton#btnStart:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #d0f4ff, stop:0.12 #47d2ff,
        stop:0.6 #16a0e0, stop:1 #115a7a);
    border-color: #a8e3ff;
}
QPushButton#btnStart:pressed {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #0a2e40, stop:1 #0f5a7a);
}
QPushButton#btnPause {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #b9ffee, stop:0.12 #2fe3c4,
        stop:0.6 #14a98f, stop:1 #0c5a52);
    border: 1px solid #7fffdd;
    border-top-color: #e8fffa;
    color: #04251f;
}
QPushButton#btnPause:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #d4fff5, stop:0.12 #4bf0d2,
        stop:0.6 #1ac4a6, stop:1 #0f6f66);
    border-color: #a8ffea;
}
QPushButton#btnPause:pressed {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #0a352f, stop:1 #117067);
}
QPushButton#btnResume {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #baffe0, stop:0.12 #3ecf8e,
        stop:0.6 #1d945c, stop:1 #0c4a34);
    border: 1px solid #7fffc0;
    border-top-color: #e8fff2;
    color: #04261a;
}
QPushButton#btnResume:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #d4fff0, stop:0.12 #55e0a4,
        stop:0.6 #23ab6c, stop:1 #0f5c40);
    border-color: #a8ffd9;
}
QPushButton#btnResume:pressed {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #0a3526, stop:1 #115c40);
}
QPushButton#btnCancel {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #b8d3da, stop:0.12 #5a8a94,
        stop:0.6 #3a5d68, stop:1 #2b4852);
    border: 1px solid #9cc2cb;
    border-top-color: #e6f2f5;
    color: #0c1a1f;
}
QPushButton#btnCancel:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #d2e6ec, stop:0.12 #6a9aa6,
        stop:0.6 #446b78, stop:1 #33535e);
    border-color: #c0dce2;
}
QPushButton#btnCancel:pressed {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #1e333a, stop:1 #2e4d57);
}
QPushButton#btnReplace {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #ffd2c2, stop:0.12 #ff7a5c,
        stop:0.6 #c74a2f, stop:1 #7a2b1e);
    border: 1px solid #ffb49e;
    border-top-color: #fff0e8;
    color: #2b0c05;
}
QPushButton#btnReplace:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #ffe2d6, stop:0.12 #ff8f73,
        stop:0.6 #d9583a, stop:1 #8a3722);
    border-color: #ffcab8;
}
QPushButton#btnReplace:pressed {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #4a1f12, stop:1 #71301c);
}

QPushButton#btnStart, QPushButton#btnPause, QPushButton#btnResume,
QPushButton#btnCancel, QPushButton#btnReplace {
    font-size: 13px;
    padding: 6px 14px;
}

QListWidget {
    background: rgba(1,16,24,0.80);
    border: 1px solid rgba(120,200,210,0.22);
    border-radius: 10px;
    padding: 4px;
    outline: none;
}
QListWidget::item {
    border-radius: 6px;
    padding: 4px 10px;
    margin: 1px 2px;
}
QListWidget::item:hover { background: rgba(110,240,220,0.10); }
QListWidget::item:selected {
    background: rgba(70,230,210,0.20);
    border: 1px solid rgba(100,235,210,0.55);
    padding: 3px 9px;
}

QPlainTextEdit {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 rgba(0,14,22,0.88), stop:1 rgba(0,6,12,0.94));
    border: 1px solid rgba(120,200,210,0.22);
    border-radius: 10px;
    padding: 8px;
    selection-background-color: rgba(70,230,210,0.35);
    selection-color: #eaf6f5;
}
QPlainTextEdit:focus { border: 1px solid #47e8e0; }

QScrollBar:vertical {
    background: transparent; width: 9px; margin: 2px;
}
QScrollBar::handle:vertical {
    background: #2e5257; border-radius: 4px; min-height: 24px;
}
QScrollBar::handle:vertical:hover { background: #3fe0cf; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: transparent; }
QScrollBar:horizontal {
    background: transparent; height: 9px; margin: 2px;
}
QScrollBar::handle:horizontal {
    background: #2e5257; border-radius: 4px; min-width: 24px;
}
QScrollBar::handle:horizontal:hover { background: #3fe0cf; }
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width: 0; }
QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal { background: transparent; }

QMenu {
    background: #0f2a32;
    border: 1px solid rgba(120,200,210,0.26);
    border-radius: 10px;
    padding: 5px;
}
QMenu::item { padding: 6px 24px; border-radius: 6px; }
QMenu::item:selected { background: rgba(70,230,210,0.22); color: #eaf6f5; }

QToolTip {
    background: #0f2a32;
    color: #eaf6f5;
    border: 1px solid #47e8e0;
    border-radius: 7px;
    padding: 6px 10px;
}
QMessageBox {
    background: #0d242c;
}
QMessageBox QLabel { color: #eaf6f5; }
'''

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
        NavigationEvent.EVENT_GOAL_COMPLETED: QColor(0, 220, 90),
        NavigationEvent.EVENT_GOAL_CANCELED: QColor(255, 152, 0),
        NavigationEvent.EVENT_GOAL_ABORTED: QColor(255, 23, 68),
        NavigationEvent.EVENT_GOAL_PAUSED: QColor(255, 193, 7),
        NavigationEvent.EVENT_GOAL_RESUMED: QColor(0, 188, 212),
        NavigationEvent.EVENT_GOAL_REPLACED: QColor(156, 39, 176),
    }

_EVENT_FALLBACK = {
    NavigationEvent.EVENT_GOAL_SUBMITTED: 'mission submitted for execution',
    NavigationEvent.EVENT_GOAL_ACCEPTED: 'mission accepted by Nav2',
    NavigationEvent.EVENT_GOAL_REJECTED: 'mission rejected',
    NavigationEvent.EVENT_FEEDBACK: '',
    NavigationEvent.EVENT_GOAL_COMPLETED: 'mission completed',
    NavigationEvent.EVENT_GOAL_CANCELED: 'mission canceled',
    NavigationEvent.EVENT_GOAL_ABORTED: 'mission aborted',
    NavigationEvent.EVENT_GOAL_PAUSED: 'mission paused',
    NavigationEvent.EVENT_GOAL_RESUMED: 'mission resumed',
    NavigationEvent.EVENT_GOAL_REPLACED: 'mission replaced',
}

# Status messages we want to surface in the log even though the events already
# render the main flow -> anything that hints at a problem or a preemption.
_DIAG_HINTS = ('ignored', 'reject', 'unavailable', 'failed', 'error',
               'abort', 'replacing', 'paused at waypoint', 'recovery', 'stuck')


def _yaw_to_quat(yaw):
    q = PoseStamped().pose.orientation
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


# Command-line fragments used to detect our own sim/nav/rviz stacks so the GUI
# never double-launches and can clean up cleanly on close.
_SIM_MARKERS = {
    'office': ['worlds/office.world'],
    'warehouse': ['worlds/warehouse.world'],
}
_NAV_MARKERS = {
    'office': ['params/office_nav2_params.yaml'],
    'warehouse': ['params/warehouse_nav2_params.yaml'],
}
_RVIZ_MARKER = ['rviz2', 'nav2_gui_view.rviz']

# Process families that belong to our stack but carry no per-env marker
# (Gazebo GUI clients, bridge, relay, RSP, lifecycle managers, coordinator,
# explorer, and the ros2 launch parents). Used by cleanup-on-close only.
_STACK_MARKERS = (
    ['gz sim -g'],
    ['turtlebot3_waffle_bridge.yaml'],
    ['cmd_vel_relay'],
    ['robot_state_publisher'],
    ['lifecycle_manager'],
    ['navigation_coordinator_node'],
    ['auto_slam_explorer'],
    ['ros2 launch ms04_autonomous_navigation'],
)


def _pids_matching(needles):
    """Return set of live PIDs whose command line contains ALL needle strings."""
    try:
        out = subprocess.run(['ps', '-eo', 'pid,args'],
                             capture_output=True, text=True, timeout=3)
    except (OSError, subprocess.TimeoutExpired):
        return set()
    found = set()
    for line in out.stdout.splitlines():
        parts = line.split(None, 1)
        if len(parts) != 2:
            continue
        pid, args = parts
        if all(n in args for n in needles):
            try:
                found.add(int(pid))
            except ValueError:
                pass
    return found - {os.getpid()}


def _kill_pids(pids, grace=1.5):
    """SIGTERM then SIGKILL: send SIGTERM to the whole set, wait briefly,
    then escalate anything still alive. Favours simple process trees over
    expensive negative-pgid walking."""
    for pid in list(pids):
        try:
            os.kill(pid, signal.SIGTERM)
        except (OSError, ProcessLookupError):
            pass
    deadline = time.monotonic() + grace
    remaining = list(pids)
    while remaining and time.monotonic() < deadline:
        time.sleep(0.1)
        remaining = [p for p in remaining if os.path.exists(f'/proc/{p}')]
    for pid in remaining:
        try:
            os.kill(pid, signal.SIGKILL)
        except (OSError, ProcessLookupError):
            pass


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


class BackdropWidget(QWidget):
    """Fills the window behind every container with soft, randomly-shaped
    blobs tinted to the current mission-state glow colour. The paint is done
    with radial gradients, giving the shapes a blurred, glowing look."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self._tint = QColor(41, 198, 255)
        self._shapes = None
        self._seed_points()

    def _seed_points(self):
        rng = random
        shapes = []
        for _ in range(5):
            cx = rng.uniform(0.05, 0.95)
            cy = rng.uniform(0.05, 0.95)
            r = rng.uniform(0.08, 0.30)
            kind = rng.choice('etq')
            shapes.append((cx, cy, r, rng.uniform(0, math.tau), kind))
        self._shapes = shapes

    def set_tint(self, hex_color):
        self._tint = QColor(hex_color)
        self.update()

    def paintEvent(self, _event):
        if not self._shapes:
            return
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        for cx, cy, r, rot, kind in self._shapes:
            size = r * min(w, h)
            cx, cy = cx * w, cy * h
            grad = QRadialGradient(cx, cy, size)
            grad.setColorAt(0.0, QColor(self._tint.red(), self._tint.green(),
                                        self._tint.blue(), 110))
            grad.setColorAt(0.55, QColor(self._tint.red(), self._tint.green(),
                                         self._tint.blue(), 42))
            grad.setColorAt(1.0, QColor(self._tint.red(), self._tint.green(),
                                        self._tint.blue(), 0))
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(grad))
            p.save()
            p.translate(cx, cy)
            p.rotate(rot)
            if kind == 't':
                tri = QPolygonF([QPointF(-size, size * 0.9),
                                 QPointF(size, size * 0.9),
                                 QPointF(0, -size)])
                p.drawPolygon(tri)
            elif kind == 'q':
                p.drawRect(QRectF(-size, -size * 0.7, size * 2, size * 1.4))
            else:
                p.drawEllipse(QRectF(-size, -size, size * 2, size * 2))
            p.restore()
        p.end()


class MissionControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Mission Control - ms04_autonomous_navigation')
        self.resize(1150, 780)

        self._env_key = 'office'
        self._mission_seq = 0
        self._spawned_procs = []

        self._status_state = 'IDLE'
        self._last_logged_line = ''
        self._fb_ctx = {'mission': None, 'wp': -1, 'rec': -1}
        self._tracked_mission = None
        self._tracked_waypoints = False
        self._tracked_plan = None
        self._plan_pose = None
        self._wp_offset = 0

        self._pose_set = False
        self._sim_up = False
        self._nav_up = False
        self._exec_up = False
        self._nav_launch_requested = False
        self._nav_fault_logged = False
        self._custom_selected = False
        self._syncing_coords = False
        self._pulse_timers = {}
        self._error_timer = None

        self._build_ros()
        self._build_ui()
        self._apply_banner('IDLE')

        self._ros_thread = RosSpinnerThread(self._node)
        self._ros_thread.event_received.connect(self._on_event, Qt.QueuedConnection)
        self._ros_thread.status_received.connect(self._on_status, Qt.QueuedConnection)
        self._ros_thread.start()

        self._populate_env('office')
        self._log_stack_status()
        self._verify_stack()
        self._refresh_workflow()
        self._refresh_status_wp()
        self._verify_timer = QTimer(self)
        self._verify_timer.setInterval(2000)
        self._verify_timer.timeout.connect(self._verify_stack)
        self._verify_timer.start()
        self._log('system',
                  'GUI ready. Workflow: 1) Launch Map, '
                  '2) Load Map + Nav (wait for it to fully start), '
                  '3) Set Initial Pose, '
                  '4) dispatch a goal below. Follow the enabled buttons.')

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

    def _log_stack_status(self):
        for key in ENVS:
            label = ENVS[key]['label']
            sim = _pids_matching(_SIM_MARKERS[key])
            nav = _pids_matching(_NAV_MARKERS[key])
            bits = []
            if sim:
                bits.append(f'sim up (pids {sorted(sim)})')
            if nav:
                bits.append(f'nav up (pids {sorted(nav)})')
            if bits:
                self._log('info', f'{label}: ' + '; '.join(bits))
            else:
                self._log('info', f'{label}: not running')

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
        outer = QVBoxLayout(central)
        outer.setSpacing(12)

        self.backdrop = BackdropWidget(central)
        self.backdrop.lower()
        self.backdrop.setGeometry(central.rect())
        blur_eff = QGraphicsBlurEffect(self.backdrop)
        blur_eff.setBlurRadius(26)
        self.backdrop.setGraphicsEffect(blur_eff)

        self.error_bar = QLabel('')
        self.error_bar.setWordWrap(True)
        self.error_bar.setStyleSheet(
            'background:#c62828;color:#ffffff;font-weight:bold;'
            'padding:8px 14px;border-radius:8px;')
        self.error_bar.setVisible(False)
        outer.addWidget(self.error_bar)

        self.state_banner = QLabel('MISSION STATE: IDLE')
        self.state_banner.setAlignment(Qt.AlignCenter)
        self.state_banner.setMinimumHeight(62)
        glow = QGraphicsDropShadowEffect(self.state_banner)
        glow.setBlurRadius(22)
        glow.setOffset(0, 0)
        glow.setColor(QColor(60, 224, 200, 90))
        self.state_banner.setGraphicsEffect(glow)
        self._banner_glow = glow
        outer.addWidget(self.state_banner)

        root = QHBoxLayout()

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
        outer.addLayout(root, 1)

    def _build_env_group(self):
        group = QGroupBox('1. Environment & Navigation')
        layout = QVBoxLayout()
        layout.setSpacing(8)

        env_row = QHBoxLayout()
        env_row.addWidget(QLabel('Environment:'))
        self.env_combo = QComboBox()
        for key, cfg in ENVS.items():
            self.env_combo.addItem(cfg['label'], key)
        self.env_combo.currentIndexChanged.connect(self._on_env_changed)
        env_row.addWidget(self.env_combo, 1)
        layout.addLayout(env_row)

        btn_row = QHBoxLayout()
        self.btn_launch_sim = QPushButton('Launch Map')
        self.btn_launch_sim.clicked.connect(self._on_launch_sim)
        self.btn_activate_nav = QPushButton('Load Map + Nav')
        self.btn_activate_nav.clicked.connect(self._on_activate_nav)
        btn_row.addWidget(self.btn_launch_sim)
        btn_row.addWidget(self.btn_activate_nav)
        layout.addLayout(btn_row)

        hint = QLabel(
            'Loader launches the AMCL navigation stack using the saved map.')
        hint.setObjectName('envHint')
        layout.addWidget(hint)

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
        layout.setSpacing(8)

        form = QFormLayout()
        form.setSpacing(6)
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
        self.spin_x.valueChanged.connect(self._sync_poi_from_coords)
        self.spin_y.valueChanged.connect(self._sync_poi_from_coords)
        for spin in (self.spin_x, self.spin_y, self.spin_yaw):
            spin.valueChanged.connect(self._refresh_workflow)
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
        self.wp_list.setMaximumHeight(360)
        layout.addWidget(self.wp_list)

        legend = QHBoxLayout()
        legend.setSpacing(8)
        self._lamps = {}
        for key, label, on_c, on_g, off_c in (
                ('loaded', 'Loaded', (215, 228, 255), (170, 200, 255),
                 (34, 44, 58)),
                ('progress', 'Progress', (200, 255, 250), (120, 255, 240),
                 (24, 64, 60)),
                ('paused', 'Paused', (255, 240, 170), (255, 215, 100),
                 (64, 52, 16)),
                ('reached', 'Reached', (210, 255, 230), (110, 255, 190),
                 (22, 46, 34))):
            wrap = QWidget()
            pair = QHBoxLayout(wrap)
            pair.setContentsMargins(0, 0, 0, 0)
            pair.setSpacing(5)
            pair.setAlignment(Qt.AlignCenter)
            dot = QLabel('\u25cf')
            dot.setStyleSheet(
                f'color:rgb({off_c[0]},{off_c[1]},{off_c[2]});'
                'font-size:24px;padding:0px;')
            glow = QGraphicsDropShadowEffect(dot)
            glow.setBlurRadius(10)
            glow.setOffset(0, 0)
            glow.setColor(QColor(0, 0, 0, 0))
            dot.setGraphicsEffect(glow)
            name = QLabel(label)
            name.setStyleSheet('color:#bdeee8;')
            pair.addWidget(dot)
            pair.addWidget(name)
            legend.addWidget(wrap, 1)
            self._lamps[key] = (dot, glow, on_c, on_g, off_c)
        layout.addLayout(legend)

        group.setLayout(layout)
        return group

    def _update_lamps(self, counts):
        """Light the loaded/progress/paused/reached legend lamps like switches:
        a bright glowing bulb when the waypoint-tracking state is present, a
        dull dark bulb when not. Single-goal missions never light any lamp."""
        for key, (dot, glow, on_c, on_g, off_c) in self._lamps.items():
            if counts.get(key, 0) > 0:
                dot.setStyleSheet(
                    f'color:rgb({on_c[0]},{on_c[1]},{on_c[2]});'
                    'font-size:24px;padding:0px;')
                glow.setColor(QColor(*on_g, 215))
                glow.setBlurRadius(22)
            else:
                dot.setStyleSheet(
                    f'color:rgb({off_c[0]},{off_c[1]},{off_c[2]});'
                    'font-size:24px;padding:0px;')
                glow.setColor(QColor(0, 0, 0, 0))
                glow.setBlurRadius(4)

    def _build_control_group(self):
        group = QGroupBox('3. Execution Control')
        layout = QHBoxLayout()
        layout.setSpacing(8)
        layout.setContentsMargins(10, 6, 10, 8)
        layout.setAlignment(Qt.AlignCenter)

        self.btn_start = QPushButton('Start Mission')
        self.btn_start.setObjectName('btnStart')
        self.btn_start.clicked.connect(self._on_start)

        self.btn_pause = QPushButton('Pause')
        self.btn_pause.setObjectName('btnPause')
        self.btn_pause.clicked.connect(self._on_pause)

        self.btn_resume = QPushButton('Resume')
        self.btn_resume.setObjectName('btnResume')
        self.btn_resume.clicked.connect(self._on_resume)

        self.btn_cancel = QPushButton('Cancel Goal')
        self.btn_cancel.setObjectName('btnCancel')
        self.btn_cancel.clicked.connect(self._on_cancel)

        self.btn_replace = QPushButton('Replace Goal')
        self.btn_replace.setObjectName('btnReplace')
        self.btn_replace.clicked.connect(self._on_replace)

        for btn in (self.btn_start, self.btn_pause, self.btn_resume,
                    self.btn_cancel, self.btn_replace):
            layout.addWidget(btn)
            glow = _BTN_GLOW.get(btn.objectName())
            if glow:
                eff = QGraphicsDropShadowEffect(btn)
                eff.setBlurRadius(14)
                eff.setOffset(0, 2)
                eff.setColor(QColor(*glow))
                btn.setGraphicsEffect(eff)

        self.btn_start.setToolTip(
            'Send a fresh mission. Rejected by the coordinator if a mission is '
            'already active - use Replace to preempt.')
        self.btn_pause.setToolTip(
            'Stop the robot; the mission keeps its remaining waypoints so they '
            'can be resumed.')
        self.btn_resume.setToolTip(
            'Continue a paused mission from the remaining waypoints.')
        self.btn_cancel.setToolTip(
            'Cancel the active mission and return to IDLE.')
        self.btn_replace.setToolTip(
            'Cancel the current goal and immediately dispatch the plan currently '
            'in the Dispatcher (works while paused too).')

        group.setLayout(layout)
        return group

    def _build_status_group(self):
        group = QGroupBox('Mission Status')
        layout = QVBoxLayout()
        layout.setSpacing(6)

        self.status_state = QLabel('IDLE')
        self.status_state.setStyleSheet(
            'font-weight:bold;font-size:20px;letter-spacing:2px;')
        self.status_mission = QLabel('-')
        self.status_mission.setObjectName('monoVal')

        self.status_target = QLabel('target: -')
        self.status_target.setObjectName('monoVal')

        rows = QVBoxLayout()
        rows.setSpacing(8)
        self.status_pose = QLabel('pose: -')
        self.status_pose.setObjectName('monoVal')
        self.status_dist = QLabel('-')
        self.status_dist.setObjectName('monoVal')
        rows.addWidget(self.status_pose)
        rows.addWidget(self.status_dist)

        cols = QVBoxLayout()
        cols.setSpacing(8)
        self.status_eta = QLabel('-')
        self.status_eta.setObjectName('monoVal')
        self.status_wp = QLabel('-')
        self.status_wp.setObjectName('monoVal')
        cols.addWidget(self.status_eta)
        cols.addWidget(self.status_wp)

        for lbl in (self.status_mission, self.status_target, self.status_pose,
                    self.status_dist, self.status_eta, self.status_wp):
            lbl.setFont(QFont('DejaVu Sans Mono', 13))

        grid = QHBoxLayout()
        grid.addLayout(rows, 1)
        grid.addLayout(cols, 1)

        layout.addWidget(self.status_state)
        layout.addWidget(self.status_mission)
        layout.addWidget(self.status_target)
        layout.addLayout(grid)
        group.setLayout(layout)
        return group

    def _build_log_group(self):
        group = QGroupBox('Real-Time Navigation Log')
        layout = QVBoxLayout()
        layout.setSpacing(6)

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(5000)
        self.log_view.setFont(QFont('Monospace', 11))
        layout.addWidget(self.log_view)

        clear_row = QHBoxLayout()
        btn_clear = QPushButton('Clear Log')
        btn_clear.clicked.connect(self.log_view.clear)
        self.lbl_src = QLabel('sources: /navigation/events /navigation/status')
        self.lbl_src.setStyleSheet('color:#6fa5ab;font-size:12px;')
        clear_row.addWidget(btn_clear)
        clear_row.addWidget(self.lbl_src, 1)
        layout.addLayout(clear_row)

        group.setLayout(layout)
        return group

    # ------------------------------------------------------- event handlers
    def _on_env_changed(self, _index):
        if self.env_combo.currentData() == self._env_key:
            return
        self.wp_list.clear()
        self._tracked_mission = None
        self._tracked_waypoints = False
        self._tracked_plan = None
        self._plan_pose = None
        self._wp_offset = 0
        self._pose_set = False
        self._sim_up = False
        self._nav_up = False
        self._exec_up = False
        self._nav_launch_requested = False
        self._nav_fault_logged = False
        self._env_key = self.env_combo.currentData()
        self._populate_env(self._env_key)
        self._refresh_status_wp()
        self._refresh_workflow()
        self._verify_stack()
        self._log('warn',
                  f'Environment switched to {ENVS[self._env_key]["label"]}: '
                  'waypoints cleared, initial pose reset')

    def _populate_env(self, key):
        cfg = ENVS[key]
        self.poi_combo.blockSignals(True)
        self.poi_combo.clear()
        for name in cfg['pois']:
            self.poi_combo.addItem(name)
        self.poi_combo.addItem('Custom (type coords below)')
        self.poi_combo.blockSignals(False)
        ix, iy, iyaw = cfg['initial_pose']
        self.spawn_label.setText(
            f'map initial pose ({ix:.1f}, {iy:.1f}, {iyaw:.2f} rad) '
            f'· world spawn ({cfg["spawn"][0]:.1f}, {cfg["spawn"][1]:.1f})')
        self._custom_selected = False
        self._apply_poi(0)

    def _on_poi_changed(self, index):
        self._apply_poi(index)

    def _apply_poi(self, index):
        cfg = ENVS[self._env_key]
        pois = list(cfg['pois'].keys())
        if index >= len(pois):  # the trailing 'Custom' entry
            self._custom_selected = True
            self._update_pose_inputs()
            self._refresh_workflow()
            return
        self._custom_selected = False
        name = pois[index]
        x, y, yaw = cfg['pois'][name]
        self._syncing_coords = True
        try:
            self.spin_x.setValue(x)
            self.spin_y.setValue(y)
            self.spin_yaw.setValue(yaw)
        finally:
            self._syncing_coords = False
        self._update_pose_inputs()
        self._refresh_workflow()

    def _sync_poi_from_coords(self):
        """Unified POI <-> coordinate input: the X/Y coordinates you typed are
        the single source of truth. Matching a predefined POI (within the
        tolerance) automatically selects that POI and snaps the pose to its
        exact values; any other coordinate selects 'Custom'. The two controls
        can never disagree about which goal would actually be dispatched."""
        if self._syncing_coords:
            return
        cfg = ENVS[self._env_key]
        x = self.spin_x.value()
        y = self.spin_y.value()
        poi_keys = list(cfg['pois'].keys())
        match = None
        for name, (px, py, pyaw) in cfg['pois'].items():
            if math.hypot(x - px, y - py) <= POI_MATCH_TOLERANCE:
                match = (name, px, py, pyaw)
                break
        if match is None:
            if not self._custom_selected:
                self.poi_combo.blockSignals(True)
                self.poi_combo.setCurrentIndex(self.poi_combo.count() - 1)
                self.poi_combo.blockSignals(False)
                self._custom_selected = True
            self._update_pose_inputs()
            return
        name, px, py, pyaw = match
        self._syncing_coords = True
        try:
            self.poi_combo.blockSignals(True)
            self.poi_combo.setCurrentIndex(poi_keys.index(name))
            self.poi_combo.blockSignals(False)
            self._custom_selected = False
            self.spin_x.setValue(px)
            self.spin_y.setValue(py)
            self.spin_yaw.setValue(pyaw)
        finally:
            self._syncing_coords = False
        self._update_pose_inputs()

    def _on_mode_changed(self, _index):
        self._refresh_lamps()
        self._refresh_status_wp()
        self._refresh_workflow()

    # --------------------------------------------------------- workflow rails
    _DIM_BTN = (
        'QPushButton { background:#252b32; color:#5d6670; border:1px solid '
        '#333b44; border-radius:8px; padding:8px 14px; } '
        'QComboBox, QDoubleSpinBox, QLabel { color:#5d6670; } ')

    def _set_dim(self, widget, on):
        """Visually grey a widget out but keep it clickable, so pressing it can
        raise the guidance error bar (only mode-gated items are truly disabled)."""
        if getattr(widget, '_dimmed', False) == on:
            return
        widget._dimmed = on
        widget.setStyleSheet(self._DIM_BTN if on else '')

    def _pulse_button(self, button, on):
        """Start/stop a soft glow on a button to point the user at the next
        required action: the colour alternates gently between white and red
        (previous version flashed the border too aggressively)."""
        if on:
            if id(button) in self._pulse_timers:
                return
            eff = QGraphicsDropShadowEffect(button)
            eff.setBlurRadius(22)
            eff.setOffset(0, 0)
            button.setGraphicsEffect(eff)
            timer = QTimer(button)
            timer.setInterval(800)
            state = {'white': True}

            def tick():
                state['white'] = not state['white']
                if state['white']:
                    eff.setColor(QColor(255, 255, 255, 70))
                    eff.setBlurRadius(30)
                else:
                    eff.setColor(QColor(255, 70, 70, 85))
                    eff.setBlurRadius(24)

            timer.timeout.connect(tick)
            timer.start()
            tick()
            self._pulse_timers[id(button)] = (timer, eff, button)
        else:
            entry = self._pulse_timers.pop(id(button), None)
            if entry is None:
                return
            timer, eff, _btn = entry
            timer.stop()
            timer.deleteLater()
            if button.graphicsEffect() is eff:
                button.setGraphicsEffect(None)
            if _BTN_GLOW.get(button.objectName()):
                self._reapply_base_glow(button)

    def _reapply_base_glow(self, button):
        glow = _BTN_GLOW.get(button.objectName())
        if not glow:
            return
        eff = QGraphicsDropShadowEffect(button)
        eff.setBlurRadius(14)
        eff.setOffset(0, 2)
        eff.setColor(QColor(*glow))
        button.setGraphicsEffect(eff)

    def _show_error(self, message, buttons=()):
        self.error_bar.setText('\u26a0  ' + message)
        self.error_bar.setVisible(True)
        for b in (buttons or ()):
            if b is not None:
                self._pulse_button(b, True)
        if self._error_timer is not None:
            self._error_timer.stop()
        self._error_timer = QTimer(self)
        self._error_timer.setSingleShot(True)
        self._error_timer.timeout.connect(self._clear_error)
        self._error_timer.start(10000)

    def _clear_error(self):
        self.error_bar.setVisible(False)
        for _timer, _eff, button in list(self._pulse_timers.values()):
            self._pulse_button(button, False)

    _NAV_REQUIRED = (['lifecycle_manager'], ['navigation_coordinator_node'])

    def _nav_online(self):
        """Full online check for the current env's Nav2/map stack: the env
        params marker plus the shared lifecycle manager and coordinator must all
        be running together before the step counts as complete."""
        missing = []
        for needles in [_NAV_MARKERS[self._env_key]] + list(self._NAV_REQUIRED):
            if not _pids_matching(needles):
                missing.append(needles[0])
        return (not missing), missing

    def _verify_stack(self):
        """Background guardrail check: watch the current env's sim + Nav2/map
        processes, unlock the workflow stages in order (Launch Map -> Load Map +
        Nav fully online -> initial pose -> executor online) and surface
        incomplete/faulty startup with the exact missing component."""
        cfg = ENVS[self._env_key]
        sim = bool(_pids_matching(_SIM_MARKERS[self._env_key]))
        exec_up = bool(_pids_matching(['navigation_coordinator_node']))
        nav_ok, missing = self._nav_online()

        if sim and not self._sim_up:
            self._log('info', f"{cfg['label']} simulation verified running")
        if nav_ok and not self._nav_up:
            self._log('info',
                      f"{cfg['label']} Nav2/map stack fully online "
                      '(params + lifecycle_manager + coordinator)')
            self._nav_launch_requested = False
            self._nav_fault_logged = False
        elif (self._nav_launch_requested and not nav_ok and not self._nav_fault_logged
              and not _pids_matching(['ros2 launch ms04_autonomous_navigation'])):
            # the launch parent died before the stack came fully online
            self._nav_fault_logged = True
            self._show_error(
                f'{cfg["label"]} Nav2/map startup failed before going fully '
                f'online: missing {", ".join(missing)}. Stop the stack and '
                'press "Load Map + Nav" again.')
            self._log('error',
                      f"{cfg['label']} Nav2/map startup incomplete: "
                      f'{", ".join(missing)} not detected')

        self._sim_up = sim
        self._nav_up = nav_ok
        self._exec_up = exec_up
        self._refresh_workflow()

    def _refresh_workflow(self):
        """Recompute which widgets are dimmed vs active from the workflow stage
        (boot -> Launch Map -> Load Map + Nav fully online -> initial pose ->
        executor online) and from the selected mode (single vs waypoints)."""
        sim_up = self._sim_up
        nav_up = self._nav_up
        exec_up = self._exec_up
        wp_mode = self.mode_combo.currentIndex() == 1
        dispatch_ready = self._pose_set and nav_up and exec_up

        self._set_dim(self.btn_launch_sim, False)
        self._set_dim(self.btn_activate_nav, not sim_up)
        self._set_dim(self.btn_set_initial_pose, not nav_up)

        self._set_dim(self.mode_combo, not dispatch_ready)
        self._set_dim(self.poi_combo, not dispatch_ready)
        self._update_pose_inputs()

        wp_area = dispatch_ready and wp_mode
        run_active = self._status_state == 'NAVIGATING' and self._tracked_waypoints
        adder_ok = wp_area and not run_active
        for b in (self.btn_add_wp, self.btn_remove_wp, self.btn_clear_wp):
            b.setEnabled(wp_area)
            self._set_dim(b, not adder_ok)
        self.wp_list.setEnabled(wp_area)

        st = self._status_state
        idle = st in ('IDLE', 'COMPLETED', 'CANCELED', 'ABORTED', 'REJECTED')
        start_ok = dispatch_ready and idle and (not wp_mode or self.wp_list.count() > 0)
        ctrl = {
            self.btn_start: start_ok,
            self.btn_pause: dispatch_ready and st == 'NAVIGATING',
            self.btn_resume: dispatch_ready and st == 'PAUSED',
            self.btn_cancel: dispatch_ready and st in ('NAVIGATING', 'PAUSED'),
            self.btn_replace: self._replace_usable(),
        }
        for btn, on in ctrl.items():
            self._set_dim(btn, not on)

    def _update_pose_inputs(self):
        """Coordinate/spin area: usable only once the dispatch stage is ready,
        and only in Single Goal + Custom, or in Waypoint mode (where the same
        box is how waypoints are composed)."""
        ready = self._pose_set and self._nav_up and self._exec_up
        wp_mode = self.mode_combo.currentIndex() == 1
        enable = ready and (self._custom_selected or wp_mode)
        for spin in (self.spin_x, self.spin_y, self.spin_yaw):
            spin.setEnabled(enable)
            self._set_dim(spin, not enable)

    def _refresh_status_wp(self, current=None):
        """Waypoint line: reflect the tracker count (N) in Waypoint mode and
        show a flat 1/1 for the single-goal mode."""
        if self.mode_combo.currentIndex() == 1:
            total = self.wp_list.count()
            if self._tracked_waypoints and current is not None:
                idx = min(max(self._wp_offset + current + 1, 0), total)
            else:
                idx = 0
            self.status_wp.setText(f'waypoint: {idx}/{total}')
        else:
            self.status_wp.setText('waypoint: 1/1')

    def _require_ready(self):
        """Dispatch gate that follows the 4-stage workflow order. The initial
        pose is only required if it has not been set yet."""
        if not self._sim_up:
            self._show_error('No world is loaded yet. Press "Launch Map" first.',
                             [self.btn_launch_sim])
            return False
        if not self._nav_up:
            self._show_error('The map is loaded but Nav2 is not fully online '
                             'yet. Press "Load Map + Nav" and wait for the '
                             'stack to finish starting.',
                             [self.btn_activate_nav])
            return False
        if not self._pose_set:
            self._show_error('The robot has no initial pose yet. Press '
                             '"Set Initial Pose".',
                             [self.btn_set_initial_pose])
            return False
        return True

    def _require_executor(self):
        if self._exec_up:
            return True
        self._show_error(
            'The navigation executor (coordinator) is not online yet. Wait for '
            '"Load Map + Nav" to finish starting, then try again.',
            [self.btn_activate_nav])
        return False

    def _guard_executor(self, allowed, message, button):
        """Block an in-flight executor command and explain via the red bar
        instead of silently publishing a command the coordinator would ignore."""
        if self._status_state in allowed:
            return True
        self._log('warn', f'{message} (state={self._status_state})')
        self._show_error(f'{message} (current state: {self._status_state}).',
                         [button])
        return False

    def _capture_plan(self, mode, pose, waypoints):
        """Snapshot the plan actually dispatched so Replace can compare the
        current goal/waypoints against what the robot is running."""
        if waypoints is not None:
            coords = []
            for i in range(self.wp_list.count()):
                wx, wy, wyaw, _n, _s = self.wp_list.item(i).data(Qt.UserRole)
                coords.append((wx, wy, wyaw))
            return (NavigationMission.MODE_WAYPOINTS, tuple(coords))
        return (NavigationMission.MODE_GO_TO_POSE,
                (self.spin_x.value(), self.spin_y.value(), self.spin_yaw.value()))

    def _active_plan_differs(self):
        """True when the plan currently on the form differs (in coordinates or
        order) from the plan the paused/running mission was dispatched with."""
        tracked = self._tracked_plan
        if tracked is None:
            return True
        tmode, tcoords = tracked
        if tmode == NavigationMission.MODE_WAYPOINTS:
            if self.mode_combo.currentIndex() != 1:
                return True
            cur = []
            for i in range(self.wp_list.count()):
                wx, wy, wyaw, _n, _s = self.wp_list.item(i).data(Qt.UserRole)
                cur.append((wx, wy, wyaw))
            return tuple(cur) != tcoords
        if self.mode_combo.currentIndex() != 0:
            return True
        x, y, yaw = self.spin_x.value(), self.spin_y.value(), self.spin_yaw.value()
        tx, ty, tyaw = tcoords
        return (math.hypot(x - tx, y - ty) > POI_MATCH_TOLERANCE
                or abs(yaw - tyaw) > 1e-4)

    def _replace_usable(self):
        """Replace is only meaningful while a mission is PAUSED and the plan on
        the form has been changed to differ from the one currently running."""
        if not (self._pose_set and self._nav_up and self._exec_up):
            return False
        if self._status_state != 'PAUSED':
            return False
        if self._tracked_plan is None:
            return False
        return self._active_plan_differs()

    def _waypoint_edit_blocked(self):
        """Waypoint editing is greyed out while a waypoint mission is driving;
        it reopens when the run is paused (so waypoints can be changed and
        then Replace applied)."""
        if self._status_state == 'NAVIGATING' and self._tracked_waypoints:
            self._show_error('A waypoint mission is currently running. Pause it '
                             'before editing waypoints.',
                             [self.btn_pause])
            return True
        return False

    def _stop_other_env_stack(self):
        """Cleanly shut down every stack that belongs to a different
        environment than the one currently selected, so switching envs swaps
        stacks instead of piling them on top of each other."""
        others = [key for key in ENVS if key != self._env_key]
        pids = set()
        stopped = []
        for key in others:
            sim = _pids_matching(_SIM_MARKERS[key])
            nav = _pids_matching(_NAV_MARKERS[key])
            if sim or nav:
                stopped.append(ENVS[key]['label'])
            pids.update(sim)
            pids.update(nav)
        if not pids:
            return
        for needles in _STACK_MARKERS:
            pids.update(_pids_matching(needles))
        pids.update(_pids_matching(_RVIZ_MARKER))
        if pids:
            self._log('warn',
                      f"Stopping {', '.join(stopped) or 'other env'} stack "
                      f'(pids {sorted(pids)}) before switching environment')
            _kill_pids(pids)

    def _on_launch_sim(self):
        cfg = ENVS[self._env_key]
        running = _pids_matching(_SIM_MARKERS[self._env_key])
        if running:
            self._log('warn', f"{cfg['label']} simulation is already running "
                              f'(pids {sorted(running)}) - not launching a duplicate')
            return
        self._stop_other_env_stack()
        self._run_launch(cfg['sim_launch'], args={'gui': 'true'})
        self._log('system', f"Launched simulation: {cfg['label']} ({cfg['sim_launch']})")

    def _on_activate_nav(self):
        cfg = ENVS[self._env_key]
        running = _pids_matching(_NAV_MARKERS[self._env_key])
        if running:
            self._log('warn', f"{cfg['label']} Nav2/map stack is already running "
                              f'(pids {sorted(running)}) - not launching a duplicate')
            return
        self._stop_other_env_stack()
        sim_pids = _pids_matching(_SIM_MARKERS[self._env_key])
        if not sim_pids:
            self._log('warn', f"No {cfg['label']} simulation detected - start "
                              'the world first, otherwise the robot will be missing')
            self._show_error(
                f'No {cfg["label"]} world running. Press "Launch Map" first.',
                [self.btn_launch_sim])
            return
        other_sim = [k for k in ENVS
                     if k != self._env_key and _pids_matching(_SIM_MARKERS[k])]
        if other_sim:
            self._log('warn', f"{cfg['label']} nav selected but "
                              f"{ENVS[other_sim[0]]['label']} sim is running - "
                              'start the matching env or stop the other first')
            self._show_error(
                f'{ENVS[other_sim[0]]["label"]} simulation is running. '
                f'Switch to that environment or stop it before activating '
                f'{cfg["label"]} Nav2.')
            return
        self._nav_launch_requested = True
        self._nav_fault_logged = False
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
        x, y, yaw = cfg['initial_pose']
        if not _pids_matching(_SIM_MARKERS[self._env_key]):
            self._show_error(
                f'{cfg["label"]} is not running yet. Press "Launch Map" '
                'before localising the robot.', [self.btn_launch_sim])
            return
        if not _pids_matching(_NAV_MARKERS[self._env_key]):
            self._show_error(
                f'Nav2 is not active for {cfg["label"]}. Press "Load Map + '
                'Nav" before setting the initial pose.', [self.btn_activate_nav])
            return
        if self._pose_set:
            QMessageBox.information(
                self, 'Already localised',
                f'The robot is already localised at the map initial pose '
                f'({x:.2f}, {y:.2f}, {yaw:.2f} rad).\n\n'
                'Pose remains valid until the simulation is restarted.')
            return
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
        self._pose_set = True
        self._refresh_workflow()
        self._log('pose', f'Set initial pose (map) -> ({x:.2f}, {y:.2f}, {yaw:.2f}); '
                          'robot localised, workflow unlocked')

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
        if not self._require_ready():
            return None
        x = self.spin_x.value()
        y = self.spin_y.value()
        yaw = self.spin_yaw.value()
        waypoint_mode = self.mode_combo.currentIndex() == 1
        if not waypoint_mode:
            return (NavigationMission.MODE_GO_TO_POSE, self._build_pose(x, y, yaw), None)
        if self.wp_list.count() == 0:
            self._log('warn', 'Waypoint mode selected but no waypoints in list')
            self._show_error('Waypoint mode selected but no waypoints in the list. '
                             'Add waypoints below first.', [self.btn_add_wp])
            return None
        poses = []
        for i in range(self.wp_list.count()):
            wx, wy, wyaw, _name, _state = self.wp_list.item(i).data(Qt.UserRole)
            poses.append(self._build_pose(wx, wy, wyaw))
        return (NavigationMission.MODE_WAYPOINTS, None, poses)

    def _on_start(self):
        if not (self._require_ready() and self._require_executor()):
            return
        if self._status_state not in ('IDLE', 'COMPLETED', 'CANCELED',
                                      'ABORTED', 'REJECTED'):
            self._show_error(
                f'A mission is currently {self._status_state}. Pause it, '
                'change the goal/waypoints to differ, then press '
                '"Replace Goal" to swap the active plan.',
                [self.btn_replace])
            return
        plan = self._current_plan()
        if plan is None:
            return
        mode, pose, waypoints = plan
        mission_id = self._next_mission_id()
        self._tracked_plan = self._capture_plan(mode, pose, waypoints)
        self._publish_mission(NavigationMission.COMMAND_START, mode, mission_id,
                              pose=pose, waypoints=waypoints)
        self._start_tracking(mission_id, waypoints, pose=pose)
        desc = ('POI/goal' if waypoints is None
                else f'{len(waypoints)} waypoints')
        self._log('command', f'START mission {mission_id} ({desc})')

    def _on_pause(self):
        if not (self._require_ready() and self._require_executor()):
            return
        if not self._guard_executor(
                {'NAVIGATING'},
                'Pause needs an active (NAVIGATING) mission', self.btn_pause):
            return
        self._publish_mission(NavigationMission.COMMAND_PAUSE,
                              NavigationMission.MODE_GO_TO_POSE, self._next_mission_id())
        self._log('command', 'PAUSE sent - robot stops, mission kept for resume')

    def _on_resume(self):
        if not (self._require_ready() and self._require_executor()):
            return
        if not self._guard_executor(
                {'PAUSED'},
                'Resume needs a PAUSED mission', self.btn_resume):
            return
        self._publish_mission(NavigationMission.COMMAND_RESUME,
                              NavigationMission.MODE_GO_TO_POSE, self._next_mission_id())
        self._log('command', 'RESUME sent - remaining waypoints continue')

    def _on_cancel(self):
        if not (self._require_ready() and self._require_executor()):
            return
        if not self._guard_executor(
                {'NAVIGATING', 'PAUSED'},
                'Cancel needs an active mission to stop', self.btn_cancel):
            return
        self._publish_mission(NavigationMission.COMMAND_CANCEL,
                              NavigationMission.MODE_GO_TO_POSE, self._next_mission_id())
        self._log('command', 'CANCEL sent - mission aborted')

    def _on_replace(self):
        if not (self._require_ready() and self._require_executor()):
            return
        if self._status_state != 'PAUSED':
            self._show_error(
                'Replace needs the active mission to be PAUSED first. Pause '
                'the run, then edit the waypoints/goal to differ from the '
                'current mission.', [self.btn_pause])
            return
        if not self._active_plan_differs():
            self._show_error(
                'The goal/waypoints are unchanged from the running mission. '
                'Change their coordinates or order to differ before pressing '
                '"Replace Goal".', [self.btn_add_wp])
            return
        self._dispatch_replace()

    def _dispatch_replace(self, plan=None):
        if plan is None:
            plan = self._current_plan()
            if plan is None:
                return
        mode, pose, waypoints = plan
        mission_id = self._next_mission_id()
        self._tracked_plan = self._capture_plan(mode, pose, waypoints)
        self._publish_mission(NavigationMission.COMMAND_REPLACE, mode, mission_id,
                              pose=pose, waypoints=waypoints)
        self._start_tracking(mission_id, waypoints, pose=pose)
        desc = ('new goal' if waypoints is None else f'{len(waypoints)} waypoints')
        self._log('command', f'REPLACE mission {mission_id} ({desc}): paused '
                             'mission cancelled at current position, new plan dispatched')

    def _start_tracking(self, mission_id, waypoints, pose=None):
        self._tracked_mission = mission_id
        self._tracked_waypoints = waypoints is not None
        self._wp_offset = 0
        self._plan_pose = pose
        self._wipe_wp_progress()
        if pose is not None:
            p = pose.pose
            self.status_target.setText(
                f'target: ({p.position.x:.2f}, {p.position.y:.2f})')
        elif waypoints is not None and waypoints:
            p = waypoints[0].pose
            self.status_target.setText(
                f'target: wp1 ({p.position.x:.2f}, {p.position.y:.2f})')
        else:
            self.status_target.setText('target: -')

    def _refresh_lamps(self):
        """Recompute the loaded/progress/paused/reached lamps from the current
        tracked waypoint mission state. In single-goal dispatch no waypoint list
        is tracked, so every lamp stays off (nothing meaningful to report)."""
        counts = {'loaded': 0, 'progress': 0, 'paused': 0, 'reached': 0}
        if self._tracked_waypoints:
            st = self._status_state
            if st == 'NAVIGATING':
                counts['progress'] = 1
            elif st == 'PAUSED':
                counts['paused'] = 1
            elif st == 'COMPLETED':
                counts['reached'] = 1
            elif st not in ('CANCELED', 'ABORTED', 'REJECTED'):
                counts['loaded'] = 1
        self._update_lamps(counts)

    def _on_add_waypoint(self):
        if self._waypoint_edit_blocked():
            return
        x = self.spin_x.value()
        y = self.spin_y.value()
        yaw = self.spin_yaw.value()
        if self.wp_list.count() > 0:
            lx, ly, lyaw, _lname, _lstate = self.wp_list.item(
                self.wp_list.count() - 1).data(Qt.UserRole)
            if math.hypot(x - lx, y - ly) <= POI_MATCH_TOLERANCE and yaw == lyaw:
                self._log('warn',
                          f'Waypoint #{self.wp_list.count() + 1} rejected: it '
                          f'duplicates #{self.wp_list.count()} '
                          f'({lx:.2f}, {ly:.2f}, {lyaw:.2f}) - consecutive '
                          'duplicate waypoints are not allowed')
                self._show_error(
                    f'Waypoint rejected: duplicates the previous waypoint '
                    f'#{self.wp_list.count()} ({lx:.2f}, {ly:.2f}, {lyaw:.2f}). '
                    'Consecutive duplicate waypoints are not allowed.')
                return
        name = self._match_poi(x, y)
        self.wp_list.addItem(self._wp_label(self.wp_list.count(), x, y, yaw, name))
        item = self.wp_list.item(self.wp_list.count() - 1)
        item.setData(Qt.UserRole, (x, y, yaw, name, 'pending'))
        self._style_wp_item(item, 'pending')
        self._refresh_lamps()
        self._refresh_status_wp()
        self._refresh_workflow()

    def _match_poi(self, x, y):
        cfg = ENVS[self._env_key]
        for name, (px, py, _pyaw) in cfg['pois'].items():
            if math.hypot(x - px, y - py) <= POI_MATCH_TOLERANCE:
                return name
        return None

    def _wp_label(self, index, x, y, yaw, name):
        num = f'#{index + 1} '
        coord = f'({x:.2f}, {y:.2f}, {yaw:.2f})'
        return num + (f'{name} ' if name else '') + coord

    def _style_wp_item(self, item, state):
        if state == 'done':
            item.setForeground(QColor(165, 214, 167))
            item.setBackground(QColor(20, 40, 26))
        elif state == 'active':
            item.setForeground(QColor(255, 213, 79))
            item.setBackground(QColor(64, 56, 18))
        else:
            item.setForeground(QColor(144, 164, 174))
            item.setBackground(QColor(20, 22, 28))
        item.setData(Qt.UserRole, (*item.data(Qt.UserRole)[:4], state))

    def _renumber_wp_list(self):
        for i in range(self.wp_list.count()):
            item = self.wp_list.item(i)
            x, y, yaw, name, _state = item.data(Qt.UserRole)
            item.setText(self._wp_label(i, x, y, yaw, name))

    def _wipe_wp_progress(self):
        for i in range(self.wp_list.count()):
            item = self.wp_list.item(i)
            x, y, yaw, name, _state = item.data(Qt.UserRole)
            item.setData(Qt.UserRole, (x, y, yaw, name, 'pending'))
            self._style_wp_item(item, 'pending')
        self._refresh_lamps()

    def _update_wp_progress(self, current):
        """Style waypoint list rows as done / active / pending based on the
        waypoint index the coordinator is currently following (0-based), plus
        the offset accumulated across pause/resume cycles."""
        index = self._wp_offset + current
        for i in range(self.wp_list.count()):
            item = self.wp_list.item(i)
            if i < index:
                self._style_wp_item(item, 'done')
            elif i == index:
                self._style_wp_item(item, 'active')
            else:
                self._style_wp_item(item, 'pending')
        self._refresh_lamps()

    def _on_remove_waypoint(self):
        if self._waypoint_edit_blocked():
            return
        row = self.wp_list.currentRow()
        if row >= 0:
            self.wp_list.takeItem(row)
            self._renumber_wp_list()
            self._refresh_lamps()
            self._refresh_status_wp()
            self._refresh_workflow()

    def _on_clear_waypoints(self):
        if self._waypoint_edit_blocked():
            return
        self.wp_list.clear()
        self._refresh_lamps()
        self._refresh_status_wp()
        self._refresh_workflow()

    # ------------------------------------------------------------ Qt slots
    def _on_event(self, msg):
        name = EVENT_NAMES.get(msg.event_type, f'EVENT_{msg.event_type}')
        color = EVENT_COLORS.get(msg.event_type, QColor(255, 255, 255))
        if msg.event_type in (NavigationEvent.EVENT_GOAL_COMPLETED,
                              NavigationEvent.EVENT_GOAL_CANCELED,
                              NavigationEvent.EVENT_GOAL_ABORTED,
                              NavigationEvent.EVENT_GOAL_REJECTED):
            self._wipe_wp_progress()
            self._tracked_mission = None
            self._tracked_plan = None
            self._refresh_workflow()
        if msg.mission_id != self._fb_ctx['mission']:
            self._fb_ctx = {'mission': msg.mission_id, 'wp': -1, 'rec': -1}
        if msg.event_type == NavigationEvent.EVENT_GOAL_PAUSED:
            if (self._tracked_mission and msg.mission_id == self._tracked_mission
                    and self._tracked_waypoints):
                self._wp_offset += msg.current_waypoint
        if msg.event_type == NavigationEvent.EVENT_FEEDBACK:
            if self._tracked_mission and msg.mission_id == self._tracked_mission:
                self._status_from_feedback(msg)
                if self._tracked_waypoints:
                    self._update_wp_progress(msg.current_waypoint)
            line = self._render_feedback(msg)
            if line:
                self._log(name, line, color)
            return
        self._log(name, self._render_event(msg), color)

    def _render_event(self, msg):
        text = (msg.message or '').strip() or _EVENT_FALLBACK.get(
            msg.event_type, '')
        if not text:
            return 'telemetry update'
        if msg.mission_id and msg.mission_id not in text:
            return f'{msg.mission_id} — {text}'
        return text

    def _render_feedback(self, msg):
        """Turn the 5 Hz Nav2 telemetry flood into compact milestone lines.
        Live pose / distance / ETA are shown in the Mission Status panel by
        _status_from_feedback instead of filling the log with odometry noise."""
        ctx = self._fb_ctx
        wp_changed = msg.current_waypoint != ctx['wp']
        rec_increased = msg.number_of_recoveries > ctx['rec']
        ctx['wp'] = msg.current_waypoint
        ctx['rec'] = max(ctx['rec'], msg.number_of_recoveries)
        parts = []
        if wp_changed and msg.current_waypoint > 0:
            parts.append(f'waypoint {msg.current_waypoint}/{msg.total_waypoints}')
        if rec_increased and msg.number_of_recoveries > 0:
            parts.append(f'recovery #{msg.number_of_recoveries} triggered (Nav2 replanning)')
        return ' · '.join(parts) if parts else None

    def _status_from_feedback(self, msg):
        """Mirror the live 5 Hz feedback telemetry into the Mission Status
        panel (pose, remaining distance, ETA, current waypoint, target)."""
        p = msg.current_pose.pose
        self.status_pose.setText(f'x {p.position.x:.2f} y {p.position.y:.2f} '
                                 f'yaw {p.orientation.w:.2f}')
        self.status_dist.setText(f'remaining: {msg.distance_remaining:.2f} m')
        self.status_eta.setText(f'ETA: {msg.estimated_time_remaining.sec}s')
        self._refresh_status_wp(msg.current_waypoint)
        if not self._tracked_waypoints:
            if self._plan_pose is not None:
                gp = self._plan_pose.pose
                self.status_target.setText(
                    f'target: ({gp.position.x:.2f}, {gp.position.y:.2f})')
            else:
                self.status_target.setText(
                    f'target: ({p.position.x:.2f}, {p.position.y:.2f})')
        else:
            idx = self._wp_offset + msg.current_waypoint
            if 0 <= idx < self.wp_list.count():
                item = self.wp_list.item(idx)
                x, y, yaw, name, _ = item.data(Qt.UserRole)
                self.status_target.setText(
                    f'target: wp{idx + 1} ({x:.2f}, {y:.2f})'
                    + (f' {name}' if name else ''))

    def _on_status(self, msg):
        state = STATE_NAMES.get(msg.state, f'STATE_{msg.state}')
        self._status_state = state
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
        self.status_state.setStyleSheet(f'font-weight:bold;font-size:18px;color:{color};')
        self._apply_banner(state)
        self.status_mission.setText(f'mission: {msg.mission_id}')
        self._refresh_status_wp(msg.current_waypoint)
        if state in ('COMPLETED', 'CANCELED', 'ABORTED', 'REJECTED'):
            self.status_dist.setText('remaining: 0.00 m')
            self.status_eta.setText('ETA: 0s')

        self._refresh_lamps()
        self._refresh_workflow()

        if (self._tracked_mission and msg.mission_id == self._tracked_mission
                and state == 'NAVIGATING'):
            self._update_wp_progress(msg.current_waypoint)

        message = (msg.message or '').strip()
        if not message:
            pass
        elif (state == 'COMPLETED'
              and any(k in message.lower() for k in ('complete', 'goal reached'))
              and not self._last_logged_line.endswith(message)):
            self._log('info', message, QColor(0, 220, 90))
        elif (any(h in message.lower() for h in _DIAG_HINTS)
              and not self._last_logged_line.endswith(message)):
            self._log('info', message, QColor(255, 205, 210))

    def _apply_banner(self, state):
        glow = STATE_GLOW.get(state, STATE_GLOW['IDLE'])
        self._banner_glow.setColor(QColor(*(_hex_rgb(glow[0]) + (170,))))
        self.backdrop.set_tint(glow[0])
        self.state_banner.setStyleSheet(
            'QLabel { background: qlineargradient(x1:0, y1:0, x2:1, y2:0, '
            f'stop:0 {glow[0]}, stop:1 {glow[1]}); '
            'color: #f7f2ff; border: 1px solid rgba(255,255,255,0.22); '
            'border-radius: 10px; padding: 10px 18px; '
            'font-weight: 600; font-size: 20px; letter-spacing: 4px; }')
        self.state_banner.setText(f'MISSION STATE: {state}')

    def resizeEvent(self, event):
        super().resizeEvent(event)
        central = self.centralWidget()
        if central is not None:
            self.backdrop.setGeometry(central.rect())

    _STYLE = {
        'system': ('#9e9e9e', 'SYS'),
        'command': ('#cddc39', 'CMD'),
        'pose': ('#64b5f6', 'POSE'),
        'info': ('#eceff1', 'INFO'),
        'warn': ('#ffb74d', 'WARN'),
        'error': ('#e57373', 'ERROR'),
    }

    def _log(self, kind, text, color=None):
        self._last_logged_line = text
        if color is None:
            color, tag = self._STYLE.get(kind, ('#ffffff', kind.upper()))
        else:
            tag = kind.upper()
        html = (f'<span style="color:#616161;">{time.strftime("%H:%M:%S")}</span> '
                f'<span style="color:{color};">[{tag}]</span> '
                f'<span style="color:{color};">{text}</span>')
        vbar = self.log_view.verticalScrollBar()
        at_bottom = vbar.value() >= vbar.maximum() - 8
        self.log_view.appendHtml(html)
        if at_bottom:
            vbar.setValue(vbar.maximum())

    # ------------------------------------------------------------- cleanup
    def _stop_named_stack(self, needles, label):
        pids = _pids_matching(needles)
        if not pids:
            return
        self._log('warn', f'Stopping {label} (pids {sorted(pids)})')
        _kill_pids(pids)

    def _cleanup_all(self):
        """Kill every ms04 stack process we know about: sims, bridges, relays,
        nav nodes, lifecycle managers, coordinators, rviz and launch parents."""
        pids = set()
        for key in ENVS:
            pids.update(_pids_matching(_SIM_MARKERS[key]))
            pids.update(_pids_matching(_NAV_MARKERS[key]))
        pids.update(_pids_matching(_RVIZ_MARKER))
        for needles in _STACK_MARKERS:
            pids.update(_pids_matching(needles))
        if pids:
            self._log('warn', f'Stopping stack (pids {sorted(pids)})')
            _kill_pids(pids)

    def closeEvent(self, event):
        for proc in self._spawned_procs:
            if proc.poll() is None:
                proc.terminate()
        self._cleanup_all()
        self.shutdown_ros()
        super().closeEvent(event)


def main(argv=None):
    app = QApplication(argv or sys.argv)
    app.setStyle('Fusion')
    app.setStyleSheet(DARK_QSS)
    gui = MissionControlGUI()
    gui.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())