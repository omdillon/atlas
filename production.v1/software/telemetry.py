"""
BLE central + PyQt6 GUI for the hand-teleoperation system: a motor
telemetry dashboard, a Gesture Controls / Individual Fingers panel with a
thin-BLE-trigger command model, plus:

  - A Safe Override box (OVERRIDE to OPEN / OVERRIDE to PACK): top-priority
    commands the firmware always accepts, abandoning whatever the hand is
    doing and driving straight to a known-safe posture.
  - A third column: the Gesture Graph panel. Draws every gesture node (WAVE
    excluded, it's a scripted animation, not a graph node) and highlights
    the node the hand is currently at (green), the node a just-issued
    command is driving toward while in transit (amber), and the edge
    between them for that duration.

    Every node pair is drawn as a real edge, not a display simplification:
    the firmware's transition engine (see transition_engine.h) reaches any
    node from any other in a single direct hop, with no forced via-OPEN
    routing, so the hand can move directly between any two gestures.

    current/target/in-progress are read straight from telemetry every frame
    (current_node_id, target_node_id, transition_active_flag); the GUI does
    not track "what did I last click" client-side, so the graph panel is
    correct from telemetry alone even on a fresh reconnect mid-transition.

Usage:
    pip install -r requirements.txt
    python telemetry.py

The ESP32 must already be running the firmware and advertising (check its
Serial monitor for "[BLE] Advertising as: ProHand-EXP13").
"""
import asyncio
import math
import struct
import sys
import time

import cv2
from bleak import BleakClient, BleakScanner
from PyQt6.QtCore import Qt, QTimer, QPointF, QRectF
from PyQt6.QtGui import QColor, QPainter, QPen, QBrush, QImage, QPixmap
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QTableWidget, QTableWidgetItem, QHeaderView, QAbstractItemView, QGroupBox, QSizePolicy,
    QDialog, QTextBrowser, QFrame, QDoubleSpinBox,
)
import qasync
from qasync import asyncSlot

from cv_dispatch_gate import DispatchGate
from gesture_classifier import TARGET_GESTURE_MASKS, classify_mask
from hand_tracker import HandTracker, finger_states, load_thresholds, states_to_mask

DEVICE_NAME = "ProHand-EXP13"
CHAR_TELEMETRY_UUID = "12df09d3-200c-4abe-b423-cbfe105cb2b6"
CHAR_MOTOR_CMD_UUID = "c6975e18-d186-4acb-bab3-6138e560b4de"
SCAN_TIMEOUT_S = 10.0

MOTOR_COUNT = 5
STATE_NAMES = {0: "IDLE", 1: "MOVING", 2: "HOLDING", 3: "STALLED"}

MOTOR_ROW_LABELS = [f"M{i + 1}" for i in range(MOTOR_COUNT)]

# Frame layout (little-endian, 55 bytes): uint16 seq, uint8
# transition_active_flag, uint8 current_node_id, uint8 target_node_id, then
# 5x (uint8 state, float32 voltage, float32 current, uint8 fault_byte).
# Must stay in sync with the firmware's build_telemetry_frame() and
# docs/protocol.md; there is no shared schema file between the two languages.
FRAME_FORMAT = "<HBBB" + "BffB" * MOTOR_COUNT
FRAME_SIZE = struct.calcsize(FRAME_FORMAT)  # 55
MIN_MTU_PAYLOAD = FRAME_SIZE

# current_node_id/target_node_id encode (graspName + 1): PACK(-1)->0 ..
# WAVE(6)->7 (WAVE never actually appears here, it's not a graph node).
# 0xFF on current_node_id only means "off-node/mid-transition". target_node_id
# has no sentinel: it's always the last-requested node (defaults to OPEN),
# so it equals current_node_id whenever the hand is settled.
NODE_ID_TO_NAME = {
    0: "PACK", 1: "OPEN", 2: "POWER", 3: "TRIPOD",
    4: "PINCH", 5: "POINTER", 6: "ROCK", 7: "WAVE",
    # CV-teleoperation expansion set, appended after WAVE so no existing id shifts.
    8: "BIRD", 9: "TWO", 10: "JAMBO", 11: "THREE", 12: "THUMB",
}
OFF_NODE_ID = 0xFF

# DRV8214 FAULT register bit positions; see docs/protocol.md for the full table.
FAULT_BITS = [
    ("STALL", 0x20),
    ("OCP", 0x10),
    ("OVP", 0x08),
    ("TSD", 0x04),
    ("NPOR", 0x02),
    ("CNT_DONE", 0x01),
]


def _decode_fault_names(fault_byte: int) -> list[str]:
    return [name for name, bit in FAULT_BITS if fault_byte & bit]


GRASP_NAMES = ["OPEN", "POWER", "TRIPOD", "PINCH", "POINTER", "ROCK", "WAVE", "PACK",
               "BIRD", "TWO", "JAMBO", "THREE", "THUMB"]
FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]

# Gesture Controls grid layout: OPEN/PACK/POWER each get a full-width row of
# their own (OPEN is the graph's "home" reference; OPEN and PACK are also
# the two Safe Override targets), then the remaining rows pair up the rest.
# WAVE is excluded entirely; it lives in its own Dynamic box (see
# WAVE_GRASP_NAME), not the grid. THUMB is the odd one out so it gets its
# own full-width row rather than an uneven half-empty pairing.
GRASP_GRID_ROWS = [
    ["OPEN"], ["PACK"], ["POWER"],
    ["TRIPOD", "PINCH"], ["POINTER", "ROCK"],
    ["BIRD", "TWO"], ["JAMBO", "THREE"], ["THUMB"],
]
WAVE_GRASP_NAME = "WAVE"

# WAVE bypasses TransitionEngine entirely and drives motors 2-5 (index..pinky)
# directly; it never touches the thumb (motor 1), see transition_engine.h.
# If some other finger (most importantly the thumb) is closed when WAVE
# starts, it stays closed for the whole animation and can collide with the
# waving fingers. The Wave button therefore sends an explicit "OPEN" first
# and waits for it to actually finish before sending WAVE, see
# _send_wave_command(). Opening is never stall-terminated early (always
# runs the full 2000ms ceiling, gesture_graph.h's FINGER_FULL_OPEN_MS), so
# 2.5s covers the worst case plus margin for BLE write/round-trip latency.
WAVE_PRE_OPEN_SETTLE_S = 2.5

# Computer-vision hand-tracking control. CV is an alternate command source,
# not a new control mode: a classified gesture is sent through the exact
# same _send_command() every button already uses (see _on_cv_tick()), never
# a parallel write path and never a continuous/analog channel into the
# firmware; see hand_tracker.py/gesture_classifier.py/cv_dispatch_gate.py
# for the classification pipeline this feeds from.
CV_TICK_INTERVAL_MS = 33  # matches HandTracker's own ~30Hz camera cadence
CV_STABILITY_DEFAULT_S = 1.0
CV_STABILITY_MIN_S = 0.3
CV_STABILITY_MAX_S = 3.0
CV_PREVIEW_LABEL_WIDTH = 320  # display width in px; height follows the camera's own aspect ratio

# Graph-panel layout order; WAVE excluded (scripted animation, not a graph
# node; see gesture_graph.h). OPEN placed first/top as a familiar "home"
# reference point only; it carries no special routing role. Every pair
# below is a real direct edge (see GestureGraphWidget).
GRAPH_NODE_ORDER = ["OPEN", "POWER", "TRIPOD", "PINCH", "POINTER", "ROCK", "PACK",
                     "BIRD", "TWO", "JAMBO", "THREE", "THUMB"]

BOX_MIN_WIDTH = 460
# 12 nodes sit on the circle (see GRAPH_NODE_ORDER), each 30 degrees apart.
# 480 keeps the circle's radius large enough that neighboring labels
# (GestureGraphWidget's label_rect) stay clear of each other (see
# NODE_RADIUS's comment for the other half of this fix). Shared by
# graph_box and cv_preview_box so they stay flush in the middle column.
GRAPH_BOX_MIN_WIDTH = 480
MAX_RECENT_COMMANDS = 5

# Runtime display scaling. All the fixed pixel sizes above (and the handful
# scattered through TelemetryWindow/GestureGraphWidget/HelpDialog below) were
# tuned against a single un-scaled desktop. A 1920x1080 monitor commonly runs
# at 125-150% Windows scaling, which leaves the app less logical space than
# that; without this, the window's minimum size can exceed the actual
# available desktop and panels get clipped/pushed off-screen instead of
# shrinking to fit. UI_SCALE is computed once in main() (via
# apply_ui_scale(), called before TelemetryWindow() is constructed) from the
# real screen's available geometry.
UI_SCALE = 1.0
# The width/height these fixed pixel sizes were designed against. UI_SCALE
# only shrinks below 1.0 when the actual screen has less room than this,
# never grows beyond it.
REFERENCE_WIDTH = 1660
REFERENCE_HEIGHT = 700
MIN_UI_SCALE = 0.6
MAX_UI_SCALE = 1.0


def apply_ui_scale(app: QApplication) -> None:
    """Shrinks BOX_MIN_WIDTH/GRAPH_BOX_MIN_WIDTH/CV_PREVIEW_LABEL_WIDTH (and
    sets UI_SCALE for the constructor-time sizes read from it below) to fit
    the screen the app is about to open on. Must run after the QApplication
    exists (screens aren't queryable before that) and before TelemetryWindow()
    is constructed (everything below reads these at __init__ time)."""
    global UI_SCALE, BOX_MIN_WIDTH, GRAPH_BOX_MIN_WIDTH, CV_PREVIEW_LABEL_WIDTH
    screen = app.primaryScreen()
    if screen is None:
        return
    avail = screen.availableGeometry()
    scale = min(avail.width() / REFERENCE_WIDTH, avail.height() / REFERENCE_HEIGHT)
    UI_SCALE = max(MIN_UI_SCALE, min(MAX_UI_SCALE, scale))
    BOX_MIN_WIDTH = round(BOX_MIN_WIDTH * UI_SCALE)
    GRAPH_BOX_MIN_WIDTH = round(GRAPH_BOX_MIN_WIDTH * UI_SCALE)
    CV_PREVIEW_LABEL_WIDTH = round(CV_PREVIEW_LABEL_WIDTH * UI_SCALE)


TABLE_QSS = (
    "QTableWidget { background-color: #2a2a2e; gridline-color: #46464c; border: none; } "
    "QTableWidget::item { padding: 0px 4px; } "
    "QHeaderView::section { background-color: #34343a; padding: 1px 4px; border: 1px solid #46464c; }"
)

DARK_STYLESHEET = """
QWidget { background-color: #1e1e22; color: #e8e8ec; }
QGroupBox { background-color: #2a2a2e; border: 1px solid #46464c;
            border-radius: 6px; margin-top: 6px; font-weight: bold; }
QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
QPushButton { background-color: #2c2c31; border: 1px solid #46464c;
              border-radius: 4px; padding: 3px 8px; }
QPushButton:disabled { color: #77777c; }
QPushButton:hover:!disabled { background-color: #35353b; }
""" + TABLE_QSS
# QDoubleSpinBox deliberately left unstyled: styling its frame at all (even
# just background/border) makes Qt switch it to the "styled" subcontrol
# rendering path, which broke the up/down increment buttons without also
# hand-defining ::up-button/::down-button/::up-arrow/::down-arrow.


def _tinted_button_qss(bg: str, border: str) -> str:
    """A self-contained per-button style override (background + colored
    border), fully specified so it composes correctly with
    DARK_STYLESHEET's cascade. Passing "" to setStyleSheet() clears the
    override and the button reverts to DARK_STYLESHEET's plain rule."""
    return (
        f"QPushButton {{ background-color: {bg}; border: 2px solid {border}; "
        f"color: #e8e8ec; }} "
        f"QPushButton:hover:!disabled {{ background-color: {bg}; }} "
        f"QPushButton:disabled {{ background-color: #2c2c31; border: 1px solid #46464c; color: #77777c; }}"
    )


# Grasp button live-state highlight; same green/amber hexes as
# GestureGraphWidget's _GOOD/_WARN, so a settled/in-transit grasp reads
# identically whether you're looking at the graph or the button you clicked.
GRASP_CURRENT_QSS = _tinted_button_qss("#16321a", "#0ca30c")
GRASP_TARGET_QSS = _tinted_button_qss("#3d2f0c", "#fab219")

# Individual-finger button live-state tint, keyed by the motor `state` byte
# (see STATE_NAMES). Both Close/Open buttons for a finger get the same tint
# together, since the telemetry frame carries motor state, not commanded
# direction, so there's no reliable way to light up only one of the pair.
# IDLE (0) isn't listed; .get() falls back to "" (clears to default).
FINGER_STATE_QSS = {
    1: _tinted_button_qss("#3d2f0c", "#fab219"),  # MOVING
    2: _tinted_button_qss("#16321a", "#0ca30c"),  # HOLDING
    3: _tinted_button_qss("#3a1414", "#dc3c37"),  # STALLED (matches fault-red elsewhere)
}

HELP_HTML = f"""
<h2>Using the GUI</h2>
<p>
Click <b>Connect</b> to scan for and connect to a device advertising as
<code>{DEVICE_NAME}</code> (the ESP32 must already be flashed with the
firmware and running). Once connected, Motor Telemetry and the Gesture
Graph update automatically and the grasp buttons, Clear Faults, and Safe
Override become enabled.
</p>
<ul>
<li><b>Gesture Controls</b>: the {len(GRAPH_NODE_ORDER)} real grasp buttons
({', '.join(GRAPH_NODE_ORDER)}) each send a single BLE command. The firmware's
transition engine computes which fingers actually need to move from wherever
the hand currently is and drives only those; <b>gestures never return to
OPEN in between</b> (see the Gesture Graph section below). Whichever button
matches the hand's current/target node lights up green/amber, mirroring the
Gesture Graph panel. The <b>Individual Fingers</b> Close/Open buttons drive
one finger at a time, routed through the same engine, and tint
amber/green/red while that finger is moving/holding/stalled.</li>
<li><b>Dynamic</b>: <code>WAVE</code> is a scripted animation, not a real
grasp/graph state (it never appears as a Gesture Graph node), so it gets its
own small box instead of sitting alongside the real grasps. Clicking it
first sends the hand to <b>OPEN</b> and waits for that to settle, then plays
the wave animation, avoiding a collision with a still-closed finger (most
importantly the thumb, which WAVE never drives itself).</li>
<li><b>Safe Override</b>: <code>OVERRIDE &rarr; OPEN</code> /
<code>OVERRIDE &rarr; PACK</code> are top-priority commands
(<code>override_open</code> / <code>override_pack</code>) that abandon
whatever the hand is doing and drive straight to that safe posture.</li>
<li><b>Hand Tracking (CV)</b>: an alternate source for the exact same
grasp commands above, derived from a webcam. Click <b>ARM CV CONTROL</b>
(unpressed by default, a safety backstop for a feature that can move the
hand on its own) to start: your tracked hand pose is classified once per
camera frame, and only dispatched as a real command after it holds steady
for the <b>Stability window</b> (default 1.0s) and differs from whatever
was last actually sent. The live camera feed (with a curl%/mask HUD
overlay) appears in the <b>Camera Preview</b> panel under the Gesture Graph
once armed. Losing the camera's view of your hand simply stops new commands
from being sent; the physical hand holds its last commanded gesture rather
than doing anything on its own. A manual button click always takes effect
immediately and does not disarm CV; if the live camera still disagrees, CV
can reassert itself once its own next stability window elapses. Run
<code>cv_calibrate.py</code> (a one-time bench step) to calibrate
per-finger thresholds against your own hand/camera/lighting setup; see that
script and <code>cv_dry_run.py</code> for a BLE-free way to test
classification before ever arming this panel.</li>
<li><b>Clear Faults</b> zeroes sticky fault bits.</li>
<li><b>Motor Telemetry / Fault Detection</b>: live per-motor status read
from the device.</li>
<li><b>Gesture Graph</b>: see below.</li>
<li><b>Recent (activity strip, top of window)</b>: the last
{MAX_RECENT_COMMANDS} commands actually sent over BLE, newest first. A
narration aid for demos; it is a client-side log of outgoing commands only,
unrelated to the telemetry-driven state shown elsewhere in the GUI.</li>
</ul>

<h2>Gesture Graph</h2>
<p>
The right-hand panel draws all {len(GRAPH_NODE_ORDER)} gesture nodes
({', '.join(GRAPH_NODE_ORDER)}) with a line between every pair.
<b>Every pair here is a real, always-available direct edge</b>: the
firmware never reads a measured position back to decide if a move is safe,
it drives each finger by commanded time from a depth it tracked itself, so
a direct A&rarr;B move is correct by construction for any A, B.
</p>
<p>
<span style="color:#0ca30c;">&#9679;</span> <b>Current</b>: the node the
firmware last confirmed the hand settled at (<code>current_node_id</code>).<br>
<span style="color:#fab219;">&#9679;</span> <b>Target / transitioning</b>:
while <code>transition_active_flag</code> is set, the node a just-issued
command is driving toward (<code>target_node_id</code>), with the edge from
Current to Target highlighted for the duration.
</p>
<p>
All three fields come from telemetry every frame; the GUI does not track
"what did I last click." This means the graph panel is correct even on a
fresh reconnect mid-transition, which a click-tracking approach could not
guarantee.
</p>

<h2>Fault Codes</h2>
<p>
Each motor reports an 8-bit <code>fault_byte</code> (mirroring the real
DRV8214 motor driver's FAULT register), decoded bit-by-bit in the Fault
Detection table.
</p>
<table border="1" cellspacing="0" cellpadding="4">
<tr><th>Bit</th><th>Name</th><th>Meaning</th></tr>
<tr><td>0x20</td><td>STALL</td><td>Motor stall detected by the firmware's software stall watchdog (ripple-stagnation while driven).</td></tr>
<tr><td>0x10</td><td>OCP</td><td>Overcurrent protection tripped: output current exceeded the driver's safe limit.</td></tr>
<tr><td>0x08</td><td>OVP</td><td>Overvoltage protection tripped: supply voltage exceeded the driver's safe limit.</td></tr>
<tr><td>0x04</td><td>TSD</td><td>Thermal shutdown: the driver IC overheated and disabled its outputs.</td></tr>
<tr><td>0x02</td><td>NPOR</td><td>Power-on-reset occurred (e.g. after a brownout or power-up).</td></tr>
<tr><td>0x01</td><td>CNT_DONE</td><td>Ripple-counting threshold exceeded. A status flag rather than a fault, reusing the same register.</td></tr>
</table>
<p>
Bits latch until an explicit <b>Clear Faults</b> command clears them; the
overall "All Clear" / "FAULT: ..." readout above the table is every motor's
fault_byte OR'd together.
</p>

<h2>Data Pipeline</h2>
<p>
Everything after the initial BLE connection flows over GATT characteristics
on one custom service; no serial/USB traffic once connected.
</p>
<p><b>Device &rarr; GUI (telemetry):</b> every 60ms (~17Hz), while connected
and subscribed, the firmware pushes a {FRAME_SIZE}-byte NOTIFY frame on
<code>CHAR_TELEMETRY_UUID</code>: uint16 seq, uint8 transition_active_flag,
uint8 current_node_id, uint8 target_node_id, then 5 per-motor records of
{{state (1B), voltage (4B float32), current (4B float32), fault_byte (1B)}},
10 bytes/motor &times; 5 motors + 5 header bytes = {FRAME_SIZE} bytes,
little-endian. Position and duty cycle were dropped from this frame as
redundant: neither fed anything the GUI needed beyond a display column.
</p>
<p>
<b>Motor index convention:</b> motor 0 is the thumb; motors 1-4 are
index/middle/ring/pinky.
</p>
<p><b>GUI &rarr; device (commands):</b> ASCII text, case-insensitive, on
<code>CHAR_MOTOR_CMD_UUID</code>. Grasp buttons send one of:
{', '.join(GRASP_NAMES)}. Individual-finger buttons send
<code>&lt;FINGER&gt;_CLOSE</code> / <code>&lt;FINGER&gt;_OPEN</code>, where
FINGER is one of {', '.join(n.upper() for n in FINGER_NAMES)}. Safe Override
sends <code>override_open</code> / <code>override_pack</code>. Clear Faults
sends <code>clear_faults</code>. The firmware runs every move itself; the
GUI never streams individual per-motor steps.</p>
"""


class HelpDialog(QDialog):
    """Reference documentation for the GUI/BLE system. Non-modal (see
    on_help_clicked()) so it can stay open for reference while using the
    rest of the GUI."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Help")
        self.setStyleSheet(DARK_STYLESHEET)
        self.resize(round(680 * UI_SCALE), round(760 * UI_SCALE))

        browser = QTextBrowser()
        browser.setOpenExternalLinks(False)
        browser.setHtml(HELP_HTML)

        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.close)

        layout = QVBoxLayout()
        layout.addWidget(browser)
        layout.addWidget(close_btn, alignment=Qt.AlignmentFlag.AlignRight)
        self.setLayout(layout)


class GestureGraphWidget(QWidget):
    """Paints the gesture graph: one node per GRAPH_NODE_ORDER entry, a
    faint edge between every pair (a real direct edge, not a display
    simplification; see module docstring), the current node highlighted
    green, and, while a transition is in progress, the target node
    highlighted amber with the current->target edge highlighted amber too.

    Driven entirely by set_state(), called from handle_notify() with the
    frame's (current_node_id, target_node_id, transition_active_flag); no
    client-side click-tracking.
    """

    # With the 12-node expansion set, the original 24px marker size left too
    # little room between adjacent nodes' labels around the circle; see
    # GRAPH_BOX_MIN_WIDTH's comment for the other half of that fix.
    NODE_RADIUS = 18

    # Status colors: green=good(current)/amber=warning(target, mid-transition),
    # picked to be distinct from the app's own categorical/UI colors and to
    # each carry a text label alongside (see the legend row below this widget)
    # rather than relying on hue alone.
    _GOOD = QColor(12, 163, 12)
    _WARN = QColor(250, 178, 25)
    _IDLE_FILL = QColor(44, 44, 49)
    _IDLE_EDGE = QColor(70, 70, 76)
    _BORDER = QColor(90, 90, 97)
    _TEXT = QColor(232, 232, 236)

    def __init__(self, parent=None):
        super().__init__(parent)
        # Overrides the class-level default above with the value scaled for
        # the actual screen (see apply_ui_scale()), read at instantiation
        # time so it reflects whatever UI_SCALE main() computed.
        self.NODE_RADIUS = round(18 * UI_SCALE)
        # Height trimmed from 280 to shrink the panel's contribution to the
        # window's minimum height (width floor never binds, since the group
        # box's own GRAPH_BOX_MIN_WIDTH already imposes a taller floor).
        self.setMinimumSize(round(280 * UI_SCALE), round(230 * UI_SCALE))
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._settled_name: str | None = None   # last node telemetry reported as "current" (not mid-transition)
        self._target_name: str | None = None
        self._transitioning = False

    def set_state(self, current_id: int, target_id: int, transitioning: bool) -> str | None:
        """Update the painted state and return the resolved 'settled' node
        name (or None if never yet settled), so callers can build a status
        label without reaching into this widget's internals."""
        if current_id != OFF_NODE_ID:
            self._settled_name = NODE_ID_TO_NAME.get(current_id)
        self._target_name = NODE_ID_TO_NAME.get(target_id)
        self._transitioning = transitioning
        self.update()
        return self._settled_name

    def reset(self):
        self._settled_name = None
        self._target_name = None
        self._transitioning = False
        self.update()

    def _node_positions(self) -> dict[str, QPointF]:
        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2
        radius = max(60.0, min(w, h) / 2 - self.NODE_RADIUS - 22)
        n = len(GRAPH_NODE_ORDER)
        positions = {}
        for i, name in enumerate(GRAPH_NODE_ORDER):
            angle = math.radians(-90 + i * (360 / n))
            positions[name] = QPointF(cx + radius * math.cos(angle), cy + radius * math.sin(angle))
        return positions

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        positions = self._node_positions()

        idle_pen = QPen(self._IDLE_EDGE, 1)
        for i, a in enumerate(GRAPH_NODE_ORDER):
            for b in GRAPH_NODE_ORDER[i + 1:]:
                painter.setPen(idle_pen)
                painter.drawLine(positions[a], positions[b])

        if (self._transitioning and self._settled_name and self._target_name
                and self._settled_name != self._target_name):
            painter.setPen(QPen(self._WARN, 3))
            painter.drawLine(positions[self._settled_name], positions[self._target_name])

        for name, pos in positions.items():
            is_target = self._transitioning and (name == self._target_name)
            is_current = (name == self._settled_name)

            if is_target:
                fill = self._WARN
            elif is_current:
                fill = self._GOOD
            else:
                fill = self._IDLE_FILL

            painter.setBrush(QBrush(fill))
            painter.setPen(QPen(self._BORDER, 1))
            painter.drawEllipse(pos, self.NODE_RADIUS, self.NODE_RADIUS)

            painter.setPen(QPen(self._TEXT))
            label_rect = QRectF(pos.x() - 44, pos.y() + self.NODE_RADIUS + 3, 88, 16)
            painter.drawText(label_rect, Qt.AlignmentFlag.AlignCenter, name)


class TelemetryWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RoboLimb Telemetry Dashboard (EXP-13)")
        self.setStyleSheet(DARK_STYLESHEET)
        self.client: BleakClient | None = None
        self.last_seq = None

        self.status_label = QLabel("Disconnected")
        self.connect_btn = QPushButton("Connect")
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)
        self.rate_label = QLabel("Transmission Rate: -- Hz")

        self.help_btn = QPushButton("Help")
        self._help_dialog: HelpDialog | None = None

        # Recent-command strip: narration aid for demos ("what did we just
        # send"). Populated from _send_command() itself, after a successful
        # write, so it only ever reflects commands actually sent over BLE.
        self._recent_commands: list[str] = []
        self.activity_label = QLabel("Recent: --")
        self.activity_label.setStyleSheet("color: #9a9aa2;")
        self.activity_label.setWordWrap(True)

        self.grasp_buttons: dict[str, QPushButton] = {}
        for name in GRASP_NAMES:
            btn = QPushButton(name)
            btn.setEnabled(False)
            self.grasp_buttons[name] = btn

        self.finger_close_buttons: dict[str, QPushButton] = {}
        self.finger_open_buttons: dict[str, QPushButton] = {}
        for name in FINGER_NAMES:
            for action, store in (("CLOSE", self.finger_close_buttons),
                                  ("OPEN", self.finger_open_buttons)):
                btn = QPushButton(action.title())
                btn.setEnabled(False)
                store[name] = btn

        # Safe Override: top-priority thin BLE triggers ("override_open" /
        # "override_pack") the firmware always accepts, abandoning whatever
        # the hand is doing (gesture or finger move) to drive straight to a
        # known-safe posture.
        self.override_open_btn = QPushButton("OVERRIDE to OPEN")
        self.override_pack_btn = QPushButton("OVERRIDE to PACK")
        # Burnt-orange tint so these two read as visually distinct from every
        # other button, a deliberate "this pre-empts everything" cue, not
        # just another grasp button. Disabled state falls back to the normal
        # muted/grey QPushButton:disabled rule from DARK_STYLESHEET.
        override_btn_qss = (
            "QPushButton { background-color: #7a3210; border: 1px solid #b3541c; "
            "color: #ffe4cf; font-weight: bold; } "
            "QPushButton:hover:!disabled { background-color: #9c4315; } "
            "QPushButton:pressed:!disabled { background-color: #612508; } "
            "QPushButton:disabled { background-color: #2c2c31; border: 1px solid #46464c; color: #77777c; }"
        )
        for btn in (self.override_open_btn, self.override_pack_btn):
            btn.setEnabled(False)
            btn.setStyleSheet(override_btn_qss)

        self.clear_faults_btn = QPushButton("Clear Faults")
        self.clear_faults_btn.setEnabled(False)

        # Hand Tracking (CV): see the module-level comment above
        # CV_TICK_INTERVAL_MS. self.cv_gate exists unconditionally (not just
        # while armed) so _sync_cv_dedupe() is always safe to call from
        # _send_command(), no matter when a manual command is sent relative
        # to ever touching this panel.
        self.cv_tracker: HandTracker | None = None
        self.cv_thresholds = None
        self.cv_gate = DispatchGate(stability_seconds=CV_STABILITY_DEFAULT_S)
        self._transition_active = False  # latest telemetry transition_active_flag
        self._cv_classifiable_names = {g.upper() for g in TARGET_GESTURE_MASKS}

        self.cv_stability_spin = QDoubleSpinBox()
        self.cv_stability_spin.setRange(CV_STABILITY_MIN_S, CV_STABILITY_MAX_S)
        self.cv_stability_spin.setSingleStep(0.1)
        self.cv_stability_spin.setValue(CV_STABILITY_DEFAULT_S)
        self.cv_stability_spin.setSuffix(" s")
        self.cv_stability_spin.valueChanged.connect(self._on_cv_stability_changed)

        # Defaults unchecked/unpressed: the primary safety backstop for a
        # feature that can move a real motor-driven mechanism on its own. A
        # checkable QPushButton rather than a QCheckBox so the armed/disarmed
        # state is unmissable (label text + a solid red "pressed" fill), not
        # just a small tick mark. Burnt-red tint (distinct from Safe
        # Override's orange) so it reads as its own deliberate "this can
        # start driving the hand by itself" toggle.
        self.cv_arm_btn = QPushButton("ARM CV CONTROL")
        self.cv_arm_btn.setCheckable(True)
        self.cv_arm_btn.setEnabled(False)
        self.cv_arm_btn.setStyleSheet(
            "QPushButton { background-color: #2c2c31; border: 1px solid #b34040; "
            "color: #e08a8a; font-weight: bold; } "
            "QPushButton:hover:!disabled { background-color: #35353b; } "
            "QPushButton:checked { background-color: #7a1414; border: 2px solid #dc3c37; "
            "color: #ffe4e4; } "
            "QPushButton:checked:hover { background-color: #8c1818; } "
            "QPushButton:disabled { background-color: #2c2c31; border: 1px solid #46464c; color: #77777c; }"
        )
        self.cv_arm_btn.toggled.connect(self._on_cv_arm_toggled)

        self.cv_status_label = QLabel("Not armed")
        self.cv_status_label.setStyleSheet("color: #9a9aa2; background: transparent;")
        self.cv_status_label.setWordWrap(True)

        # Embedded camera preview (no separate native cv2.imshow window):
        # placeholder text shown until armed, then _on_cv_tick() keeps it
        # updated via setPixmap(). Positioned in the GUI under the Gesture
        # Graph panel (see middle_column below), not in this control box, so
        # it reads as the graph's live "what the camera sees" companion
        # rather than just another control-panel widget.
        self.cv_preview_label = QLabel("Not armed")
        self.cv_preview_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.cv_preview_label.setStyleSheet("background-color: #000000; color: #77777c;")
        self.cv_preview_label.setMinimumHeight(round(140 * UI_SCALE))
        self.cv_preview_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

        self.cv_timer = QTimer(self)
        self.cv_timer.setInterval(CV_TICK_INTERVAL_MS)
        self.cv_timer.timeout.connect(self._on_cv_tick)

        self._motor_faults = [0] * MOTOR_COUNT

        self._frame_count = 0
        self._last_rate_frame_count = 0
        self.rate_timer = QTimer(self)
        self.rate_timer.setInterval(1000)
        self.rate_timer.timeout.connect(self._update_rate_label)
        self.rate_timer.start()

        TABLE_ROW_HEIGHT = round(22 * UI_SCALE)
        # 56 was too narrow for header text ("Voltage (V)"/"Current (A)", 9 chars)
        # plus the QHeaderView section padding (2px 4px) at the ambient font
        # size; 80 gives real margin.
        TABLE_MIN_COLUMN_WIDTH = round(80 * UI_SCALE)
        MOTOR_TABLE_HEADERS = ["State", "Voltage (V)", "Current (A)"]
        self.table = QTableWidget(MOTOR_COUNT, len(MOTOR_TABLE_HEADERS))
        self._configure_readout_table(self.table, MOTOR_TABLE_HEADERS, "--")

        h_header = self.table.horizontalHeader()
        v_header = self.table.verticalHeader()
        # Fixed, not Stretch: QHeaderView's Stretch mode splits available
        # width proportionally to each column's own size hint, not evenly --
        # with the removed Ripple/Pos%/Duty% columns gone, that left this
        # table's 3 remaining columns badly uneven (the last one nearly
        # clipped). _sync_motor_table_column_widths() (called from
        # resizeEvent/showEvent below) divides the table's actual width
        # evenly across columns itself instead.
        h_header.setSectionResizeMode(QHeaderView.ResizeMode.Fixed)
        v_header.setSectionResizeMode(QHeaderView.ResizeMode.Fixed)
        v_header.setDefaultSectionSize(TABLE_ROW_HEIGHT)

        table_min_width = (v_header.width() + TABLE_MIN_COLUMN_WIDTH * len(MOTOR_TABLE_HEADERS)
                            + 2 * self.table.frameWidth())
        table_height = h_header.height() + MOTOR_COUNT * TABLE_ROW_HEIGHT + 2 * self.table.frameWidth()
        self.table.setMinimumWidth(table_min_width)
        self.table.setFixedHeight(table_height)

        motor_data_box = QGroupBox("Motor Telemetry")
        motor_data_layout = QVBoxLayout()
        motor_data_layout.setContentsMargins(6, 10, 6, 4)
        motor_data_layout.addWidget(self.table)
        motor_data_box.setLayout(motor_data_layout)
        motor_data_box.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        motor_data_box.setMinimumWidth(BOX_MIN_WIDTH)

        gesture_box = QGroupBox("Gesture Controls")
        gesture_layout = QVBoxLayout()
        gesture_layout.setContentsMargins(6, 10, 6, 4)
        grasp_grid = QGridLayout()
        grasp_grid.setSpacing(4)
        for row, names in enumerate(GRASP_GRID_ROWS):
            span = 2 if len(names) == 1 else 1
            for col, name in enumerate(names):
                btn = self.grasp_buttons[name]
                btn.clicked.connect(lambda _checked=False, n=name: self._send_command(n))
                grasp_grid.addWidget(btn, row, col, 1, span)
        gesture_layout.addLayout(grasp_grid)

        finger_divider = QFrame()
        finger_divider.setFixedHeight(1)
        finger_divider.setStyleSheet("background-color: #46464c; border: none;")
        gesture_layout.addSpacing(4)
        gesture_layout.addWidget(finger_divider)
        finger_header = QLabel("Individual Fingers")
        finger_header.setStyleSheet("font-weight: bold; color: #b8b8c0; background: transparent;")
        gesture_layout.addWidget(finger_header)

        finger_label_qss = "color: #9a9aa2; background: transparent;"
        finger_grid = QGridLayout()
        finger_grid.setSpacing(4)
        finger_grid.setColumnStretch(1, 1)
        finger_grid.setColumnStretch(2, 1)
        for row, name in enumerate(FINGER_NAMES):
            name_label = QLabel(name)
            name_label.setStyleSheet(finger_label_qss)
            finger_grid.addWidget(
                name_label, row, 0,
                alignment=Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            )
            close_btn = self.finger_close_buttons[name]
            open_btn = self.finger_open_buttons[name]
            close_btn.clicked.connect(lambda _c=False, n=name: self._send_command(f"{n.upper()}_CLOSE"))
            open_btn.clicked.connect(lambda _c=False, n=name: self._send_command(f"{n.upper()}_OPEN"))
            finger_grid.addWidget(close_btn, row, 1)
            finger_grid.addWidget(open_btn, row, 2)
        gesture_layout.addLayout(finger_grid)

        gesture_box.setLayout(gesture_layout)
        gesture_box.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        gesture_box.setMinimumWidth(BOX_MIN_WIDTH)

        # Dynamic box: WAVE is a scripted animation, not a real grasp/graph
        # state (it bypasses TransitionEngine entirely; see the module
        # docstring and firmware transition_engine.h), so it gets its own
        # small group instead of sitting in the grid next to the real
        # grasps. Cool teal accent (vs. Safe Override's warning amber) so it
        # reads as "demo extra", not "danger". Its button is wired to
        # _send_wave_command() rather than _send_command() directly; see
        # that method and WAVE_PRE_OPEN_SETTLE_S for why (auto-OPEN first).
        wave_box = QGroupBox("Dynamic")
        wave_box.setStyleSheet(
            "QGroupBox { border: 1px solid #2c7a7a; } "
            "QGroupBox::title { color: #5fd0d0; }"
        )
        wave_layout = QVBoxLayout()
        wave_layout.setContentsMargins(6, 10, 6, 4)
        wave_btn = self.grasp_buttons[WAVE_GRASP_NAME]
        wave_btn.clicked.connect(self._send_wave_command)
        wave_layout.addWidget(wave_btn)
        wave_box.setLayout(wave_layout)
        wave_box.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        wave_box.setMinimumWidth(BOX_MIN_WIDTH)

        # Safe Override box: its own small group so it reads as a distinct,
        # deliberate "escape hatch" rather than just two more grasp buttons.
        # Burnt-orange border/title tint (matching the buttons inside) so the
        # whole box reads as a visually separate "pre-empts everything" zone
        # even before you look at the buttons themselves.
        override_box = QGroupBox("Overrides")
        override_box.setStyleSheet(
            "QGroupBox { border: 1px solid #b3541c; } "
            "QGroupBox::title { color: #e08a4f; }"
        )
        override_layout = QVBoxLayout()
        override_layout.setContentsMargins(6, 10, 6, 4)
        override_layout.addWidget(self.override_open_btn)
        override_layout.addWidget(self.override_pack_btn)
        override_box.setLayout(override_layout)
        override_box.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        override_box.setMinimumWidth(BOX_MIN_WIDTH)

        # Hand Tracking (CV) box: a peer to Gesture Controls/Dynamic as a
        # third command source (see CV_TICK_INTERVAL_MS comment), but placed
        # in the left column underneath Safe Override rather than stacked in
        # the rightmost column, freeing that column's vertical room for
        # future dynamic gesture buttons. Uses BOX_MIN_WIDTH (same floor as
        # every other box in this column) so it stays flush with the rest.
        cv_box = QGroupBox("Hand Tracking (CV)")
        cv_box.setStyleSheet(
            "QGroupBox { border: 1px solid #5a4fc7; } "
            "QGroupBox::title { color: #a79bf0; }"
        )
        cv_layout = QVBoxLayout()
        cv_layout.setContentsMargins(6, 10, 6, 4)
        cv_stab_row = QHBoxLayout()
        cv_stability_header = QLabel("Tracking Window:")
        cv_stability_header.setStyleSheet("background: transparent;")
        cv_stab_row.addWidget(cv_stability_header)
        cv_stab_row.addWidget(self.cv_stability_spin)
        cv_stab_row.addStretch(1)
        cv_layout.addLayout(cv_stab_row)
        cv_layout.addWidget(self.cv_arm_btn)
        cv_layout.addWidget(self.cv_status_label)
        cv_box.setLayout(cv_layout)
        cv_box.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        cv_box.setMinimumWidth(BOX_MIN_WIDTH)

        self.fault_health_label = QLabel("All Clear")
        self.fault_health_label.setStyleSheet("background: transparent;")
        fault_col_header_labels = ["STALL", "OCP", "OVP", "TSD", "NPOR", "CNT"]
        self.fault_table = QTableWidget(MOTOR_COUNT, len(FAULT_BITS))
        self._configure_readout_table(self.fault_table, fault_col_header_labels, "-")

        f_v_header = self.fault_table.verticalHeader()
        f_h_header = self.fault_table.horizontalHeader()

        f_v_header.setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self.fault_table.resizeRowsToContents()
        fault_row_height = f_v_header.sectionSize(0)
        f_v_header.setSectionResizeMode(QHeaderView.ResizeMode.Fixed)
        f_v_header.setDefaultSectionSize(fault_row_height)

        fault_available_width = BOX_MIN_WIDTH - 12 - f_v_header.width() - 2 * self.fault_table.frameWidth()
        fault_col_min_width = fault_available_width // len(FAULT_BITS)
        fault_min_width = f_v_header.width() + fault_col_min_width * len(FAULT_BITS) + 2 * self.fault_table.frameWidth()
        fault_table_height = f_h_header.height() + MOTOR_COUNT * fault_row_height + 2 * self.fault_table.frameWidth() + 2
        self.fault_table.setMinimumWidth(fault_min_width)
        self.fault_table.setFixedHeight(fault_table_height)

        fault_box = QGroupBox("Fault Detection")
        fault_layout = QVBoxLayout()
        fault_layout.setContentsMargins(6, 10, 6, 4)
        fault_layout.addWidget(self.fault_health_label)
        fault_layout.addWidget(self.fault_table)
        fault_layout.addWidget(self.clear_faults_btn)
        fault_box.setLayout(fault_layout)
        fault_box.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        fault_box.setMinimumWidth(BOX_MIN_WIDTH)

        # Gesture Graph panel: sits in the middle column. Canvas widget
        # above, a textual status line + color-key legend below it
        # (accessibility: current/target state is never conveyed by node
        # color alone).
        self.graph_widget = GestureGraphWidget()
        self.graph_status_label = QLabel("Not connected")
        self.graph_status_label.setStyleSheet("color: #b8b8c0; background: transparent;")
        self.graph_status_label.setWordWrap(True)

        legend_row = QHBoxLayout()
        legend_current = QLabel("Current")
        legend_current.setStyleSheet("color: #0ca30c; font-weight: bold; background: transparent;")
        legend_target = QLabel("Target")
        legend_target.setStyleSheet("color: #fab219; font-weight: bold; background: transparent;")
        legend_row.addWidget(legend_current)
        legend_row.addWidget(legend_target)
        legend_row.addStretch(1)

        graph_box = QGroupBox("Gesture Graph")
        graph_layout = QVBoxLayout()
        graph_layout.setContentsMargins(6, 10, 6, 4)
        graph_layout.addWidget(self.graph_status_label)
        graph_layout.addWidget(self.graph_widget, 1)
        graph_layout.addLayout(legend_row)
        graph_box.setLayout(graph_layout)
        graph_box.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        graph_box.setMinimumWidth(GRAPH_BOX_MIN_WIDTH)

        # Camera Preview: positioned under the Gesture Graph in the middle
        # column (see middle_column below), not inside the Hand Tracking (CV)
        # control box in the left column: this is the live "what the
        # camera sees" companion to the graph above it, not a control.
        # Preferred (not Expanding) vertical size policy so it takes only
        # the space its content needs and graph_box keeps first claim on
        # available vertical space (matches its own stretch factor below).
        cv_preview_box = QGroupBox("Camera Preview")
        cv_preview_box.setStyleSheet(
            "QGroupBox { border: 1px solid #5a4fc7; } "
            "QGroupBox::title { color: #a79bf0; }"
        )
        cv_preview_layout = QVBoxLayout()
        cv_preview_layout.setContentsMargins(6, 10, 6, 4)
        cv_preview_layout.addWidget(self.cv_preview_label)
        cv_preview_box.setLayout(cv_preview_layout)
        cv_preview_box.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        cv_preview_box.setMinimumWidth(GRAPH_BOX_MIN_WIDTH)

        top_bar = QHBoxLayout()
        top_bar.addWidget(self.status_label)
        top_bar.addWidget(self.connect_btn)
        top_bar.addWidget(self.disconnect_btn)
        top_bar.addStretch(1)
        top_bar.addWidget(self.rate_label)
        top_bar.addWidget(self.help_btn)

        # Left column: live data readouts, Safe Override underneath Fault
        # Detection, then Hand Tracking (CV) underneath Safe Override,
        # grouped here (rather than in the rightmost column) so that
        # column's Gesture Controls / Dynamic boxes have more vertical room
        # to grow their own button sets (e.g. future dynamic gesture buttons).
        # Explicit tight spacing on all three columns (rather than the
        # style's default ~11px QVBoxLayout spacing): with 4 stacked boxes in
        # left_column, the default spacing alone was pushing a restored
        # (non-maximized) window's bottom panel off the visible desktop.
        COLUMN_SPACING = 4

        left_column = QVBoxLayout()
        left_column.setSpacing(COLUMN_SPACING)
        left_column.addWidget(motor_data_box)
        left_column.addWidget(fault_box)
        left_column.addWidget(override_box)
        left_column.addWidget(cv_box)
        left_column.addStretch(1)

        # Middle column: the gesture graph (expanding to fill available
        # height/width since it's a diagram, not a fixed-content table),
        # with the live Camera Preview underneath it.
        middle_column = QVBoxLayout()
        middle_column.setSpacing(COLUMN_SPACING)
        middle_column.addWidget(graph_box, 1)
        middle_column.addWidget(cv_preview_box)

        # Right column: all button controls, Dynamic added beneath Gesture
        # Controls. Hand Tracking (CV)'s control box lives in the left
        # column instead (see above), freeing this column for future
        # dynamic gesture buttons.
        right_column = QVBoxLayout()
        right_column.setSpacing(COLUMN_SPACING)
        right_column.addWidget(gesture_box)
        right_column.addWidget(wave_box)
        right_column.addStretch(1)

        content_row = QHBoxLayout()
        content_row.addLayout(left_column, 1)
        content_row.addLayout(middle_column, 1)
        content_row.addLayout(right_column, 1)

        layout = QVBoxLayout()
        layout.addLayout(top_bar)
        layout.addWidget(self.activity_label)
        layout.addLayout(content_row, 1)
        self.setLayout(layout)

        self.connect_btn.clicked.connect(self.on_connect_clicked)
        self.disconnect_btn.clicked.connect(self.on_disconnect_clicked)
        self.clear_faults_btn.clicked.connect(lambda: self._send_command("clear_faults"))
        self.override_open_btn.clicked.connect(lambda: self._send_command("override_open"))
        self.override_pack_btn.clicked.connect(lambda: self._send_command("override_pack"))
        self.help_btn.clicked.connect(self.on_help_clicked)

    def _configure_readout_table(self, table, headers, placeholder):
        table.setHorizontalHeaderLabels(headers)
        table.setVerticalHeaderLabels(MOTOR_ROW_LABELS)
        table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        table.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        table.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        table.setCornerButtonEnabled(False)
        table.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        table.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        table.setStyleSheet(TABLE_QSS)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        for row in range(MOTOR_COUNT):
            for col in range(len(headers)):
                item = QTableWidgetItem(placeholder)
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                table.setItem(row, col, item)
        table.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

    def _sync_table_heights(self):
        for table in (self.table, self.fault_table):
            needed = (table.horizontalHeader().height()
                      + table.verticalHeader().length()
                      + 2 * table.frameWidth())
            if table.height() != needed:
                table.setFixedHeight(needed)

    def _sync_motor_table_column_widths(self):
        """Force the Motor Telemetry table's columns to exactly equal
        widths, splitting the table's actual current width. Manual instead
        of QHeaderView.ResizeMode.Stretch; see the comment where
        h_header's resize mode is set to Fixed in __init__ for why."""
        n = self.table.columnCount()
        total = self.table.viewport().width()
        if n == 0 or total <= 0:
            return
        base = total // n
        for col in range(n):
            # Last column absorbs the rounding remainder so the columns'
            # widths sum exactly to the viewport width (no leftover gap).
            self.table.setColumnWidth(col, total - base * (n - 1) if col == n - 1 else base)

    def showEvent(self, event):
        super().showEvent(event)
        self._sync_table_heights()
        self._sync_motor_table_column_widths()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._sync_table_heights()
        self._sync_motor_table_column_widths()

    def closeEvent(self, event):
        # HandTracker.close() is idempotent by design (see hand_tracker.py)
        # specifically so a defensive call here and an explicit disarm both
        # calling it is safe. No cv2 native window to destroy: the camera
        # preview is an embedded QLabel (cv_preview_label), cleaned up
        # automatically with the rest of the Qt widget tree.
        if self.cv_tracker is not None:
            self.cv_tracker.close()
        super().closeEvent(event)

    def on_help_clicked(self):
        if self._help_dialog is None:
            self._help_dialog = HelpDialog(self)
        self._help_dialog.show()
        self._help_dialog.raise_()
        self._help_dialog.activateWindow()

    @asyncSlot()
    async def on_connect_clicked(self):
        self.connect_btn.setEnabled(False)
        self.status_label.setText(f"Scanning for '{DEVICE_NAME}'...")

        device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=SCAN_TIMEOUT_S)
        if device is None:
            self.status_label.setText(f"'{DEVICE_NAME}' not found, is it advertising?")
            self.connect_btn.setEnabled(True)
            return

        self.client = BleakClient(device, disconnected_callback=self.on_disconnected)
        await self.client.connect()

        mtu_payload = self.client.mtu_size - 3
        if mtu_payload < MIN_MTU_PAYLOAD:
            self.status_label.setText(
                f"Connected, but MTU too small ({mtu_payload}B < {MIN_MTU_PAYLOAD}B needed) "
                f"-- telemetry will be corrupt!"
            )
        else:
            self.status_label.setText(f"Connected (MTU payload {mtu_payload}B)")

        await self.client.start_notify(CHAR_TELEMETRY_UUID, self.handle_notify)

        self.disconnect_btn.setEnabled(True)
        self._set_command_buttons_enabled(True)

    @asyncSlot()
    async def on_disconnect_clicked(self):
        if self.client is not None and self.client.is_connected:
            await self.client.disconnect()

    def on_disconnected(self, client):
        self.status_label.setText("Disconnected")
        self.disconnect_btn.setEnabled(False)
        # Force-disarm CV before disabling the rest of the command surface --
        # setChecked(False) fires _on_cv_arm_toggled(), which stops the
        # camera timer/releases the webcam. A no-op if it was already unarmed.
        self.cv_arm_btn.setChecked(False)
        self._set_command_buttons_enabled(False)
        self.connect_btn.setEnabled(True)
        self.graph_widget.reset()
        self.graph_status_label.setText("Not connected")
        self._update_grasp_highlight(None, None, False)
        for finger in FINGER_NAMES:
            self._set_finger_pair_style(finger, -1)

    def _set_command_buttons_enabled(self, enabled: bool):
        for btn in (*self.grasp_buttons.values(),
                    *self.finger_close_buttons.values(),
                    *self.finger_open_buttons.values(),
                    self.clear_faults_btn,
                    self.override_open_btn, self.override_pack_btn,
                    self.cv_arm_btn):
            btn.setEnabled(enabled)

    def handle_notify(self, _characteristic, data: bytearray):
        if len(data) != FRAME_SIZE:
            self.status_label.setText(f"Bad frame size {len(data)} (expected {FRAME_SIZE}), MTU issue?")
            return

        self._frame_count += 1

        fields = struct.unpack(FRAME_FORMAT, data)
        seq, transitioning, current_id, target_id = fields[0:4]
        # Consumed by _on_cv_tick(): CV dispatch is suppressed while a
        # transition (from ANY source, button or CV) is still in flight, so
        # CV's ~30Hz cadence never re-targets the transition engine mid-move.
        self._transition_active = bool(transitioning)
        if self.last_seq is not None and (seq - self.last_seq) % 65536 != 1:
            print(f"[warn] dropped frame(s): seq jumped {self.last_seq} -> {seq}")
        self.last_seq = seq

        for i in range(MOTOR_COUNT):
            state, voltage, current, fault_byte = fields[4 + i * 4: 4 + i * 4 + 4]

            self.table.item(i, 0).setText(STATE_NAMES.get(state, f"?{state}"))
            self.table.item(i, 1).setText(f"{voltage:.2f}")
            self.table.item(i, 2).setText(f"{current:.2f}")

            self._motor_faults[i] = fault_byte
            self._set_finger_pair_style(FINGER_NAMES[i], state)

        self._refresh_fault_table()

        settled_name = self.graph_widget.set_state(current_id, target_id, bool(transitioning))
        target_name = NODE_ID_TO_NAME.get(target_id, "?")
        if transitioning:
            self.graph_status_label.setText(f"{settled_name or '?'} -> {target_name}  (transitioning...)")
        else:
            self.graph_status_label.setText(f"Current: {settled_name or target_name}")
        self._update_grasp_highlight(settled_name, target_name, bool(transitioning))

    def _update_grasp_highlight(self, settled_name: str | None, target_name: str | None, transitioning: bool):
        """Border-highlight the grasp button matching the graph's live
        current/target node, mirroring GestureGraphWidget's own green/amber
        precedence exactly (target wins over current if a node is somehow
        both at once). WAVE never matches: it's not a graph node (see
        NODE_ID_TO_NAME) and lives in its own Dynamic box."""
        for name, btn in self.grasp_buttons.items():
            if transitioning and name == target_name:
                btn.setStyleSheet(GRASP_TARGET_QSS)
            elif name == settled_name:
                btn.setStyleSheet(GRASP_CURRENT_QSS)
            else:
                btn.setStyleSheet("")

    def _set_finger_pair_style(self, finger: str, state: int):
        """Tint a finger's Close/Open buttons together by its live motor
        state (MOVING/HOLDING/STALLED); any other value clears to default."""
        qss = FINGER_STATE_QSS.get(state, "")
        self.finger_close_buttons[finger].setStyleSheet(qss)
        self.finger_open_buttons[finger].setStyleSheet(qss)

    def _refresh_fault_table(self):
        overall = 0
        for i in range(MOTOR_COUNT):
            fault_byte = self._motor_faults[i]
            overall |= fault_byte
            for col, (_name, bit) in enumerate(FAULT_BITS):
                item = self.fault_table.item(i, col)
                if fault_byte & bit:
                    item.setText("●")
                    item.setForeground(QColor(220, 60, 55))
                else:
                    item.setText("-")
                    item.setForeground(QColor(120, 120, 126))

        if overall == 0:
            self.fault_health_label.setText("All Clear")
            self.fault_health_label.setStyleSheet("color: #4caf50; background: transparent;")
        else:
            names = _decode_fault_names(overall)
            self.fault_health_label.setText(f"FAULT: {', '.join(names)}")
            self.fault_health_label.setStyleSheet("color: #dc3c37; font-weight: bold; background: transparent;")

    @asyncSlot()
    async def _send_command(self, text: str):
        if self.client is None or not self.client.is_connected:
            self.status_label.setText("Not connected")
            return
        await self.client.write_gatt_char(CHAR_MOTOR_CMD_UUID, text.encode("utf-8"), response=True)
        self._log_command(text)
        self._sync_cv_dedupe(text)

    def _sync_cv_dedupe(self, text: str):
        """Keep the CV dispatch gate's dedupe memory in sync with a command
        that just reached the hand, whatever its source (manual click,
        override, or CV itself via _on_cv_tick(); this runs for all of them,
        since they all funnel through _send_command() above).

        Mirrors DispatchGate.sync_external_command()'s "momentary nudge"
        contract: this only updates dedupe memory, never CV's own live
        stability timer, so an armed CV loop keeps evaluating the camera
        uninterrupted and can still reassert itself after its own next
        stability window if the live hand disagrees with what was just sent.
        """
        upper = text.upper()
        if upper == "CLEAR_FAULTS":
            return  # doesn't move the hand, irrelevant to CV dedupe memory
        if upper == "OVERRIDE_OPEN":
            self.cv_gate.sync_external_command("open")
        elif upper == "OVERRIDE_PACK":
            self.cv_gate.sync_external_command("pack")
        elif upper in self._cv_classifiable_names:
            self.cv_gate.sync_external_command(upper.lower())
        else:
            # WAVE (not a graph node) or a <FINGER>_CLOSE/_OPEN command:
            # both leave the hand in a pose that can't be confidently mapped
            # onto a known mask, so force CV to require a fresh stability
            # window rather than assume/guess what state it's in.
            self.cv_gate.sync_external_command(None)

    def _on_cv_stability_changed(self, value: float):
        self.cv_gate.stability_seconds = value

    def _on_cv_arm_toggled(self, checked: bool):
        if checked:
            try:
                # Camera index is fixed at 0 (the OS's default/first camera);
                # the picker was removed as unneeded for a single-camera setup.
                # If a different index is ever needed, cv_camera_check.py opens
                # a given index standalone to identify which is which.
                self.cv_tracker = HandTracker(camera_index=0)
                self.cv_tracker.open()
            except RuntimeError as exc:
                self.cv_status_label.setText(f"Camera error: {exc}")
                self.cv_tracker = None
                self.cv_arm_btn.blockSignals(True)
                self.cv_arm_btn.setChecked(False)
                self.cv_arm_btn.blockSignals(False)
                return
            # Reloaded on every arm (not just at startup) so a freshly
            # recalibrated cv_calibration.json takes effect without
            # restarting the GUI.
            self.cv_thresholds = load_thresholds(quiet=True)
            self.cv_gate = DispatchGate(stability_seconds=self.cv_stability_spin.value())
            self.cv_arm_btn.setText("DISARM CV CONTROL")
            self.cv_status_label.setText("Armed, tracking...")
            self.cv_timer.start()
        else:
            self.cv_timer.stop()
            if self.cv_tracker is not None:
                self.cv_tracker.close()
                self.cv_tracker = None
            self.cv_arm_btn.setText("ARM CV CONTROL")
            self.cv_status_label.setText("Not armed")
            self.cv_preview_label.setText("Not armed")

    def _on_cv_tick(self):
        if self.cv_tracker is None:
            return
        try:
            frame, curl_pcts, detected = self.cv_tracker.step()
        except RuntimeError as exc:
            self.cv_status_label.setText(f"Camera error: {exc}, disarming")
            self.cv_arm_btn.setChecked(False)  # triggers cleanup above
            return

        states = finger_states(curl_pcts, self.cv_thresholds)
        mask = states_to_mask(states)
        candidate = classify_mask(mask)

        now = time.monotonic()
        dispatched = None
        if not self._transition_active:
            # gate.update() is only called when NOT transitioning: this
            # freezes the stability window (rather than computing-but-
            # discarding a dispatch) so the gate's dedupe bookkeeping is
            # never told a gesture was dispatched when it wasn't.
            dispatched = self.cv_gate.update(candidate, now)
        if dispatched is not None:
            asyncio.ensure_future(self._send_command(dispatched.upper()))

        stable_candidate, remaining, last_dispatched = self.cv_gate.status(now)
        suffix = "  (paused: hand mid-transition)" if self._transition_active else ""
        self.cv_status_label.setText(
            f"{'Hand detected' if detected else 'No hand detected'}  |  "
            f"candidate: {stable_candidate or '-'}  |  hold: {remaining:.1f}s  |  "
            f"last sent: {last_dispatched or '-'}{suffix}"
        )

        curl_str = " ".join(
            f"{name[0].upper()}:{pct:3.0f}" for name, pct in zip(FINGER_NAMES, curl_pcts)
        )
        cv2.putText(frame, f"{curl_str}  mask:0x{mask:02X}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
        self.cv_preview_label.setPixmap(self._frame_to_pixmap(frame))

    def _frame_to_pixmap(self, frame_bgr) -> QPixmap:
        """Converts an HxWx3 BGR numpy frame (as returned by HandTracker.step(),
        HUD already drawn on it) into a QPixmap scaled to CV_PREVIEW_LABEL_WIDTH,
        for embedding directly in the GUI (replaces the old separate cv2.imshow
        native window). QPixmap.fromImage() copies the pixel data internally,
        so there's no risk of the numpy buffer being reused/overwritten by the
        next tick out from under Qt."""
        h, w, _ch = frame_bgr.shape
        bytes_per_line = frame_bgr.strides[0]
        qimg = QImage(frame_bgr.data, w, h, bytes_per_line, QImage.Format.Format_BGR888)
        return QPixmap.fromImage(qimg).scaledToWidth(
            CV_PREVIEW_LABEL_WIDTH, Qt.TransformationMode.SmoothTransformation
        )

    @asyncSlot()
    async def _send_wave_command(self):
        """Wave-only pre-step: send OPEN and give it time to actually settle
        before sending WAVE, so the animation never starts with some other
        finger (most importantly the thumb, which WAVE never drives) still
        closed in its path. Scoped to this one button; every other grasp/
        finger command still goes straight through _send_command()."""
        await self._send_command("OPEN")
        await asyncio.sleep(WAVE_PRE_OPEN_SETTLE_S)
        await self._send_command(WAVE_GRASP_NAME)

    def _format_command_label(self, text: str) -> str:
        if text in self.grasp_buttons:
            return f"-> {text}"
        if text == "override_open":
            return "OPEN (Safe Override)"
        if text == "override_pack":
            return "PACK (Safe Override)"
        if text == "clear_faults":
            return "Clear Faults"
        if "_" in text:
            finger, _, action = text.partition("_")
            return f"-> {finger.title()} {action.title()}"
        return text

    def _log_command(self, text: str):
        self._recent_commands.insert(0, self._format_command_label(text))
        del self._recent_commands[MAX_RECENT_COMMANDS:]
        self.activity_label.setText("Recent: " + "   |   ".join(self._recent_commands))

    def _update_rate_label(self):
        rate = self._frame_count - self._last_rate_frame_count
        self._last_rate_frame_count = self._frame_count
        self.rate_label.setText(f"Transmission Rate: {rate} Hz")


def main():
    app = QApplication(sys.argv)
    apply_ui_scale(app)  # must run before TelemetryWindow(); see its docstring
    window = TelemetryWindow()

    # Fill the actual screen (e.g. a 1920x1080 display) rather than opening
    # at the layout's un-scaled preferred size, which left dead space on a
    # generous screen and (pre-apply_ui_scale) could exceed a scaled-down
    # one. showMaximized(), not resize(), so the window still behaves like a
    # normal maximized window (restorable, movable) instead of a borderless
    # fixed-size one.
    window.showMaximized()

    loop = qasync.QEventLoop(app)
    asyncio.set_event_loop(loop)
    with loop:
        loop.run_forever()


if __name__ == "__main__":
    main()
