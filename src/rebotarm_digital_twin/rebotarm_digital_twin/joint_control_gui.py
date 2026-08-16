#!/usr/bin/env python3
import sys

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)
import rclpy
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from rebotarm_digital_twin.twin_utils import (
    JOINT_NAMES,
    LOWER_LIMITS,
    UPPER_LIMITS,
)


SLIDER_STEPS = 10000


class JointTargetNode(Node):
    def __init__(self):
        super().__init__("joint_control_gui")
        self.publisher = self.create_publisher(
            JointState,
            "/host/raw_joint_states",
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )

    def publish_positions(self, positions):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINT_NAMES)
        message.position = list(positions)
        self.publisher.publish(message)


class JointRow(QFrame):
    def __init__(self, index, lower, upper, initial, changed):
        super().__init__()
        self.lower = lower
        self.upper = upper
        self.changed = changed
        self.setObjectName("jointRow")

        layout = QGridLayout(self)
        layout.setContentsMargins(18, 12, 18, 12)
        layout.setHorizontalSpacing(16)
        layout.setVerticalSpacing(6)
        layout.setColumnStretch(1, 1)

        name = QLabel(f"J{index + 1}")
        name.setObjectName("jointName")
        description = QLabel(f"joint{index + 1}")
        description.setObjectName("jointDescription")
        name_box = QVBoxLayout()
        name_box.setSpacing(0)
        name_box.addWidget(name)
        name_box.addWidget(description)
        layout.addLayout(name_box, 0, 0)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, SLIDER_STEPS)
        self.slider.setSingleStep(1)
        self.slider.setPageStep(100)
        self.slider.setMinimumWidth(360)
        self.slider.setFixedHeight(32)
        self.slider.setCursor(Qt.PointingHandCursor)
        self.slider.setAccessibleName(f"joint{index + 1} target")
        layout.addWidget(self.slider, 0, 1)

        self.spin = QDoubleSpinBox()
        self.spin.setRange(lower, upper)
        self.spin.setDecimals(4)
        self.spin.setSingleStep(0.01)
        self.spin.setSuffix(" rad")
        self.spin.setMinimumWidth(132)
        layout.addWidget(self.spin, 0, 2)

        limit_row = QHBoxLayout()
        limit_row.setContentsMargins(1, 0, 1, 0)
        lower_limit = QLabel(f"{lower:.2f}")
        lower_limit.setObjectName("limitLabel")
        upper_limit = QLabel(f"{upper:.2f}")
        upper_limit.setObjectName("limitLabel")
        limit_row.addWidget(lower_limit)
        limit_row.addStretch()
        limit_row.addWidget(upper_limit)
        layout.addLayout(limit_row, 1, 1)

        self.slider.valueChanged.connect(self._slider_changed)
        self.spin.valueChanged.connect(self._spin_changed)
        self.set_value(initial, emit=False)

    def _slider_changed(self, raw):
        value = self.lower + (self.upper - self.lower) * raw / SLIDER_STEPS
        self.spin.blockSignals(True)
        self.spin.setValue(value)
        self.spin.blockSignals(False)
        self.changed()

    def _spin_changed(self, value):
        ratio = (value - self.lower) / (self.upper - self.lower)
        self.slider.blockSignals(True)
        self.slider.setValue(round(ratio * SLIDER_STEPS))
        self.slider.blockSignals(False)
        self.changed()

    def set_value(self, value, emit=True):
        value = max(self.lower, min(self.upper, value))
        ratio = (value - self.lower) / (self.upper - self.lower)
        self.slider.blockSignals(True)
        self.spin.blockSignals(True)
        self.slider.setValue(round(ratio * SLIDER_STEPS))
        self.spin.setValue(value)
        self.slider.blockSignals(False)
        self.spin.blockSignals(False)
        if emit:
            self.changed()

    def value(self):
        return self.spin.value()


class JointControlWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("reBotArm Joint Control")
        self.setMinimumSize(760, 690)
        self.resize(820, 720)

        root = QWidget()
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(24, 20, 24, 20)
        outer.setSpacing(14)

        header = QHBoxLayout()
        title_box = QVBoxLayout()
        title_box.setSpacing(2)
        title = QLabel("关节目标控制")
        title.setObjectName("title")
        subtitle = QLabel("reBotArm · 6-DOF · rad")
        subtitle.setObjectName("subtitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        header.addLayout(title_box)
        header.addStretch()
        status_dot = QLabel("●  SIM ONLY")
        status_dot.setObjectName("status")
        header.addWidget(status_dot)
        outer.addLayout(header)

        initial = tuple(
            max(low, min(high, 0.0))
            for low, high in zip(LOWER_LIMITS, UPPER_LIMITS)
        )
        self.rows = []
        for index, (lower, upper, value) in enumerate(
            zip(LOWER_LIMITS, UPPER_LIMITS, initial)
        ):
            row = JointRow(index, lower, upper, value, self.publish)
            self.rows.append(row)
            outer.addWidget(row)

        footer = QHBoxLayout()
        self.feedback = QLabel("目标已就绪")
        self.feedback.setObjectName("footerText")
        footer.addWidget(self.feedback)
        footer.addStretch()
        center = QPushButton("回到中位")
        center.setObjectName("secondaryButton")
        center.clicked.connect(self.center_all)
        footer.addWidget(center)
        outer.addLayout(footer)

        self.setStyleSheet(STYLESHEET)
        QTimer.singleShot(0, self.publish)

    def publish(self):
        positions = tuple(row.value() for row in self.rows)
        self.node.publish_positions(positions)
        self.feedback.setText("目标已更新")

    def center_all(self):
        for row, lower, upper in zip(self.rows, LOWER_LIMITS, UPPER_LIMITS):
            row.set_value((lower + upper) / 2.0, emit=False)
        self.publish()


STYLESHEET = """
QMainWindow, QWidget {
    background: #f4f6f8;
    color: #18212b;
    font-family: "Noto Sans CJK SC", "Noto Sans", sans-serif;
    font-size: 14px;
}
QLabel#title { font-size: 25px; font-weight: 700; }
QLabel#subtitle { color: #687482; font-size: 13px; }
QLabel#status {
    color: #176b45; background: #e3f4ea; border: 1px solid #b8dfc9;
    border-radius: 4px; padding: 7px 12px; font-weight: 700;
}
QFrame#jointRow {
    background: #ffffff; border: 1px solid #dce2e8; border-radius: 6px;
}
QLabel#jointName { font-size: 18px; font-weight: 700; color: #17212b; }
QLabel#jointDescription, QLabel#limitLabel, QLabel#footerText {
    color: #75818e; font-size: 12px;
}
QSlider { background: transparent; }
QSlider::groove:horizontal {
    height: 4px; background: #dfe5e9; border-radius: 2px;
}
QSlider::sub-page:horizontal {
    background: #1b8a73; border-radius: 2px;
}
QSlider::add-page:horizontal {
    background: #dfe5e9; border-radius: 2px;
}
QSlider::handle:horizontal {
    width: 16px; height: 16px; margin: -7px 0;
    background: #1b8a73; border: 2px solid #ffffff; border-radius: 9px;
}
QSlider::handle:horizontal:hover {
    width: 18px; height: 18px; margin: -8px -1px;
    background: #14745f; border: 2px solid #d8eee8; border-radius: 10px;
}
QSlider::handle:horizontal:pressed {
    background: #105d4e; border-color: #b7ddd3;
}
QDoubleSpinBox {
    min-height: 34px; background: #f8fafb; border: 1px solid #cfd7df;
    border-radius: 4px; padding: 0 8px; font-family: monospace;
}
QPushButton#secondaryButton {
    min-height: 36px; padding: 0 18px; background: #ffffff;
    border: 1px solid #bcc7d1; border-radius: 4px; font-weight: 600;
}
QPushButton#secondaryButton:hover { background: #e9eef2; }
"""


def main(args=None):
    rclpy.init(args=args)
    node = JointTargetNode()
    app = QApplication(sys.argv)
    app.setFont(QFont("Noto Sans", 10))
    window = JointControlWindow(node)
    window.show()
    timer = QTimer()

    def spin_ros():
        if not rclpy.ok():
            timer.stop()
            app.quit()
            return
        try:
            rclpy.spin_once(node, timeout_sec=0.0)
        except RCLError:
            timer.stop()
            app.quit()

    timer.timeout.connect(spin_ros)
    timer.start(10)
    try:
        return app.exec_()
    finally:
        timer.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
