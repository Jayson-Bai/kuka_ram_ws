from python_qt_binding import QtCore, QtWidgets, QtGui
from rqt_gui_py.plugin import Plugin
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
import subprocess
import os
import signal
import json
import threading

from my_project_interfaces.msg import UiStatus

from std_msgs.msg import String as StringMsg
import re
from pathlib import Path


LAUNCH_PARAMS = [
    # (param_name, default_value, description, group)
    ("center_start_delay_s", "1.0", "中心节点启动延迟（秒）", "中心节点"),
    ("npz_preload_chunks", "2", "NPZ 预加载块数", "中心节点"),
    ("queue_low", "1000", "轨迹队列低水位", "中心节点"),
    ("queue_high", "2000", "轨迹队列高水位", "中心节点"),
    ("plan_qos_depth", "2000", "Plan 话题 QoS 深度", "中心节点"),
    ("traj_prefill", "1000", "启动时轨迹预填充数量", "中心节点"),
    ("traj_low", "500", "轨迹积压低阈值", "中心节点"),
    ("traj_high", "1500", "轨迹积压高阈值", "中心节点"),
    ("xyzabc_decimals", "6", "位姿小数精度", "中心节点"),
    ("e_decimals", "2", "挤出小数精度", "中心节点"),
    ("kuka_status_raw", "false", "打印 KUKA 原始 XML 长度", "中心节点"),
    ("summary_period_ms", "200", "控制中心发布周期（ms）", "中心节点"),
    ("sen_type", "PythonDemo", "RSI XML SEN 类型", "RSI 节点"),
    ("decimal_precision", "4", "RSI 数据小数精度", "RSI 节点"),
    ("local_ip", "192.168.1.1", "RSI 本地监听 IP", "RSI 节点"),
    ("local_port", "49152", "RSI 本地监听端口", "RSI 节点"),
    ("abort_lift_mm", "100.0", "ABORT 时 Z 轴抬升距离（mm）", "RSI 节点"),
    ("abort_lift_speed_mm_s", "10.0", "ABORT 时 Z 轴抬升速度（mm/s）", "RSI 节点"),
    ("port", "/dev/ttyUSB0", "UART 串口设备路径", "UART 节点"),
    ("baudrate", "115200", "UART 波特率", "UART 节点"),
    ("extrude_scale", "1.0", "UART 挤出倍率因子", "UART 节点"),
    ("ui_publish_period_ms", "200", "UI 状态发布周期（ms）", "系统管理器"),
    ("heartbeat_timeout_s", "1.0", "心跳超时（秒）", "系统管理器"),
    ("traj_queue_limit", "5000", "UI 轨迹队列上限", "系统管理器"),
    ("event_queue_limit", "2000", "UI 事件队列上限", "系统管理器"),
]

_LAUNCH_DEFAULTS = {p[0]: p[1] for p in LAUNCH_PARAMS}
_LAUNCH_GROUPS_ORDER = ["中心节点", "RSI 节点", "UART 节点", "系统管理器"]


class _LaunchSettingsDialog(QtWidgets.QDialog):
    """Dialog for editing all launch parameters grouped by node."""

    def __init__(self, current_params, parent=None):
        super().__init__(parent)
        self.setWindowTitle("启动参数设置")
        self.setMinimumSize(620, 520)
        self._inputs = {}
        self._build_ui(current_params)

    def _build_ui(self, current_params):
        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(10)

        title = QtWidgets.QLabel("启动参数配置")
        title.setStyleSheet("font-size: 15px; font-weight: 700; color: #2b2b2b;")
        main_layout.addWidget(title)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll_widget = QtWidgets.QWidget()
        scroll_layout = QtWidgets.QVBoxLayout(scroll_widget)
        scroll_layout.setSpacing(10)
        scroll_layout.setContentsMargins(0, 0, 6, 0)

        # Group params by node
        groups = {}
        for name, default, desc, group in LAUNCH_PARAMS:
            groups.setdefault(group, []).append((name, default, desc))

        group_colors = {
            "中心节点": "#1a73e8",
            "RSI 节点": "#b15e00",
            "UART 节点": "#1b6e3c",
            "系统管理器": "#7b1fa2",
        }

        for group_name in _LAUNCH_GROUPS_ORDER:
            if group_name not in groups:
                continue
            group_box = QtWidgets.QGroupBox(group_name)
            color = group_colors.get(group_name, "#333333")
            group_box.setStyleSheet(
                "QGroupBox { font-weight: 600; margin-top: 4px;"
                " padding: 10px 8px 8px 8px;"
                " border: 1px solid #d0d0d0; border-radius: 6px;"
                " background: #ffffff; }"
                "QGroupBox::title { subcontrol-origin: margin;"
                " subcontrol-position: top left;"
                f" padding: 0 6px; color: {color}; }}"
            )
            form = QtWidgets.QFormLayout(group_box)
            form.setLabelAlignment(
                QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter
            )
            form.setFieldGrowthPolicy(
                QtWidgets.QFormLayout.AllNonFixedFieldsGrow
            )
            form.setHorizontalSpacing(12)
            form.setVerticalSpacing(8)

            for param_name, default_val, description in groups[group_name]:
                current_val = current_params.get(param_name, default_val)
                label = QtWidgets.QLabel(param_name)
                label.setToolTip(description)
                label.setStyleSheet("color: #444444; font-size: 12px;")

                if param_name == "kuka_status_raw":
                    widget = QtWidgets.QCheckBox(description)
                    widget.setChecked(current_val.lower() == "true")
                    form.addRow(label, widget)

                else:
                    widget = QtWidgets.QLineEdit(current_val)
                    widget.setToolTip(description)
                    widget.setStyleSheet(
                        "border: 1px solid #d0d0d0; border-radius: 4px;"
                        " padding: 4px 6px; background: #ffffff;"
                    )
                    form.addRow(label, widget)

                self._inputs[param_name] = widget

            scroll_layout.addWidget(group_box)

        scroll_layout.addStretch(1)
        scroll.setWidget(scroll_widget)
        main_layout.addWidget(scroll, 1)

        # Bottom buttons
        btn_row = QtWidgets.QHBoxLayout()
        btn_row.setSpacing(8)
        btn_reset = QtWidgets.QPushButton("恢复默认")
        btn_reset.setMinimumHeight(32)
        btn_reset.setCursor(QtCore.Qt.PointingHandCursor)
        btn_reset.setStyleSheet(
            "font-weight: 600; border: 1px solid #c0c0c0;"
            " border-radius: 5px; background: #ffffff;"
            " color: #666666; padding: 4px 16px;"
        )
        btn_reset.clicked.connect(self._reset_defaults)
        btn_ok = QtWidgets.QPushButton("确定")
        btn_ok.setMinimumHeight(32)
        btn_ok.setCursor(QtCore.Qt.PointingHandCursor)
        btn_ok.setStyleSheet(
            "font-weight: 600; border: 1px solid #1a73e8;"
            " border-radius: 5px; background: #1a73e8;"
            " color: #ffffff; padding: 4px 20px;"
        )
        btn_ok.clicked.connect(self.accept)
        btn_cancel = QtWidgets.QPushButton("取消")
        btn_cancel.setMinimumHeight(32)
        btn_cancel.setCursor(QtCore.Qt.PointingHandCursor)
        btn_cancel.setStyleSheet(
            "font-weight: 600; border: 1px solid #c0c0c0;"
            " border-radius: 5px; background: #ffffff;"
            " color: #333333; padding: 4px 16px;"
        )
        btn_cancel.clicked.connect(self.reject)
        btn_row.addWidget(btn_reset)
        btn_row.addStretch(1)
        btn_row.addWidget(btn_cancel)
        btn_row.addWidget(btn_ok)
        main_layout.addLayout(btn_row)

        self.setStyleSheet("QWidget { background: #f7f7f7; }")



    def _reset_defaults(self):
        for name, default_val in _LAUNCH_DEFAULTS.items():
            widget = self._inputs.get(name)
            if widget is None:
                continue
            if isinstance(widget, QtWidgets.QCheckBox):
                widget.setChecked(default_val.lower() == "true")
            elif isinstance(widget, QtWidgets.QLineEdit):
                widget.setText(default_val)

    def get_params(self):
        result = {}
        for name, widget in self._inputs.items():
            if isinstance(widget, QtWidgets.QCheckBox):
                result[name] = "true" if widget.isChecked() else "false"
            elif isinstance(widget, QtWidgets.QLineEdit):
                val = widget.text().strip()
                result[name] = val if val else _LAUNCH_DEFAULTS.get(name, "")
        return result


class _AutoScaleLabel(QtWidgets.QLabel):
    def __init__(self, text=""):
        super().__init__(text)
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        self._color = "#a0a0a0"

    def set_color(self, color):
        self._color = color
        self.update_style()

    def update_style(self):
        self.setStyleSheet(f"color: {self._color}; font-weight: 900;")

    def resizeEvent(self, event):
        super().resizeEvent(event)
        rect = self.rect()
        if rect.height() > 0:
            font = self.font()
            pixel_size = max(12, int(rect.height() * 0.55))
            font.setPixelSize(pixel_size)
            self.setFont(font)



_OFFSET_CONFIG_DIR = os.path.expanduser("~/.config/my_project")
_OFFSET_CONFIG_PATH = os.path.join(_OFFSET_CONFIG_DIR, "tool_offset.json")
_OFFSET_DEFAULTS = {"tool_offset_x": 0.64, "tool_offset_y": -1.29, "tool_offset_z": 0.17}


def _load_offset_config():
    try:
        with open(_OFFSET_CONFIG_PATH, "r") as f:
            data = json.load(f)
        return {
            "tool_offset_x": float(data.get("tool_offset_x", _OFFSET_DEFAULTS["tool_offset_x"])),
            "tool_offset_y": float(data.get("tool_offset_y", _OFFSET_DEFAULTS["tool_offset_y"])),
            "tool_offset_z": float(data.get("tool_offset_z", _OFFSET_DEFAULTS["tool_offset_z"])),
        }
    except Exception:
        return dict(_OFFSET_DEFAULTS)


def _save_offset_config(x, y, z):
    os.makedirs(_OFFSET_CONFIG_DIR, exist_ok=True)
    with open(_OFFSET_CONFIG_PATH, "w") as f:
        json.dump({"tool_offset_x": x, "tool_offset_y": y, "tool_offset_z": z}, f, indent=2)


class _ZoomableGraphicsView(QtWidgets.QGraphicsView):
    zoom_changed = QtCore.pyqtSignal(float)
    ZOOM_MIN = 0.1
    ZOOM_MAX = 20.0

    def wheelEvent(self, event):
        factor = 1.15
        if event.angleDelta().y() > 0:
            zoom = factor
        else:
            zoom = 1.0 / factor
        new_scale = self.transform().m11() * zoom
        if new_scale < self.ZOOM_MIN or new_scale > self.ZOOM_MAX:
            return
        self.scale(zoom, zoom)
        self.zoom_changed.emit(self.transform().m11())


class _LayerViewerDialog(QtWidgets.QDialog):
    def __init__(self, npz_dir: str, parent=None):
        super().__init__(parent)
        self._npz_dir = npz_dir
        self._images: list[Path] = []
        self._index = 0
        self._zoom = 1.0

        self.setWindowTitle(f"层预览 - {Path(npz_dir).name}")
        self.resize(900, 700)

        self._scan_images()
        self._build_ui()
        self._show_current()

    def _scan_images(self):
        preview_dir = Path(self._npz_dir) / "layer_previews"
        if not preview_dir.is_dir():
            return
        pattern = re.compile(r"layer_(\d+)\.png$")
        files = []
        for f in sorted(preview_dir.iterdir()):
            m = pattern.match(f.name)
            if m:
                files.append((int(m.group(1)), f))
        files.sort(key=lambda x: x[0])
        self._images = [f[1] for f in files]

    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # Top bar: prev / label / next
        top_bar = QtWidgets.QHBoxLayout()
        self._btn_prev = QtWidgets.QPushButton("← 上一层")
        self._btn_prev.setFixedWidth(100)
        self._btn_prev.setCursor(QtCore.Qt.PointingHandCursor)
        self._btn_next = QtWidgets.QPushButton("下一层 →")
        self._btn_next.setFixedWidth(100)
        self._btn_next.setCursor(QtCore.Qt.PointingHandCursor)
        self._label_index = QtWidgets.QLabel("")
        self._label_index.setAlignment(QtCore.Qt.AlignCenter)
        top_bar.addWidget(self._btn_prev)
        top_bar.addStretch()
        top_bar.addWidget(self._label_index)
        top_bar.addStretch()
        top_bar.addWidget(self._btn_next)
        layout.addLayout(top_bar)

        # Image area
        self._scene = QtWidgets.QGraphicsScene(self)
        self._view = _ZoomableGraphicsView(self._scene)
        self._view.setDragMode(QtWidgets.QGraphicsView.ScrollHandDrag)
        self._view.setRenderHints(
            QtGui.QPainter.Antialiasing | QtGui.QPainter.SmoothPixmapTransform
        )
        self._view.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)
        layout.addWidget(self._view, 1)

        # Bottom bar: zoom label + reset + close
        bottom_bar = QtWidgets.QHBoxLayout()
        self._label_zoom = QtWidgets.QLabel("缩放: 100%")
        btn_reset = QtWidgets.QPushButton("重置视图")
        btn_reset.setCursor(QtCore.Qt.PointingHandCursor)
        btn_close = QtWidgets.QPushButton("关闭")
        btn_close.setCursor(QtCore.Qt.PointingHandCursor)
        bottom_bar.addWidget(self._label_zoom)
        bottom_bar.addStretch()
        bottom_bar.addWidget(btn_reset)
        bottom_bar.addWidget(btn_close)
        layout.addLayout(bottom_bar)

        # Connections
        self._btn_prev.clicked.connect(self._on_prev)
        self._btn_next.clicked.connect(self._on_next)
        btn_reset.clicked.connect(self._on_reset)
        btn_close.clicked.connect(self.accept)
        self._view.zoom_changed.connect(self._on_zoom_changed)

    def _show_current(self):
        if not self._images:
            self._label_index.setText("未找到图像")
            self._btn_prev.setEnabled(False)
            self._btn_next.setEnabled(False)
            return
        total = len(self._images)
        self._label_index.setText(f"层 {self._index + 1} / {total}")
        self._btn_prev.setEnabled(self._index > 0)
        self._btn_next.setEnabled(self._index < total - 1)

        pixmap = QtGui.QPixmap(str(self._images[self._index]))
        if pixmap.isNull():
            self._label_index.setText("图像加载失败")
            return
        self._scene.clear()
        self._scene.addPixmap(pixmap)
        self._scene.setSceneRect(QtCore.QRectF(pixmap.rect()))
        self._on_reset()

    def _on_prev(self):
        if self._index > 0:
            self._index -= 1
            self._show_current()

    def _on_next(self):
        if self._index < len(self._images) - 1:
            self._index += 1
            self._show_current()

    def _on_reset(self):
        self._view.fitInView(self._scene.sceneRect(), QtCore.Qt.KeepAspectRatio)
        self._zoom = self._view.transform().m11()
        self._label_zoom.setText(f"缩放: {self._zoom * 100:.0f}%")

    def _on_zoom_changed(self, scale):
        self._zoom = scale
        self._label_zoom.setText(f"缩放: {scale * 100:.0f}%")

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Left:
            self._on_prev()
        elif event.key() == QtCore.Qt.Key_Right:
            self._on_next()
        else:
            super().keyPressEvent(event)


class _UiStatusWidget(QtWidgets.QWidget):
    status_received = QtCore.pyqtSignal(object)
    scale_submit = QtCore.pyqtSignal(float)
    command_submit = QtCore.pyqtSignal(str)
    uart_command_submit = QtCore.pyqtSignal(str)
    export_finished = QtCore.pyqtSignal(bool, str)  # (success, message)
    export_progress = QtCore.pyqtSignal(str)  # status text
    export_progress_val = QtCore.pyqtSignal(int)  # percentage (0-100)
    rsi_xml_received = QtCore.pyqtSignal(str)  # RSI 发出 XML 日志
    uart_log_received = QtCore.pyqtSignal(str)  # UART 原始日志

    def __init__(self):
        super().__init__()
        self._extrude_scale_current = 1.0
        self._last_npz_dir = None
        self._build_ui()
        self.status_received.connect(self._update_ui)
        self.rsi_xml_received.connect(self._on_rsi_xml)
        self.uart_log_received.connect(self._on_uart_log)
        self.export_progress_val.connect(self._on_export_progress_val)
        
        self._align_timer = QtCore.QTimer(self)
        self._align_timer.timeout.connect(self._dynamic_align)
        self._align_timer.start(50)

    def _dynamic_align(self):
        if not hasattr(self, '_system_box'): return
        h1 = self._col1_layout.sizeHint().height()
        h0_widgets = [self._kuka_box, self._rsi_log_box, self._traj_box]
        h0_rest = sum(w.height() for w in h0_widgets)
        h0_rest += self._col0_layout.spacing() * len(h0_widgets)
        target_h = h1 - h0_rest
        if target_h > 40 and abs(self._system_box.height() - target_h) > 1:
            self._system_box.setFixedHeight(target_h)

    def _build_ui(self):
        layout = QtWidgets.QGridLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(6)
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 1)

        self._labels = {}
        value_min_width = QtWidgets.QLabel("0").fontMetrics().horizontalAdvance("0") * 5
        cf_labels = [
            ("Carbon Fiber State", "当前状态"),
            ("Carbon Fiber Fan OK", "风扇状态"),
            ("Carbon Fiber Current Temp", "当前温度"),
            ("Carbon Fiber Target Temp", "目标温度"),
        ]
        resin_labels = [
            ("Resin State", "当前状态"),
            ("Resin Fan OK", "风扇状态"),
            ("Resin Current Temp", "当前温度"),
            ("Resin Target Temp", "目标温度"),
        ]
        label_metrics = QtWidgets.QLabel("X").fontMetrics()
        cf_resin_label_titles = [title for _, title in (cf_labels + resin_labels)]
        cf_resin_label_min_width = max(
            label_metrics.horizontalAdvance(text) for text in cf_resin_label_titles
        )

        title = QtWidgets.QLabel("系统控制面板")
        title.setObjectName("titleLabel")
        title.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        title.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        layout.addWidget(title, 0, 0, 1, 3)

        def add_group(
            group_title,
            rows,
            parent_layout=None,
            object_name=None,
            add_to_layout=True,
            value_alignment=QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter,
            label_min_width=None,
        ):
            group_box = QtWidgets.QGroupBox(group_title)
            if object_name:
                group_box.setObjectName(object_name)
            
            form = QtWidgets.QFormLayout(group_box)
            form.setLabelAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
            form.setFormAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
            form.setFieldGrowthPolicy(QtWidgets.QFormLayout.AllNonFixedFieldsGrow)
            form.setHorizontalSpacing(12)
            form.setVerticalSpacing(6)
            for row_title in rows:
                if isinstance(row_title, tuple):
                    key, title = row_title
                else:
                    key, title = row_title, row_title
                label_title = QtWidgets.QLabel(title)
                label_title.setObjectName("fieldLabel")
                if label_min_width is not None:
                    label_title.setMinimumWidth(label_min_width)
                label_value = QtWidgets.QLabel("-")
                label_value.setObjectName("valueLabel")
                label_value.setAlignment(value_alignment)
                label_value.setMinimumWidth(value_min_width)
                label_value.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
                form.addRow(label_title, label_value)
                self._labels[key] = label_value

            if parent_layout is not None:
                parent_layout.addWidget(group_box)
            elif add_to_layout:
                layout.addWidget(group_box)
            return group_box

        col0_layout = QtWidgets.QVBoxLayout()
        col0_layout.setSpacing(6)
        self._col0_layout = col0_layout
        col1_layout = QtWidgets.QVBoxLayout()
        col1_layout.setSpacing(6)
        self._col1_layout = col1_layout
        col2_layout = QtWidgets.QVBoxLayout()
        col2_layout.setSpacing(6)

        system_box = QtWidgets.QGroupBox("系统状态")
        system_box.setObjectName("groupSystem")
        system_box.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        self._system_box = system_box
        system_layout = QtWidgets.QVBoxLayout(system_box)
        system_layout.setSpacing(0)
        system_layout.setContentsMargins(4, 12, 4, 12)
        
        sys_val = _AutoScaleLabel("离线")
        sys_val.setObjectName("valueLabel")
        sys_val.setAlignment(QtCore.Qt.AlignCenter)
        sys_val.setMinimumHeight(40)
        
        system_layout.addWidget(sys_val, 1)
        
        self._labels["System State"] = sys_val
        col0_layout.addWidget(system_box)

        kuka_box = QtWidgets.QGroupBox("KUKA 实时位姿")
        self._kuka_box = kuka_box
        kuka_box.setObjectName("groupKuka")
        kuka_box.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        kuka_layout = QtWidgets.QGridLayout(kuka_box)
        kuka_layout.setSpacing(6)
        for i, axis in enumerate(("X", "Y", "Z", "A", "B", "C")):
            axis_widget = QtWidgets.QWidget()
            axis_layout = QtWidgets.QVBoxLayout(axis_widget)
            axis_layout.setContentsMargins(2, 2, 2, 2)
            axis_layout.setSpacing(2)
            axis_label = QtWidgets.QLabel(axis)
            axis_label.setObjectName("axisLabel")
            axis_label.setProperty("axis", axis)
            axis_label.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
            axis_value = QtWidgets.QLabel("-")
            axis_value.setObjectName("axisValue")
            axis_value.setProperty("axis", axis)
            axis_value.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
            axis_value.setMinimumWidth(value_min_width)
            axis_value.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            axis_layout.addWidget(axis_label)
            axis_layout.addWidget(axis_value)
            kuka_layout.addWidget(axis_widget, i // 3, i % 3)
            self._labels[f"KUKA {axis}"] = axis_value
        col0_layout.addWidget(kuka_box)
        
        traj_box = QtWidgets.QGroupBox("RSI 节点")
        self._traj_box = traj_box
        traj_box.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        traj_layout = QtWidgets.QVBoxLayout(traj_box)
        traj_layout.setSpacing(6)
        add_group("概览", [
            ("Traj Backlog", "待发送"),
            ("Next Traj Seq", "下一帧UDP发送"),
        ], parent_layout=traj_layout)
        traj_row = QtWidgets.QHBoxLayout()
        traj_row.setSpacing(8)
        add_group("当前帧UDP发送", [
            ("Traj Seq", "序号"),
            ("Traj Tool", "工具"),
            ("Traj X", "X"),
            ("Traj Y", "Y"),
            ("Traj Z", "Z"),
            ("Traj A", "A"),
            ("Traj B", "B"),
            ("Traj C", "C"),
            ("Traj E", "E"),
        ], parent_layout=traj_row)
        add_group("下一帧UDP发送", [
            ("Traj Seq (Next)", "序号"),
            ("Traj Tool (Next)", "工具"),
            ("Traj X (Next)", "X"),
            ("Traj Y (Next)", "Y"),
            ("Traj Z (Next)", "Z"),
            ("Traj A (Next)", "A"),
            ("Traj B (Next)", "B"),
            ("Traj C (Next)", "C"),
            ("Traj E (Next)", "E"),
        ], parent_layout=traj_row)
        traj_row.setStretch(0, 1)
        traj_row.setStretch(1, 1)
        traj_layout.addLayout(traj_row)
        col0_layout.addWidget(traj_box)

        # ======== RSI 日志区域 ========
        rsi_log_box = QtWidgets.QGroupBox("RSI 日志")
        rsi_log_box.setObjectName("groupRsiLog")
        rsi_log_box.setSizePolicy(
            QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum
        )
        rsi_log_layout = QtWidgets.QVBoxLayout(rsi_log_box)
        rsi_log_layout.setContentsMargins(4, 8, 4, 4)
        rsi_log_layout.setSpacing(2)

        self._rsi_log_text = QtWidgets.QPlainTextEdit()
        self._rsi_log_text.setReadOnly(True)
        self._rsi_log_text.setMaximumBlockCount(30)
        self._rsi_log_text.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self._rsi_log_text.setStyleSheet(
            "QPlainTextEdit {"
            "  background: #1e1e1e;"
            "  color: #d4d4d4;"
            "  font-family: 'Courier New', 'Noto Mono', monospace;"
            "  font-size: 11px;"
            "  border: 1px solid #3c3c3c;"
            "  border-radius: 4px;"
            "  padding: 4px;"
            "}"
        )
        rsi_log_layout.addWidget(self._rsi_log_text)

        self._rsi_log_text.setMaximumHeight(100)

        self._rsi_log_last_xml = ""
        self._rsi_log_dup_count = 0
        col0_layout.addWidget(rsi_log_box)
        self._rsi_log_box = rsi_log_box

        # ======== Column 1: Printhead ========
        
        ph_overview_box = QtWidgets.QGroupBox("Uart 节点")
        ph_overview_box.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        ph_overview_layout = QtWidgets.QVBoxLayout(ph_overview_box)
        ph_overview_layout.setSpacing(6)
        
        events_summary_box = QtWidgets.QGroupBox("事件概览")
        events_summary_box.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        events_summary_layout = QtWidgets.QHBoxLayout(events_summary_box)
        events_summary_layout.setSpacing(12)
        
        lbl1 = QtWidgets.QLabel("下一序号")
        lbl1.setObjectName("fieldLabel")
        val1 = QtWidgets.QLabel("-")
        val1.setObjectName("valueLabel")
        val1.setMinimumWidth(value_min_width)
        val1.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        events_summary_layout.addWidget(lbl1)
        events_summary_layout.addWidget(val1)
        
        lbl2 = QtWidgets.QLabel("待处理")
        lbl2.setObjectName("fieldLabel")
        val2 = QtWidgets.QLabel("-")
        val2.setObjectName("valueLabel")
        val2.setMinimumWidth(value_min_width)
        val2.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        events_summary_layout.addStretch(1)
        
        events_summary_layout.addWidget(lbl2)
        events_summary_layout.addWidget(val2)
        events_summary_layout.addStretch(1)
        ph_overview_layout.addWidget(events_summary_box)
        
        self._labels["Next Event Seq"] = val1
        self._labels["Events Pending"] = val2
        
        events_row = QtWidgets.QHBoxLayout()
        events_row.setSpacing(8)
        add_group("当前事件", [
            ("Event Type", "类型"),
            ("Event Payload", "载荷"),
            ("Event Src Line", "源码行"),
            ("Event Trigger Seq", "触发序号"),
        ], parent_layout=events_row)
        add_group("下一事件", [
            ("Event Type (Next)", "类型"),
            ("Event Payload (Next)", "载荷"),
            ("Event Src Line (Next)", "源码行"),
            ("Event Trigger Seq (Next)", "触发序号"),
        ], parent_layout=events_row)
        events_row.setStretch(0, 1)
        events_row.setStretch(1, 1)
        ph_overview_layout.addLayout(events_row)
        
        col1_layout.addWidget(ph_overview_box)
        
        ph_tools_box = QtWidgets.QGroupBox("工具管理")
        ph_tools_box.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        ph_tools_layout = QtWidgets.QVBoxLayout(ph_tools_box)
        ph_tools_layout.setSpacing(8)
        
        tool_row = QtWidgets.QHBoxLayout()
        tool_row.setSpacing(8)

        cur_tool_title = QtWidgets.QLabel("当前工具")
        cur_tool_title.setObjectName("fieldLabel")
        tool_row.addWidget(cur_tool_title)
        self._current_tool_value = QtWidgets.QLabel("-")
        self._current_tool_value.setObjectName("valueLabel")
        tool_row.addWidget(self._current_tool_value)

        sep = QtWidgets.QLabel("│")
        sep.setStyleSheet("color: #5a5a5a; font-size: 16px;")
        sep.setAlignment(QtCore.Qt.AlignCenter)
        tool_row.addWidget(sep)

        switch_title = QtWidgets.QLabel("切换工具")
        switch_title.setObjectName("fieldLabel")
        tool_row.addWidget(switch_title)
        self._btn_tool_cf = QtWidgets.QPushButton("碳纤维")
        self._btn_tool_cf.setObjectName("btnToolCF")
        self._btn_tool_cf.setMinimumHeight(24)
        self._btn_tool_cf.setMaximumHeight(24)
        self._btn_tool_cf.setCursor(QtCore.Qt.PointingHandCursor)
        self._btn_tool_resin = QtWidgets.QPushButton("树脂")
        self._btn_tool_resin.setObjectName("btnToolResin")
        self._btn_tool_resin.setMinimumHeight(24)
        self._btn_tool_resin.setMaximumHeight(24)
        self._btn_tool_resin.setCursor(QtCore.Qt.PointingHandCursor)
        tool_row.addWidget(self._btn_tool_cf)
        tool_row.addWidget(self._btn_tool_resin)
        tool_row.addStretch(1)

        ph_tools_layout.addLayout(tool_row)
        
        extrude_row_widget = QtWidgets.QWidget()
        extrude_row_widget.setFixedHeight(26)
        extrude_inner = QtWidgets.QHBoxLayout(extrude_row_widget)
        extrude_inner.setContentsMargins(0, 0, 0, 0)
        extrude_inner.setSpacing(10)
        extrude_cur_label = QtWidgets.QLabel("当前挤出倍率")
        extrude_cur_label.setObjectName("fieldLabel")
        self._extrude_scale_value = QtWidgets.QLabel("1.000")
        self._extrude_scale_value.setObjectName("valueLabel")
        self._extrude_scale_value.setMinimumWidth(value_min_width)
        extrude_set_label = QtWidgets.QLabel("设置")
        extrude_set_label.setObjectName("fieldLabel")
        self._extrude_scale_input = QtWidgets.QLineEdit()
        self._extrude_scale_input.setPlaceholderText("1.0")
        self._extrude_scale_input.setMaximumWidth(80)
        validator = QtGui.QDoubleValidator(0.001, 1000.0, 3, self._extrude_scale_input)
        validator.setNotation(QtGui.QDoubleValidator.StandardNotation)
        self._extrude_scale_input.setValidator(validator)
        self._extrude_scale_apply = QtWidgets.QPushButton("应用")
        self._extrude_scale_apply.setObjectName("btnTempApply_extrude")
        self._extrude_scale_apply.setCursor(QtCore.Qt.PointingHandCursor)
        self._extrude_scale_status = QtWidgets.QLabel("-")
        self._extrude_scale_status.setObjectName("valueLabel")
        extrude_inner.addWidget(extrude_cur_label)
        extrude_inner.addWidget(self._extrude_scale_value)
        extrude_inner.addSpacing(8)
        extrude_inner.addWidget(extrude_set_label)
        extrude_inner.addWidget(self._extrude_scale_input)
        extrude_inner.addWidget(self._extrude_scale_apply)
        extrude_inner.addSpacing(8)
        extrude_inner.addWidget(self._extrude_scale_status, 1)
        ph_tools_layout.addWidget(extrude_row_widget)
        
        head_panels_row = QtWidgets.QHBoxLayout()
        head_panels_row.setSpacing(12)
        
        for head_id, head_name in (("cf", "碳纤维"), ("resin", "树脂")):
            master_panel = QtWidgets.QGroupBox(head_name)
            master_panel.setObjectName(f"groupMaster{head_id}")
            master_panel.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Maximum)
            master_layout = QtWidgets.QVBoxLayout(master_panel)
            master_layout.setSpacing(6)
            
            status_box = add_group("概况", cf_labels if head_id == "cf" else resin_labels, add_to_layout=False,
                label_min_width=cf_resin_label_min_width,
                value_alignment=QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            master_layout.addWidget(status_box)
            
            ctrl_box = QtWidgets.QGroupBox("控制")
            ctrl_layout = QtWidgets.QVBoxLayout(ctrl_box)
            ctrl_layout.setSpacing(6)
            
            fan_row = QtWidgets.QHBoxLayout()
            fan_row.setSpacing(8)
            fan_label = QtWidgets.QLabel("风扇")
            fan_label.setObjectName("fieldLabel")
            fan_label.setMinimumWidth(30)
            btn_fan_on = QtWidgets.QPushButton("开")
            btn_fan_on.setObjectName(f"btnFanOn_{head_id}")
            btn_fan_on.setMinimumHeight(24)
            btn_fan_on.setMaximumHeight(24)
            btn_fan_on.setCursor(QtCore.Qt.PointingHandCursor)
            btn_fan_off = QtWidgets.QPushButton("关")
            btn_fan_off.setObjectName(f"btnFanOff_{head_id}")
            btn_fan_off.setMinimumHeight(24)
            btn_fan_off.setMaximumHeight(24)
            btn_fan_off.setCursor(QtCore.Qt.PointingHandCursor)
            fan_row.addWidget(fan_label)
            fan_row.addWidget(btn_fan_on)
            fan_row.addWidget(btn_fan_off)
            ctrl_layout.addLayout(fan_row)
            
            temp_row = QtWidgets.QHBoxLayout()
            temp_row.setSpacing(8)
            temp_label = QtWidgets.QLabel("温度")
            temp_label.setObjectName("fieldLabel")
            temp_label.setMinimumWidth(30)
            temp_input = QtWidgets.QLineEdit()
            temp_input.setPlaceholderText("°C")
            temp_input.setMaximumWidth(80)
            temp_validator = QtGui.QDoubleValidator(0.0, 500.0, 1, temp_input)
            temp_validator.setNotation(QtGui.QDoubleValidator.StandardNotation)
            temp_input.setValidator(temp_validator)
            btn_temp_apply = QtWidgets.QPushButton("设定")
            btn_temp_apply.setObjectName(f"btnTempApply_{head_id}")
            btn_temp_apply.setMinimumHeight(24)
            btn_temp_apply.setMaximumHeight(24)
            btn_temp_apply.setCursor(QtCore.Qt.PointingHandCursor)
            temp_row.addWidget(temp_label)
            temp_row.addWidget(temp_input, 1)
            temp_row.addWidget(btn_temp_apply)
            ctrl_layout.addLayout(temp_row)
            
            master_layout.addWidget(ctrl_box)
            head_panels_row.addWidget(master_panel)
            
            setattr(self, f"_btn_fan_on_{head_id}", btn_fan_on)
            setattr(self, f"_btn_fan_off_{head_id}", btn_fan_off)
            setattr(self, f"_temp_input_{head_id}", temp_input)
            setattr(self, f"_btn_temp_apply_{head_id}", btn_temp_apply)
            
        ph_tools_layout.addLayout(head_panels_row)
        col1_layout.addWidget(ph_tools_box)

        # ======== UART 日志区域 ========
        uart_log_box = QtWidgets.QGroupBox("UART 日志")
        uart_log_box.setObjectName("groupUartLog")
        uart_log_box.setSizePolicy(
            QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum
        )
        uart_log_layout = QtWidgets.QVBoxLayout(uart_log_box)
        uart_log_layout.setContentsMargins(4, 8, 4, 4)
        uart_log_layout.setSpacing(2)

        self._uart_log_text = QtWidgets.QPlainTextEdit()
        self._uart_log_text.setReadOnly(True)
        self._uart_log_text.setMaximumBlockCount(30)
        self._uart_log_text.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self._uart_log_text.setStyleSheet(
            "QPlainTextEdit {"
            "  background: #1e1e1e;"
            "  color: #d4d4d4;"
            "  font-family: 'Courier New', 'Noto Mono', monospace;"
            "  font-size: 11px;"
            "  border: 1px solid #3c3c3c;"
            "  border-radius: 4px;"
            "  padding: 4px;"
            "}"
        )
        uart_log_layout.addWidget(self._uart_log_text)
        self._uart_log_text.setMaximumHeight(100)

        col1_layout.addWidget(uart_log_box)
        col1_layout.addStretch(1)

        # ======== Print Control 区域 ========
        control_box = QtWidgets.QGroupBox("打印控制")
        control_box.setObjectName("groupControl")
        control_box.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        control_layout = QtWidgets.QVBoxLayout(control_box)
        control_layout.setSpacing(8)

        btn_row = QtWidgets.QHBoxLayout()
        btn_row.setSpacing(12)

        self._btn_pause = QtWidgets.QPushButton("暂停")
        self._btn_pause.setObjectName("btnPause")
        self._btn_pause.setMinimumHeight(36)
        self._btn_pause.setCursor(QtCore.Qt.PointingHandCursor)

        self._btn_resume = QtWidgets.QPushButton("继续")
        self._btn_resume.setObjectName("btnResume")
        self._btn_resume.setMinimumHeight(36)
        self._btn_resume.setCursor(QtCore.Qt.PointingHandCursor)
        self._btn_resume.setEnabled(False)

        self._btn_stop = QtWidgets.QPushButton("停止")
        self._btn_stop.setObjectName("btnStop")
        self._btn_stop.setMinimumHeight(36)
        self._btn_stop.setCursor(QtCore.Qt.PointingHandCursor)

        btn_row.addWidget(self._btn_pause)
        btn_row.addWidget(self._btn_resume)
        btn_row.addWidget(self._btn_stop)
        control_layout.addLayout(btn_row)



        # Wait to add control_box until after launch_box

        # ======== GCode Export 区域 ========
        export_box = QtWidgets.QGroupBox("GCode 导出")
        export_box.setObjectName("groupExport")
        export_box.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        export_layout = QtWidgets.QVBoxLayout(export_box)
        export_layout.setSpacing(6)

        # Subtitle: Tool Offset
        offset_subtitle = QtWidgets.QLabel("工具偏移")
        offset_subtitle.setStyleSheet("font-weight: bold; color: #1a73e8; font-size: 12px; margin-top: 2px;")
        export_layout.addWidget(offset_subtitle)

        # Tool Offset Description & Input Fields (placed inside GCode Export)
        offset_desc = QtWidgets.QLabel(
            "切换到工具 1（碳纤维）时的移动量，\n"
            "相对于工具 2（树脂）基准位置。"
        )
        offset_desc.setObjectName("fieldLabel")
        offset_desc.setWordWrap(True)
        export_layout.addWidget(offset_desc)

        offset_cfg = _load_offset_config()
        offset_grid = QtWidgets.QHBoxLayout()
        offset_grid.setSpacing(8)
        self._offset_spins = {}
        for axis, default_val in [("X", offset_cfg["tool_offset_x"]),
                                   ("Y", offset_cfg["tool_offset_y"]),
                                   ("Z", offset_cfg["tool_offset_z"])]:
            axis_w = QtWidgets.QWidget()
            axis_lay = QtWidgets.QVBoxLayout(axis_w)
            axis_lay.setContentsMargins(0, 0, 0, 0)
            axis_lay.setSpacing(2)
            lbl = QtWidgets.QLabel(f"{axis} (mm)")
            lbl.setObjectName("fieldLabel")
            lbl.setAlignment(QtCore.Qt.AlignCenter)
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(-100.0, 100.0)
            spin.setDecimals(2)
            spin.setSingleStep(0.01)
            spin.setValue(default_val)
            spin.setMinimumHeight(28)
            axis_lay.addWidget(lbl)
            axis_lay.addWidget(spin)
            offset_grid.addWidget(axis_w)
            self._offset_spins[axis] = spin
            spin.valueChanged.connect(self._on_offset_changed)
        export_layout.addLayout(offset_grid)

        self._offset_status = QtWidgets.QLabel("已加载配置。")
        self._offset_status.setObjectName("fieldLabel")
        export_layout.addWidget(self._offset_status)

        # Visual Separator Line
        separator = QtWidgets.QFrame()
        separator.setFrameShape(QtWidgets.QFrame.HLine)
        separator.setFrameShadow(QtWidgets.QFrame.Sunken)
        export_layout.addWidget(separator)

        # Subtitle: GCode File
        gcode_subtitle = QtWidgets.QLabel("GCode 文件")
        gcode_subtitle.setStyleSheet("font-weight: bold; color: #1a73e8; font-size: 12px; margin-top: 4px;")
        export_layout.addWidget(gcode_subtitle)

        # GCode file selector
        gcode_row = QtWidgets.QHBoxLayout()
        gcode_row.setSpacing(4)
        gcode_lbl = QtWidgets.QLabel("GCode")
        gcode_lbl.setObjectName("fieldLabel")
        gcode_lbl.setMinimumWidth(50)
        self._gcode_path_input = QtWidgets.QLineEdit()
        self._gcode_path_input.setPlaceholderText("选择 .gcode 文件...")
        self._btn_browse_gcode = QtWidgets.QPushButton("…")
        self._btn_browse_gcode.setFixedWidth(32)
        self._btn_browse_gcode.setFixedHeight(28)
        self._btn_browse_gcode.setCursor(QtCore.Qt.PointingHandCursor)
        self._btn_browse_gcode.setObjectName("btnBrowseGcode")
        gcode_row.addWidget(gcode_lbl)
        gcode_row.addWidget(self._gcode_path_input, 1)
        gcode_row.addWidget(self._btn_browse_gcode)
        export_layout.addLayout(gcode_row)

        # NPZ output path (internal, hidden from UI)
        self._npz_out_input = QtWidgets.QLineEdit()
        self._npz_out_input.setPlaceholderText("根据 GCode 文件名自动生成")

        # Planner settings (collapsible)
        planner_toggle = QtWidgets.QPushButton("▶ 规划器设置")
        planner_toggle.setObjectName("btnPlannerToggle")
        planner_toggle.setMinimumHeight(28)
        planner_toggle.setCursor(QtCore.Qt.PointingHandCursor)
        planner_toggle.setCheckable(True)
        export_layout.addWidget(planner_toggle)

        planner_container = QtWidgets.QWidget()
        planner_container.setVisible(False)
        planner_form = QtWidgets.QFormLayout(planner_container)
        planner_form.setLabelAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        planner_form.setFieldGrowthPolicy(QtWidgets.QFormLayout.AllNonFixedFieldsGrow)
        planner_form.setHorizontalSpacing(8)
        planner_form.setVerticalSpacing(4)

        _PLANNER_PARAMS = [
            ("dt", "0.004", "采样周期（s）"),
            ("default_feed_mm_s", "10.0", "默认进给速度（mm/s）"),
            ("corner_angle_deg", "10.0", "拐角角度阈值（°）"),
            ("corner_retreat_ratio", "0.2", "拐角回退比例"),
            ("density", "0", "点密度倍率"),
            ("degree", "3", "B 样条阶数"),
            ("max_fit_points_per_segment", "20000", "每段最大拟合点数"),
            ("export_sleep_ms", "0", "导出节流休眠（ms）"),
            ("export_yield_every", "0", "导出 yield 间隔"),
            ("split_by_layer_type", "true", "按层+类型拆分 NPZ"),
            ("plot_layer_xy", "true", "每层生成 XY 路径图"),
            ("plot_stride", "5", "绘图采样步长"),
        ]
        self._planner_inputs = {}
        for param_name, default_val, desc in _PLANNER_PARAMS:
            lbl = QtWidgets.QLabel(param_name)
            lbl.setObjectName("fieldLabel")
            lbl.setToolTip(desc)
            if param_name in ("split_by_layer_type", "plot_layer_xy"):
                inp = QtWidgets.QCheckBox()
                inp.setChecked(default_val.lower() == "true")
                inp.setToolTip(desc)
                planner_form.addRow(lbl, inp)
            else:
                inp = QtWidgets.QLineEdit(default_val)
                inp.setToolTip(desc)
                planner_form.addRow(lbl, inp)
            self._planner_inputs[param_name] = inp

        export_layout.addWidget(planner_container)
        self._planner_container = planner_container

        def _toggle_planner(checked):
            planner_container.setVisible(checked)
            planner_toggle.setText(
                "▼ 规划器设置" if checked else "▶ 规划器设置"
            )
        planner_toggle.toggled.connect(_toggle_planner)

        # Export button + progress
        export_btn_row = QtWidgets.QHBoxLayout()
        export_btn_row.setSpacing(8)
        self._btn_export_npz = QtWidgets.QPushButton("导出 NPZ")
        self._btn_export_npz.setObjectName("btnExportNpz")
        self._btn_export_npz.setMinimumHeight(36)
        self._btn_export_npz.setCursor(QtCore.Qt.PointingHandCursor)
        export_btn_row.addWidget(self._btn_export_npz)
        export_layout.addLayout(export_btn_row)

        # View Layer Images button
        view_row = QtWidgets.QHBoxLayout()
        view_row.setSpacing(8)
        self._btn_view_layers = QtWidgets.QPushButton("查看层图像")
        self._btn_view_layers.setObjectName("btnViewLayers")
        self._btn_view_layers.setMinimumHeight(36)
        self._btn_view_layers.setCursor(QtCore.Qt.PointingHandCursor)
        self._btn_view_layers.setEnabled(False)
        view_row.addWidget(self._btn_view_layers)
        export_layout.addLayout(view_row)

        self._export_progress = QtWidgets.QProgressBar()
        self._export_progress.setMinimumHeight(18)
        self._export_progress.setRange(0, 0)  # indeterminate
        self._export_progress.setVisible(False)
        export_layout.addWidget(self._export_progress)

        self._export_status = QtWidgets.QLabel("")
        self._export_status.setObjectName("fieldLabel")
        self._export_status.setWordWrap(True)
        export_layout.addWidget(self._export_status)

        # Connect export signals
        self._btn_browse_gcode.clicked.connect(self._on_browse_gcode)
        self._gcode_path_input.textChanged.connect(self._on_gcode_path_changed)
        self._btn_export_npz.clicked.connect(self._on_export_npz)
        self.export_finished.connect(self._on_export_finished)
        self.export_progress.connect(self._on_export_progress)
        self._btn_view_layers.clicked.connect(self._on_view_layers)


        # ======== Launch Control 区域 ========
        launch_box = QtWidgets.QGroupBox("启动")
        launch_box.setObjectName("groupLaunch")
        launch_box.setSizePolicy(
            QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum
        )
        launch_inner = QtWidgets.QVBoxLayout(launch_box)
        launch_inner.setSpacing(8)

        launch_btn_row = QtWidgets.QHBoxLayout()
        launch_btn_row.setSpacing(12)

        self._btn_launch_settings = QtWidgets.QPushButton("设置")
        self._btn_launch_settings.setObjectName("btnLaunchSettings")
        self._btn_launch_settings.setMinimumHeight(36)
        self._btn_launch_settings.setCursor(QtCore.Qt.PointingHandCursor)

        self._btn_launch = QtWidgets.QPushButton("启动")
        self._btn_launch.setObjectName("btnLaunch")
        self._btn_launch.setMinimumHeight(36)
        self._btn_launch.setCursor(QtCore.Qt.PointingHandCursor)

        self._btn_stop_launch = QtWidgets.QPushButton("停止节点")
        self._btn_stop_launch.setObjectName("btnStopLaunch")
        self._btn_stop_launch.setMinimumHeight(36)
        self._btn_stop_launch.setCursor(QtCore.Qt.PointingHandCursor)
        self._btn_stop_launch.setEnabled(False)

        launch_btn_row.addWidget(self._btn_launch_settings)
        launch_btn_row.addWidget(self._btn_launch)
        launch_btn_row.addWidget(self._btn_stop_launch)
        launch_inner.addLayout(launch_btn_row)

        launch_status_row = QtWidgets.QHBoxLayout()
        launch_status_row.setSpacing(8)
        launch_label = QtWidgets.QLabel("启动状态")
        launch_label.setObjectName("fieldLabel")
        self._launch_status = QtWidgets.QLabel("未启动")
        self._launch_status.setObjectName("launchStatus")
        launch_status_row.addWidget(launch_label)
        launch_status_row.addWidget(self._launch_status, 1)
        launch_inner.addLayout(launch_status_row)

        # Add all boxes to col2_layout in the desired order
        col2_layout.addWidget(export_box)
        col2_layout.addWidget(launch_box)
        col2_layout.addWidget(control_box)
        col2_layout.addStretch(1)

        col0_layout.addStretch(1)

        layout.addLayout(col0_layout, 1, 0)
        layout.addLayout(col1_layout, 1, 1)
        layout.addLayout(col2_layout, 1, 2)

        layout.setRowStretch(0, 0)
        layout.setRowStretch(1, 1)

        self.setStyleSheet(
            "QWidget { background: #f7f7f7; font-family: 'WenQuanYi Micro Hei', 'Noto Sans CJK SC', 'Microsoft YaHei', sans-serif; }"
            "QGroupBox {"
            "  font-weight: 600;"
            "  margin-top: 4px;"
            "  padding: 10px 8px 8px 8px;"
            "  border: 1px solid #d0d0d0;"
            "  border-radius: 6px;"
            "  background: #ffffff;"
            "}"
            "QGroupBox::title {"
            "  subcontrol-origin: margin;"
            "  subcontrol-position: top left;"
            "  padding: 0 6px;"
            "}"
            "QLabel#titleLabel {"
            "  font-size: 16px;"
            "  font-weight: 700;"
            "  color: #2b2b2b;"
            "}"
            "QLabel#fieldLabel { color: #666666; }"
            "QLabel#valueLabel {"
            "  font-weight: 600;"
            "  color: #222222;"
            "}"
            "QGroupBox#groupHeartbeat::title { color: #b42318; }"
            "QGroupBox#groupKuka::title { color: #b15e00; }"
            "QGroupBox#groupPrintheadCF::title { color: #000000; }"
            "QGroupBox#groupExtrudeScale::title { color: #000000; }"
            "QGroupBox#groupPrintheadResin::title { color: #444444; }"
            "QLabel#axisLabel {"
            "  font-size: 10px;"
            "  color: #666666;"
            "}"
            "QLabel#axisValue {"
            "  font-weight: 600;"
            "  border: 1px solid #d0d0d0;"
            "  border-radius: 4px;"
            "  background: #fafafa;"
            "  padding: 2px 4px;"
            "  color: #666666;"
            "}"
            "QPushButton#btnPause {"
            "  font-weight: 600;"
            "  font-size: 13px;"
            "  border: 1px solid #c0c0c0;"
            "  border-radius: 6px;"
            "  background: #ffffff;"
            "  color: #333333;"
            "  padding: 6px 16px;"
            "}"
            "QPushButton#btnPause:hover {"
            "  background: #f0f0f0;"
            "  border-color: #999999;"
            "}"
            "QPushButton#btnPause:disabled {"
            "  background: #eeeeee;"
            "  color: #aaaaaa;"
            "  border-color: #dddddd;"
            "}"
            "QPushButton#btnResume {"
            "  font-weight: 600;"
            "  font-size: 13px;"
            "  border: 1px solid #34a853;"
            "  border-radius: 6px;"
            "  background: #ffffff;"
            "  color: #1b6e3c;"
            "  padding: 6px 16px;"
            "}"
            "QPushButton#btnResume:hover {"
            "  background: #e6f4ea;"
            "  border-color: #1b6e3c;"
            "}"
            "QPushButton#btnResume:disabled {"
            "  background: #eeeeee;"
            "  color: #aaaaaa;"
            "  border-color: #dddddd;"
            "}"
            "QPushButton#btnStop {"
            "  font-weight: 600;"
            "  font-size: 13px;"
            "  border: 1px solid #d93025;"
            "  border-radius: 6px;"
            "  background: #ffffff;"
            "  color: #b42318;"
            "  padding: 6px 16px;"
            "}"
            "QPushButton#btnStop:hover {"
            "  background: #fce8e6;"
            "  border-color: #b42318;"
            "}"
            "QPushButton#btnStop:disabled {"
            "  background: #eeeeee;"
            "  color: #aaaaaa;"
            "  border-color: #dddddd;"
            "}"
            "QLabel#controlStatus {"
            "  font-weight: 700;"
            "  font-size: 13px;"
            "  color: #2b2b2b;"
            "}"
            "QGroupBox#groupControl::title { color: #1a73e8; }"
            "QGroupBox#groupPrintheadControl::title { color: #1a73e8; }"
            "QGroupBox#groupCtrlcf::title { color: #000000; }"
            "QGroupBox#groupCtrlresin::title { color: #444444; }"
            "QPushButton[objectName^='btnFanOn'] {"
            "  font-weight: 600;"
            "  font-size: 12px;"
            "  border: 1px solid #34a853;"
            "  border-radius: 5px;"
            "  background: #ffffff;"
            "  color: #1b6e3c;"
            "  padding: 4px 14px;"
            "}"
            "QPushButton[objectName^='btnFanOn']:hover {"
            "  background: #e6f4ea;"
            "  border-color: #1b6e3c;"
            "}"
            "QPushButton[objectName^='btnFanOff'] {"
            "  font-weight: 600;"
            "  font-size: 12px;"
            "  border: 1px solid #c0c0c0;"
            "  border-radius: 5px;"
            "  background: #ffffff;"
            "  color: #666666;"
            "  padding: 4px 14px;"
            "}"
            "QPushButton[objectName^='btnFanOff']:hover {"
            "  background: #f0f0f0;"
            "  border-color: #999999;"
            "}"
            "QPushButton[objectName^='btnTempApply'] {"
            "  font-weight: 600;"
            "  font-size: 12px;"
            "  border: 1px solid #1a73e8;"
            "  border-radius: 5px;"
            "  background: #ffffff;"
            "  color: #1a73e8;"
            "  padding: 4px 14px;"
            "}"
            "QPushButton[objectName^='btnTempApply']:hover {"
            "  background: #e8f0fe;"
            "  border-color: #1558b0;"
            "}"
            "QPushButton#btnToolCF, QPushButton#btnToolResin {"
            "  font-weight: 600;"
            "  font-size: 12px;"
            "  border: 1px solid #b15e00;"
            "  border-radius: 5px;"
            "  background: #ffffff;"
            "  color: #b15e00;"
            "  padding: 4px 16px;"
            "}"
            "QPushButton#btnToolCF:hover, QPushButton#btnToolResin:hover {"
            "  background: #fff3e0;"
            "  border-color: #8a4500;"
            "}"
            "QGroupBox#groupPrintheadControl QLineEdit {"
            "  border: 1px solid #d0d0d0;"
            "  border-radius: 4px;"
            "  padding: 4px 6px;"
            "  background: #ffffff;"
            "}"
            "QGroupBox#groupPrintheadControl QLineEdit:focus {"
            "  border-color: #1a73e8;"
            "}"
            "QGroupBox#groupLaunch::title { color: #1a73e8; }"
            "QPushButton#btnLaunchSettings {"
            "  font-weight: 600;"
            "  font-size: 13px;"
            "  border: 1px solid #c0c0c0;"
            "  border-radius: 6px;"
            "  background: #ffffff;"
            "  color: #333333;"
            "  padding: 6px 16px;"
            "}"
            "QPushButton#btnLaunchSettings:hover {"
            "  background: #f0f0f0;"
            "  border-color: #999999;"
            "}"
            "QPushButton#btnLaunch {"
            "  font-weight: 600;"
            "  font-size: 13px;"
            "  border: 1px solid #1a73e8;"
            "  border-radius: 6px;"
            "  background: #1a73e8;"
            "  color: #ffffff;"
            "  padding: 6px 16px;"
            "}"
            "QPushButton#btnLaunch:hover {"
            "  background: #1558b0;"
            "  border-color: #1558b0;"
            "}"
            "QPushButton#btnLaunch:disabled {"
            "  background: #eeeeee;"
            "  color: #aaaaaa;"
            "  border-color: #dddddd;"
            "}"
            "QPushButton#btnStopLaunch {"
            "  font-weight: 600;"
            "  font-size: 13px;"
            "  border: 1px solid #d93025;"
            "  border-radius: 6px;"
            "  background: #ffffff;"
            "  color: #b42318;"
            "  padding: 6px 16px;"
            "}"
            "QPushButton#btnStopLaunch:hover {"
            "  background: #fce8e6;"
            "  border-color: #b42318;"
            "}"
            "QPushButton#btnStopLaunch:disabled {"
            "  background: #eeeeee;"
            "  color: #aaaaaa;"
            "  border-color: #dddddd;"
            "}"
            "QLabel#launchStatus {"
            "  font-weight: 700;"
            "  font-size: 13px;"
            "  color: #666666;"
            "}"
            "QGroupBox#groupExport QDoubleSpinBox {"
            "  border: 1px solid #d0d0d0;"
            "  border-radius: 4px;"
            "  padding: 4px 6px;"
            "  background: #ffffff;"
            "}"
            "QGroupBox#groupExport QDoubleSpinBox:focus {"
            "  border-color: #1a73e8;"
            "}"
            "QGroupBox#groupExport::title { color: #1a73e8; }"
            "QGroupBox#groupExport QLineEdit {"
            "  border: 1px solid #d0d0d0;"
            "  border-radius: 4px;"
            "  padding: 4px 6px;"
            "  background: #ffffff;"
            "}"
            "QGroupBox#groupExport QLineEdit:focus {"
            "  border-color: #1a73e8;"
            "}"
            "QPushButton#btnBrowseGcode {"
            "  border: 1px solid #1a73e8;"
            "  border-radius: 4px;"
            "  background: #ffffff;"
            "  color: #1a73e8;"
            "  font-weight: 600;"
            "}"
            "QPushButton#btnBrowseGcode:hover {"
            "  background: #e8f0fe;"
            "}"
            "QPushButton#btnPlannerToggle {"
            "  font-weight: 600;"
            "  font-size: 12px;"
            "  border: 1px solid #d0d0d0;"
            "  border-radius: 5px;"
            "  background: #fafafa;"
            "  color: #555555;"
            "  padding: 4px 12px;"
            "  text-align: left;"
            "}"
            "QPushButton#btnPlannerToggle:hover {"
            "  background: #f0f0f0;"
            "  border-color: #999999;"
            "}"
            "QPushButton#btnExportNpz {"
            "  font-weight: 600;"
            "  font-size: 13px;"
            "  border: 1px solid #1a73e8;"
            "  border-radius: 6px;"
            "  background: #1a73e8;"
            "  color: #ffffff;"
            "  padding: 6px 16px;"
            "}"
            "QPushButton#btnExportNpz:hover {"
            "  background: #1558b0;"
            "  border-color: #1558b0;"
            "}"
            "QPushButton#btnExportNpz:disabled {"
            "  background: #eeeeee;"
            "  color: #aaaaaa;"
            "  border-color: #dddddd;"
            "}"
        )

        self._extrude_scale_apply.clicked.connect(self._on_extrude_scale_apply)
        self._extrude_scale_input.returnPressed.connect(self._on_extrude_scale_apply)
        self._btn_pause.clicked.connect(self._on_pause)
        self._btn_resume.clicked.connect(self._on_resume)
        self._btn_stop.clicked.connect(self._on_stop)

        # Printhead control connections
        self._btn_fan_on_cf.clicked.connect(lambda: self._on_fan("cf", True))
        self._btn_fan_off_cf.clicked.connect(lambda: self._on_fan("cf", False))
        self._btn_fan_on_resin.clicked.connect(lambda: self._on_fan("resin", True))
        self._btn_fan_off_resin.clicked.connect(lambda: self._on_fan("resin", False))
        self._btn_temp_apply_cf.clicked.connect(lambda: self._on_temp_apply("cf"))
        self._temp_input_cf.returnPressed.connect(lambda: self._on_temp_apply("cf"))
        self._btn_temp_apply_resin.clicked.connect(lambda: self._on_temp_apply("resin"))
        self._temp_input_resin.returnPressed.connect(lambda: self._on_temp_apply("resin"))
        self._btn_tool_cf.clicked.connect(lambda: self._on_tool_switch("cf"))
        self._btn_tool_resin.clicked.connect(lambda: self._on_tool_switch("resin"))

    def _set_value(self, key, text, color=None):
        label = self._labels.get(key)
        if not label:
            return
        label.setText(text)
        if isinstance(label, _AutoScaleLabel):
            if color:
                label.set_color(color)
        else:
            if color:
                label.setStyleSheet(f"color: {color};")
            else:
                label.setStyleSheet("")

    def _format_tool(self, tool_id):
        if tool_id == 1:
            return "碳纤维"
        if tool_id == 2:
            return "树脂"
        return str(tool_id)

    def _on_extrude_scale_apply(self):
        text = self._extrude_scale_input.text().strip()
        if not text:
            self._set_extrude_status("请输入倍率值。", "#b42318")
            return
        try:
            val = float(text)
        except ValueError:
            self._set_extrude_status("无效倍率值。", "#b42318")
            return
        if not (val > 0.0):
            self._set_extrude_status("倍率必须大于 0。", "#b42318")
            return
        self._set_extrude_status("提交中...", "#b15e00")
        self.scale_submit.emit(val)

    def _set_extrude_status(self, text, color=None):
        self._extrude_scale_status.setText(text)
        if color:
            self._extrude_scale_status.setStyleSheet(f"color: {color};")
        else:
            self._extrude_scale_status.setStyleSheet("")

    def set_extrude_scale(self, value, status_text=None, status_color=None):
        self._extrude_scale_current = value
        self._extrude_scale_value.setText(f"{value:.3f}")
        if status_text is not None:
            self._set_extrude_status(status_text, status_color)

    def set_extrude_status(self, text, color=None):
        self._set_extrude_status(text, color)

    def current_extrude_scale(self):
        return self._extrude_scale_current

    def _update_ui(self, msg: UiStatus):
        state_str = msg.state or "-"
        state_color = self._STATE_COLORS.get(state_str, "#a0a0a0")
        self._set_value("System State", state_str, state_color)
        self._update_control_buttons(msg.state or "")


        if msg.printhead_status_valid:
            ps = msg.printhead_status
            self._current_tool_value.setText(self._format_tool(ps.current_tool))

            using_color = "#1b6e3c"
            cf_state = "使用中" if ps.current_tool == 1 else "空闲"
            resin_state = "使用中" if ps.current_tool == 2 else "空闲"
            self._set_value("Carbon Fiber State", cf_state, using_color if cf_state == "使用中" else "#2b2b2b")
            self._set_value("Resin State", resin_state, using_color if resin_state == "使用中" else "#2b2b2b")

            cf_fan_color = "#1b6e3c" if ps.fan_ok_cf else "#b42318"
            self._set_value("Carbon Fiber Fan OK", "开" if ps.fan_ok_cf else "关", cf_fan_color)
            self._set_value("Carbon Fiber Current Temp", f"{ps.current_temp_cf:.1f}", "#2b2b2b")
            self._set_value("Carbon Fiber Target Temp", f"{ps.target_temp_cf:.1f}", "#2b2b2b")

            resin_fan_color = "#1b6e3c" if ps.fan_ok_resin else "#b42318"
            self._set_value("Resin Fan OK", "开" if ps.fan_ok_resin else "关", resin_fan_color)
            self._set_value("Resin Current Temp", f"{ps.current_temp_resin:.1f}", "#2b2b2b")
            self._set_value("Resin Target Temp", f"{ps.target_temp_resin:.1f}", "#2b2b2b")
        else:
            missing_keys = [
                "Carbon Fiber State",
                "Carbon Fiber Fan OK",
                "Carbon Fiber Current Temp",
                "Carbon Fiber Target Temp",
                "Resin State",
                "Resin Fan OK",
                "Resin Current Temp",
                "Resin Target Temp",
            ]
            for key in missing_keys:
                self._set_value(key, "-", "#b42318")
            self._current_tool_value.setText("-")

        if msg.kuka_status_valid:
            ks = msg.kuka_status
            self._set_value("KUKA X", f"{ks.x:.2f}")
            self._set_value("KUKA Y", f"{ks.y:.2f}")
            self._set_value("KUKA Z", f"{ks.z:.2f}")
            self._set_value("KUKA A", f"{ks.a:.2f}")
            self._set_value("KUKA B", f"{ks.b:.2f}")
            self._set_value("KUKA C", f"{ks.c:.2f}")
        else:
            for axis in ("X", "Y", "Z", "A", "B", "C"):
                self._set_value(f"KUKA {axis}", "-", "#b42318")

        self._set_value("Traj Backlog", str(msg.traj_backlog), "#2b2b2b")
        self._set_value("Next Traj Seq", str(msg.traj_next_seq), "#2b2b2b")
        self._set_value("Next Event Seq", str(msg.event_next_seq), "#2b2b2b")
        self._set_value("Events Pending", str(msg.event_pending), "#2b2b2b")
        if msg.current_traj_valid:
            ct = msg.current_traj
            self._set_value("Traj Seq", str(ct.seq), "#2b2b2b")
            self._set_value("Traj Tool", self._format_tool(ct.tool_id), "#2b2b2b")
            self._set_value("Traj X", f"{ct.x:.2f}", "#2b2b2b")
            self._set_value("Traj Y", f"{ct.y:.2f}", "#2b2b2b")
            self._set_value("Traj Z", f"{ct.z:.2f}", "#2b2b2b")
            self._set_value("Traj A", f"{ct.a:.2f}", "#2b2b2b")
            self._set_value("Traj B", f"{ct.b:.2f}", "#2b2b2b")
            self._set_value("Traj C", f"{ct.c:.2f}", "#2b2b2b")
            self._set_value("Traj E", f"{ct.e:.3f}", "#2b2b2b")
        else:
            for key in (
                "Traj Seq",
                "Traj Tool",
                "Traj X",
                "Traj Y",
                "Traj Z",
                "Traj A",
                "Traj B",
                "Traj C",
                "Traj E",
            ):
                self._set_value(key, "-", "#b42318")

        if msg.next_traj_valid:
            nt = msg.next_traj
            self._set_value("Traj Seq (Next)", str(nt.seq), "#2b2b2b")
            self._set_value("Traj Tool (Next)", self._format_tool(nt.tool_id), "#2b2b2b")
            self._set_value("Traj X (Next)", f"{nt.x:.2f}", "#2b2b2b")
            self._set_value("Traj Y (Next)", f"{nt.y:.2f}", "#2b2b2b")
            self._set_value("Traj Z (Next)", f"{nt.z:.2f}", "#2b2b2b")
            self._set_value("Traj A (Next)", f"{nt.a:.2f}", "#2b2b2b")
            self._set_value("Traj B (Next)", f"{nt.b:.2f}", "#2b2b2b")
            self._set_value("Traj C (Next)", f"{nt.c:.2f}", "#2b2b2b")
            self._set_value("Traj E (Next)", f"{nt.e:.3f}", "#2b2b2b")
        else:
            for key in (
                "Traj Seq (Next)",
                "Traj Tool (Next)",
                "Traj X (Next)",
                "Traj Y (Next)",
                "Traj Z (Next)",
                "Traj A (Next)",
                "Traj B (Next)",
                "Traj C (Next)",
                "Traj E (Next)",
            ):
                self._set_value(key, "-", "#b42318")

        if msg.current_event_valid:
            ce = msg.current_event
            self._set_value("Event Type", ce.event_type or "-", "#2b2b2b")
            self._set_value("Event Payload", ce.payload or "-", "#2b2b2b")
            self._set_value("Event Src Line", str(ce.event_src_line), "#2b2b2b")
            self._set_value("Event Trigger Seq", str(ce.trigger_seq), "#2b2b2b")
        else:
            for key in (
                "Event Type",
                "Event Payload",
                "Event Src Line",
                "Event Trigger Seq",
            ):
                self._set_value(key, "-", "#b42318")

        if msg.next_event_valid:
            ne = msg.next_event
            self._set_value("Event Type (Next)", ne.event_type or "-", "#2b2b2b")
            self._set_value("Event Payload (Next)", ne.payload or "-", "#2b2b2b")
            self._set_value("Event Src Line (Next)", str(ne.event_src_line), "#2b2b2b")
            self._set_value("Event Trigger Seq (Next)", str(ne.trigger_seq), "#2b2b2b")
        else:
            for key in (
                "Event Type (Next)",
                "Event Payload (Next)",
                "Event Src Line (Next)",
                "Event Trigger Seq (Next)",
            ):
                self._set_value(key, "-", "#b42318")

    def _on_pause(self):
        self.command_submit.emit("PAUSE")

    def _on_resume(self):
        self.command_submit.emit("RESUME")

    def _on_stop(self):
        reply = QtWidgets.QMessageBox.warning(
            self,
            "确认停止",
            "确定要停止打印吗？\n\n"
            "这将抬升 Z 轴，然后切断所有通信。\n"
            "KUKA 将触发安全停机。",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply == QtWidgets.QMessageBox.Yes:
            self.command_submit.emit("ABORT")

    def _on_fan(self, head_id, on):
        val = "1" if on else "0"
        cmd = f"EV 0 fan_{head_id} {val}\n"
        self.uart_command_submit.emit(cmd)

    def _on_temp_apply(self, head_id):
        temp_input = getattr(self, f"_temp_input_{head_id}")
        text = temp_input.text().strip()
        if not text:
            return
        try:
            temp = float(text)
        except ValueError:
            return
        if temp < 0:
            return
        cmd = f"EV 0 heat_{head_id} {temp}\n"
        self.uart_command_submit.emit(cmd)

    def _on_tool_switch(self, head_id):
        tool_id = "1" if head_id == "cf" else "2"
        event_type = f"tool_change_{head_id}"
        cmd = f"EV 0 {event_type} {tool_id}\n"
        self.uart_command_submit.emit(cmd)

    # ---- Offset persistence ----

    def _on_offset_changed(self, _value=None):
        x = self._offset_spins["X"].value()
        y = self._offset_spins["Y"].value()
        z = self._offset_spins["Z"].value()
        try:
            _save_offset_config(x, y, z)
            self._offset_status.setText(f"已保存: X={x:.2f}  Y={y:.2f}  Z={z:.2f}")
            self._offset_status.setStyleSheet("color: #1b6e3c;")
        except Exception as exc:
            self._offset_status.setText(f"保存失败: {exc}")
            self._offset_status.setStyleSheet("color: #b42318;")

    def get_tool_offset(self):
        return (
            self._offset_spins["X"].value(),
            self._offset_spins["Y"].value(),
            self._offset_spins["Z"].value(),
        )

    # ---- GCode Export ----

    def _on_browse_gcode(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "选择 GCode 文件",
            os.path.dirname(self._gcode_path_input.text()) or os.path.expanduser("~"),
            "GCode Files (*.gcode *.gc *.g);;All Files (*)",
        )
        if path:
            self._gcode_path_input.setText(path)

    def _on_gcode_path_changed(self, text):
        if not text:
            return
        base = os.path.splitext(os.path.basename(text))[0]
        data_root = os.path.expanduser("~/kuka_ram_ws/data/output_npz")
        self._npz_out_input.setText(os.path.join(data_root, base + ".npz"))

    def _on_export_npz(self):
        gcode_path = self._gcode_path_input.text().strip()
        if not gcode_path or not os.path.isfile(gcode_path):
            self._export_status.setText("请选择有效的 GCode 文件。")
            self._export_status.setStyleSheet("color: #b42318;")
            return

        npz_out = self._npz_out_input.text().strip()
        if not npz_out:
            self._export_status.setText("请指定 NPZ 输出路径。")
            self._export_status.setStyleSheet("color: #b42318;")
            return

        # Gather planner params
        try:
            params = {
                "dt": float(self._planner_inputs["dt"].text()),
                "default_feed_mm_s": float(self._planner_inputs["default_feed_mm_s"].text()),
                "corner_angle_deg": float(self._planner_inputs["corner_angle_deg"].text()),
                "corner_retreat_ratio": float(self._planner_inputs["corner_retreat_ratio"].text()),
                "density": int(self._planner_inputs["density"].text()),
                "degree": int(self._planner_inputs["degree"].text()),
                "max_fit_points_per_segment": int(self._planner_inputs["max_fit_points_per_segment"].text()),
                "export_sleep_ms": int(self._planner_inputs["export_sleep_ms"].text()),
                "export_yield_every": int(self._planner_inputs["export_yield_every"].text()),
                "split_by_layer_type": self._planner_inputs["split_by_layer_type"].isChecked(),
                "plot_layer_xy": self._planner_inputs["plot_layer_xy"].isChecked(),
                "plot_stride": int(self._planner_inputs["plot_stride"].text()),
            }
        except (ValueError, KeyError) as exc:
            self._export_status.setText(f"无效的规划器参数: {exc}")
            self._export_status.setStyleSheet("color: #b42318;")
            return

        offset = self.get_tool_offset()

        # Disable button, show progress
        self._btn_export_npz.setEnabled(False)
        self._export_progress.setVisible(True)
        self._export_progress.setRange(0, 100)
        self._export_progress.setValue(0)
        self._export_status.setText("导出中...")
        self._export_status.setStyleSheet("color: #b15e00;")

        def _worker():
            try:
                self.export_progress.emit("读取 GCode...")
                from gcode_planner.gcode_parser import load_gcode_lines, parse_gcode_lines
                from gcode_planner.npz_exporter import export_npz

                def progress_cb(ratio):
                    self.export_progress_val.emit(int(ratio * 100))

                lines = load_gcode_lines(gcode_path)
                self.export_progress.emit("解析 GCode...")
                parsed = parse_gcode_lines(lines)
                self.export_progress.emit(f"导出 NPZ（{len(parsed)} 条指令）...")

                out_dir = os.path.dirname(npz_out)
                if out_dir:
                    os.makedirs(out_dir, exist_ok=True)
                    # Also create a subdirectory named after the GCode base name
                    gcode_base = os.path.splitext(os.path.basename(gcode_path))[0]
                    os.makedirs(os.path.join(out_dir, gcode_base), exist_ok=True)

                stats = export_npz(
                    parsed,
                    npz_out,
                    dt=params["dt"],
                    chunk_size=5000000,
                    default_feed_mm_s=params["default_feed_mm_s"],
                    corner_angle_deg=params["corner_angle_deg"],
                    corner_retreat_ratio=params["corner_retreat_ratio"],
                    density=params["density"],
                    degree=params["degree"],
                    max_fit_points_per_segment=params["max_fit_points_per_segment"],
                    export_sleep_ms=params["export_sleep_ms"],
                    export_yield_every=params["export_yield_every"],
                    tool_offset=offset,
                    progress_callback=progress_cb,
                    split_by_layer_type=params["split_by_layer_type"],
                    plot_layer_xy=params["plot_layer_xy"],
                    plot_stride=params["plot_stride"],
                )
                rows = stats.get("rows", 0)
                parts = stats.get("parts", 0)
                total_s = stats.get("total_s", 0.0)
                msg = (
                    f"完成: {rows} 行, {parts} 分块, {total_s:.1f}秒\n"
                    f"偏移: ({offset[0]:.2f}, {offset[1]:.2f}, {offset[2]:.2f})"
                )
                self.export_finished.emit(True, msg)
            except Exception as exc:
                self.export_finished.emit(False, str(exc))

        t = threading.Thread(target=_worker, daemon=True)
        t.start()

    def _on_export_progress(self, text):
        self._export_status.setText(text)
        self._export_status.setStyleSheet("color: #b15e00;")

    def _on_export_progress_val(self, val):
        self._export_progress.setValue(val)

    def _on_export_finished(self, success, message):
        self._btn_export_npz.setEnabled(True)
        self._export_progress.setVisible(False)
        if success:
            self._export_status.setText(message)
            self._export_status.setStyleSheet("color: #1b6e3c;")
            npz_path = self._npz_out_input.text().strip()
            if npz_path:
                self._last_npz_dir = os.path.splitext(npz_path)[0]
                self._btn_view_layers.setEnabled(True)
        else:
            self._export_status.setText(f"导出失败: {message}")
            self._export_status.setStyleSheet("color: #b42318;")

    def _on_view_layers(self):
        if not self._last_npz_dir or not os.path.isdir(self._last_npz_dir):
            self._export_status.setText("未找到 NPZ 导出目录。")
            self._export_status.setStyleSheet("color: #b42318;")
            return
        dlg = _LayerViewerDialog(self._last_npz_dir, self)
        dlg.exec_()

    _STATE_COLORS = {
        "RUNNING": "#1b6e3c",
        "WAIT_HEARTBEAT": "#b15e00",
        "HEARTBEAT_LOST": "#b42318",
        "PAUSED": "#1a73e8",
        "ABORTING": "#b42318",
        "ABORTED": "#b42318",
    }

    def _update_control_buttons(self, state):
        if state == "PAUSED":
            self._btn_pause.setEnabled(False)
            self._btn_resume.setEnabled(True)
            self._btn_stop.setEnabled(True)
        elif state == "WAIT_HEARTBEAT":
            self._btn_pause.setEnabled(False)
            self._btn_resume.setEnabled(True)
            self._btn_stop.setEnabled(True)
        elif state in ("ABORTING", "ABORTED"):
            self._btn_pause.setEnabled(False)
            self._btn_resume.setEnabled(False)
            self._btn_stop.setEnabled(False)
        else:
            self._btn_pause.setEnabled(True)
            self._btn_resume.setEnabled(False)
            self._btn_stop.setEnabled(True)

    def _on_rsi_xml(self, xml_text):
        import time, re
        xml_stripped = re.sub(r'<IPOC>[^<]*</IPOC>', '<IPOC>...</IPOC>', xml_text)
        if xml_stripped == self._rsi_log_last_xml:
            return
        self._rsi_log_last_xml = xml_stripped
        t = time.localtime()
        ts = f"{t.tm_hour:02d}:{t.tm_min:02d}:{t.tm_sec:02d}"
        lines = xml_text.strip().split("\n")
        for line in lines:
            self._rsi_log_text.appendPlainText(f"[{ts}] {line}")

    def _on_uart_log(self, line_text):
        import time
        t = time.localtime()
        ts = f"{t.tm_hour:02d}:{t.tm_min:02d}:{t.tm_sec:02d}"
        self._uart_log_text.appendPlainText(f"[{ts}] {line_text}")


class MyProjectUiPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("MyProjectUiPlugin")

        if not rclpy.ok():
            rclpy.init(args=None)

        self._node = context.node
        self._param_client = self._node.create_client(
            SetParameters, "/uart_node/set_parameters"
        )
        self._cmd_pub = self._node.create_publisher(
            StringMsg, "/system/command", 10
        )
        self._uart_cmd_pub = self._node.create_publisher(
            StringMsg, "/uart/manual_command", 10
        )
        self._widget = _UiStatusWidget()
        self._widget.scale_submit.connect(self._on_scale_submit)
        self._widget.command_submit.connect(self._on_command_submit)
        self._widget.uart_command_submit.connect(self._on_uart_command_submit)
        context.add_widget(self._widget)

        # Launch state
        self._launch_params = dict(_LAUNCH_DEFAULTS)
        self._launch_params["gcode_path"] = "/home/jayson/kuka_ram_ws/data/input_gcode/test.gcode"
        self._launch_process = None
        self._pending_launch = False
        
        # Initialize text inputs in widget from launch state
        self._widget._gcode_path_input.setText(self._launch_params["gcode_path"])
        
        # Sync changes from main UI to launch params
        self._widget._gcode_path_input.textChanged.connect(self._on_gcode_path_changed_sync)
        # Connect export signals to launch flow
        self._widget.export_finished.connect(self._on_export_finished_launcher)

        self._widget._btn_launch_settings.clicked.connect(
            self._on_launch_settings
        )
        self._widget._btn_launch.clicked.connect(self._on_launch)
        self._widget._btn_stop_launch.clicked.connect(self._on_stop_launch)

        self._node.create_subscription(
            UiStatus, "/ui/status", self._on_status, 10
        )
        self._node.create_subscription(
            StringMsg, "/rsi/sent_xml", self._on_rsi_xml, 10
        )
        self._node.create_subscription(
            StringMsg, "/uart/raw", self._on_uart_log_msg, 10
        )

        self._spin_timer = QtCore.QTimer(self._widget)
        self._spin_timer.timeout.connect(self._spin_once)
        self._spin_timer.start(50)

        self._launch_check_timer = QtCore.QTimer(self._widget)
        self._launch_check_timer.timeout.connect(self._check_launch_process)
        self._launch_check_timer.start(1000)

    def _on_status(self, msg: UiStatus):
        self._widget.status_received.emit(msg)

    def _on_rsi_xml(self, msg: StringMsg):
        self._widget.rsi_xml_received.emit(msg.data)

    def _on_uart_log_msg(self, msg: StringMsg):
        self._widget.uart_log_received.emit(msg.data)

    def _on_command_submit(self, cmd: str):
        msg = StringMsg()
        msg.data = cmd
        self._cmd_pub.publish(msg)

    def _on_uart_command_submit(self, cmd: str):
        msg = StringMsg()
        msg.data = cmd
        self._uart_cmd_pub.publish(msg)

    def _on_scale_submit(self, value: float):
        if not self._param_client.service_is_ready():
            self._widget.set_extrude_status("UART 参数服务未就绪。", "#b42318")
            return

        req = SetParameters.Request()
        req.parameters = [
            Parameter("extrude_scale", Parameter.Type.DOUBLE, value).to_parameter_msg()
        ]
        future = self._param_client.call_async(req)

        def _done(fut):
            try:
                resp = fut.result()
            except Exception:
                self._widget.set_extrude_scale(value, "提交失败。", "#b42318")
                return
            results = resp.results if resp is not None else []
            if results and all(r.successful for r in results):
                self._widget.set_extrude_scale(value, "已应用。", "#1b6e3c")
            else:
                reason = results[0].reason if results else "提交失败"
                self._widget.set_extrude_scale(value, reason or "提交失败。", "#b42318")

        future.add_done_callback(_done)

    def _spin_once(self):
        rclpy.spin_once(self._node, timeout_sec=0.0)

    # ---- Launch control ----

    def _on_launch_settings(self):
        dialog = _LaunchSettingsDialog(self._launch_params, self._widget)
        if dialog.exec_() == QtWidgets.QDialog.Accepted:
            self._launch_params.update(dialog.get_params())

    def _on_gcode_path_changed_sync(self, text):
        self._launch_params["gcode_path"] = text

    def _on_export_finished_launcher(self, success, message):
        if not self._pending_launch:
            return
        self._pending_launch = False
        if success:
            self._do_launch()
        else:
            self._widget._btn_launch.setEnabled(True)
            self._widget._launch_status.setText("启动中止: GCode 导出失败。")
            self._widget._launch_status.setStyleSheet(
                "color: #b42318; font-weight: 700; font-size: 13px;"
            )

    def _check_npz_exists(self, gcode_path):
        if not gcode_path:
            return False
        base = os.path.splitext(os.path.basename(gcode_path))[0]
        data_root = os.path.expanduser("~/kuka_ram_ws/data/output_npz")
        
        # Check standard single NPZ file
        npz_file = os.path.join(data_root, base + ".npz")
        if os.path.isfile(npz_file):
            return True
            
        # Check multi-part NPZ files (e.g. switch_test_part0000.npz)
        npz_part = os.path.join(data_root, base + "_part0000.npz")
        if os.path.isfile(npz_part):
            return True
            
        # Check directory case
        npz_dir = os.path.join(data_root, base)
        if os.path.isdir(npz_dir):
            try:
                if os.listdir(npz_dir):
                    return True
            except Exception:
                pass
                
        return False

    def _check_npz_and_offset_match(self, gcode_path):
        if not gcode_path:
            return False, "missing"
            
        if not self._check_npz_exists(gcode_path):
            return False, "missing"
            
        base = os.path.splitext(os.path.basename(gcode_path))[0]
        data_root = os.path.expanduser("~/kuka_ram_ws/data/output_npz")
        offset_file = os.path.join(data_root, base + ".offset.json")
        
        if not os.path.isfile(offset_file):
            return False, "mismatch"
            
        try:
            with open(offset_file, "r") as f:
                data = json.load(f)
            saved_offset = data.get("tool_offset")
            if not saved_offset or len(saved_offset) != 3:
                return False, "mismatch"
                
            cur_offset = self._widget.get_tool_offset() # (x, y, z)
            
            # Compare up to 0.005 mm tolerance (since spin boxes are 2 decimal places)
            for val_saved, val_cur in zip(saved_offset, cur_offset):
                if abs(val_saved - val_cur) > 0.005:
                    return False, "mismatch"
            
            return True, "ok"
        except Exception:
            return False, "mismatch"

    def _on_launch(self):
        if (
            self._launch_process is not None
            and self._launch_process.poll() is None
        ):
            return

        gcode_path = self._launch_params.get("gcode_path", "").strip()
        if not gcode_path or not os.path.isfile(gcode_path):
            self._widget._launch_status.setText("启动失败: GCode 文件未找到。")
            self._widget._launch_status.setStyleSheet(
                "color: #b42318; font-weight: 700; font-size: 13px;"
            )
            return

        # Enforce check for matching NPZ files/folders and offsets
        exists, status = self._check_npz_and_offset_match(gcode_path)
        if not exists:
            if status == "missing":
                msg = (
                    "未找到与此 GCode 匹配的 NPZ 文件。\n\n"
                    "请先点击“导出 NPZ”按钮生成轨迹数据。"
                )
                title = "NPZ 文件缺失"
            else:  # mismatch
                msg = (
                    "当前工具偏移值与已保存的 NPZ 轨迹中的偏移值不匹配。\n\n"
                    "请先点击“导出 NPZ”按钮使用新偏移重新生成轨迹。"
                )
                title = "偏移量不匹配"

            QtWidgets.QMessageBox.warning(
                self._widget,
                title,
                msg,
                QtWidgets.QMessageBox.Ok
            )
            self._widget._launch_status.setText(f"启动已取消: NPZ {status}。")
            self._widget._launch_status.setStyleSheet(
                "color: #b42318; font-weight: 700; font-size: 13px;"
            )
            return

        # Directly launch since NPZ is verified to exist and offsets match
        self._do_launch()

    def _do_launch(self):
        cmd = ["ros2", "launch", "my_project_startup", "startup.launch.py"]
        for name, value in self._launch_params.items():
            if name == "gcode_path":
                base = os.path.splitext(os.path.basename(value))[0]
                data_root = os.path.expanduser("~/kuka_ram_ws/data/output_npz")
                npz_val = os.path.join(data_root, base + ".npz")
                cmd.append(f"npz_path:={npz_val}")
            else:
                default = _LAUNCH_DEFAULTS.get(name, "")
                if value != default:
                    cmd.append(f"{name}:={value}")
        try:
            self._launch_process = subprocess.Popen(
                cmd,
                preexec_fn=os.setsid,
            )
            self._widget._btn_launch.setEnabled(False)
            self._widget._btn_stop_launch.setEnabled(True)
            self._widget._btn_launch_settings.setEnabled(False)
            self._widget._launch_status.setText("运行中（等待 KUKA 首包...")
            self._widget._launch_status.setStyleSheet(
                "color: #1b6e3c; font-weight: 700; font-size: 13px;"
            )
        except Exception as exc:
            self._widget._btn_launch.setEnabled(True)
            self._widget._launch_status.setText(f"启动失败: {exc}")
            self._widget._launch_status.setStyleSheet(
                "color: #b42318; font-weight: 700; font-size: 13px;"
            )

    def _on_stop_launch(self):
        if (
            self._launch_process is not None
            and self._launch_process.poll() is None
        ):
            try:
                os.killpg(
                    os.getpgid(self._launch_process.pid), signal.SIGTERM
                )
            except OSError:
                self._launch_process.terminate()
        self._launch_process = None
        self._widget._btn_launch.setEnabled(True)
        self._widget._btn_stop_launch.setEnabled(False)
        self._widget._btn_launch_settings.setEnabled(True)
        self._widget._launch_status.setText("已停止")
        self._widget._launch_status.setStyleSheet(
            "color: #b42318; font-weight: 700; font-size: 13px;"
        )

    def _check_launch_process(self):
        if self._launch_process is None:
            return
        rc = self._launch_process.poll()
        if rc is not None:
            self._launch_process = None
            self._widget._btn_launch.setEnabled(True)
            self._widget._btn_stop_launch.setEnabled(False)
            self._widget._btn_launch_settings.setEnabled(True)
            if rc == 0:
                self._widget._launch_status.setText("已退出")
                self._widget._launch_status.setStyleSheet(
                    "color: #666666; font-weight: 700; font-size: 13px;"
                )
            else:
                self._widget._launch_status.setText(
                    f"异常退出（代码 {rc}）"
                )
                self._widget._launch_status.setStyleSheet(
                    "color: #b42318; font-weight: 700; font-size: 13px;"
                )

    def shutdown_plugin(self):
        if self._launch_check_timer.isActive():
            self._launch_check_timer.stop()
        if self._spin_timer.isActive():
            self._spin_timer.stop()
        # Terminate launch process if running
        if (
            self._launch_process is not None
            and self._launch_process.poll() is None
        ):
            try:
                os.killpg(
                    os.getpgid(self._launch_process.pid), signal.SIGTERM
                )
            except OSError:
                self._launch_process.terminate()
            self._launch_process = None
        self._node.destroy_node()
