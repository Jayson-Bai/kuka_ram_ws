from python_qt_binding import QtCore, QtWidgets, QtGui
from rqt_gui_py.plugin import Plugin
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
import subprocess
import os
import signal

from my_project_interfaces.msg import UiStatus

from std_msgs.msg import String as StringMsg


LAUNCH_PARAMS = [
    # (param_name, default_value, description, group)
    ("center_start_delay_s", "1.0", "Delay before starting center_node (seconds)", "Center Node"),
    ("npz_path", "/home/jayson/kuka_ram_ws/data/output_npz/test.npz",
     "Trajectory/event NPZ file path", "Center Node"),
    ("npz_preload_chunks", "2", "NPZ preload chunk count", "Center Node"),
    ("queue_low", "1000", "Trajectory queue low watermark", "Center Node"),
    ("queue_high", "2000", "Trajectory queue high watermark", "Center Node"),
    ("plan_qos_depth", "2000", "Plan topic QoS depth", "Center Node"),
    ("traj_prefill", "1000", "Trajectory prefill count on startup", "Center Node"),
    ("traj_low", "500", "Trajectory backlog low threshold", "Center Node"),
    ("traj_high", "1500", "Trajectory backlog high threshold", "Center Node"),
    ("xyzabc_decimals", "6", "Pose decimal precision", "Center Node"),
    ("e_decimals", "2", "Extrusion decimal precision", "Center Node"),
    ("kuka_status_raw", "false", "Print KUKA raw XML length", "Center Node"),
    ("summary_period_ms", "200", "Control center publish period (ms)", "Center Node"),
    ("sen_type", "PosCorr", "RSI XML SEN type", "RSI Node"),
    ("decimal_precision", "6", "RSI data decimal precision", "RSI Node"),
    ("local_ip", "192.168.1.1", "RSI local listen IP", "RSI Node"),
    ("local_port", "49152", "RSI local listen port", "RSI Node"),
    ("abort_lift_mm", "100.0", "Z lift distance on ABORT (mm)", "RSI Node"),
    ("abort_lift_speed_mm_s", "10.0", "Z lift speed on ABORT (mm/s)", "RSI Node"),
    ("port", "/dev/ttyUSB0", "UART serial device path", "UART Node"),
    ("baudrate", "115200", "UART baud rate", "UART Node"),
    ("extrude_scale", "1.0", "UART extrusion scale factor", "UART Node"),
    ("ui_publish_period_ms", "200", "UI status publish period (ms)", "System Manager"),
    ("heartbeat_timeout_s", "1.0", "Heartbeat timeout (seconds)", "System Manager"),
    ("traj_queue_limit", "5000", "UI trajectory queue limit", "System Manager"),
    ("event_queue_limit", "2000", "UI event queue limit", "System Manager"),
]

_LAUNCH_DEFAULTS = {p[0]: p[1] for p in LAUNCH_PARAMS}
_LAUNCH_GROUPS_ORDER = ["Center Node", "RSI Node", "UART Node", "System Manager"]


class _LaunchSettingsDialog(QtWidgets.QDialog):
    """Dialog for editing all launch parameters grouped by node."""

    def __init__(self, current_params, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Launch Settings")
        self.setMinimumSize(620, 520)
        self._inputs = {}
        self._build_ui(current_params)

    def _build_ui(self, current_params):
        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(10)

        title = QtWidgets.QLabel("Launch Parameter Configuration")
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
            "Center Node": "#1a73e8",
            "RSI Node": "#b15e00",
            "UART Node": "#1b6e3c",
            "System Manager": "#7b1fa2",
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
                elif param_name == "npz_path":
                    row_widget = QtWidgets.QWidget()
                    row_lay = QtWidgets.QHBoxLayout(row_widget)
                    row_lay.setContentsMargins(0, 0, 0, 0)
                    row_lay.setSpacing(4)
                    line_edit = QtWidgets.QLineEdit(current_val)
                    line_edit.setToolTip(description)
                    line_edit.setStyleSheet(
                        "border: 1px solid #d0d0d0; border-radius: 4px;"
                        " padding: 4px 6px; background: #ffffff;"
                    )
                    browse_btn = QtWidgets.QPushButton("…")
                    browse_btn.setFixedWidth(32)
                    browse_btn.setFixedHeight(28)
                    browse_btn.setCursor(QtCore.Qt.PointingHandCursor)
                    browse_btn.setStyleSheet(
                        "border: 1px solid #1a73e8; border-radius: 4px;"
                        " background: #ffffff; color: #1a73e8;"
                        " font-weight: 600;"
                    )
                    browse_btn.clicked.connect(
                        lambda checked, le=line_edit: self._browse_file(le)
                    )
                    row_lay.addWidget(line_edit, 1)
                    row_lay.addWidget(browse_btn)
                    form.addRow(label, row_widget)
                    widget = line_edit
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
        btn_reset = QtWidgets.QPushButton("Reset Defaults")
        btn_reset.setMinimumHeight(32)
        btn_reset.setCursor(QtCore.Qt.PointingHandCursor)
        btn_reset.setStyleSheet(
            "font-weight: 600; border: 1px solid #c0c0c0;"
            " border-radius: 5px; background: #ffffff;"
            " color: #666666; padding: 4px 16px;"
        )
        btn_reset.clicked.connect(self._reset_defaults)
        btn_ok = QtWidgets.QPushButton("OK")
        btn_ok.setMinimumHeight(32)
        btn_ok.setCursor(QtCore.Qt.PointingHandCursor)
        btn_ok.setStyleSheet(
            "font-weight: 600; border: 1px solid #1a73e8;"
            " border-radius: 5px; background: #1a73e8;"
            " color: #ffffff; padding: 4px 20px;"
        )
        btn_ok.clicked.connect(self.accept)
        btn_cancel = QtWidgets.QPushButton("Cancel")
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

    def _browse_file(self, line_edit):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Select NPZ / Manifest File",
            os.path.dirname(line_edit.text()) or "",
            "NPZ Files (*.npz);;JSON Manifest (*.json);;All Files (*)",
        )
        if path:
            line_edit.setText(path)

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


class _UiStatusWidget(QtWidgets.QWidget):
    status_received = QtCore.pyqtSignal(object)
    scale_submit = QtCore.pyqtSignal(float)
    command_submit = QtCore.pyqtSignal(str)
    uart_command_submit = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._extrude_scale_current = 1.0
        self._build_ui()
        self.status_received.connect(self._update_ui)

    def _build_ui(self):
        layout = QtWidgets.QGridLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(6)
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)

        self._labels = {}
        value_min_width = QtWidgets.QLabel("0").fontMetrics().horizontalAdvance("0") * 5
        cf_labels = [
            ("Carbon Fiber State", "State"),
            ("Carbon Fiber Fan OK", "Fan OK"),
            ("Carbon Fiber Current Temp", "Current Temp"),
            ("Carbon Fiber Target Temp", "Target Temp"),
        ]
        resin_labels = [
            ("Resin State", "State"),
            ("Resin Fan OK", "Fan OK"),
            ("Resin Current Temp", "Current Temp"),
            ("Resin Target Temp", "Target Temp"),
        ]
        label_metrics = QtWidgets.QLabel("X").fontMetrics()
        cf_resin_label_titles = [title for _, title in (cf_labels + resin_labels)]
        cf_resin_label_min_width = max(
            label_metrics.horizontalAdvance(text) for text in cf_resin_label_titles
        )

        title = QtWidgets.QLabel("Status Overview")
        title.setObjectName("titleLabel")
        title.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        title.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        layout.addWidget(title, 0, 0, 1, 2)

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

        left_column = QtWidgets.QVBoxLayout()
        left_column.setSpacing(6)
        right_column = QtWidgets.QVBoxLayout()
        right_column.setSpacing(6)

        system_box = add_group("System", [
            ("System State", "State"),
        ], add_to_layout=False)
        system_box.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        printhead_box = QtWidgets.QGroupBox("Printhead")
        printhead_box.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        printhead_layout = QtWidgets.QVBoxLayout(printhead_box)
        printhead_layout.setSpacing(6)
        add_group("General", [
            ("Printhead Ready", "Ready"),
            ("Printhead Age", "Age"),
            ("Printhead Stamp", "Stamp"),
            ("Ready Event Seq", "Ready Seq"),
            ("Ready Event Type", "Ready Type"),
            ("Current Tool", "Tool"),
        ], parent_layout=printhead_layout)
        printhead_row = QtWidgets.QHBoxLayout()
        printhead_row.setSpacing(8)
        cf_box = add_group("Carbon Fiber", cf_labels,
        parent_layout=printhead_row, object_name="groupPrintheadCF",
            label_min_width=cf_resin_label_min_width,
            value_alignment=QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        resin_box = add_group("Resin", resin_labels,
        parent_layout=printhead_row, object_name="groupPrintheadResin",
            label_min_width=cf_resin_label_min_width,
            value_alignment=QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        cf_box.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        resin_box.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        printhead_row.setStretch(0, 1)
        printhead_row.setStretch(1, 1)
        printhead_layout.addLayout(printhead_row)
        printhead_box.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        right_column.addWidget(printhead_box)
        kuka_box = QtWidgets.QGroupBox("KUKA Pos")
        kuka_box.setObjectName("groupKuka")
        kuka_box.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        kuka_layout = QtWidgets.QVBoxLayout(kuka_box)
        kuka_layout.setSpacing(6)
        axes_row = QtWidgets.QHBoxLayout()
        axes_row.setSpacing(6)
        for axis in ("X", "Y", "Z", "A", "B", "C"):
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
            axes_row.addWidget(axis_widget)
            self._labels[f"KUKA {axis}"] = axis_value
        kuka_layout.addLayout(axes_row)
        left_column.addWidget(kuka_box)
        heartbeat_box = add_group("Heartbeat", [
            ("Heartbeat Age", "Age"),
            ("Heartbeat Seq", "Seq"),
            ("Heartbeat IPOC", "IPOC"),
            ("Heartbeat Tool", "Tool"),
            ("Heartbeat Extrude", "Extrude"),
        ], parent_layout=left_column, object_name="groupHeartbeat")
        heartbeat_box.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        traj_box = QtWidgets.QGroupBox("Trajectory")
        traj_box.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        traj_layout = QtWidgets.QVBoxLayout(traj_box)
        traj_layout.setSpacing(6)
        add_group("Summary", [
            ("Traj Backlog", "Backlog"),
            ("Next Traj Seq", "Next Seq"),
        ], parent_layout=traj_layout)
        traj_row = QtWidgets.QHBoxLayout()
        traj_row.setSpacing(8)
        add_group("Current", [
            ("Traj Seq", "Seq"),
            ("Traj Tool", "Tool"),
            ("Traj X", "X"),
            ("Traj Y", "Y"),
            ("Traj Z", "Z"),
            ("Traj A", "A"),
            ("Traj B", "B"),
            ("Traj C", "C"),
            ("Traj E", "E"),
        ], parent_layout=traj_row)
        add_group("Next", [
            ("Traj Seq (Next)", "Seq"),
            ("Traj Tool (Next)", "Tool"),
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
        left_column.addWidget(traj_box)

        events_box = QtWidgets.QGroupBox("Events")
        events_box.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        events_layout = QtWidgets.QVBoxLayout(events_box)
        events_layout.setSpacing(6)
        add_group("Summary", [
            ("Next Event Seq", "Next Seq"),
            ("Events Pending", "Pending"),
        ], parent_layout=events_layout)
        events_row = QtWidgets.QHBoxLayout()
        events_row.setSpacing(8)
        add_group("Current", [
            ("Event Type", "Type"),
            ("Event Payload", "Payload"),
            ("Event Src Line", "Src Line"),
            ("Event Trigger Seq", "Trigger Seq"),
        ], parent_layout=events_row)
        add_group("Next", [
            ("Event Type (Next)", "Type"),
            ("Event Payload (Next)", "Payload"),
            ("Event Src Line (Next)", "Src Line"),
            ("Event Trigger Seq (Next)", "Trigger Seq"),
        ], parent_layout=events_row)
        events_row.setStretch(0, 1)
        events_row.setStretch(1, 1)
        events_layout.addLayout(events_row)
        right_column.addWidget(events_box)

        # ======== Printhead Control 区域 ========
        ph_control_box = QtWidgets.QGroupBox("Printhead Control")
        ph_control_box.setObjectName("groupPrintheadControl")
        ph_control_box.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        ph_control_outer = QtWidgets.QVBoxLayout(ph_control_box)
        ph_control_outer.setSpacing(8)

        # -- Tool Switch row --
        tool_row = QtWidgets.QHBoxLayout()
        tool_row.setSpacing(12)
        tool_label = QtWidgets.QLabel("Switch Tool")
        tool_label.setObjectName("fieldLabel")
        tool_row.addWidget(tool_label)
        self._btn_tool_cf = QtWidgets.QPushButton("Carbon Fiber")
        self._btn_tool_cf.setObjectName("btnToolCF")
        self._btn_tool_cf.setMinimumHeight(32)
        self._btn_tool_cf.setCursor(QtCore.Qt.PointingHandCursor)
        self._btn_tool_resin = QtWidgets.QPushButton("Resin")
        self._btn_tool_resin.setObjectName("btnToolResin")
        self._btn_tool_resin.setMinimumHeight(32)
        self._btn_tool_resin.setCursor(QtCore.Qt.PointingHandCursor)
        tool_row.addWidget(self._btn_tool_cf)
        tool_row.addWidget(self._btn_tool_resin)
        ph_control_outer.addLayout(tool_row)

        # -- Carbon Fiber / RESIN side-by-side panels --
        head_panels_row = QtWidgets.QHBoxLayout()
        head_panels_row.setSpacing(12)

        for head_id, head_name in (("cf", "Carbon Fiber"), ("resin", "Resin")):
            panel = QtWidgets.QGroupBox(head_name)
            panel.setObjectName(f"groupCtrl{head_id}")
            panel.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Maximum)
            panel_layout = QtWidgets.QVBoxLayout(panel)
            panel_layout.setSpacing(6)

            # Fan row
            fan_row = QtWidgets.QHBoxLayout()
            fan_row.setSpacing(8)
            fan_label = QtWidgets.QLabel("Fan")
            fan_label.setObjectName("fieldLabel")
            fan_label.setMinimumWidth(30)
            btn_fan_on = QtWidgets.QPushButton("ON")
            btn_fan_on.setObjectName(f"btnFanOn_{head_id}")
            btn_fan_on.setMinimumHeight(28)
            btn_fan_on.setCursor(QtCore.Qt.PointingHandCursor)
            btn_fan_off = QtWidgets.QPushButton("OFF")
            btn_fan_off.setObjectName(f"btnFanOff_{head_id}")
            btn_fan_off.setMinimumHeight(28)
            btn_fan_off.setCursor(QtCore.Qt.PointingHandCursor)
            fan_row.addWidget(fan_label)
            fan_row.addWidget(btn_fan_on)
            fan_row.addWidget(btn_fan_off)
            panel_layout.addLayout(fan_row)

            # Temperature row
            temp_row = QtWidgets.QHBoxLayout()
            temp_row.setSpacing(8)
            temp_label = QtWidgets.QLabel("Temp")
            temp_label.setObjectName("fieldLabel")
            temp_label.setMinimumWidth(30)
            temp_input = QtWidgets.QLineEdit()
            temp_input.setPlaceholderText("°C")
            temp_input.setMaximumWidth(80)
            temp_validator = QtGui.QDoubleValidator(0.0, 500.0, 1, temp_input)
            temp_validator.setNotation(QtGui.QDoubleValidator.StandardNotation)
            temp_input.setValidator(temp_validator)
            btn_temp_apply = QtWidgets.QPushButton("Set")
            btn_temp_apply.setObjectName(f"btnTempApply_{head_id}")
            btn_temp_apply.setMinimumHeight(28)
            btn_temp_apply.setCursor(QtCore.Qt.PointingHandCursor)
            temp_row.addWidget(temp_label)
            temp_row.addWidget(temp_input, 1)
            temp_row.addWidget(btn_temp_apply)
            panel_layout.addLayout(temp_row)

            head_panels_row.addWidget(panel)

            # Store references
            setattr(self, f"_btn_fan_on_{head_id}", btn_fan_on)
            setattr(self, f"_btn_fan_off_{head_id}", btn_fan_off)
            setattr(self, f"_temp_input_{head_id}", temp_input)
            setattr(self, f"_btn_temp_apply_{head_id}", btn_temp_apply)

        ph_control_outer.addLayout(head_panels_row)

        # -- Extrude Scale row (inside Printhead Control) --
        extrude_group = QtWidgets.QGroupBox("Extrude Scale")
        extrude_group.setObjectName("groupExtrudeScale")
        extrude_group.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        extrude_inner = QtWidgets.QHBoxLayout(extrude_group)
        extrude_inner.setSpacing(10)

        extrude_cur_label = QtWidgets.QLabel("Current")
        extrude_cur_label.setObjectName("fieldLabel")
        self._extrude_scale_value = QtWidgets.QLabel("1.000")
        self._extrude_scale_value.setObjectName("valueLabel")
        self._extrude_scale_value.setMinimumWidth(value_min_width)

        extrude_set_label = QtWidgets.QLabel("Set")
        extrude_set_label.setObjectName("fieldLabel")
        self._extrude_scale_input = QtWidgets.QLineEdit()
        self._extrude_scale_input.setPlaceholderText("1.0")
        self._extrude_scale_input.setMaximumWidth(80)
        validator = QtGui.QDoubleValidator(0.001, 1000.0, 3, self._extrude_scale_input)
        validator.setNotation(QtGui.QDoubleValidator.StandardNotation)
        self._extrude_scale_input.setValidator(validator)
        self._extrude_scale_apply = QtWidgets.QPushButton("Apply")
        self._extrude_scale_apply.setObjectName("btnTempApply_extrude")
        self._extrude_scale_apply.setMinimumHeight(28)
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

        ph_control_outer.addWidget(extrude_group)

        layout.addWidget(system_box, 1, 0, 1, 2)
        layout.addLayout(left_column, 2, 0, 1, 1)
        layout.addLayout(right_column, 2, 1, 1, 1)
        layout.addWidget(ph_control_box, 3, 0, 1, 2)

        # ======== Print Control 区域 ========
        control_box = QtWidgets.QGroupBox("Print Control")
        control_box.setObjectName("groupControl")
        control_box.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        control_layout = QtWidgets.QVBoxLayout(control_box)
        control_layout.setSpacing(8)

        btn_row = QtWidgets.QHBoxLayout()
        btn_row.setSpacing(12)

        self._btn_pause = QtWidgets.QPushButton("Pause")
        self._btn_pause.setObjectName("btnPause")
        self._btn_pause.setMinimumHeight(36)
        self._btn_pause.setCursor(QtCore.Qt.PointingHandCursor)

        self._btn_resume = QtWidgets.QPushButton("Resume")
        self._btn_resume.setObjectName("btnResume")
        self._btn_resume.setMinimumHeight(36)
        self._btn_resume.setCursor(QtCore.Qt.PointingHandCursor)
        self._btn_resume.setEnabled(False)

        self._btn_stop = QtWidgets.QPushButton("Stop")
        self._btn_stop.setObjectName("btnStop")
        self._btn_stop.setMinimumHeight(36)
        self._btn_stop.setCursor(QtCore.Qt.PointingHandCursor)

        btn_row.addWidget(self._btn_pause)
        btn_row.addWidget(self._btn_resume)
        btn_row.addWidget(self._btn_stop)
        control_layout.addLayout(btn_row)

        ctrl_status_row = QtWidgets.QHBoxLayout()
        ctrl_status_row.setSpacing(8)
        ctrl_label = QtWidgets.QLabel("Status")
        ctrl_label.setObjectName("fieldLabel")
        self._control_status = QtWidgets.QLabel("WAIT_HEARTBEAT")
        self._control_status.setObjectName("controlStatus")
        ctrl_status_row.addWidget(ctrl_label)
        ctrl_status_row.addWidget(self._control_status, 1)
        control_layout.addLayout(ctrl_status_row)

        layout.addWidget(control_box, 4, 0, 1, 2, QtCore.Qt.AlignTop)

        # ======== Launch Control 区域 ========
        launch_box = QtWidgets.QGroupBox("Launch")
        launch_box.setObjectName("groupLaunch")
        launch_box.setSizePolicy(
            QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum
        )
        launch_inner = QtWidgets.QVBoxLayout(launch_box)
        launch_inner.setSpacing(8)

        launch_btn_row = QtWidgets.QHBoxLayout()
        launch_btn_row.setSpacing(12)

        self._btn_launch_settings = QtWidgets.QPushButton("Settings")
        self._btn_launch_settings.setObjectName("btnLaunchSettings")
        self._btn_launch_settings.setMinimumHeight(36)
        self._btn_launch_settings.setCursor(QtCore.Qt.PointingHandCursor)

        self._btn_launch = QtWidgets.QPushButton("Start")
        self._btn_launch.setObjectName("btnLaunch")
        self._btn_launch.setMinimumHeight(36)
        self._btn_launch.setCursor(QtCore.Qt.PointingHandCursor)

        self._btn_stop_launch = QtWidgets.QPushButton("Stop Nodes")
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
        launch_label = QtWidgets.QLabel("Launch Status")
        launch_label.setObjectName("fieldLabel")
        self._launch_status = QtWidgets.QLabel("Not Started")
        self._launch_status.setObjectName("launchStatus")
        launch_status_row.addWidget(launch_label)
        launch_status_row.addWidget(self._launch_status, 1)
        launch_inner.addLayout(launch_status_row)

        layout.addWidget(launch_box, 5, 0, 1, 2, QtCore.Qt.AlignTop)

        layout.setRowStretch(0, 0)
        layout.setRowStretch(1, 0)
        layout.setRowStretch(2, 0)
        layout.setRowStretch(3, 0)
        layout.setRowStretch(4, 0)
        layout.setRowStretch(5, 0)
        layout.setRowStretch(6, 1)

        self.setStyleSheet(
            "QWidget { background: #f7f7f7; }"
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
        if color:
            label.setStyleSheet(f"color: {color};")
        else:
            label.setStyleSheet("")

    def _format_tool(self, tool_id):
        if tool_id == 1:
            return "Carbon Fiber"
        if tool_id == 2:
            return "Resin"
        return str(tool_id)

    def _on_extrude_scale_apply(self):
        text = self._extrude_scale_input.text().strip()
        if not text:
            self._set_extrude_status("Enter a scale value.", "#b42318")
            return
        try:
            val = float(text)
        except ValueError:
            self._set_extrude_status("Invalid scale value.", "#b42318")
            return
        if not (val > 0.0):
            self._set_extrude_status("Scale must be > 0.", "#b42318")
            return
        self._set_extrude_status("Submitting...", "#b15e00")
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
        self._set_value("System State", msg.state or "-", "#2b2b2b")
        self._update_control_buttons(msg.state or "")
        if msg.rsi_heartbeat_valid:
            self._set_value("Heartbeat Age", f"{msg.rsi_heartbeat_age_s:.3f}", "#1b6e3c")
            self._set_value("Heartbeat Seq", str(msg.rsi_heartbeat.seq_used), "#1b6e3c")
            self._set_value("Heartbeat IPOC", msg.rsi_heartbeat.ipoc or "-", "#2b2b2b")
            self._set_value("Heartbeat Tool", self._format_tool(msg.rsi_heartbeat.tool_id), "#2b2b2b")
            self._set_value("Heartbeat Extrude", f"{msg.rsi_heartbeat.extrude_abs:.3f}", "#2b2b2b")
        else:
            self._set_value("Heartbeat Age", "-", "#b42318")
            self._set_value("Heartbeat Seq", "-", "#b42318")
            self._set_value("Heartbeat IPOC", "-", "#b42318")
            self._set_value("Heartbeat Tool", "-", "#b42318")
            self._set_value("Heartbeat Extrude", "-", "#b42318")

        if msg.printhead_status_valid:
            ps = msg.printhead_status
            ready = "Yes" if ps.ready_for_motion else "No"
            ready_color = "#1b6e3c" if ps.ready_for_motion else "#b42318"
            self._set_value("Printhead Ready", ready, ready_color)
            self._set_value("Printhead Age", f"{msg.printhead_status_age_s:.3f}", "#1b6e3c")
            stamp_text = f"{ps.stamp.sec}.{ps.stamp.nanosec:09d}"
            self._set_value("Printhead Stamp", stamp_text, "#2b2b2b")
            self._set_value("Ready Event Seq", str(ps.ready_event_seq), "#2b2b2b")
            self._set_value("Ready Event Type", ps.ready_event_type or "-", "#2b2b2b")
            self._set_value("Current Tool", self._format_tool(ps.current_tool), "#2b2b2b")

            using_color = "#1b6e3c"
            cf_state = "USING" if ps.current_tool == 1 else "HOME"
            resin_state = "USING" if ps.current_tool == 2 else "HOME"
            self._set_value("Carbon Fiber State", cf_state, using_color if cf_state == "USING" else "#2b2b2b")
            self._set_value("Resin State", resin_state, using_color if resin_state == "USING" else "#2b2b2b")

            cf_fan_color = "#1b6e3c" if ps.fan_ok_cf else "#b42318"
            self._set_value("Carbon Fiber Fan OK", "Yes" if ps.fan_ok_cf else "No", cf_fan_color)
            self._set_value("Carbon Fiber Current Temp", f"{ps.current_temp_cf:.1f}", "#2b2b2b")
            self._set_value("Carbon Fiber Target Temp", f"{ps.target_temp_cf:.1f}", "#2b2b2b")

            resin_fan_color = "#1b6e3c" if ps.fan_ok_resin else "#b42318"
            self._set_value("Resin Fan OK", "Yes" if ps.fan_ok_resin else "No", resin_fan_color)
            self._set_value("Resin Current Temp", f"{ps.current_temp_resin:.1f}", "#2b2b2b")
            self._set_value("Resin Target Temp", f"{ps.target_temp_resin:.1f}", "#2b2b2b")
        else:
            missing_keys = [
                "Printhead Ready",
                "Printhead Age",
                "Printhead Stamp",
                "Ready Event Seq",
                "Ready Event Type",
                "Current Tool",
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
            "Confirm Stop",
            "Are you sure you want to STOP the print?\n\n"
            "This will raise the Z axis and then cut all communication.\n"
            "KUKA will trigger a safety stop.",
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

    _STATE_COLORS = {
        "RUNNING": "#1b6e3c",
        "WAIT_HEARTBEAT": "#b15e00",
        "HEARTBEAT_LOST": "#b42318",
        "PAUSED": "#1a73e8",
        "ABORTING": "#b42318",
        "ABORTED": "#b42318",
    }

    def _update_control_buttons(self, state):
        color = self._STATE_COLORS.get(state, "#2b2b2b")
        self._control_status.setText(state or "-")
        self._control_status.setStyleSheet(f"color: {color}; font-weight: 700; font-size: 13px;")
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


class MyProjectUiPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("MyProjectUiPlugin")

        if not rclpy.ok():
            rclpy.init(args=None)

        self._node = rclpy.create_node("my_project_ui_panel")
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
        self._launch_process = None
        self._widget._btn_launch_settings.clicked.connect(
            self._on_launch_settings
        )
        self._widget._btn_launch.clicked.connect(self._on_launch)
        self._widget._btn_stop_launch.clicked.connect(self._on_stop_launch)

        self._node.create_subscription(
            UiStatus, "/ui/status", self._on_status, 10
        )

        self._spin_timer = QtCore.QTimer(self._widget)
        self._spin_timer.timeout.connect(self._spin_once)
        self._spin_timer.start(50)

        self._launch_check_timer = QtCore.QTimer(self._widget)
        self._launch_check_timer.timeout.connect(self._check_launch_process)
        self._launch_check_timer.start(1000)

    def _on_status(self, msg: UiStatus):
        self._widget.status_received.emit(msg)

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
            self._widget.set_extrude_status("UART parameter service not ready.", "#b42318")
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
                self._widget.set_extrude_scale(value, "Submit failed.", "#b42318")
                return
            results = resp.results if resp is not None else []
            if results and all(r.successful for r in results):
                self._widget.set_extrude_scale(value, "Applied.", "#1b6e3c")
            else:
                reason = results[0].reason if results else "Submit failed"
                self._widget.set_extrude_scale(value, reason or "Submit failed.", "#b42318")

        future.add_done_callback(_done)

    def _spin_once(self):
        rclpy.spin_once(self._node, timeout_sec=0.0)

    # ---- Launch control ----

    def _on_launch_settings(self):
        dialog = _LaunchSettingsDialog(self._launch_params, self._widget)
        if dialog.exec_() == QtWidgets.QDialog.Accepted:
            self._launch_params = dialog.get_params()

    def _on_launch(self):
        if (
            self._launch_process is not None
            and self._launch_process.poll() is None
        ):
            return
        cmd = ["ros2", "launch", "my_project_startup", "startup.launch.py"]
        for name, value in self._launch_params.items():
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
            self._widget._launch_status.setText("Running (waiting for KUKA first packet...)")
            self._widget._launch_status.setStyleSheet(
                "color: #1b6e3c; font-weight: 700; font-size: 13px;"
            )
        except Exception as exc:
            self._widget._launch_status.setText(f"Launch failed: {exc}")
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
        self._widget._launch_status.setText("Stopped")
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
                self._widget._launch_status.setText("Exited")
                self._widget._launch_status.setStyleSheet(
                    "color: #666666; font-weight: 700; font-size: 13px;"
                )
            else:
                self._widget._launch_status.setText(
                    f"Exited abnormally (code {rc})"
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
