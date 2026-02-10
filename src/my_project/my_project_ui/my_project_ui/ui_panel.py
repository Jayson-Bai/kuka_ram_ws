from python_qt_binding import QtCore, QtWidgets, QtGui
from rqt_gui_py.plugin import Plugin
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters

from my_project_interfaces.msg import UiStatus


class _UiStatusWidget(QtWidgets.QWidget):
    status_received = QtCore.pyqtSignal(object)
    scale_submit = QtCore.pyqtSignal(float)

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
            ("CF State", "State"),
            ("CF Fan OK", "Fan OK"),
            ("CF Current Temp", "Current Temp"),
            ("CF Target Temp", "Target Temp"),
        ]
        resin_labels = [
            ("RESIN State", "State"),
            ("RESIN Fan OK", "Fan OK"),
            ("RESIN Current Temp", "Current Temp"),
            ("RESIN Target Temp", "Target Temp"),
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
        cf_box = add_group("CF", cf_labels,
        parent_layout=printhead_row, object_name="groupPrintheadCF",
            label_min_width=cf_resin_label_min_width,
            value_alignment=QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        resin_box = add_group("RESIN", resin_labels,
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
        extrude_box = QtWidgets.QGroupBox("Extrude Scale")
        extrude_box.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        extrude_layout = QtWidgets.QFormLayout(extrude_box)
        extrude_layout.setLabelAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        extrude_layout.setFormAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        extrude_layout.setFieldGrowthPolicy(QtWidgets.QFormLayout.AllNonFixedFieldsGrow)
        extrude_layout.setHorizontalSpacing(12)
        extrude_layout.setVerticalSpacing(6)
        label_title = QtWidgets.QLabel("Current")
        label_title.setObjectName("fieldLabel")
        self._extrude_scale_value = QtWidgets.QLabel("1.000")
        self._extrude_scale_value.setObjectName("valueLabel")
        self._extrude_scale_value.setMinimumWidth(value_min_width)
        extrude_layout.addRow(label_title, self._extrude_scale_value)

        input_row = QtWidgets.QWidget()
        input_row_layout = QtWidgets.QHBoxLayout(input_row)
        input_row_layout.setContentsMargins(0, 0, 0, 0)
        input_row_layout.setSpacing(6)
        self._extrude_scale_input = QtWidgets.QLineEdit()
        self._extrude_scale_input.setPlaceholderText("1.0")
        validator = QtGui.QDoubleValidator(0.001, 1000.0, 3, self._extrude_scale_input)
        validator.setNotation(QtGui.QDoubleValidator.StandardNotation)
        self._extrude_scale_input.setValidator(validator)
        self._extrude_scale_apply = QtWidgets.QPushButton("Apply")
        input_row_layout.addWidget(self._extrude_scale_input)
        input_row_layout.addWidget(self._extrude_scale_apply)
        extrude_layout.addRow(QtWidgets.QLabel("Set"), input_row)

        self._extrude_scale_status = QtWidgets.QLabel("-")
        self._extrude_scale_status.setObjectName("valueLabel")
        extrude_layout.addRow(QtWidgets.QLabel("Status"), self._extrude_scale_status)
        # extrude_box 采用与原 Logs 相同的整行布局

        layout.addWidget(system_box, 1, 0, 1, 2, QtCore.Qt.AlignTop)
        layout.addLayout(left_column, 2, 0, 1, 1)
        layout.addLayout(right_column, 2, 1, 1, 1)
        layout.addWidget(extrude_box, 3, 0, 1, 2, QtCore.Qt.AlignTop)
        layout.setRowStretch(0, 0)
        layout.setRowStretch(1, 0)
        layout.setRowStretch(2, 0)
        layout.setRowStretch(3, 0)
        layout.setRowStretch(4, 1)

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
        )

        self._extrude_scale_apply.clicked.connect(self._on_extrude_scale_apply)
        self._extrude_scale_input.returnPressed.connect(self._on_extrude_scale_apply)

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
            return "CF"
        if tool_id == 2:
            return "RESIN"
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
            self._set_value("CF State", cf_state, using_color if cf_state == "USING" else "#2b2b2b")
            self._set_value("RESIN State", resin_state, using_color if resin_state == "USING" else "#2b2b2b")

            cf_fan_color = "#1b6e3c" if ps.fan_ok_cf else "#b42318"
            self._set_value("CF Fan OK", "Yes" if ps.fan_ok_cf else "No", cf_fan_color)
            self._set_value("CF Current Temp", f"{ps.current_temp_cf:.1f}", "#2b2b2b")
            self._set_value("CF Target Temp", f"{ps.target_temp_cf:.1f}", "#2b2b2b")

            resin_fan_color = "#1b6e3c" if ps.fan_ok_resin else "#b42318"
            self._set_value("RESIN Fan OK", "Yes" if ps.fan_ok_resin else "No", resin_fan_color)
            self._set_value("RESIN Current Temp", f"{ps.current_temp_resin:.1f}", "#2b2b2b")
            self._set_value("RESIN Target Temp", f"{ps.target_temp_resin:.1f}", "#2b2b2b")
        else:
            missing_keys = [
                "Printhead Ready",
                "Printhead Age",
                "Printhead Stamp",
                "Ready Event Seq",
                "Ready Event Type",
                "Current Tool",
                "CF State",
                "CF Fan OK",
                "CF Current Temp",
                "CF Target Temp",
                "RESIN State",
                "RESIN Fan OK",
                "RESIN Current Temp",
                "RESIN Target Temp",
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
        self._widget = _UiStatusWidget()
        self._widget.scale_submit.connect(self._on_scale_submit)
        context.add_widget(self._widget)

        self._node.create_subscription(
            UiStatus, "/ui/status", self._on_status, 10
        )

        self._spin_timer = QtCore.QTimer(self._widget)
        self._spin_timer.timeout.connect(self._spin_once)
        self._spin_timer.start(50)

    def _on_status(self, msg: UiStatus):
        self._widget.status_received.emit(msg)

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
                reason = results[0].reason if results else "提交失败"
                self._widget.set_extrude_scale(value, reason or "Submit failed.", "#b42318")

        future.add_done_callback(_done)

    def _spin_once(self):
        rclpy.spin_once(self._node, timeout_sec=0.0)

    def shutdown_plugin(self):
        if self._spin_timer.isActive():
            self._spin_timer.stop()
        self._node.destroy_node()
