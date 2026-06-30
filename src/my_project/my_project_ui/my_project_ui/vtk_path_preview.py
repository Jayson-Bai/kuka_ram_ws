from __future__ import annotations

from pathlib import Path
import json
import math
import threading

try:
    from python_qt_binding import QtCore, QtWidgets
except ImportError:
    try:
        from PyQt5 import QtCore, QtWidgets
    except ImportError:
        from PySide6 import QtCore, QtWidgets

from gcode_planner.path_preview import (
    PathType,
    PreviewPath,
    extract_layer_preview_paths,
    list_preview_layers,
)


_MAX_POINTS_PER_PATH = 1000
_FIXED_PLANE_SIZE_MM = 500.0
_FIXED_PLANE_GRID_STEP_MM = 50.0
_BEAD_DIMENSIONS_MM = {
    PathType.FIBER_PRINT: (1.0, 0.1),
    PathType.RESIN_PRINT: (2.0, 0.5),
}

_PATH_COLORS = {
    PathType.FIBER_PRINT: (0.0, 0.75, 0.45),
    PathType.RESIN_PRINT: (0.1, 0.35, 1.0),
    PathType.TRAVEL: (0.55, 0.55, 0.55),
    PathType.TOOL_CHANGE_EVENT: (1.0, 0.45, 0.05),
    PathType.EVENT: (0.8, 0.8, 0.2),
}


_SIGNAL = getattr(QtCore, "pyqtSignal", None) or getattr(QtCore, "Signal")


def _load_vtk_modules():
    from vtkmodules.qt.QVTKRenderWindowInteractor import (
        QVTKRenderWindowInteractor,
    )
    from vtkmodules.vtkCommonCore import vtkPoints
    from vtkmodules.vtkCommonDataModel import (
        vtkCellArray,
        vtkPolyData,
        vtkPolyLine,
    )
    from vtkmodules.vtkInteractionStyle import (
        vtkInteractorStyleTrackballCamera,
    )
    from vtkmodules.vtkFiltersCore import vtkAppendPolyData
    from vtkmodules.vtkFiltersSources import (
        vtkArrowSource,
        vtkCylinderSource,
        vtkSphereSource,
    )
    from vtkmodules.vtkRenderingFreeType import vtkVectorText
    from vtkmodules.vtkRenderingCore import (
        vtkActor,
        vtkFollower,
        vtkPolyDataMapper,
        vtkRenderer,
    )
    from vtkmodules.vtkCommonTransforms import vtkTransform
    from vtkmodules.vtkFiltersGeneral import vtkTransformPolyDataFilter
    import vtkmodules.vtkInteractionStyle  # noqa: F401
    import vtkmodules.vtkRenderingOpenGL2  # noqa: F401

    return {
        "QVTKRenderWindowInteractor": QVTKRenderWindowInteractor,
        "vtkActor": vtkActor,
        "vtkAppendPolyData": vtkAppendPolyData,
        "vtkArrowSource": vtkArrowSource,
        "vtkCellArray": vtkCellArray,
        "vtkCylinderSource": vtkCylinderSource,
        "vtkPoints": vtkPoints,
        "vtkPolyData": vtkPolyData,
        "vtkPolyDataMapper": vtkPolyDataMapper,
        "vtkFollower": vtkFollower,
        "vtkPolyLine": vtkPolyLine,
        "vtkSphereSource": vtkSphereSource,
        "vtkTransform": vtkTransform,
        "vtkTransformPolyDataFilter": vtkTransformPolyDataFilter,
        "vtkVectorText": vtkVectorText,
        "vtkInteractorStyleTrackballCamera": vtkInteractorStyleTrackballCamera,
        "vtkRenderer": vtkRenderer,
    }


def _sample_points(points):
    if len(points) <= _MAX_POINTS_PER_PATH:
        return points
    step = max(1, len(points) // _MAX_POINTS_PER_PATH)
    sampled = points[::step]
    if sampled[-1] != points[-1]:
        sampled = sampled + (points[-1],)
    return sampled


class VtkPathPreviewDialog(QtWidgets.QDialog):
    _layers_loaded = _SIGNAL(object, object)
    _paths_loaded = _SIGNAL(int, object, object)

    def __init__(self, npz_root: str, parent=None):
        super().__init__(parent)
        self._npz_root = Path(npz_root).expanduser()
        self._tool_offset_xyz, self._preview_z_origin = (
            self._read_preview_offsets()
        )
        self._layers = []
        self._current_paths: list[PreviewPath] = []
        self._loading_layer = None
        self._vtk = None
        self._renderer = None
        self._vtk_widget = None
        self._actors = []

        self.setWindowTitle(f"VTK Path Preview - {self._npz_root.name}")
        self.resize(1100, 760)
        self._build_ui()
        self._try_build_vtk_view()
        self._layers_loaded.connect(self._on_layers_loaded)
        self._paths_loaded.connect(self._on_paths_loaded)
        QtCore.QTimer.singleShot(100, self._load_layers)

    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        top_row = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel(f"复合打印工艺路径 - {self._npz_root.name}")
        title.setStyleSheet(
            "font-size: 15px; font-weight: 700; color: #2b2b2b;"
        )
        btn_close = QtWidgets.QPushButton("关闭")
        btn_close.setFixedWidth(56)
        btn_close.clicked.connect(self.accept)
        top_row.addWidget(title, 1)
        top_row.addWidget(btn_close)
        layout.addLayout(top_row)

        filter_row = QtWidgets.QHBoxLayout()
        self._show_fiber = QtWidgets.QCheckBox("纤维路径")
        self._show_resin = QtWidgets.QCheckBox("树脂路径")
        self._show_travel = QtWidgets.QCheckBox("空走路径")
        self._show_tool_change = QtWidgets.QCheckBox("工具切换")
        self._show_endpoints = QtWidgets.QCheckBox("起/终点")
        default_enabled = {
            self._show_fiber,
            self._show_resin,
        }
        for checkbox in (
            self._show_fiber,
            self._show_resin,
            self._show_travel,
            self._show_tool_change,
        ):
            checkbox.setChecked(checkbox in default_enabled)
            checkbox.stateChanged.connect(self._on_filter_changed)
            filter_row.addWidget(checkbox)
        self._show_endpoints.setChecked(False)
        self._show_endpoints.stateChanged.connect(
            lambda _state: self._update_scene()
        )
        filter_row.addWidget(self._show_endpoints)
        filter_row.addStretch()
        self._btn_top_view = QtWidgets.QPushButton("顶视")
        self._btn_iso_view = QtWidgets.QPushButton("斜视")
        self._btn_top_view.clicked.connect(self._set_top_view)
        self._btn_iso_view.clicked.connect(self._set_iso_view)
        filter_row.addWidget(self._btn_top_view)
        filter_row.addWidget(self._btn_iso_view)
        layout.addLayout(filter_row)

        self._viewport = QtWidgets.QStackedWidget()
        self._fallback_label = QtWidgets.QLabel("正在初始化三维视图...")
        self._fallback_label.setAlignment(QtCore.Qt.AlignCenter)
        self._fallback_label.setWordWrap(True)
        self._fallback_label.setStyleSheet(
            "color: #7a4b00; font-size: 14px;"
        )
        self._viewport.addWidget(self._fallback_label)
        layout.addWidget(self._viewport, 1)

        slider_grid = QtWidgets.QGridLayout()
        self._layer_label = QtWidgets.QLabel("层: 正在扫描...")
        self._layer_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self._layer_slider.setMinimum(0)
        self._layer_slider.valueChanged.connect(self._load_current_layer)
        self._path_label = QtWidgets.QLabel("当前层路径: 等待加载")
        self._path_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self._path_slider.setMinimum(0)
        self._path_slider.valueChanged.connect(
            lambda _value: self._update_scene()
        )
        slider_grid.addWidget(self._layer_label, 0, 0)
        slider_grid.addWidget(self._layer_slider, 0, 1)
        slider_grid.addWidget(self._path_label, 1, 0)
        slider_grid.addWidget(self._path_slider, 1, 1)
        layout.addLayout(slider_grid)

        self._layer_slider.setEnabled(False)
        self._path_slider.setEnabled(False)

    def _try_build_vtk_view(self):
        try:
            self._vtk = _load_vtk_modules()
        except Exception as exc:
            self._fallback_label.setText(
                "VTK 未安装或无法加载。\n"
                "Linux 可安装 python3-vtk9；Windows 发布包需要内置 vtk 运行库。\n"
                f"错误: {exc}"
            )
            self._viewport.setCurrentWidget(self._fallback_label)
            return

        self._vtk_widget = self._vtk["QVTKRenderWindowInteractor"](self)
        self._renderer = self._vtk["vtkRenderer"]()
        self._renderer.SetBackground(1.0, 1.0, 1.0)
        self._vtk_widget.GetRenderWindow().AddRenderer(self._renderer)
        interactor = self._vtk_widget.GetRenderWindow().GetInteractor()
        if interactor is not None:
            style = self._vtk["vtkInteractorStyleTrackballCamera"]()
            interactor.SetInteractorStyle(style)
        self._viewport.addWidget(self._vtk_widget)
        self._viewport.setCurrentWidget(self._vtk_widget)
        self._vtk_widget.Initialize()
        self._vtk_widget.Start()

    def _read_preview_offsets(self):
        for path in self._offset_sidecar_candidates():
            try:
                with path.open("r", encoding="utf-8") as f:
                    data = json.load(f)
                offset = data.get("tool_offset", (0.0, 0.0, 0.0))
                if len(offset) != 3:
                    offset = (0.0, 0.0, 0.0)
                preview_z_origin = float(
                    data.get("resin_z_print_compensation_mm", 0.0)
                )
                return tuple(float(v) for v in offset), preview_z_origin
            except Exception:
                continue
        return (0.0, 0.0, 0.0), 0.0

    def _offset_sidecar_candidates(self):
        root = self._npz_root
        candidates = []
        if root.is_file():
            candidates.append(root.with_suffix(".offset.json"))
        else:
            candidates.append(root / f"{root.name}.offset.json")
            candidates.extend(sorted(root.glob("*.offset.json")))
        seen = set()
        for path in candidates:
            if path in seen:
                continue
            seen.add(path)
            yield path

    def _run_background(self, target):
        thread = threading.Thread(target=target, daemon=True)
        thread.start()

    def _load_layers(self):
        self._layer_label.setText("层: 正在扫描...")

        def worker():
            try:
                layers = list_preview_layers(self._npz_root)
                self._layers_loaded.emit(layers, None)
            except Exception as exc:
                self._layers_loaded.emit([], str(exc))

        self._run_background(worker)

    def _on_layers_loaded(self, layers, error):
        if error:
            self._layer_label.setText("层: 加载失败")
            self._path_label.setText(f"当前层路径: {error}")
            return

        self._layers = list(layers)
        if self._layers:
            self._layer_slider.setEnabled(True)
            self._layer_slider.setMaximum(len(self._layers) - 1)
            self._path_slider.setEnabled(False)
        else:
            self._layer_slider.setEnabled(False)
            self._path_slider.setEnabled(False)
        self._load_current_layer()

    def _load_current_layer(self):
        if not self._layers:
            self._current_paths = []
            self._layer_label.setText("层: 0 / 0")
            self._path_label.setText("当前层路径: 0 / 0")
            self._update_scene()
            return

        layer = self._layers[self._layer_slider.value()]
        self._loading_layer = layer
        self._current_paths = []
        self._path_slider.blockSignals(True)
        self._path_slider.setMaximum(0)
        self._path_slider.setValue(0)
        self._path_slider.setEnabled(False)
        self._path_slider.blockSignals(False)
        self._layer_label.setText(
            f"层: {layer} "
            f"({self._layer_slider.value() + 1} / {len(self._layers)})"
        )
        self._path_label.setText("当前层路径: 正在加载...")
        self._update_scene()

        def worker():
            try:
                paths = extract_layer_preview_paths(
                    self._npz_root,
                    layer,
                    max_paths=2000,
                )
                self._paths_loaded.emit(layer, paths, None)
            except Exception as exc:
                self._paths_loaded.emit(layer, [], str(exc))

        self._run_background(worker)

    def _on_paths_loaded(self, layer, paths, error):
        if layer != self._loading_layer:
            return
        if error:
            self._path_label.setText(f"当前层路径: {error}")
            return

        self._current_paths = list(paths)
        self._path_slider.blockSignals(True)
        visible_count = self._enabled_path_count()
        self._path_slider.setMaximum(visible_count)
        self._path_slider.setValue(visible_count)
        self._path_slider.setEnabled(bool(visible_count))
        self._path_slider.blockSignals(False)
        self._update_scene(reset_camera=True)

    def _enabled_types(self) -> set[PathType]:
        enabled = set()
        if self._show_fiber.isChecked():
            enabled.add(PathType.FIBER_PRINT)
        if self._show_resin.isChecked():
            enabled.add(PathType.RESIN_PRINT)
        if self._show_travel.isChecked():
            enabled.add(PathType.TRAVEL)
        if self._show_tool_change.isChecked():
            enabled.add(PathType.TOOL_CHANGE_EVENT)
            enabled.add(PathType.EVENT)
        return enabled

    def _filtered_paths(self) -> list[PreviewPath]:
        enabled = self._enabled_types()
        return [
            path for path in self._current_paths
            if path.path_type in enabled
        ]

    def _display_paths(self) -> list[PreviewPath]:
        enabled_visible = self._filtered_paths()
        return enabled_visible or list(self._current_paths)

    def _visible_paths(self) -> list[PreviewPath]:
        return self._display_paths()[: self._path_slider.value()]

    def _enabled_path_count(self):
        return len(self._display_paths())

    def _filter_status_text(self):
        if self._current_paths and not self._filtered_paths():
            return "（当前过滤无路径，已显示全部类型）"
        return ""

    def _on_filter_changed(self):
        visible_count = self._enabled_path_count()
        self._path_slider.blockSignals(True)
        self._path_slider.setMaximum(visible_count)
        self._path_slider.setValue(visible_count)
        self._path_slider.setEnabled(bool(visible_count))
        self._path_slider.blockSignals(False)
        self._update_scene()

    def _update_scene(self, reset_camera=False):
        visible = self._visible_paths()
        self._path_label.setText(
            f"当前层路径: {min(self._path_slider.value(), len(visible))} / "
            f"{self._enabled_path_count()}"
            f"{self._filter_status_text()}"
        )
        if self._renderer is None:
            return

        for actor in self._actors:
            self._renderer.RemoveActor(actor)
        self._actors = []

        for actor in self._base_plane_actors(self._current_paths or visible):
            self._renderer.AddActor(actor)
            self._actors.append(actor)

        for actor in self._actors_for_paths(visible):
            self._renderer.AddActor(actor)
            self._actors.append(actor)

        current_path = visible[-1] if visible else None
        if current_path is not None:
            actor = self._nozzle_actor_for_path(current_path, visible)
            self._renderer.AddActor(actor)
            self._actors.append(actor)
        if self._show_endpoints.isChecked() and current_path is not None:
            for actor in self._endpoint_actors_for_path(current_path, visible):
                self._renderer.AddActor(actor)
                self._actors.append(actor)

        if reset_camera:
            self._renderer.ResetCamera()
        self._render()

    def _actors_for_paths(self, paths: list[PreviewPath]):
        grouped = {}
        for path in paths:
            grouped.setdefault(path.path_type, []).append(path)

        actors = []
        for path_type, grouped_paths in grouped.items():
            if path_type in (PathType.TOOL_CHANGE_EVENT, PathType.EVENT):
                actors.extend(
                    self._event_marker_actors(path_type, grouped_paths)
                )
                continue
            actor = self._actor_for_paths(path_type, grouped_paths)
            if actor is not None:
                actors.append(actor)

        return actors

    def _actor_for_paths(self, path_type: PathType, paths: list[PreviewPath]):
        if path_type in _BEAD_DIMENSIONS_MM:
            return self._bead_actor_for_paths(path_type, paths)
        return self._line_actor_for_paths(path_type, paths)

    def _bead_actor_for_paths(
        self,
        path_type: PathType,
        paths: list[PreviewPath],
    ):
        width, height = _BEAD_DIMENSIONS_MM[path_type]
        half_width = width / 2.0
        vtk_points = self._vtk["vtkPoints"]()
        cells = self._vtk["vtkCellArray"]()
        point_index = 0

        def add_quad(indices):
            cells.InsertNextCell(4)
            for index in indices:
                cells.InsertCellPoint(index)

        def bead_corners(point, normal, path_top_z):
            top_z = path_top_z
            bottom_z = top_z - height
            nx, ny = normal
            left_x = point[0] + nx * half_width
            left_y = point[1] + ny * half_width
            right_x = point[0] - nx * half_width
            right_y = point[1] - ny * half_width
            return (
                (left_x, left_y, top_z),
                (right_x, right_y, top_z),
                (left_x, left_y, bottom_z),
                (right_x, right_y, bottom_z),
            )

        for path in paths:
            points = [
                self._display_point_for_path(path, point)
                for point in _sample_points(path.points)
            ]
            if len(points) < 2:
                continue
            path_top_z = max(point[2] for point in points)
            for start, end in zip(points, points[1:]):
                dx = end[0] - start[0]
                dy = end[1] - start[1]
                length_xy = math.hypot(dx, dy)
                if length_xy <= 1e-9:
                    continue
                normal = (-dy / length_xy, dx / length_xy)
                corners = (
                    bead_corners(start, normal, path_top_z)
                    + bead_corners(end, normal, path_top_z)
                )
                indices = list(
                    range(point_index, point_index + len(corners))
                )
                for corner in corners:
                    vtk_points.InsertNextPoint(*corner)
                point_index += len(corners)

                add_quad((indices[0], indices[4], indices[5], indices[1]))
                add_quad((indices[2], indices[3], indices[7], indices[6]))
                add_quad((indices[0], indices[2], indices[6], indices[4]))
                add_quad((indices[1], indices[5], indices[7], indices[3]))
                add_quad((indices[0], indices[1], indices[3], indices[2]))
                add_quad((indices[4], indices[6], indices[7], indices[5]))

        if point_index == 0:
            return None

        poly_data = self._vtk["vtkPolyData"]()
        poly_data.SetPoints(vtk_points)
        poly_data.SetPolys(cells)

        mapper = self._vtk["vtkPolyDataMapper"]()
        mapper.SetInputData(poly_data)
        actor = self._vtk["vtkActor"]()
        actor.SetMapper(mapper)
        color = _PATH_COLORS.get(path_type, (1.0, 1.0, 1.0))
        actor.GetProperty().SetColor(*color)
        actor.GetProperty().SetOpacity(0.82)
        return actor

    def _line_actor_for_paths(
        self,
        path_type: PathType,
        paths: list[PreviewPath],
    ):
        vtk_points = self._vtk["vtkPoints"]()
        cells = self._vtk["vtkCellArray"]()
        point_index = 0

        for path in paths:
            points = _sample_points(path.points)
            if not points:
                continue
            polyline = self._vtk["vtkPolyLine"]()
            polyline.GetPointIds().SetNumberOfIds(len(points))
            for local_index, point in enumerate(points):
                vtk_points.InsertNextPoint(
                    *self._display_point_for_path(path, point)
                )
                polyline.GetPointIds().SetId(local_index, point_index)
                point_index += 1
            cells.InsertNextCell(polyline)

        if point_index == 0:
            return None

        poly_data = self._vtk["vtkPolyData"]()
        poly_data.SetPoints(vtk_points)
        poly_data.SetLines(cells)

        mapper = self._vtk["vtkPolyDataMapper"]()
        mapper.SetInputData(poly_data)
        actor = self._vtk["vtkActor"]()
        actor.SetMapper(mapper)
        color = _PATH_COLORS.get(path_type, (1.0, 1.0, 1.0))
        actor.GetProperty().SetColor(*color)
        actor.GetProperty().SetLineWidth(2.0)
        if path_type == PathType.TRAVEL:
            actor.GetProperty().SetOpacity(0.45)
        return actor

    def _base_plane_actors(self, paths: list[PreviewPath]):
        del paths
        min_x = 0.0
        max_x = _FIXED_PLANE_SIZE_MM
        min_y = 0.0
        max_y = _FIXED_PLANE_SIZE_MM
        z = 0.0
        step = _FIXED_PLANE_GRID_STEP_MM

        actors = [self._grid_actor(min_x, max_x, min_y, max_y, z, step)]
        axis_len = _FIXED_PLANE_SIZE_MM * 0.18
        for axis, color, direction, label_position in (
            (
                "X",
                (0.9, 0.05, 0.05),
                (axis_len, 0.0, 0.0),
                (axis_len * 1.04, 0.0, z),
            ),
            (
                "Y",
                (0.05, 0.55, 0.1),
                (0.0, axis_len, 0.0),
                (0.0, axis_len * 1.04, z),
            ),
            (
                "Z",
                (0.1, 0.25, 0.95),
                (0.0, 0.0, axis_len),
                (0.0, 0.0, z + axis_len * 1.04),
            ),
        ):
            actors.append(
                self._axis_arrow_actor((0.0, 0.0, z), direction, color)
            )
            actors.append(
                self._dimension_label_actor(
                    axis, label_position, color, step * 0.18
                )
            )
        actors.append(
            self._dimension_label_actor(
                f"{_FIXED_PLANE_SIZE_MM:.0f} x {_FIXED_PLANE_SIZE_MM:.0f} mm",
                (min_x, max_y, z),
                (0.2, 0.2, 0.2),
                step * 0.18,
            )
        )
        return actors

    def _path_bounds(self, paths: list[PreviewPath]):
        points = [point for path in paths for point in path.points]
        if not points:
            return -50.0, 50.0, -50.0, 50.0, 0.0, 50.0
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        zs = [point[2] for point in points]
        return min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)

    def _grid_step(self, span):
        raw = max(float(span) / 8.0, 1.0)
        magnitude = 10 ** math.floor(math.log10(raw))
        normalized = raw / magnitude
        if normalized <= 2.0:
            return 2.0 * magnitude
        if normalized <= 5.0:
            return 5.0 * magnitude
        return 10.0 * magnitude

    def _grid_actor(self, min_x, max_x, min_y, max_y, z, step):
        vtk_points = self._vtk["vtkPoints"]()
        cells = self._vtk["vtkCellArray"]()
        idx = 0

        def add_line(start, end):
            nonlocal idx
            line = self._vtk["vtkPolyLine"]()
            line.GetPointIds().SetNumberOfIds(2)
            vtk_points.InsertNextPoint(*start)
            vtk_points.InsertNextPoint(*end)
            line.GetPointIds().SetId(0, idx)
            line.GetPointIds().SetId(1, idx + 1)
            idx += 2
            cells.InsertNextCell(line)

        x0 = math.floor(min_x / step) * step
        while x0 <= max_x + 1e-6:
            add_line((x0, min_y, z), (x0, max_y, z))
            x0 += step
        y0 = math.floor(min_y / step) * step
        while y0 <= max_y + 1e-6:
            add_line((min_x, y0, z), (max_x, y0, z))
            y0 += step

        poly_data = self._vtk["vtkPolyData"]()
        poly_data.SetPoints(vtk_points)
        poly_data.SetLines(cells)
        mapper = self._vtk["vtkPolyDataMapper"]()
        mapper.SetInputData(poly_data)
        actor = self._vtk["vtkActor"]()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.82, 0.84, 0.86)
        actor.GetProperty().SetLineWidth(1.0)
        return actor

    def _axis_arrow_actor(self, origin, direction, color):
        source = self._vtk["vtkArrowSource"]()
        source.SetShaftRadius(0.025)
        source.SetTipRadius(0.08)
        source.SetTipLength(0.25)
        length = max(
            (direction[0] ** 2 + direction[1] ** 2 + direction[2] ** 2) ** 0.5,
            1.0,
        )
        transform = self._vtk["vtkTransform"]()
        transform.Translate(*origin)
        if abs(direction[1]) > 1e-9:
            transform.RotateZ(90.0)
        elif abs(direction[2]) > 1e-9:
            transform.RotateY(-90.0)
        transform.Scale(length, length, length)
        filt = self._vtk["vtkTransformPolyDataFilter"]()
        filt.SetInputConnection(source.GetOutputPort())
        filt.SetTransform(transform)
        mapper = self._vtk["vtkPolyDataMapper"]()
        mapper.SetInputConnection(filt.GetOutputPort())
        actor = self._vtk["vtkActor"]()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(*color)
        actor.GetProperty().SetOpacity(0.42)
        return actor

    def _dimension_label_actor(self, text, position, color, scale):
        source = self._vtk["vtkVectorText"]()
        source.SetText(str(text))
        mapper = self._vtk["vtkPolyDataMapper"]()
        mapper.SetInputConnection(source.GetOutputPort())
        actor = self._vtk["vtkFollower"]()
        actor.SetMapper(mapper)
        actor.SetScale(scale, scale, scale)
        actor.SetPosition(*position)
        actor.GetProperty().SetColor(*color)
        actor.GetProperty().SetOpacity(0.68)
        if self._renderer is not None:
            actor.SetCamera(self._renderer.GetActiveCamera())
        return actor

    def _event_marker_actors(
        self,
        path_type: PathType,
        paths: list[PreviewPath],
    ):
        color = _PATH_COLORS.get(path_type, (1.0, 0.45, 0.05))
        all_points = [
            point for path in self._current_paths for point in path.points
        ]
        radius = self._endpoint_radius(all_points) * 1.5
        return [
            self._sphere_actor(
                self._display_point_for_path(path, path.end), radius, color
            )
            for path in paths
        ]

    def _nozzle_actor_for_path(
        self,
        current_path: PreviewPath,
        visible_paths: list[PreviewPath],
    ):
        all_points = [
            self._display_point_for_path(path, point)
            for path in (visible_paths or [current_path])
            for point in path.points
        ]
        radius = self._endpoint_radius(all_points)
        body_radius = radius * 0.45
        body_height = radius * 7.0
        body = self._vtk["vtkCylinderSource"]()
        body.SetRadius(body_radius)
        body.SetHeight(body_height)
        body.SetResolution(24)
        hemisphere = self._vtk["vtkSphereSource"]()
        hemisphere.SetRadius(radius)
        hemisphere.SetThetaResolution(24)
        hemisphere.SetPhiResolution(12)
        hemisphere.SetStartPhi(90.0)
        hemisphere.SetEndPhi(180.0)

        body_transform = self._vtk["vtkTransform"]()
        body_transform.PostMultiply()
        body_transform.RotateX(90.0)
        body_transform.Translate(0.0, 0.0, radius + body_height / 2.0)
        hemisphere_transform = self._vtk["vtkTransform"]()
        hemisphere_transform.Translate(0.0, 0.0, radius)
        append = self._vtk["vtkAppendPolyData"]()
        for source, transform in (
            (body, body_transform),
            (hemisphere, hemisphere_transform),
        ):
            filt = self._vtk["vtkTransformPolyDataFilter"]()
            filt.SetInputConnection(source.GetOutputPort())
            filt.SetTransform(transform)
            append.AddInputConnection(filt.GetOutputPort())

        mapper = self._vtk["vtkPolyDataMapper"]()
        mapper.SetInputConnection(append.GetOutputPort())
        actor = self._vtk["vtkActor"]()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(
            *self._nozzle_color_for_path(current_path)
        )
        actor.GetProperty().SetOpacity(0.92)
        display_end = self._display_point_for_path(
            current_path, current_path.end
        )
        self._apply_xyzabc_transform(
            actor, display_end, current_path.end_abc
        )
        return actor

    def _display_point_for_path(self, path: PreviewPath, point):
        x, y, z = point
        if int(path.tool_id) == 1:
            x -= self._tool_offset_xyz[0]
            y -= self._tool_offset_xyz[1]
            z -= self._tool_offset_xyz[2]
        z -= self._preview_z_origin
        return x, y, z

    def _nozzle_color_for_path(self, path: PreviewPath):
        if path.path_type == PathType.FIBER_PRINT or path.tool_id == 1:
            return _PATH_COLORS[PathType.FIBER_PRINT]
        if path.path_type == PathType.RESIN_PRINT or path.tool_id == 2:
            return _PATH_COLORS[PathType.RESIN_PRINT]
        return _PATH_COLORS.get(path.path_type, (0.12, 0.12, 0.12))

    def _apply_xyzabc_transform(self, actor, xyz, abc):
        # ABC=0 keeps the local tangent point at xyz; the nozzle body extends
        # upward from that point, so its rounded bottom touches the path.
        transform = self._vtk["vtkTransform"]()
        transform.PostMultiply()
        transform.RotateZ(float(abc[0]))
        transform.RotateY(float(abc[1]))
        transform.RotateX(float(abc[2]))
        transform.Translate(*xyz)
        actor.SetUserTransform(transform)

    def _endpoint_actors_for_path(
        self,
        path: PreviewPath,
        visible_paths: list[PreviewPath],
    ):
        bounds_points = [
            self._display_point_for_path(item, point)
            for item in visible_paths
            for point in item.points
        ]
        radius = self._endpoint_radius(bounds_points)
        return [
            self._sphere_actor(
                self._display_point_for_path(path, path.start),
                radius,
                (0.0, 0.9, 0.25),
            ),
            self._sphere_actor(
                self._display_point_for_path(path, path.end),
                radius,
                (1.0, 0.15, 0.05),
            ),
        ]

    def _endpoint_radius(self, points):
        if not points:
            return 1.0
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        zs = [point[2] for point in points]
        dx = max(xs) - min(xs)
        dy = max(ys) - min(ys)
        dz = max(zs) - min(zs)
        diagonal = max((dx * dx + dy * dy + dz * dz) ** 0.5, 1.0)
        return max(0.6, min(3.0, diagonal * 0.01))

    def _sphere_actor(self, center, radius, color):
        source = self._vtk["vtkSphereSource"]()
        source.SetCenter(*center)
        source.SetRadius(float(radius))
        source.SetThetaResolution(16)
        source.SetPhiResolution(16)
        mapper = self._vtk["vtkPolyDataMapper"]()
        mapper.SetInputConnection(source.GetOutputPort())
        actor = self._vtk["vtkActor"]()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(*color)
        actor.GetProperty().SetOpacity(1.0)
        return actor

    def _set_top_view(self):
        if self._renderer is None:
            return
        camera = self._renderer.GetActiveCamera()
        camera.SetPosition(0.0, 0.0, 1.0)
        camera.SetFocalPoint(0.0, 0.0, 0.0)
        camera.SetViewUp(0.0, 1.0, 0.0)
        self._renderer.ResetCamera()
        self._render()

    def _set_iso_view(self):
        if self._renderer is None:
            return
        camera = self._renderer.GetActiveCamera()
        camera.SetPosition(1.0, -1.0, 0.7)
        camera.SetFocalPoint(0.0, 0.0, 0.0)
        camera.SetViewUp(0.0, 0.0, 1.0)
        self._renderer.ResetCamera()
        self._render()

    def _render(self):
        if self._vtk_widget is not None:
            self._vtk_widget.GetRenderWindow().Render()
