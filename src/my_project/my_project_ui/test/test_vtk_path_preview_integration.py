from pathlib import Path


UI_PACKAGE = Path(__file__).resolve().parents[1] / "my_project_ui"
UI_PANEL = UI_PACKAGE / "ui_panel.py"
VTK_PREVIEW = UI_PACKAGE / "vtk_path_preview.py"


def test_vtk_path_preview_module_keeps_vtk_optional():
    src = VTK_PREVIEW.read_text(encoding="utf-8")

    assert "class VtkPathPreviewDialog" in src
    assert "def _load_vtk_modules" in src
    assert (
        "from vtkmodules.qt.QVTKRenderWindowInteractor import" in src
    )
    assert "VTK 未安装" in src


def test_vtk_path_preview_dialog_exposes_process_path_controls():
    src = VTK_PREVIEW.read_text(encoding="utf-8")

    assert "self._layer_slider" in src
    assert "self._path_slider" in src
    assert "self._show_fiber" in src
    assert "self._show_resin" in src
    assert "self._show_travel" in src
    assert "self._show_tool_change" in src
    assert "当前层路径" in src
    assert "extract_layer_preview_paths" in src


def test_ui_panel_adds_vtk_path_preview_entry_without_replacing_png_preview():
    src = UI_PANEL.read_text(encoding="utf-8")

    assert (
        "self._btn_view_layers = QtWidgets.QPushButton(\"查看层图像\")"
        in src
    )
    assert (
        "self._btn_view_vtk_paths = "
        "QtWidgets.QPushButton(\"三维路径预览\")" in src
    )
    assert (
        "self._btn_view_vtk_paths.clicked.connect(self._on_view_vtk_paths)"
        in src
    )
    assert "def _on_view_vtk_paths(self):" in src
    assert (
        "from my_project_ui.vtk_path_preview import VtkPathPreviewDialog"
        in src
    )


def test_vtk_path_preview_constructor_defers_heavy_npz_loading():
    src = VTK_PREVIEW.read_text(encoding="utf-8")
    init_block = src.split("    def __init__(", 1)[1].split(
        "    def _build_ui", 1
    )[0]

    assert "list_preview_layers" not in init_block
    assert "_load_current_layer" not in init_block
    assert "QtCore.QTimer.singleShot(100, self._load_layers)" in init_block
    assert "_load_layers" in src


def test_vtk_path_preview_does_not_render_all_paths_on_layer_load():
    src = VTK_PREVIEW.read_text(encoding="utf-8")
    load_block = src.split("    def _load_current_layer", 1)[1].split(
        "    def _enabled_types", 1
    )[0]

    assert "visible_count = self._enabled_path_count()" in load_block
    assert "self._path_slider.setMaximum(visible_count)" in load_block
    assert "self._path_slider.setValue(visible_count)" in load_block
    assert "setValue(len(self._current_paths))" not in load_block


def test_vtk_origin_axes_are_hidden_by_default_and_have_toggle():
    src = VTK_PREVIEW.read_text(encoding="utf-8")

    assert 'self._show_origin_axes = QtWidgets.QCheckBox("原点坐标系")' in src
    assert 'self._show_origin_axes.setChecked(False)' in src
    assert 'self._show_origin_axes.stateChanged.connect' in src

    update_block = src.split("    def _update_scene", 1)[1].split(
        "    def _actors_for_paths", 1
    )[0]
    assert "for actor in self._base_plane_actors(" in update_block
    assert "show_origin_axes=self._show_origin_axes.isChecked()" in update_block

    base_plane_block = src.split("    def _base_plane_actors", 1)[1].split(
        "    def _grid_actor", 1
    )[0]
    assert "show_origin_axes: bool = False" in base_plane_block
    assert "actors = [self._grid_actor" in base_plane_block
    assert "if show_origin_axes:" in base_plane_block
    assert base_plane_block.index("actors = [self._grid_actor") < base_plane_block.index("if show_origin_axes:")


def test_vtk_path_preview_loads_npz_data_on_background_threads():
    src = VTK_PREVIEW.read_text(encoding="utf-8")

    assert "import threading" in src
    assert "def _run_background" in src
    assert "threading.Thread(target=target, daemon=True)" in src
    assert "_layers_loaded = _SIGNAL(object, object)" in src
    assert "_paths_loaded = _SIGNAL(int, object, object)" in src
    assert "list_preview_layers(self._npz_root)" in src
    assert (
        "extract_layer_preview_paths(" in src
        and "max_paths=2000" in src
        and "max_rows=120000" in src
    )


def test_vtk_path_preview_groups_paths_into_few_vtk_actors():
    src = VTK_PREVIEW.read_text(encoding="utf-8")

    assert "def _actors_for_paths" in src
    assert "grouped.setdefault(path.path_type, []).append(path)" in src
    assert "def _actor_for_paths" in src
    assert "_MAX_POINTS_PER_PATH" in src
    assert "def _sample_points" in src


def test_vtk_path_preview_caps_render_geometry_for_large_npz_layers():
    src = VTK_PREVIEW.read_text(encoding="utf-8")

    assert "_MAX_RENDER_POINTS_PER_ACTOR" in src
    assert "_MAX_BEAD_SEGMENTS_PER_ACTOR" in src
    assert "def _sample_limit_for_paths" in src
    assert "def _sample_points(points, max_points=_MAX_POINTS_PER_PATH)" in src

    bead_block = src.split("    def _bead_actor_for_paths", 1)[1].split(
        "    def _line_actor_for_paths", 1
    )[0]
    line_block = src.split("    def _line_actor_for_paths", 1)[1].split(
        "    def _base_plane_actors", 1
    )[0]
    assert "sample_limit = _sample_limit_for_paths(" in bead_block
    assert "_MAX_BEAD_SEGMENTS_PER_ACTOR" in bead_block
    assert "_sample_points(path.points, max_points=sample_limit)" in bead_block
    assert "sample_limit = _sample_limit_for_paths(" in line_block
    assert "_MAX_RENDER_POINTS_PER_ACTOR" in line_block
    assert "_sample_points(path.points, max_points=sample_limit)" in line_block


def test_vtk_path_preview_defaults_to_complete_print_paths_and_trackball():
    src = VTK_PREVIEW.read_text(encoding="utf-8")

    assert "vtkInteractorStyleTrackballCamera" in src
    assert "interactor.SetInteractorStyle(style)" in src
    assert "self._vtk_widget.Start()" in src
    assert "self._show_travel" in src
    assert "default_enabled = {" in src
    assert "self._show_fiber" in src
    assert "self._show_resin" in src
    assert "visible_count = self._enabled_path_count()" in src
    assert "self._path_slider.setValue(visible_count)" in src


def test_vtk_path_preview_uses_ascii_window_title_and_endpoint_toggle():
    src = VTK_PREVIEW.read_text(encoding="utf-8")

    assert (
        'self.setWindowTitle(f"VTK Path Preview - {self._npz_root.name}")'
        in src
    )
    assert 'self._show_endpoints = QtWidgets.QCheckBox("起/终点")' in src
    assert 'vtkSphereSource' in src
    assert 'def _endpoint_actors_for_path' in src
    assert 'self._show_endpoints.isChecked()' in src


def test_vtk_path_preview_falls_back_when_enabled_filters_hide_layer():
    src = VTK_PREVIEW.read_text(encoding="utf-8")

    assert "def _display_paths" in src
    assert "enabled_visible = self._filtered_paths()" in src
    assert "return enabled_visible or list(self._current_paths)" in src
    assert "当前过滤无路径" in src


def test_vtk_path_preview_adds_white_background_dimension_plane_and_axes():
    src = VTK_PREVIEW.read_text(encoding="utf-8")

    assert "SetBackground(1.0, 1.0, 1.0)" in src
    assert "def _base_plane_actors" in src
    assert "def _dimension_label_actor" in src
    assert "vtkArrowSource" in src
    assert (
        "from vtkmodules.vtkRenderingFreeType import vtkVectorText"
        in src
    )
    filters_sources_import = src.split(
        "from vtkmodules.vtkFiltersSources import", 1
    )[1].split(")", 1)[0]
    assert "vtkVectorText," not in filters_sources_import
    assert '"X",' in src
    assert "(0.9, 0.05, 0.05)," in src
    assert '"Y",' in src
    assert "(0.05, 0.55, 0.1)," in src
    assert '"Z",' in src
    assert "(0.1, 0.25, 0.95)," in src


def test_vtk_base_plane_is_fixed_500mm_and_keeps_real_z_height():
    src = VTK_PREVIEW.read_text(encoding="utf-8")
    plane_block = src.split("    def _base_plane_actors", 1)[1].split(
        "    def _path_bounds", 1
    )[0]

    assert "_FIXED_PLANE_SIZE_MM = 500.0" in src
    assert "min_x = 0.0" in plane_block
    assert "max_x = _FIXED_PLANE_SIZE_MM" in plane_block
    assert "min_y = 0.0" in plane_block
    assert "max_y = _FIXED_PLANE_SIZE_MM" in plane_block
    assert "z = 0.0" in plane_block
    assert "_path_bounds(paths)" not in plane_block
    assert "min(0.0, min_z)" not in plane_block


def test_vtk_preview_uses_material_coordinates_from_offset_sidecar():
    src = VTK_PREVIEW.read_text(encoding="utf-8")

    assert "import json" in src
    assert "def _read_preview_offsets" in src
    assert "resin_z_print_compensation_mm" in src
    assert "tool_offset" in src
    assert "self._tool_offset_xyz" in src
    assert "self._preview_z_origin" in src
    assert "def _display_point_for_path" in src
    assert "if int(path.tool_id) == 1:" in src
    assert "x -= self._tool_offset_xyz[0]" in src
    assert "y -= self._tool_offset_xyz[1]" in src
    assert "z -= self._tool_offset_xyz[2]" in src
    assert "z -= self._preview_z_origin" in src


def test_vtk_preview_renders_paths_events_and_nozzle_with_display_points():
    src = VTK_PREVIEW.read_text(encoding="utf-8")

    assert "self._display_point_for_path(path, point)" in src
    assert "self._display_point_for_path(path, path.end)" in src
    assert "display_end = self._display_point_for_path(" in src
    assert "current_path.end, current_path.end_abc" not in src


def test_vtk_path_preview_renders_tool_change_markers_and_moving_nozzle():
    src = VTK_PREVIEW.read_text(encoding="utf-8")

    assert "def _event_marker_actors" in src
    assert "PathType.TOOL_CHANGE_EVENT" in src
    assert "def _nozzle_actor_for_path" in src
    assert "def _apply_xyzabc_transform" in src
    assert "current_path.end_abc" in src
    assert "vtkCylinderSource" in src
    assert "vtkConeSource" not in src


def test_vtk_nozzle_uses_slender_cylinder_and_hemisphere_at_path_tangent():
    src = VTK_PREVIEW.read_text(encoding="utf-8")
    nozzle_block = src.split("    def _nozzle_actor_for_path", 1)[1].split(
        "    def _apply_xyzabc_transform", 1
    )[0]

    assert "body_radius = radius * 0.45" in nozzle_block
    assert "body_height = radius * 7.0" in nozzle_block
    assert "body_transform.PostMultiply()" in nozzle_block
    assert "vtkSphereSource" in nozzle_block
    assert "hemisphere.SetStartPhi(90.0)" in nozzle_block
    assert "hemisphere.SetEndPhi(180.0)" in nozzle_block
    assert "hemisphere_transform.Translate(0.0, 0.0, radius)" in nozzle_block
    assert (
        "body_transform.Translate(0.0, 0.0, radius + body_height / 2.0)"
        in nozzle_block
    )
    assert "self._nozzle_color_for_path(current_path)" in nozzle_block


def test_vtk_axes_are_translucent_and_labels_stay_near_axis_tips():
    src = VTK_PREVIEW.read_text(encoding="utf-8")

    assert "axis_len * 1.04" in src
    assert "actor.GetProperty().SetOpacity(0.42)" in src
    assert "actor.GetProperty().SetOpacity(0.68)" in src


def test_vtk_nozzle_color_follows_current_print_tool():
    src = VTK_PREVIEW.read_text(encoding="utf-8")

    assert "def _nozzle_color_for_path" in src
    assert "PathType.FIBER_PRINT" in src
    assert "PathType.RESIN_PRINT" in src
    assert "path.tool_id == 1" in src
    assert "path.tool_id == 2" in src
    assert "_PATH_COLORS[PathType.FIBER_PRINT]" in src
    assert "_PATH_COLORS[PathType.RESIN_PRINT]" in src


def test_vtk_abc_zero_keeps_nozzle_tip_on_path_with_body_above_it():
    src = VTK_PREVIEW.read_text(encoding="utf-8")
    transform_block = src.split("    def _apply_xyzabc_transform", 1)[1].split(
        "    def _endpoint_actors_for_path", 1
    )[0]

    assert "# ABC=0 keeps the local tangent point at xyz" in transform_block
    assert "transform.Translate(*xyz)" in transform_block
    assert "transform.RotateZ(float(abc[0]))" in transform_block
    assert "transform.RotateY(float(abc[1]))" in transform_block
    assert "transform.RotateX(float(abc[2]))" in transform_block


def test_vtk_print_paths_use_real_world_bead_width_and_layer_height():
    src = VTK_PREVIEW.read_text(encoding="utf-8")

    assert "_BEAD_DIMENSIONS_MM" in src
    assert "PathType.FIBER_PRINT: (1.0, 0.1)" in src
    assert "PathType.RESIN_PRINT: (2.0, 0.5)" in src
    assert "if path_type in _BEAD_DIMENSIONS_MM:" in src
    assert "return self._bead_actor_for_paths(path_type, paths)" in src


def test_vtk_bead_mesh_uses_material_coordinates_and_physical_z_thickness():
    src = VTK_PREVIEW.read_text(encoding="utf-8")
    bead_block = src.split("    def _bead_actor_for_paths", 1)[1].split(
        "    def _line_actor_for_paths", 1
    )[0]

    assert "width, height = _BEAD_DIMENSIONS_MM[path_type]" in bead_block
    assert "half_width = width / 2.0" in bead_block
    assert "path_top_z = max(point[2] for point in points)" in bead_block
    assert "top_z = path_top_z" in bead_block
    assert "bottom_z = top_z - height" in bead_block
    assert "self._display_point_for_path(path, point)" in bead_block
    assert "poly_data.SetPolys(cells)" in bead_block


def test_vtk_travel_paths_are_yellow_and_visible_by_default():
    src = VTK_PREVIEW.read_text(encoding="utf-8")

    assert "PathType.TRAVEL: (1.0, 0.85, 0.0)" in src
    default_block = src.split("        default_enabled = {", 1)[1].split("        }", 1)[0]
    assert "self._show_travel" in default_block


def test_vtk_tool_change_markers_use_event_row_display_position():
    src = VTK_PREVIEW.read_text(encoding="utf-8")
    marker_block = src.split("    def _event_marker_actors", 1)[1].split(
        "    def _nozzle_actor_for_path", 1
    )[0]

    assert "self._display_point_for_path(path, path.end)" in marker_block
    assert "path.start" not in marker_block
    assert "self._tool_offset_xyz" not in marker_block


def test_vtk_preview_cleans_up_render_window_on_close_and_ignores_late_callbacks():
    src = VTK_PREVIEW.read_text(encoding="utf-8")

    assert "self._closing = False" in src
    assert "def closeEvent(self, event):" in src
    assert "def done(self, result):" in src
    assert "def _cleanup_vtk(self):" in src
    cleanup_block = src.split("    def _cleanup_vtk", 1)[1].split(
        "    def _build_ui", 1
    )[0]
    assert "self._closing = True" in cleanup_block
    assert "render_window.Finalize()" in cleanup_block
    assert "self._vtk_widget.Finalize()" in cleanup_block
    assert "self._renderer.RemoveAllViewProps()" in cleanup_block
    assert "self._vtk_widget = None" in cleanup_block
    assert "self._renderer = None" in cleanup_block
    assert "if self._closing:" in src
