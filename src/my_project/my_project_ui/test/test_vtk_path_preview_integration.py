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
    )


def test_vtk_path_preview_groups_paths_into_few_vtk_actors():
    src = VTK_PREVIEW.read_text(encoding="utf-8")

    assert "def _actors_for_paths" in src
    assert "grouped.setdefault(path.path_type, []).append(path)" in src
    assert "def _actor_for_paths" in src
    assert "_MAX_POINTS_PER_PATH" in src
    assert "def _sample_points" in src


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
