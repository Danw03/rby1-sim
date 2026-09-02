from pathlib import Path

import pytest

from rby1_control_ui.main_window import MainWindow
from rby1_control_ui.mock_backend import MockRby1Backend
from rby1_control_ui.qt_compat import QApplication


@pytest.fixture(scope="module", autouse=True)
def qt_application():
    app = QApplication.instance() or QApplication([])
    yield app


def test_scenario_tab_loads_tasks_and_captures_q(monkeypatch, tmp_path: Path):
    monkeypatch.setenv("RBY1_TASK_CATALOG", str(tmp_path / "catalog.json"))
    backend = MockRby1Backend()
    window = MainWindow(backend)
    try:
        panel = window.scenario_panel
        names = {
            panel.task_list.item(index).text()
            for index in range(panel.task_list.count())
        }
        assert "wait_example" in names
        assert "ready_pose" in names

        window.tabs.setCurrentIndex(2)
        assert window.tabs.currentWidget() is panel

        panel.get_q()
        captured = panel.capture_value.toPlainText()
        assert captured.startswith("[")
        assert len(captured.strip("[]").split(",")) == 7
    finally:
        window.close()
