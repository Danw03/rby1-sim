import pytest

from rby1_control_ui.mock_backend import MockRby1Backend
from rby1_control_ui.qt_compat import QApplication
from rby1_control_ui.task_commands import Task
from rby1_control_ui.task_runner import TaskRunner


@pytest.fixture(scope="module", autouse=True)
def qt_application():
    app = QApplication.instance() or QApplication([])
    yield app


def _ready_backend():
    backend = MockRby1Backend()
    backend.prepare_robot()
    return backend


def test_runner_executes_resolved_relative_joint_command():
    backend = _ready_backend()
    task = Task("relative")
    task.joint_relative(
        "right_arm",
        [1, 2, 3, 4, 5, 6, 7],
        [2.0, 0.2, 0.2],
    )
    messages = []
    activity = []
    runner = TaskRunner(
        backend,
        on_status=messages.append,
        on_active_changed=activity.append,
    )

    runner.start(task.build())
    assert runner.active
    runner.tick()

    assert not runner.active
    assert backend.get_motion_state()["joint_groups"]["right_arm"] == [
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0
    ]
    assert activity == [True, False]
    assert any(message.startswith("Task completed") for message in messages)


def test_runner_executes_whole_body_as_one_task_step():
    backend = _ready_backend()
    task = Task("ready")
    task.whole_body_joint_absolute(
        torso=[0, 10, -20, 10, 0, 0],
        right_arm=[0, -5, 0, -90, 0, 40, 0],
        left_arm=[0, 5, 0, -90, 0, 40, 0],
        joint_motion=[8.0, 0.2, 0.2],
    )
    messages = []
    runner = TaskRunner(backend, on_status=messages.append)

    runner.start(task.build())
    assert runner.active
    runner.tick()

    state = backend.get_motion_state()["joint_groups"]
    assert not runner.active
    assert state["torso"] == [0, 10, -20, 10, 0, 0]
    assert state["right_arm"] == [0, -5, 0, -90, 0, 40, 0]
    assert state["left_arm"] == [0, 5, 0, -90, 0, 40, 0]
    assert sum("Task step 1/1" in message for message in messages) == 2


def test_cartesian_timeout_accounts_for_distance_and_velocity():
    backend = _ready_backend()
    task = Task("long_tcp")
    task.linear_absolute(
        "right_arm",
        [0.5, 0, 0, 0, 0, 0],
        [2.0, 0.05, 0.2, 0.2],
    )
    runner = TaskRunner(backend, clock=lambda: 100.0)

    runner.start(task.build())

    # 0.5 m / 0.05 m/s = 10 s estimated motion, then the TaskRunner's
    # 3x watchdog scale and 5 s margin produce a 35 s timeout.
    assert runner._command_deadline == pytest.approx(135.0)
    runner.stop()


def test_rotation_timeout_uses_shortest_quaternion_distance():
    backend = _ready_backend()
    backend._cartesian["right_arm"] = [0, 0, 0, 0, 0, 179]
    task = Task("wrapped_yaw")
    task.linear_absolute(
        "right_arm",
        [0, 0, 0, 0, 0, -179],
        [2.0, 0.05, 0.2, 0.2],
    )
    runner = TaskRunner(backend, clock=lambda: 100.0)

    runner.start(task.build())

    # The orientation delta is 2 degrees, not a raw Euler delta of 358.
    assert runner._command_deadline == pytest.approx(130.0)
    runner.stop()


def test_runner_rejects_unprepared_robot():
    backend = MockRby1Backend()
    runner = TaskRunner(backend)
    with pytest.raises(RuntimeError, match="control state"):
        runner.start(Task("wait").delay(10).build())

