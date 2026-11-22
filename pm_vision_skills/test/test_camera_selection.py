"""Camera selection of the vision correction skills (no ROS graph needed)."""

from types import SimpleNamespace

import pytest

from pm_robot_primitive_skills.py_modules.PmRobotError import PmRobotError
from pm_vision_skills.pm_vision_skills import VisionSkillsNode


class Logger:
    def info(self, *args):
        pass

    warn = info


def node(bottom_ok=True, top_ok=True):
    """VisionSkillsNode without ROS; records the camera moves it requests."""
    skills = object.__new__(VisionSkillsNode)
    client = SimpleNamespace(wait_for_service=lambda timeout_sec: True)
    skills.pm_robot_utils = SimpleNamespace(
        get_mode=lambda: 'unity', REAL_MODE='real', UNITY_MODE='unity',
        TCP_CAMERA_TOP='Cam1_TCP_top', TCP_CAMERA_BOTTOM='Camera_Station_TCP_bottom',
        client_move_robot_cam1_to_frame=client)
    skills._logger = Logger()
    skills.moves = []

    def call(client, request, name):
        bottom = request.target_frame == 'Camera_Station_TCP'
        skills.moves.append('bottom' if bottom else 'top')
        if bottom:
            assert request.endeffector_frame_override == 'part_f1'
        else:
            assert request.target_frame == 'part_f1'
        return SimpleNamespace(success=bottom_ok if bottom else top_ok)

    skills._call_service_with_timeout = call
    return skills


def test_default_prefers_the_bottom_camera():
    skills = node()
    assert skills._move_frame_to_camera('part_f1') == (
        'pm_robot_bottom_cam_2_Unity.yaml', 'Camera_Station_TCP_bottom')
    assert skills.moves == ['bottom']


def test_default_falls_back_to_the_top_camera():
    skills = node(bottom_ok=False)
    assert skills._move_frame_to_camera('part_f1') == (
        'pm_robot_top_cam_1_Unity.yaml', 'Cam1_TCP_top')
    assert skills.moves == ['bottom', 'top']
    with pytest.raises(PmRobotError, match='both cameras'):
        node(bottom_ok=False, top_ok=False)._move_frame_to_camera('part_f1')


def test_bottom_camera_only():
    skills = node()
    assert skills._move_frame_to_camera('part_f1', 'bottom')[1] == 'Camera_Station_TCP_bottom'
    failing = node(bottom_ok=False)
    with pytest.raises(PmRobotError, match='bottom cam'):
        failing._move_frame_to_camera('part_f1', 'bottom')
    assert failing.moves == ['bottom']  # no fallback


def test_top_camera_only():
    skills = node()
    assert skills._move_frame_to_camera('part_f1', 'top') == (
        'pm_robot_top_cam_1_Unity.yaml', 'Cam1_TCP_top')
    assert skills.moves == ['top']  # the bottom camera is not tried
    with pytest.raises(PmRobotError, match='top camera'):
        node(top_ok=False)._move_frame_to_camera('part_f1', 'top')


def test_unknown_camera_is_rejected():
    with pytest.raises(PmRobotError, match='Unknown camera'):
        node()._move_frame_to_camera('part_f1', 'side')


def test_correct_frame_services_pass_their_camera():
    skills = object.__new__(VisionSkillsNode)
    seen = []
    skills.correct_frame = lambda request, response, camera=None: seen.append(camera) or response
    VisionSkillsNode.correct_frame_top(skills, 'request', 'response')
    VisionSkillsNode.correct_frame_bottom(skills, 'request', 'response')
    assert seen == ['top', 'bottom']
