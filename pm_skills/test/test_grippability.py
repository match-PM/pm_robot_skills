"""Planning-only grippability and collision cleanup regression tests."""

from unittest.mock import MagicMock, call

import pm_moveit_interfaces.srv as moveit_srv
import pm_skills_interfaces.srv as skill_srv
import pytest

from assembly_scene_publisher.py_modules.scene_errors import GrippingFrameNotFoundError
from pm_robot_primitive_skills.py_modules.PmRobotError import PmRobotError
from pm_skills.py_modules.PmRobotUtils import PmRobotUtils
from pm_skills.py_modules.pm_gripper_skills import PmGripperSkills


@pytest.fixture
def skill():
    """Use real callbacks with mocked collision helpers and ROS clients."""
    utils = object.__new__(PmRobotUtils)
    utils.wait_for_initial_scene_update = MagicMock()
    utils.assembly_scene_analyzer = MagicMock()
    utils.assembly_scene_analyzer.check_component_assembled.return_value = False
    utils.assembly_scene_analyzer.get_gripping_frame_of_component.return_value = 'part_grip'
    utils.assembly_scene_analyzer.get_measurement_frames_for_gripping_frame.return_value = []
    utils.set_gripper_component_collision = MagicMock()

    domain = object.__new__(PmGripperSkills)
    domain.pm_robot_utils = utils
    domain.logger = MagicMock()
    domain.GRIP_APPROACH_OFFSET = 0.05
    domain.GRIP_SENSING_START_OFFSET = 0.0005
    domain.client_move_robot_tool_to_frame = MagicMock()
    domain.client_move_robot_tool_to_frame.call.return_value = moveit_srv.MoveToFrame.Response(
        success=True)
    return domain


def check(skill):
    """Call the service callback without starting a ROS node."""
    return skill.check_component_grippable_callback(
        skill_srv.CheckComponentGrippable.Request(component_name='part'),
        skill_srv.CheckComponentGrippable.Response(),
    )


def test_check_plans_all_grip_poses_without_execution(skill):
    result = check(skill)

    assert result.success
    requests = [args.args[0] for args in skill.client_move_robot_tool_to_frame.call.call_args_list]
    assert [req.translation.z for req in requests] == [0.05, 0.0005, 0.0]
    assert all(req.target_frame == 'part_grip' for req in requests)
    assert all(req.execute_movement is False for req in requests)
    assert skill.pm_robot_utils.set_gripper_component_collision.call_args_list == [
        call('part', False), call('part', True)]


@pytest.mark.parametrize('stage', [0, 1, 2])
def test_failed_plan_reports_stage_and_reenables_collisions(skill, stage):
    skill.client_move_robot_tool_to_frame.call.side_effect = [
        moveit_srv.MoveToFrame.Response(success=True) for _ in range(stage)
    ] + [moveit_srv.MoveToFrame.Response(success=False, message='unreachable')]

    result = check(skill)

    assert not result.success
    assert ['approach', 'sensing start', 'gripping'][stage] in result.message
    assert 'unreachable' in result.message
    assert skill.client_move_robot_tool_to_frame.call.call_count == stage + 1
    assert skill.pm_robot_utils.set_gripper_component_collision.call_args_list == (
        [call('part', False), call('part', True)] if stage == 2 else [])


@pytest.mark.parametrize('method, value', [
    ('check_object_exists', False),
    ('is_gripper_empty', False),
    ('check_component_assembled', True),
])
def test_invalid_grip_preconditions_stop_before_planning(skill, method, value):
    getattr(skill.pm_robot_utils.assembly_scene_analyzer, method).return_value = value

    assert not check(skill).success
    skill.client_move_robot_tool_to_frame.call.assert_not_called()
    skill.pm_robot_utils.set_gripper_component_collision.assert_not_called()


def test_missing_gripping_frame_returns_failure(skill):
    skill.pm_robot_utils.assembly_scene_analyzer.get_gripping_frame_of_component.side_effect = (
        GrippingFrameNotFoundError('missing gripping frame'))

    result = check(skill)

    assert not result.success
    skill.client_move_robot_tool_to_frame.call.assert_not_called()


def test_unavailable_motion_service_returns_failure(skill):
    skill.client_move_robot_tool_to_frame.wait_for_service.return_value = False

    assert not check(skill).success
    skill.client_move_robot_tool_to_frame.call.assert_not_called()


def test_missing_planner_response_returns_failure(skill):
    skill.client_move_robot_tool_to_frame.call.return_value = None

    result = check(skill)

    assert not result.success
    assert 'no response' in result.message


def test_planner_exception_reenables_collisions(skill):
    skill.client_move_robot_tool_to_frame.call.side_effect = [
        moveit_srv.MoveToFrame.Response(success=True),
        moveit_srv.MoveToFrame.Response(success=True),
        RuntimeError('planner disconnected'),
    ]

    result = check(skill)

    assert not result.success
    assert 'planner disconnected' in result.message
    assert skill.pm_robot_utils.set_gripper_component_collision.call_args_list == [
        call('part', False), call('part', True)]


def test_motion_helper_still_executes_by_default(skill):
    assert skill.move_gripper_to_frame('target')[0]
    request = skill.client_move_robot_tool_to_frame.call.call_args.args[0]
    assert request.execute_movement is True


def test_contact_setup_failure_still_attempts_collision_cleanup(skill):
    utils = skill.pm_robot_utils
    utils.set_gripper_component_collision.side_effect = [PmRobotError('setup failed'), None]

    result = check(skill)

    assert not result.success
    assert 'setup failed' in result.message
    assert utils.set_gripper_component_collision.call_args_list == [
        call('part', False), call('part', True)]
    assert skill.client_move_robot_tool_to_frame.call.call_count == 2


def test_collision_cleanup_failure_is_reported(skill):
    utils = skill.pm_robot_utils
    utils.set_gripper_component_collision.side_effect = [None, PmRobotError('cleanup failed')]

    result = check(skill)

    assert not result.success
    assert 'cleanup failed' in result.message
    assert utils.set_gripper_component_collision.call_args_list == [
        call('part', False), call('part', True)]
