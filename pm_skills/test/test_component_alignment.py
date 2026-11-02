"""Component alignment chooses its goniometer and supports checks without motion."""

from types import SimpleNamespace
from unittest.mock import MagicMock

import pm_moveit_interfaces.srv as moveit_srv
import pm_skills_interfaces.srv as skill_srv
import pytest

from assembly_scene_publisher.py_modules.scene_errors import (
    ComponentNotFoundError,
    GrippingFrameNotFoundError,
)
from pm_skills.py_modules.pm_gonio_skills import PmGonioSkills


@pytest.fixture
def skill():
    domain = object.__new__(PmGonioSkills)
    analyzer = MagicMock()
    analyzer.is_object_on_gonio_left.return_value = True
    analyzer.is_object_on_gonio_right.return_value = False
    analyzer.get_gripping_frame_of_component.return_value = 'part_grip'
    domain.pm_robot_utils = SimpleNamespace(
        wait_for_initial_scene_update=MagicMock(),
        assembly_scene_analyzer=analyzer,
        client_align_gonio_left=MagicMock(),
        client_align_gonio_right=MagicMock(),
    )
    for client in clients(domain):
        client.call.return_value = moveit_srv.AlignGonio.Response(success=True)
    domain.PM_ROBOT_GRIPPER_FRAME = 'PM_Robot_Tool_TCP'
    domain.logger = MagicMock()
    return domain


def clients(skill):
    return (skill.pm_robot_utils.client_align_gonio_left,
            skill.pm_robot_utils.client_align_gonio_right)


def invoke(skill, execute):
    if execute:
        return skill.align_component_to_gripper_callback(
            skill_srv.AlignComponentToGripper.Request(component_name='part'),
            skill_srv.AlignComponentToGripper.Response())
    return skill.check_gripper_is_alignable_callback(
        skill_srv.CheckGripperIsAlignable.Request(component_name='part'),
        skill_srv.CheckGripperIsAlignable.Response())


@pytest.mark.parametrize('execute', [True, False])
@pytest.mark.parametrize('side', ['left', 'right'])
def test_alignment_routes_to_the_components_goniometer(skill, side, execute):
    analyzer = skill.pm_robot_utils.assembly_scene_analyzer
    analyzer.is_object_on_gonio_left.return_value = side == 'left'
    analyzer.is_object_on_gonio_right.return_value = side == 'right'
    # The configured target frame must be used, rather than the helper's default.
    skill.PM_ROBOT_GRIPPER_FRAME = 'configured_gripper_tcp'

    response = invoke(skill, execute)

    assert response.success
    assert side in response.message
    skill.pm_robot_utils.wait_for_initial_scene_update.assert_called_once_with()
    analyzer.get_gripping_frame_of_component.assert_called_once_with('part')
    chosen = getattr(skill.pm_robot_utils, f'client_align_gonio_{side}')
    chosen.wait_for_service.assert_called_once_with(timeout_sec=1.0)
    chosen.call.assert_called_once()
    request = chosen.call.call_args.args[0]
    assert request.endeffector_frame_override == 'part_grip'
    assert request.target_frame == 'configured_gripper_tcp'
    assert request.execute_movement is execute
    other_side = 'right' if side == 'left' else 'left'
    getattr(skill.pm_robot_utils, f'client_align_gonio_{other_side}').call.assert_not_called()
    if not execute:
        assert 'No movement was executed' in response.message


@pytest.mark.parametrize('execute', [True, False])
@pytest.mark.parametrize('invalid', ['missing_component', 'not_on_gonio', 'missing_grip_frame'])
def test_invalid_component_does_not_call_moveit(skill, execute, invalid):
    analyzer = skill.pm_robot_utils.assembly_scene_analyzer
    if invalid == 'missing_component':
        analyzer.check_object_exists.return_value = False
    elif invalid == 'not_on_gonio':
        analyzer.is_object_on_gonio_left.return_value = False
        analyzer.is_object_on_gonio_right.return_value = False
    else:
        analyzer.get_gripping_frame_of_component.side_effect = (
            GrippingFrameNotFoundError('No gripping frame found'))

    response = invoke(skill, execute)

    assert not response.success
    assert response.message
    for client in clients(skill):
        client.call.assert_not_called()


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('failure', ['unavailable', 'no_response', 'rejected', 'disconnected'])
@pytest.mark.parametrize('execute', [True, False])
def test_moveit_failures_are_reported_without_switching_goniometers(skill, side, failure, execute):
    skill.pm_robot_utils.assembly_scene_analyzer.is_object_on_gonio_left.return_value = side == 'left'
    skill.pm_robot_utils.assembly_scene_analyzer.is_object_on_gonio_right.return_value = side == 'right'
    chosen = getattr(skill.pm_robot_utils, f'client_align_gonio_{side}')
    if failure == 'unavailable':
        chosen.wait_for_service.return_value = False
    elif failure == 'no_response':
        chosen.call.return_value = None
    elif failure == 'rejected':
        chosen.call.return_value = moveit_srv.AlignGonio.Response(
            success=False, message='Alignment not reachable')
    else:
        chosen.call.side_effect = RuntimeError('Planner disconnected')

    response = invoke(skill, execute)

    assert not response.success
    assert response.message
    if failure == 'rejected':
        assert 'Alignment not reachable' in response.message
    if failure == 'disconnected':
        assert 'Planner disconnected' in response.message
    other_side = 'right' if side == 'left' else 'left'
    getattr(skill.pm_robot_utils, f'client_align_gonio_{other_side}').call.assert_not_called()


def test_scene_lookup_error_returns_failure(skill):
    skill.pm_robot_utils.assembly_scene_analyzer.check_object_exists.side_effect = (
        ComponentNotFoundError('part'))

    assert not invoke(skill, execute=False).success
    for client in clients(skill):
        client.call.assert_not_called()


@pytest.mark.parametrize('side', ['left', 'right'])
def test_existing_goniometer_helpers_execute_by_default(skill, side):
    success, _ = getattr(skill, f'align_gonio_{side}')('part_grip')

    assert success
    request = getattr(skill.pm_robot_utils, f'client_align_gonio_{side}').call.call_args.args[0]
    assert request.execute_movement is True
    assert request.target_frame == 'PM_Robot_Tool_TCP'
