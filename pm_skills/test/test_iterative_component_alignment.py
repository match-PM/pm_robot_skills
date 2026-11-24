"""Iterative component alignment measures the gripping frame's laser dependencies."""

from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pm_moveit_interfaces.srv as moveit_srv
import pm_skills_interfaces.srv as skill_srv
import numpy as np
import pytest
from geometry_msgs.msg import Transform

from assembly_scene_publisher.py_modules.scene_errors import GrippingFrameNotFoundError
from pm_skills.py_modules.pm_gonio_skills import PmGonioSkills

FRAMES = ['part_laser_1', 'part_laser_2', 'part_laser_3']


def ref_frame(name):
    return SimpleNamespace(frame_name=name)


@pytest.fixture(autouse=True)
def no_sleep():
    with patch('pm_skills.py_modules.pm_gonio_skills.time.sleep'):
        yield


@pytest.fixture
def skill():
    domain = object.__new__(PmGonioSkills)
    analyzer = MagicMock()
    analyzer.is_object_on_gonio_left.return_value = True
    analyzer.is_object_on_gonio_right.return_value = False
    analyzer.get_gripping_frame_of_component.return_value = 'part_grip'
    analyzer.get_laser_frames_for_gripping_frame.return_value = [
        (ref_frame(name), False) for name in FRAMES]
    analyzer.get_component_for_frame_name.return_value = 'part'
    transform = Transform()
    transform.rotation.w = 1.0
    utils = SimpleNamespace(
        wait_for_initial_scene_update=MagicMock(),
        assembly_scene_analyzer=analyzer,
        client_align_gonio_left=MagicMock(),
        client_align_gonio_right=MagicMock(),
        client_move_laser_relative=MagicMock(),
        get_transform_for_frame=MagicMock(return_value=transform),
        get_current_joint_state=MagicMock(return_value=0.0),
        GONIO_LEFT_STAGE_1='l1', GONIO_LEFT_STAGE_2='l2',
        GONIO_RIGHT_STAGE_1='r1', GONIO_RIGHT_STAGE_2='r2',
    )
    for client in (utils.client_align_gonio_left, utils.client_align_gonio_right):
        client.call.return_value = moveit_srv.AlignGonio.Response(success=True)
    utils.client_move_laser_relative.call.return_value = SimpleNamespace(success=True, message='')
    domain.pm_robot_utils = utils

    measured = SimpleNamespace(success=True, message='')
    domain.node = SimpleNamespace(
        gripper_skills=MagicMock(),
        measurement_skills=MagicMock(),
    )
    domain.node.gripper_skills.move_laser_to_frame.return_value = (True, '')
    domain.node.gripper_skills.move_confocal_top_to_frame.return_value = (True, '')
    domain.node.measurement_skills.correct_frame_with_laser.return_value = measured
    domain.node.measurement_skills.correct_frame_with_confocal_top.return_value = measured
    domain.PM_ROBOT_GRIPPER_FRAME = 'PM_Robot_Tool_TCP'
    domain.logger = MagicMock()
    return domain


def invoke(skill, sensor='laser', iterations=None):
    callback = getattr(skill, f'iterative_align_component_to_gripper_{sensor}_callback')
    request = skill_srv.IterativeAlignComponentToGripper.Request(component_name='part')
    if iterations is not None:
        request.num_iterations = iterations
    return callback(request, skill_srv.IterativeAlignComponentToGripper.Response())


def set_side(skill, side):
    analyzer = skill.pm_robot_utils.assembly_scene_analyzer
    analyzer.is_object_on_gonio_left.return_value = side == 'left'
    analyzer.is_object_on_gonio_right.return_value = side == 'right'


@pytest.mark.parametrize('side', ['left', 'right'])
def test_aligns_gripping_frame_to_gripper_on_components_gonio(skill, side):
    set_side(skill, side)

    response = invoke(skill)

    assert response.success, response.message
    assert list(response.frames_measured) == FRAMES
    skill.pm_robot_utils.assembly_scene_analyzer.get_laser_frames_for_gripping_frame \
        .assert_called_once_with('part_grip')
    chosen = getattr(skill.pm_robot_utils, f'client_align_gonio_{side}')
    request = chosen.call.call_args.args[0]
    assert request.endeffector_frame_override == 'part_grip'
    assert request.target_frame == 'PM_Robot_Tool_TCP'
    assert request.execute_movement is True
    other = 'right' if side == 'left' else 'left'
    getattr(skill.pm_robot_utils, f'client_align_gonio_{other}').call.assert_not_called()


def test_interface_default_is_two_iterations(skill):
    assert skill_srv.IterativeAlignComponentToGripper.Request().num_iterations == 2

    response = invoke(skill)

    assert response.success
    assert skill.pm_robot_utils.client_align_gonio_left.call.call_count == 2
    measure = skill.node.measurement_skills.correct_frame_with_laser
    measured = [call.args[0].frame_name for call in measure.call_args_list]
    assert measured == FRAMES * 2


def test_requested_iteration_count_is_used(skill):
    assert invoke(skill, iterations=3).success
    assert skill.pm_robot_utils.client_align_gonio_left.call.call_count == 3


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('sensor', ['laser', 'confocal'])
def test_service_decides_the_sensor(skill, sensor, side):
    set_side(skill, side)

    assert invoke(skill, sensor=sensor).success

    gripper = skill.node.gripper_skills
    measurement = skill.node.measurement_skills
    if sensor == 'confocal':
        gripper.move_confocal_top_to_frame.assert_called_once_with(FRAMES[0], z_offset=0.02)
        gripper.move_laser_to_frame.assert_not_called()
        measurement.correct_frame_with_laser.assert_not_called()
        assert measurement.correct_frame_with_confocal_top.call_count == len(FRAMES) * 2
    else:
        gripper.move_laser_to_frame.assert_called_once_with(FRAMES[0], z_offset=0.02)
        gripper.move_confocal_top_to_frame.assert_not_called()
        measurement.correct_frame_with_confocal_top.assert_not_called()
        assert measurement.correct_frame_with_laser.call_count == len(FRAMES) * 2


@pytest.mark.parametrize('iterations', [0, -1])
def test_non_positive_iterations_are_rejected(skill, iterations):
    response = invoke(skill, iterations=iterations)

    assert not response.success
    assert 'num_iterations' in response.message
    skill.node.gripper_skills.move_laser_to_frame.assert_not_called()
    skill.pm_robot_utils.client_align_gonio_left.call.assert_not_called()


@pytest.mark.parametrize('invalid', [
    'missing_component', 'not_on_gonio', 'missing_grip_frame', 'no_laser_frames',
    'laser_frame_off_gonio'])
def test_invalid_setup_fails_before_any_motion(skill, invalid):
    analyzer = skill.pm_robot_utils.assembly_scene_analyzer
    if invalid == 'missing_component':
        analyzer.check_object_exists.return_value = False
    elif invalid == 'not_on_gonio':
        set_side(skill, None)
    elif invalid == 'missing_grip_frame':
        analyzer.get_gripping_frame_of_component.side_effect = (
            GrippingFrameNotFoundError('No gripping frame found'))
    elif invalid == 'no_laser_frames':
        analyzer.get_laser_frames_for_gripping_frame.return_value = []
    else:
        analyzer.get_component_for_frame_name.side_effect = (
            lambda name: 'carrier' if name == FRAMES[1] else 'part')
        analyzer.is_object_on_gonio_left.side_effect = lambda name: name == 'part'

    response = invoke(skill)

    assert not response.success
    assert response.message
    if invalid == 'laser_frame_off_gonio':
        assert FRAMES[1] in response.message
    skill.node.gripper_skills.move_laser_to_frame.assert_not_called()
    skill.pm_robot_utils.client_align_gonio_left.call.assert_not_called()
    skill.pm_robot_utils.client_align_gonio_right.call.assert_not_called()


def test_laser_frame_on_other_component_on_same_gonio_is_allowed(skill):
    analyzer = skill.pm_robot_utils.assembly_scene_analyzer
    analyzer.get_component_for_frame_name.side_effect = (
        lambda name: 'carrier' if name == FRAMES[1] else 'part')

    assert invoke(skill).success


def test_measurement_failure_stops_alignment(skill):
    skill.node.measurement_skills.correct_frame_with_laser.return_value = SimpleNamespace(
        success=False, message='no signal')

    response = invoke(skill)

    assert not response.success
    assert 'no signal' in response.message
    skill.pm_robot_utils.client_align_gonio_left.call.assert_not_called()


def test_gonio_rejection_is_reported(skill):
    skill.pm_robot_utils.client_align_gonio_left.call.return_value = (
        moveit_srv.AlignGonio.Response(success=False, message='not reachable'))

    response = invoke(skill)

    assert not response.success
    assert 'not reachable' in response.message


@pytest.mark.parametrize('side', ['left', 'right'])
def test_existing_iterative_services_still_work(skill, side):
    request = skill_srv.IterativeGonioAlign.Request(
        confocal_laser=False, target_alignment_frame='target',
        component_alignment_frame='part_grip', num_iterations=1,
        frames_to_measure=FRAMES)

    response = getattr(skill, f'iterative_align_gonio_{side}')(
        request, skill_srv.IterativeGonioAlign.Response())

    assert response.success, response.message
    assert f'gonio {side}' in response.message
    lift = skill.pm_robot_utils.client_move_laser_relative.call.call_args.args[0]
    assert lift.translation.z == pytest.approx(0.03 if side == 'right' else 0.01)


@pytest.mark.parametrize('service', ['component', 'gonio'])
def test_gonio_movement_per_iteration_is_returned(skill, service):
    # Stage 1 / stage 2 joint readings (rad): pre, pre, post, post per iteration.
    readings = iter([0.0, 0.0, np.radians(0.5), np.radians(-0.2),
                     np.radians(0.5), np.radians(-0.2), np.radians(0.6), np.radians(-0.25)])
    skill.pm_robot_utils.get_current_joint_state.side_effect = lambda joint: next(readings)

    if service == 'component':
        response = invoke(skill)
    else:
        response = skill.iterative_align_gonio_left(
            skill_srv.IterativeGonioAlign.Request(
                target_alignment_frame='target', component_alignment_frame='part_grip',
                num_iterations=2, frames_to_measure=FRAMES),
            skill_srv.IterativeGonioAlign.Response())

    assert response.success, response.message
    assert list(response.gonio_stage_1_moved_deg) == pytest.approx([0.5, 0.1])
    assert list(response.gonio_stage_2_moved_deg) == pytest.approx([-0.2, -0.05])


def test_no_gonio_movement_is_returned_on_failure(skill):
    skill.pm_robot_utils.client_align_gonio_left.call.return_value = (
        moveit_srv.AlignGonio.Response(success=False, message='not reachable'))

    response = invoke(skill)

    assert not response.success
    assert list(response.gonio_stage_1_moved_deg) == []
