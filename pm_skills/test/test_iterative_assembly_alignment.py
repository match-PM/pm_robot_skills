"""Iterative assembly alignment aligns the target component's gonio to the assembly frame."""

from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pm_moveit_interfaces.srv as moveit_srv
import pm_skills_interfaces.srv as skill_srv
import pytest
from geometry_msgs.msg import Transform

from assembly_scene_publisher.py_modules.scene_errors import (
    AssemblyFrameNotFoundError,
    AssemblyInstructionNotFoundError,
    TargetFrameNotFoundError,
)
from pm_robot_primitive_skills.py_modules.PmRobotError import PmRobotError
from pm_skills.py_modules.pm_gonio_skills import PmGonioSkills

FRAMES = ['target_laser_1', 'target_laser_2', 'target_laser_3']
OWNERS = {'part_assembly': 'part', 'base_target': 'base',
          **{name: 'base' for name in FRAMES}}


def ref_frame(name):
    return SimpleNamespace(frame_name=name)


@pytest.fixture(autouse=True)
def no_sleep():
    with patch('pm_skills.py_modules.pm_gonio_skills.time.sleep'):
        yield


@pytest.fixture
def events():
    return []


@pytest.fixture
def skill(events):
    domain = object.__new__(PmGonioSkills)
    analyzer = MagicMock()
    # The target component 'base' is on the right gonio, the assembly component 'part' is not.
    analyzer.is_object_on_gonio_left.side_effect = lambda name: False
    analyzer.is_object_on_gonio_right.side_effect = lambda name: name == 'base'
    analyzer.get_assembly_frame_for_component.return_value = 'part_assembly'
    analyzer.get_target_frame_for_assembly_frame.return_value = 'base_target'
    analyzer.get_component_for_frame_name.side_effect = lambda name: OWNERS[name]
    analyzer.get_laser_frames_for_target_frame.return_value = [
        (ref_frame(name), False) for name in FRAMES]
    analyzer.get_measurement_frames_for_assembly_frame.return_value = []
    analyzer.get_assembly_instruction.return_value = SimpleNamespace(id='instr_1')

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
        recalculate_assembly_instruction=MagicMock(
            side_effect=lambda instruction_id: events.append(('recalculate', instruction_id))),
        GONIO_LEFT_STAGE_1='l1', GONIO_LEFT_STAGE_2='l2',
        GONIO_RIGHT_STAGE_1='r1', GONIO_RIGHT_STAGE_2='r2',
    )

    def align(request):
        events.append(('align', request.endeffector_frame_override))
        return moveit_srv.AlignGonio.Response(success=True)
    for client in (utils.client_align_gonio_left, utils.client_align_gonio_right):
        client.call.side_effect = align
    utils.client_move_laser_relative.call.return_value = SimpleNamespace(success=True, message='')
    domain.pm_robot_utils = utils

    def measure(request, response):
        events.append(('measure', request.frame_name))
        return SimpleNamespace(success=True, message='')
    domain.node = SimpleNamespace(gripper_skills=MagicMock(), measurement_skills=MagicMock())
    domain.node.gripper_skills.move_laser_to_frame.return_value = (True, '')
    domain.node.gripper_skills.move_confocal_top_to_frame.return_value = (True, '')
    domain.node.measurement_skills.correct_frame_with_laser.side_effect = measure
    domain.node.measurement_skills.correct_frame_with_confocal_top.side_effect = measure
    domain.PM_ROBOT_GRIPPER_FRAME = 'PM_Robot_Tool_TCP'
    domain.logger = MagicMock()
    return domain


def invoke(skill, sensor='laser', iterations=None):
    callback = getattr(skill, f'iterative_align_assembly_component_to_target_{sensor}_callback')
    request = skill_srv.IterativeAlignAssemblyComponentToTarget.Request(component_name='part')
    if iterations is not None:
        request.num_iterations = iterations
    return callback(request, skill_srv.IterativeAlignAssemblyComponentToTarget.Response())


def no_motion(skill):
    skill.node.gripper_skills.move_laser_to_frame.assert_not_called()
    skill.node.gripper_skills.move_confocal_top_to_frame.assert_not_called()
    skill.pm_robot_utils.client_align_gonio_left.call.assert_not_called()
    skill.pm_robot_utils.client_align_gonio_right.call.assert_not_called()


def test_aligns_target_frame_to_assembly_frame_on_target_gonio(skill):
    response = invoke(skill)

    assert response.success, response.message
    assert response.assembly_frame == 'part_assembly'
    assert response.target_frame == 'base_target'
    assert response.target_component == 'base'
    assert list(response.frames_measured) == FRAMES
    assert len(response.gonio_stage_1_moved_deg) == 2
    request = skill.pm_robot_utils.client_align_gonio_right.call.call_args.args[0]
    assert request.endeffector_frame_override == 'base_target'
    assert request.target_frame == 'part_assembly'
    skill.pm_robot_utils.client_align_gonio_left.call.assert_not_called()
    skill.pm_robot_utils.assembly_scene_analyzer.get_assembly_instruction.assert_called_once_with(
        assembly_component='part', target_component='base')


@pytest.mark.parametrize('gripped_side', ['left', None])
def test_assembly_component_may_be_anywhere_but_the_same_gonio(skill, gripped_side):
    analyzer = skill.pm_robot_utils.assembly_scene_analyzer
    analyzer.is_object_on_gonio_left.side_effect = (
        lambda name: gripped_side == 'left' and name == 'part')

    assert invoke(skill).success


def test_both_components_on_same_gonio_is_rejected(skill):
    analyzer = skill.pm_robot_utils.assembly_scene_analyzer
    analyzer.is_object_on_gonio_right.side_effect = lambda name: name in ('base', 'part')

    response = invoke(skill)

    assert not response.success
    assert 'both on the right goniometer' in response.message
    no_motion(skill)


def test_measure_recalculate_align_order_with_default_two_iterations(skill, events):
    assert invoke(skill).success

    one_iteration = ([('measure', name) for name in FRAMES]
                     + [('recalculate', 'instr_1'), ('align', 'base_target')])
    assert events == one_iteration * 2


def test_requested_iteration_count_is_used(skill):
    assert invoke(skill, iterations=3).success
    assert skill.pm_robot_utils.recalculate_assembly_instruction.call_count == 3
    assert skill.pm_robot_utils.client_align_gonio_right.call.call_count == 3


@pytest.mark.parametrize('sensor', ['laser', 'confocal'])
def test_service_decides_the_sensor(skill, sensor):
    assert invoke(skill, sensor=sensor).success

    gripper = skill.node.gripper_skills
    measurement = skill.node.measurement_skills
    if sensor == 'confocal':
        gripper.move_confocal_top_to_frame.assert_called_once_with(FRAMES[0], z_offset=0.02)
        gripper.move_laser_to_frame.assert_not_called()
        measurement.correct_frame_with_laser.assert_not_called()
    else:
        gripper.move_laser_to_frame.assert_called_once_with(FRAMES[0], z_offset=0.02)
        gripper.move_confocal_top_to_frame.assert_not_called()
        measurement.correct_frame_with_confocal_top.assert_not_called()


def test_unmeasured_assembly_frame_dependencies_only_warn(skill):
    analyzer = skill.pm_robot_utils.assembly_scene_analyzer
    analyzer.get_measurement_frames_for_assembly_frame.return_value = [
        (ref_frame('part_vision_1'), False), (ref_frame('part_laser_1'), True)]

    response = invoke(skill)

    assert response.success
    warning = ' '.join(str(call.args[0]) for call in skill.logger.warn.call_args_list)
    assert 'part_vision_1' in warning
    assert 'part_laser_1' not in warning


@pytest.mark.parametrize('invalid', [
    'missing_component', 'no_assembly_frame', 'no_target_frame', 'target_not_on_gonio',
    'target_on_same_component', 'no_laser_frames', 'laser_frame_off_gonio', 'no_instruction',
    'zero_iterations'])
def test_invalid_setup_fails_before_any_motion(skill, invalid):
    analyzer = skill.pm_robot_utils.assembly_scene_analyzer
    iterations = None
    if invalid == 'missing_component':
        analyzer.check_object_exists.return_value = False
    elif invalid == 'no_assembly_frame':
        analyzer.get_assembly_frame_for_component.side_effect = (
            AssemblyFrameNotFoundError('No assembly frame found'))
    elif invalid == 'no_target_frame':
        analyzer.get_target_frame_for_assembly_frame.side_effect = (
            TargetFrameNotFoundError('No associated target frame'))
    elif invalid == 'target_not_on_gonio':
        analyzer.is_object_on_gonio_right.side_effect = lambda name: False
    elif invalid == 'target_on_same_component':
        analyzer.get_component_for_frame_name.side_effect = lambda name: 'part'
    elif invalid == 'no_laser_frames':
        analyzer.get_laser_frames_for_target_frame.return_value = []
    elif invalid == 'laser_frame_off_gonio':
        owners = {**OWNERS, FRAMES[1]: 'carrier'}
        analyzer.get_component_for_frame_name.side_effect = lambda name: owners[name]
    elif invalid == 'no_instruction':
        analyzer.get_assembly_instruction.side_effect = (
            AssemblyInstructionNotFoundError('No instruction'))
    else:
        iterations = 0

    response = invoke(skill, iterations=iterations)

    assert not response.success
    assert response.message
    if invalid == 'laser_frame_off_gonio':
        assert FRAMES[1] in response.message
    no_motion(skill)
    skill.pm_robot_utils.recalculate_assembly_instruction.assert_not_called()


def test_recalculation_failure_stops_before_aligning(skill):
    skill.pm_robot_utils.recalculate_assembly_instruction.side_effect = PmRobotError(
        'Failed to recalculate assembly instruction: instr_1')

    response = invoke(skill)

    assert not response.success
    assert 'instr_1' in response.message
    skill.pm_robot_utils.client_align_gonio_right.call.assert_not_called()


def test_measurement_failure_stops_alignment(skill):
    skill.node.measurement_skills.correct_frame_with_laser.side_effect = (
        lambda request, response: SimpleNamespace(success=False, message='no signal'))

    response = invoke(skill)

    assert not response.success
    assert 'no signal' in response.message
    skill.pm_robot_utils.recalculate_assembly_instruction.assert_not_called()
    skill.pm_robot_utils.client_align_gonio_right.call.assert_not_called()


def test_gonio_rejection_is_reported(skill):
    skill.pm_robot_utils.client_align_gonio_right.call.side_effect = (
        lambda request: moveit_srv.AlignGonio.Response(success=False, message='not reachable'))

    response = invoke(skill)

    assert not response.success
    assert 'not reachable' in response.message
    assert list(response.gonio_stage_1_moved_deg) == []
