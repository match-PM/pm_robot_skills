"""Measurement policy belongs to planning checks, not direct grip actions."""

from types import SimpleNamespace
from unittest.mock import MagicMock

import assembly_manager_interfaces.msg as msg
import pm_skills_interfaces.srv as skill_srv
import pytest

from assembly_scene_publisher.py_modules.AssemblySceneAnalyzerAdv import AssemblySceneAnalyzerAdv
from pm_robot_primitive_skills.py_modules.PmRobotError import PmRobotError
from pm_robot_modules.submodules.pm_robot_config import VacuumGripperConfig
from pm_skills.py_modules.pm_gripper_skills import PmGripperSkills


CALLBACKS = ['grip_component_callback', 'force_grip_component_callback']


@pytest.fixture
def scenario():
    scene = msg.ObjectScene()
    part = msg.Object(obj_name='part', parent_frame='Gonio_Right_Part_1_Origin')
    other = msg.Object(obj_name='support')
    scene.objects_in_scene.extend([part, other])

    grip = msg.RefFrame(frame_name='part_grip', parent_frame='part')
    grip.properties.gripping_frame_properties.is_gripping_frame = True
    grip.constraints.centroid.is_active = True
    grip.constraints.centroid.ref_frame_names = ['part_vision']
    vision = msg.RefFrame(frame_name='part_vision', parent_frame='part')
    vision.properties.vision_frame_properties.is_vision_frame = True
    vision.constraints.centroid.is_active = True
    vision.constraints.centroid.ref_frame_names = ['support_laser']
    laser = msg.RefFrame(frame_name='support_laser', parent_frame='support')
    laser.properties.laser_frame_properties.is_laser_frame = True
    unrelated = msg.RefFrame(frame_name='unrelated', parent_frame='part')
    unrelated.properties.vision_frame_properties.is_vision_frame = True
    part.ref_frames.extend([grip, vision, unrelated])
    other.ref_frames.append(laser)
    analyzer = AssemblySceneAnalyzerAdv(scene)
    analyzer.is_object_on_gonio_left = MagicMock(return_value=False)
    analyzer.is_object_on_gonio_right = MagicMock(return_value=True)

    utils = SimpleNamespace(
        assembly_scene_analyzer=analyzer,
        wait_for_initial_scene_update=MagicMock(),
        set_gripper_component_collision=MagicMock(),
        set_tool_vaccum=MagicMock(return_value=True),
        open_gripper_2_jaws=MagicMock(),
        close_gripper_2_jaws=MagicMock(),
        pm_robot_config=SimpleNamespace(tool=SimpleNamespace(
            get_active_tool_type=lambda: VacuumGripperConfig.TOOL_VACUUM_IDENT)),
    )
    domain = object.__new__(PmGripperSkills)
    domain.pm_robot_utils = utils
    domain.node = SimpleNamespace(gonio_skills=SimpleNamespace(
        align_gonio_left=MagicMock(return_value=(True, 'aligned')),
        align_gonio_right=MagicMock(return_value=(True, 'aligned'))))
    domain.logger = MagicMock()
    domain.move_gripper_to_frame = MagicMock(side_effect=PmRobotError('test motion boundary'))
    domain.attach_component_to_gripper = MagicMock()
    domain.lift_gripper_relative = MagicMock(return_value=(True, 'retracted'))
    domain.GRIP_APPROACH_OFFSET = 0.05
    domain.GRIP_SENSING_START_OFFSET = 0.0005
    domain.GRIP_RELATIVE_LIFT_DISTANCE = 0.01
    return SimpleNamespace(skill=domain, grip=grip, vision=vision, laser=laser)


def invoke(scenario, callback):
    return getattr(scenario.skill, callback)(
        skill_srv.GripComponent.Request(component_name='part', align_orientation=True),
        skill_srv.GripComponent.Response())


def check(scenario):
    return scenario.skill.check_component_grippable_callback(
        skill_srv.CheckComponentGrippable.Request(component_name='part'),
        skill_srv.CheckComponentGrippable.Response())


def mark_measured(scenario):
    scenario.vision.properties.vision_frame_properties.has_been_measured = True
    scenario.laser.properties.laser_frame_properties.has_been_measured = True


@pytest.mark.parametrize('pending', ['vision', 'laser'])
def test_planning_check_rejects_unmeasured_transitive_dependencies(scenario, pending):
    mark_measured(scenario)
    getattr(getattr(scenario, pending).properties,
            pending + '_frame_properties').has_been_measured = False

    result = check(scenario)

    assert not result.success
    assert getattr(scenario, pending).frame_name in result.message
    scenario.skill.node.gonio_skills.align_gonio_right.assert_not_called()
    scenario.skill.move_gripper_to_frame.assert_not_called()
    scenario.skill.pm_robot_utils.set_tool_vaccum.assert_not_called()
    scenario.skill.attach_component_to_gripper.assert_not_called()
    scenario.skill.lift_gripper_relative.assert_not_called()


def test_complete_dependencies_allow_planning_despite_unrelated_unmeasured_frame(scenario):
    mark_measured(scenario)

    result = check(scenario)

    assert result.message == 'test motion boundary'
    scenario.skill.node.gonio_skills.align_gonio_right.assert_not_called()
    scenario.skill.move_gripper_to_frame.assert_called_once()


def test_gripping_frame_with_both_measurement_roles_requires_both(scenario):
    mark_measured(scenario)
    scenario.grip.properties.vision_frame_properties.is_vision_frame = True
    scenario.grip.properties.vision_frame_properties.has_been_measured = True
    scenario.grip.properties.laser_frame_properties.is_laser_frame = True
    with pytest.raises(PmRobotError, match='part_grip'):
        scenario.skill._require_grip_measurements('part_grip')
    scenario.grip.properties.laser_frame_properties.has_been_measured = True
    scenario.skill._require_grip_measurements('part_grip')


@pytest.mark.parametrize('callback', CALLBACKS)
def test_direct_actions_do_not_apply_measurement_policy(scenario, callback):
    # Both measurements remain false. A manual action must still reach its
    # ordinary alignment/motion path; the mock stops before any real movement.
    scenario.skill._require_grip_measurements = MagicMock(
        side_effect=AssertionError('Planning policy called by an executing action'))
    result = invoke(scenario, callback)
    assert result.message == 'test motion boundary'
    scenario.skill.node.gonio_skills.align_gonio_right.assert_called_once()
    scenario.skill.move_gripper_to_frame.assert_called_once()
    scenario.skill._require_grip_measurements.assert_not_called()


@pytest.mark.parametrize('callback', CALLBACKS)
def test_alignment_invalidation_does_not_block_manual_action(scenario, callback):
    mark_measured(scenario)

    def align(*args):
        scenario.laser.properties.laser_frame_properties.has_been_measured = False
        return True, 'aligned'

    scenario.skill.node.gonio_skills.align_gonio_right.side_effect = align
    result = invoke(scenario, callback)
    assert result.message == 'test motion boundary'
    scenario.skill.move_gripper_to_frame.assert_called_once()


def test_manual_grip_actuation_is_not_blocked_by_measurement_invalidation(scenario):
    mark_measured(scenario)
    moves = []

    def move(*args, **kwargs):
        moves.append(kwargs['z_offset'])
        if len(moves) == 3:
            scenario.vision.properties.vision_frame_properties.has_been_measured = False
        return True, 'reached'

    scenario.skill.move_gripper_to_frame.side_effect = move
    scenario.skill.pm_robot_utils.set_gonio_right_vacuum = MagicMock(return_value=True)
    scenario.skill.attach_component_to_gripper.side_effect = PmRobotError('test attachment boundary')
    result = invoke(scenario, 'grip_component_callback')
    assert result.message == 'test attachment boundary'
    assert moves == [0.05, 0.0005, 0.0]
    scenario.skill.pm_robot_utils.set_tool_vaccum.assert_called_once_with(True)
    scenario.skill.pm_robot_utils.close_gripper_2_jaws.assert_not_called()
    scenario.skill.attach_component_to_gripper.assert_called_once_with('part')
    scenario.skill.lift_gripper_relative.assert_called_once()


@pytest.mark.parametrize('invalid', ['missing', 'cycle'])
def test_planning_check_rejects_unresolved_dependencies(scenario, invalid):
    scenario.vision.constraints.centroid.ref_frame_names = [
        'absent' if invalid == 'missing' else 'part_grip']
    result = check(scenario)
    assert not result.success
    assert 'Cannot verify measurement dependencies' in result.message
    scenario.skill.move_gripper_to_frame.assert_not_called()
    scenario.skill.node.gonio_skills.align_gonio_right.assert_not_called()
