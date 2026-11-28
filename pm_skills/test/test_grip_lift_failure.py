"""A grip whose lift fails is undone without moving the robot: the component
goes back to its support, the gripper releases it and the goniometer holds it
again, so the skill can be executed again."""

from types import SimpleNamespace
from unittest.mock import MagicMock, call

import assembly_manager_interfaces.msg as msg
import pm_skills_interfaces.srv as skill_srv
import pytest

from assembly_scene_publisher.py_modules.AssemblySceneAnalyzerAdv import AssemblySceneAnalyzerAdv
from pm_robot_modules.submodules.pm_robot_config import VacuumGripperConfig
from pm_skills.py_modules.pm_gripper_skills import PmGripperSkills


@pytest.fixture
def skill():
    scene = msg.ObjectScene()
    part = msg.Object(obj_name='part', parent_frame='Gonio_Right_Part_1_Origin')
    grip = msg.RefFrame(frame_name='part_grip', parent_frame='part')
    grip.properties.gripping_frame_properties.is_gripping_frame = True
    part.ref_frames.append(grip)
    part.properties.is_assembled = False
    scene.objects_in_scene.append(part)
    analyzer = AssemblySceneAnalyzerAdv(scene)
    analyzer.is_object_on_gonio_left = MagicMock(return_value=False)
    analyzer.is_object_on_gonio_right = MagicMock(return_value=True)

    def set_properties(name, properties):
        analyzer.get_component_by_name(name).properties = properties
        return SimpleNamespace(success=True)

    utils = SimpleNamespace(
        assembly_scene_analyzer=analyzer,
        wait_for_initial_scene_update=MagicMock(),
        set_gripper_component_collision=MagicMock(),
        set_tool_vaccum=MagicMock(return_value=True),
        set_gonio_left_vacuum=MagicMock(return_value=True),
        set_gonio_right_vacuum=MagicMock(return_value=True),
        set_component_properties=MagicMock(side_effect=set_properties),
        open_gripper_2_jaws=MagicMock(),
        close_gripper_2_jaws=MagicMock(),
        pm_robot_config=SimpleNamespace(tool=SimpleNamespace(
            get_active_tool_type=lambda: VacuumGripperConfig.TOOL_VACUUM_IDENT)),
    )
    domain = object.__new__(PmGripperSkills)
    domain.pm_robot_utils = utils
    domain.logger = MagicMock()
    domain.move_gripper_to_frame = MagicMock(return_value=(True, 'reached'))

    def attach(name):
        analyzer.get_component_by_name(name).parent_frame = 'PM_Robot_Tool_TCP'
        return True

    def reparent(name, parent):
        analyzer.get_component_by_name(name).parent_frame = parent
        return True

    domain.attach_component_to_gripper = MagicMock(side_effect=attach)
    domain.attach_component_to_component = MagicMock(side_effect=reparent)
    domain.lift_gripper_relative = MagicMock(return_value=(False, 'Planing failed!'))
    domain.GRIP_APPROACH_OFFSET = 0.05
    domain.GRIP_SENSING_START_OFFSET = 0.0005
    domain.GRIP_RELATIVE_LIFT_DISTANCE = 0.05
    return domain


def grip(domain):
    return domain.grip_component_callback(
        skill_srv.GripComponent.Request(component_name='part', align_orientation=False),
        skill_srv.GripComponent.Response())


def test_failed_lift_returns_the_part_without_moving_the_robot(skill, monkeypatch):
    monkeypatch.setattr('time.sleep', lambda *_: None)
    result = grip(skill)
    utils, analyzer = skill.pm_robot_utils, skill.pm_robot_utils.assembly_scene_analyzer
    assert not result.success
    assert 'Failed to lift gripper' in result.message
    assert "returned to 'Gonio_Right_Part_1_Origin'" in result.message
    part = analyzer.get_component_by_name('part')
    assert part.parent_frame == 'Gonio_Right_Part_1_Origin'
    assert part.properties.is_gripped is False
    # Gripper vacuum on for the grip, off again; gonio vacuum off, on again.
    assert utils.set_tool_vaccum.call_args_list == [call(True), call(False)]
    assert utils.set_gonio_right_vacuum.call_args_list == [call(False), call(True)]
    # The only motion after the grip pose is the failed lift: no further move.
    skill.lift_gripper_relative.assert_called_once_with(0.05)
    assert skill.move_gripper_to_frame.call_count == 3
    # The gripper is empty again, so the skill can run again.
    assert analyzer.is_gripper_empty()


def test_successful_lift_keeps_the_part_gripped(skill, monkeypatch):
    monkeypatch.setattr('time.sleep', lambda *_: None)
    skill.lift_gripper_relative.return_value = (True, 'lifted')
    result = grip(skill)
    assert result.success
    part = skill.pm_robot_utils.assembly_scene_analyzer.get_component_by_name('part')
    assert part.parent_frame == 'PM_Robot_Tool_TCP' and part.properties.is_gripped
    skill.attach_component_to_component.assert_not_called()


def test_failed_lift_before_the_attach_does_not_touch_the_part(skill, monkeypatch):
    monkeypatch.setattr('time.sleep', lambda *_: None)
    skill.pm_robot_utils.set_tool_vaccum.return_value = False  # grip fails before attach
    result = grip(skill)
    assert not result.success and 'returned to' not in result.message
    skill.attach_component_to_component.assert_not_called()
    skill.pm_robot_utils.set_gonio_right_vacuum.assert_not_called()


def test_incomplete_return_is_reported(skill, monkeypatch):
    monkeypatch.setattr('time.sleep', lambda *_: None)
    skill.pm_robot_utils.set_gonio_right_vacuum.side_effect = [True, False]
    result = grip(skill)
    assert 'was incomplete: switching the gonio right vacuum on failed' in result.message
