"""Shared skill parameters (config/skill_parameters.yaml)."""

from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from pm_skills.skill_parameters import SkillParameterError, load_skill_parameters

SOURCE = Path(__file__).resolve().parents[1] / 'config' / 'skill_parameters.yaml'


def write(tmp_path, data):
    path = tmp_path / 'parameters.yaml'
    path.write_text(yaml.safe_dump(data))
    return path


def source_data():
    return yaml.safe_load(SOURCE.read_text())


def test_source_file_has_the_former_constants():
    parameters = load_skill_parameters(SOURCE)
    assert parameters.gripper_frame == 'PM_Robot_Tool_TCP'
    grip = parameters.grip
    assert (grip.approach_offset, grip.sensing_start_offset, grip.sensing_end_offset,
            grip.sensing_end_rough, grip.gripping_offset, grip.lift_distance) == (
        0.05, 0.0005, -0.0005, 0.0002, 0.0001, 0.05)
    assert parameters.release.lift_distance == 0.02
    assert len(parameters.sha256) == 64


def test_source_file_has_the_line_of_sight_values():
    import math
    sight = load_skill_parameters(SOURCE).line_of_sight
    assert (sight.feature_radius.camera, sight.feature_radius.laser,
            sight.feature_radius.confocal) == (0.0003, 0.00005, 0.00005)
    assert sight.request_values('camera') == (0.0003, math.radians(0.5))
    with pytest.raises(SkillParameterError, match='Unknown sensor kind'):
        sight.request_values('sonar')


def test_node_applies_the_parameters_to_its_constants(tmp_path):
    from pm_skills.pm_skills import PmSkills

    data = source_data()
    data['grip']['approach_offset'] = 0.03
    data['gripper_frame'] = 'other_tcp'
    node = SimpleNamespace()
    PmSkills._apply_skill_parameters(node, load_skill_parameters(write(tmp_path, data)))
    assert node.GRIP_APPROACH_OFFSET == 0.03
    assert node.PM_ROBOT_GRIPPER_FRAME == 'other_tcp'
    assert node.GRIP_RELATIVE_LIFT_DISTANCE == 0.05
    assert node.RELEASE_LIFT_DISTANCE == 0.02


@pytest.mark.parametrize('change, text', [
    (lambda d: d['grip'].pop('lift_distance'), 'missing grip.lift_distance'),
    (lambda d: d['grip'].update(lift_distanse=0.05), 'unknown grip.lift_distanse'),
    (lambda d: d['grip'].update(approach_offset='5 cm'), 'grip.approach_offset must be a number'),
    (lambda d: d['release'].update(lift_distance=True), 'release.lift_distance must be a number'),
    (lambda d: d.update(gripper_frame=''), 'gripper_frame must be a frame name'),
    (lambda d: d.pop('release'), 'missing release'),
    (lambda d: d.pop('line_of_sight'), 'missing line_of_sight'),
    (lambda d: d['line_of_sight']['feature_radius'].pop('camera'),
     'missing line_of_sight.feature_radius.camera'),
    (lambda d: d['line_of_sight']['feature_radius'].update(laser=-0.1),
     'line_of_sight.feature_radius.laser must not be negative'),
    (lambda d: d['line_of_sight'].update(tilt_tolerance_deg='half'),
     'line_of_sight.tilt_tolerance_deg must be a number'),
])
def test_invalid_files_are_rejected(tmp_path, change, text):
    data = source_data()
    change(data)
    with pytest.raises(SkillParameterError, match=text):
        load_skill_parameters(write(tmp_path, data))


def test_missing_file_is_reported(tmp_path):
    with pytest.raises(SkillParameterError, match='Cannot read'):
        load_skill_parameters(tmp_path / 'none.yaml')
