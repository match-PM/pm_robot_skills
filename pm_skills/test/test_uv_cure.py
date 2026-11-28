"""uv_cure cures the placed glue points and leaves the component properties
to place and release."""

from types import SimpleNamespace
from unittest.mock import MagicMock

import assembly_manager_interfaces.msg as msg
import pm_msgs.srv as pm_msg_srv

from assembly_scene_publisher.py_modules.AssemblySceneAnalyzerAdv import AssemblySceneAnalyzerAdv
from pm_skills.py_modules.pm_uv_skills import PmUvSkills


def glue(name, placed, cured=False):
    frame = msg.RefFrame(frame_name=name, parent_frame='base')
    properties = frame.properties.glue_pt_frame_properties
    properties.is_glue_point, properties.has_been_placed = True, placed
    properties.has_been_cured = cured
    return frame


def test_uv_cure_cures_placed_glue_points_only():
    scene = msg.ObjectScene()
    base = msg.Object(obj_name='base')
    base.ref_frames.extend([glue('g1', True), glue('g2', False), glue('g3', True, cured=True)])
    part = msg.Object(obj_name='part', parent_frame='PM_Robot_Tool_TCP')
    part.properties.is_placed, part.properties.is_gripped = True, True
    scene.objects_in_scene.extend([base, part])
    analyzer = AssemblySceneAnalyzerAdv(scene)
    updated = {}
    utils = SimpleNamespace(
        assembly_scene_analyzer=analyzer,
        client_uv_cure=MagicMock(),
        set_frame_properties=MagicMock(side_effect=lambda name, props: updated.update(
            {name: props.glue_pt_frame_properties.has_been_cured})),
        set_component_properties=MagicMock(),
    )
    utils.client_uv_cure.call.return_value = pm_msg_srv.UVCuringSkill.Response(success=True)
    analyzer.wait_for_initial_scene_update = MagicMock()
    skills = object.__new__(PmUvSkills)
    skills.pm_robot_utils = utils
    skills.get_logger = MagicMock()
    response = skills.uv_cure_adv_callback(pm_msg_srv.UVCuringSkill.Request(),
                                           pm_msg_srv.UVCuringSkill.Response())
    assert response.success, response.message
    assert updated == {'g1': True}
    utils.set_component_properties.assert_not_called()
    assert part.properties.is_placed and part.properties.is_gripped
