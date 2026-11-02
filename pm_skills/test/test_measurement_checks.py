"""Measurement checks require a planned pose and assembly-manager visibility."""

import importlib
from unittest.mock import MagicMock

from geometry_msgs.msg import Pose, Quaternion, TransformStamped
import assembly_manager_interfaces.srv as assembly_srv
import pm_moveit_interfaces.srv as moveit_srv
import pm_skills_interfaces.srv as skill_srv
import pytest
from scipy.spatial.transform import Rotation

from assembly_scene_publisher.py_modules.scene_errors import TfFrameLookupError
from pm_skills.py_modules.PmRobotUtils import PmRobotUtils
from pm_skills.py_modules.pm_measurement_skills import PmMeasurementSkills
from pm_vision_skills.pm_vision_skills import VisionSkillsNode


UTILS_MODULE = importlib.import_module('pm_skills.py_modules.PmRobotUtils')
CHECKS = [
    (PmMeasurementSkills, 'check_frame_mes_confocal_top'),
    (PmMeasurementSkills, 'check_frame_mes_laser_top'),
    (PmMeasurementSkills, 'check_frame_mes_confocal_bottom'),
    (VisionSkillsNode, 'check_frame_mes_cam_top'),
    (VisionSkillsNode, 'check_frame_mes_cam_bottom'),
]


@pytest.fixture
def utils(monkeypatch):
    instance = object.__new__(PmRobotUtils)
    instance._node = MagicMock()
    instance.tf_buffer = MagicMock()
    instance.assembly_scene_analyzer = MagicMock()
    instance.client_move_robot_laser_to_frame = MagicMock()
    instance.client_move_robot_cam1_to_frame = MagicMock()
    for client in (instance.client_move_robot_laser_to_frame,
                   instance.client_move_robot_cam1_to_frame):
        pose = Pose(orientation=Quaternion(w=1.0))
        client.call.return_value = moveit_srv.MoveToFrame.Response(
            success=True, calculated_endeffector_pose=pose)
    instance.client_check_line_of_sight = MagicMock()
    instance.client_check_line_of_sight.call.return_value = (
        assembly_srv.CheckLineOfSight.Response(success=True))

    def identity(*args, **kwargs):
        transform = TransformStamped()
        transform.transform.rotation.w = 1.0
        return transform

    monkeypatch.setattr(UTILS_MODULE, 'get_transform_for_frame_in_world', identity)
    monkeypatch.setattr(UTILS_MODULE, 'get_transform_for_frame', identity)
    return instance


def run_check(utils, bottom=False):
    request = skill_srv.CheckFrameMeasurable.Request(frame_name='part_measurement')
    if bottom:
        return utils.check_frame_mes_bot(request, utils.TCP_CONFOCAL_BOTTOM, 0.04)
    return utils.check_frame_mes(request, 0.008, utils.client_move_robot_laser_to_frame)


@pytest.mark.parametrize('domain_class, method', CHECKS)
@pytest.mark.parametrize('visible', [True, False])
def test_all_sensor_callbacks_require_line_of_sight(utils, domain_class, method, visible):
    # Call the adapters without constructing a ROS node.
    domain = MagicMock()
    domain.pm_robot_utils = utils
    utils.client_check_line_of_sight.call.return_value.success = visible

    result = getattr(domain_class, method)(
        domain,
        skill_srv.CheckFrameMeasurable.Request(frame_name='part_measurement'),
        skill_srv.CheckFrameMeasurable.Response(),
    )

    assert result.success is visible
    assert 'line of sight' in result.message or 'line-of-sight' in result.message
    sight_request = utils.client_check_line_of_sight.call.call_args.args[0]
    assert sight_request.frame_name == 'part_measurement'
    clients = (utils.client_move_robot_laser_to_frame, utils.client_move_robot_cam1_to_frame)
    planned_requests = [call.args[0] for client in clients for call in client.call.call_args_list]
    assert len(planned_requests) == 1
    assert planned_requests[0].execute_movement is False


@pytest.mark.parametrize('bottom', [False, True])
@pytest.mark.parametrize('failure', ['unavailable', 'no_response', 'planning_failed', 'missing_frame'])
def test_planning_failures_stop_before_visibility_check(utils, bottom, failure):
    client = (utils.client_move_robot_cam1_to_frame if bottom
              else utils.client_move_robot_laser_to_frame)
    if failure == 'unavailable':
        client.wait_for_service.return_value = False
    elif failure == 'no_response':
        client.call.return_value = None
    elif failure == 'planning_failed':
        client.call.return_value.success = False
        client.call.return_value.message = 'unreachable'
    else:
        utils.assembly_scene_analyzer.is_frame_from_scene.return_value = False

    result = run_check(utils, bottom)

    assert not result.success
    assert result.message
    utils.client_check_line_of_sight.call.assert_not_called()


@pytest.mark.parametrize('bottom', [False, True])
@pytest.mark.parametrize('failure', ['unavailable', 'no_response', 'tf_lookup'])
def test_visibility_errors_return_failure(utils, monkeypatch, bottom, failure):
    if failure == 'unavailable':
        utils.client_check_line_of_sight.wait_for_service.return_value = False
    elif failure == 'no_response':
        utils.client_check_line_of_sight.call.return_value = None
    else:
        monkeypatch.setattr(
            UTILS_MODULE, 'get_transform_for_frame_in_world',
            MagicMock(side_effect=TfFrameLookupError('missing transform')))

    result = run_check(utils, bottom)

    assert not result.success
    assert result.message


def test_top_visibility_ray_is_expressed_in_the_measurement_frame(utils, monkeypatch):
    world_target = TransformStamped()
    world_target.transform.rotation = Quaternion(
        **dict(zip(('x', 'y', 'z', 'w'), Rotation.from_euler('x', 90, degrees=True).as_quat())))
    monkeypatch.setattr(UTILS_MODULE, 'get_transform_for_frame_in_world', lambda *a: world_target)
    utils.client_move_robot_laser_to_frame.call.return_value.calculated_endeffector_pose.orientation = (
        world_target.transform.rotation)

    assert run_check(utils).success

    ray = utils.client_check_line_of_sight.call.call_args.args[0].transform_from_frame
    assert (ray.translation.x, ray.translation.y, ray.translation.z) == pytest.approx((0, 0.008, 0))


def test_bottom_visibility_ray_preserves_the_planned_orientation(utils, monkeypatch):
    planned_pose = utils.client_move_robot_cam1_to_frame.call.return_value.calculated_endeffector_pose
    planned_pose.orientation = Quaternion(
        **dict(zip(('x', 'y', 'z', 'w'), Rotation.from_euler('z', 90, degrees=True).as_quat())))
    sensor_pose = TransformStamped()
    sensor_pose.transform.rotation.w = 1.0
    sensor_pose.transform.translation.x = 1.0
    monkeypatch.setattr(UTILS_MODULE, 'get_transform_for_frame_in_world', lambda *a: sensor_pose)

    assert run_check(utils, bottom=True).success

    ray = utils.client_check_line_of_sight.call.call_args.args[0].transform_from_frame
    assert (ray.translation.x, ray.translation.y, ray.translation.z) == pytest.approx((0, -1, -0.04))
