# Copyright 2026 OpenAI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for rectangular spiral alignment result selection."""

import csv

from geometry_msgs.msg import Pose
import pytest
from pm_robot_primitive_skills.py_modules.PmRobotError import PmRobotError
from pm_skills.py_modules.pm_alignment_search_skills import (
    PmAlignmentSearchSkills,
)


def test_bool_result_is_centroid_of_true_samples():
    """Multiple true samples select their three-dimensional centroid."""
    samples = [
        ((-4.0, 2.0, 0.0), False),
        ((1.0, 3.0, -2.0), True),
        ((5.0, 7.0, 4.0), True),
        ((8.0, 9.0, 0.0), False),
    ]

    offset, value = PmAlignmentSearchSkills._best_result(samples, True)

    assert offset == (3.0, 5.0, 1.0)
    assert value is True


def test_bool_result_with_one_true_sample_uses_its_position():
    """A single true sample remains the selected result."""
    samples = [
        ((-1.0, 0.0, 0.0), False),
        ((2.0, 3.0, 4.0), True),
        ((5.0, 0.0, 0.0), False),
    ]

    offset, value = PmAlignmentSearchSkills._best_result(samples, True)

    assert offset == (2.0, 3.0, 4.0)
    assert value is True


def test_float_result_still_selects_maximum_sample():
    """Float signals preserve maximum-value selection."""
    samples = [
        ((1.0, 2.0, 0.0), 0.2),
        ((3.0, 4.0, 0.0), 0.9),
        ((5.0, 6.0, 0.0), 0.4),
    ]

    offset, value = PmAlignmentSearchSkills._best_result(samples, False)

    assert offset == (3.0, 4.0, 0.0)
    assert value == 0.9


def test_constant_float_result_returns_to_initial_position():
    """A float signal without contrast selects the zero offset."""
    samples = [
        ((-5.0, -5.0, 0.0), 0.4),
        ((2.0, 3.0, 0.0), 0.4),
        ((5.0, 5.0, 0.0), 0.4),
    ]

    offset, value = PmAlignmentSearchSkills._best_result(samples, False)

    assert offset == (0.0, 0.0, 0.0)
    assert value == 0.4


def test_all_false_bool_result_returns_to_initial_position():
    """An all-false boolean signal selects the zero offset."""
    samples = [
        ((-5.0, -5.0, 0.0), False),
        ((5.0, 5.0, 0.0), False),
    ]

    offset, value = PmAlignmentSearchSkills._best_result(samples, True)

    assert offset == (0.0, 0.0, 0.0)
    assert value is False


def test_all_true_bool_result_returns_to_initial_position():
    """An all-true boolean signal also selects the zero offset."""
    samples = [
        ((-5.0, -5.0, 0.0), True),
        ((5.0, 5.0, 0.0), True),
    ]

    offset, value = PmAlignmentSearchSkills._best_result(samples, True)

    assert offset == (0.0, 0.0, 0.0)
    assert value is True


def test_plan_pose_uses_selected_frame_as_endeffector_override():
    """Pose checks move the selected frame instead of the default TCP."""
    class FakeResponse:
        success = True
        joint_names = ['joint']
        joint_values = [1.5]

    class FakeClient:
        srv_name = '/move_smarpod_to_pose'
        request = None

        @staticmethod
        def wait_for_service(timeout_sec):
            return timeout_sec == 1.0

        def call(self, request):
            self.request = request
            return FakeResponse()

    class FakeLogger:
        @staticmethod
        def info(_message):
            pass

    skill = PmAlignmentSearchSkills.__new__(PmAlignmentSearchSkills)
    skill._move_smarpod_to_pose_client = FakeClient()
    skill._logger = FakeLogger()
    pose = Pose()
    pose.orientation.w = 1.0

    joints = skill._plan_pose(
        pose,
        use_hexapod=True,
        endeffector_frame='component_search_frame',
        local_offset_um=(10.0, 20.0, 0.0),
    )

    request = skill._move_smarpod_to_pose_client.request
    assert request.endeffector_frame_override == 'component_search_frame'
    assert request.execute_movement is False
    assert joints == {'joint': 1.5}


def test_plan_failure_explains_initial_preflight_stage():
    """A generic planner failure reports what succeeded and what to inspect."""
    class FakeResponse:
        success = False
        message = 'Planing failed!'

    class FakeClient:
        srv_name = '/move_smarpod_to_pose'

        @staticmethod
        def wait_for_service(timeout_sec):
            return timeout_sec == 1.0

        @staticmethod
        def call(_request):
            return FakeResponse()

    class FakeLogger:
        @staticmethod
        def info(_message):
            pass

    skill = PmAlignmentSearchSkills.__new__(PmAlignmentSearchSkills)
    skill._move_smarpod_to_pose_client = FakeClient()
    skill._logger = FakeLogger()
    pose = Pose()
    pose.orientation.w = 1.0

    with pytest.raises(PmRobotError) as raised:
        skill._plan_pose(
            pose,
            use_hexapod=True,
            endeffector_frame='focus_point',
            local_offset_um=(0.0, 0.0, 0.0),
            check_index=1,
            check_count=5,
        )

    message = str(raised.value)
    assert 'Preflight pose check 1/5' in message
    assert 'initial search position' in message
    assert 'Inverse kinematics succeeded' in message
    assert 'No collision contacts were reported' in message
    assert 'alignment subscription was not started' in message


def test_measurements_csv_contains_timestamps_and_positions(tmp_path):
    """CSV output preserves both clocks and all sampled position coordinates."""
    image_path = tmp_path / 'alignment.png'
    samples = [(
        (1.0, 2.0, 3.0),
        0.75,
        12_345_678_901,
        1_700_000_000_123_456_789,
        (0.101, 0.202, 0.303),
    )]

    csv_path = PmAlignmentSearchSkills._save_csv(str(image_path), samples)

    assert csv_path == str(tmp_path / 'alignment.csv')
    with open(csv_path, newline='', encoding='utf-8') as csv_file:
        rows = list(csv.DictReader(csv_file))
    assert len(rows) == 1
    assert rows[0]['ros_time_ns'] == '12345678901'
    assert rows[0]['wall_time_unix_ns'] == '1700000000123456789'
    assert rows[0]['local_offset_y_um'] == '2.0'
    assert rows[0]['controller_position_z_m'] == '0.303'


def test_plot_with_selected_result_is_created(tmp_path):
    """The selected-result overlay can be rendered with the measurements."""
    samples = [
        ((0.0, 0.0, 0.0), 0.1, 1, 2, (0.0, 0.0, 0.0)),
        ((5.0, 5.0, 0.0), 0.9, 3, 4, (0.1, 0.2, 0.3)),
    ]

    path = PmAlignmentSearchSkills._save_plot(
        str(tmp_path / 'alignment.png'),
        (0, 1),
        ((0.0, 0.0, 0.0), (5.0, 5.0, 0.0)),
        samples,
        False,
        (5.0, 5.0, 0.0),
        0.9,
    )

    assert path.endswith('alignment.png')
    assert (tmp_path / 'alignment.png').stat().st_size > 0


def test_output_files_get_a_timestamped_measurement_folder(tmp_path):
    """Each result pair gets a folder below rect_spiral_search."""
    image_path = PmAlignmentSearchSkills._output_image_path(
        str(tmp_path),
        timestamp='20260904_123456_123456',
    )

    expected_folder = (
        tmp_path / 'rect_spiral_search' / '20260904_123456_123456'
    )
    assert image_path == str(expected_folder / 'rect_spiral_search.png')
    assert expected_folder.is_dir()


def test_image_filename_is_preserved_inside_measurement_folder(tmp_path):
    """Legacy image paths retain their filename inside the pair folder."""
    image_path = PmAlignmentSearchSkills._output_image_path(
        str(tmp_path / 'custom.svg'),
        timestamp='20260904_123456_123456',
    )

    expected_folder = (
        tmp_path / 'rect_spiral_search' / '20260904_123456_123456'
    )
    assert image_path == str(expected_folder / 'custom.svg')
