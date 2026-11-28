"""
Parameters of pm_skills shared with the planner (config/skill_parameters.yaml).

No ROS node is needed: the pm_skills node and the planner (pm_pddl_planning)
both load the installed file with :func:`load_skill_parameters`.
"""

import hashlib
import math
import os
from dataclasses import dataclass

import yaml

FILE_NAME = 'skill_parameters.yaml'


class SkillParameterError(ValueError):
    """The parameter file is missing, incomplete or has invalid values."""


@dataclass(frozen=True)
class GripParameters:
    approach_offset: float
    sensing_start_offset: float
    sensing_end_offset: float
    sensing_end_rough: float
    gripping_offset: float
    lift_distance: float


@dataclass(frozen=True)
class ReleaseParameters:
    lift_distance: float


@dataclass(frozen=True)
class FeatureRadius:
    """Surroundings each sensor kind needs clear around a frame [m]."""
    camera: float
    laser: float
    confocal: float


@dataclass(frozen=True)
class LineOfSightParameters:
    feature_radius: FeatureRadius
    tilt_tolerance_deg: float

    @property
    def tilt_tolerance(self):
        """The tilt tolerance in rad."""
        return math.radians(self.tilt_tolerance_deg)

    def request_values(self, sensor_kind):
        """``(feature_radius [m], tilt_tolerance [rad])`` for a
        CheckLineOfSight request of ``camera``, ``laser`` or ``confocal``."""
        if sensor_kind not in FeatureRadius.__dataclass_fields__:
            raise SkillParameterError(f'Unknown sensor kind {sensor_kind!r}.')
        return getattr(self.feature_radius, sensor_kind), self.tilt_tolerance

    def as_dict(self):
        return {'feature_radius': dict(self.feature_radius.__dict__),
                'tilt_tolerance_deg': self.tilt_tolerance_deg}


@dataclass(frozen=True)
class SkillParameters:
    gripper_frame: str
    grip: GripParameters
    release: ReleaseParameters
    line_of_sight: LineOfSightParameters
    path: str     # file the values were read from
    sha256: str   # of the file content, to tell parameter versions apart

    def as_dict(self):
        return {'gripper_frame': self.gripper_frame, 'grip': dict(self.grip.__dict__),
                'release': dict(self.release.__dict__),
                'line_of_sight': self.line_of_sight.as_dict()}


def default_path():
    """Return the installed file: share/pm_skills/config/skill_parameters.yaml."""
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('pm_skills'), 'config', FILE_NAME)


def load_skill_parameters(path=None):
    """Read and check the parameter file (default: the installed one)."""
    path = path or default_path()
    try:
        with open(path, 'rb') as stream:
            content = stream.read()
    except OSError as error:
        raise SkillParameterError(f'Cannot read skill parameters {path}: {error}') from error
    try:
        data = yaml.safe_load(content)
    except yaml.YAMLError as error:
        raise SkillParameterError(f'{path} is not valid YAML: {error}') from error
    if not isinstance(data, dict):
        raise SkillParameterError(f'{path} must contain a mapping.')

    _keys(data, ('gripper_frame', 'grip', 'release', 'line_of_sight'), path, '')
    frame = data['gripper_frame']
    if not isinstance(frame, str) or not frame:
        raise SkillParameterError(f'{path}: gripper_frame must be a frame name.')
    return SkillParameters(
        gripper_frame=frame,
        grip=GripParameters(**_numbers(data['grip'], GripParameters, path, 'grip')),
        release=ReleaseParameters(**_numbers(data['release'], ReleaseParameters, path,
                                             'release')),
        line_of_sight=_line_of_sight(data['line_of_sight'], path),
        path=str(path), sha256=hashlib.sha256(content).hexdigest())


def _keys(data, expected, path, section):
    where = f'{section}.' if section else ''
    missing = [key for key in expected if key not in data]
    unknown = sorted(set(data) - set(expected))
    if missing or unknown:
        problems = ([f'missing {where}{key}' for key in missing]
                    + [f'unknown {where}{key}' for key in unknown])
        raise SkillParameterError(f'{path}: ' + ', '.join(problems) + '.')


def _numbers(data, kind, path, section, names=None):
    if not isinstance(data, dict):
        raise SkillParameterError(f'{path}: {section} must be a mapping.')
    names = names or tuple(kind.__dataclass_fields__)
    _keys(data, names, path, section)
    values = {}
    for name in names:
        value = data[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)) \
                or not math.isfinite(value):
            raise SkillParameterError(f'{path}: {section}.{name} must be a number, '
                                      f'not {value!r}.')
        values[name] = float(value)
    return values


def _line_of_sight(data, path):
    if not isinstance(data, dict):
        raise SkillParameterError(f'{path}: line_of_sight must be a mapping.')
    _keys(data, ('feature_radius', 'tilt_tolerance_deg'), path, 'line_of_sight')
    radius = _numbers(data['feature_radius'], FeatureRadius, path,
                      'line_of_sight.feature_radius')
    tilt = _numbers({'tilt_tolerance_deg': data['tilt_tolerance_deg']}, LineOfSightParameters,
                    path, 'line_of_sight', ('tilt_tolerance_deg',))['tilt_tolerance_deg']
    negative = [f'line_of_sight.feature_radius.{name}' for name, value in radius.items()
                if value < 0] + (['line_of_sight.tilt_tolerance_deg'] if tilt < 0 else [])
    if negative:
        raise SkillParameterError(f'{path}: ' + ', '.join(negative) + ' must not be negative.')
    return LineOfSightParameters(FeatureRadius(**radius), tilt)
