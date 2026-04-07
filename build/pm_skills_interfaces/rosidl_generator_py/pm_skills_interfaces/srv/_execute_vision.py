# generated from rosidl_generator_py/resource/_idl.py.em
# with input from pm_skills_interfaces:srv/ExecuteVision.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ExecuteVision_Request(type):
    """Metaclass of message 'ExecuteVision_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('pm_skills_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'pm_skills_interfaces.srv.ExecuteVision_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__execute_vision__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__execute_vision__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__execute_vision__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__execute_vision__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__execute_vision__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExecuteVision_Request(metaclass=Metaclass_ExecuteVision_Request):
    """Message class 'ExecuteVision_Request'."""

    __slots__ = [
        '_process_filename',
        '_camera_config_filename',
        '_process_uid',
        '_image_display_time',
        '_run_cross_validation',
    ]

    _fields_and_field_types = {
        'process_filename': 'string',
        'camera_config_filename': 'string',
        'process_uid': 'string',
        'image_display_time': 'float',
        'run_cross_validation': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.process_filename = kwargs.get('process_filename', str())
        self.camera_config_filename = kwargs.get('camera_config_filename', str())
        self.process_uid = kwargs.get('process_uid', str())
        self.image_display_time = kwargs.get('image_display_time', float())
        self.run_cross_validation = kwargs.get('run_cross_validation', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.process_filename != other.process_filename:
            return False
        if self.camera_config_filename != other.camera_config_filename:
            return False
        if self.process_uid != other.process_uid:
            return False
        if self.image_display_time != other.image_display_time:
            return False
        if self.run_cross_validation != other.run_cross_validation:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def process_filename(self):
        """Message field 'process_filename'."""
        return self._process_filename

    @process_filename.setter
    def process_filename(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'process_filename' field must be of type 'str'"
        self._process_filename = value

    @builtins.property
    def camera_config_filename(self):
        """Message field 'camera_config_filename'."""
        return self._camera_config_filename

    @camera_config_filename.setter
    def camera_config_filename(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'camera_config_filename' field must be of type 'str'"
        self._camera_config_filename = value

    @builtins.property
    def process_uid(self):
        """Message field 'process_uid'."""
        return self._process_uid

    @process_uid.setter
    def process_uid(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'process_uid' field must be of type 'str'"
        self._process_uid = value

    @builtins.property
    def image_display_time(self):
        """Message field 'image_display_time'."""
        return self._image_display_time

    @image_display_time.setter
    def image_display_time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'image_display_time' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'image_display_time' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._image_display_time = value

    @builtins.property
    def run_cross_validation(self):
        """Message field 'run_cross_validation'."""
        return self._run_cross_validation

    @run_cross_validation.setter
    def run_cross_validation(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'run_cross_validation' field must be of type 'bool'"
        self._run_cross_validation = value


# Import statements for member types

# Member 'points'
import array  # noqa: E402, I100

# already imported above
# import builtins

# already imported above
# import math

# already imported above
# import rosidl_parser.definition


class Metaclass_ExecuteVision_Response(type):
    """Metaclass of message 'ExecuteVision_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('pm_skills_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'pm_skills_interfaces.srv.ExecuteVision_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__execute_vision__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__execute_vision__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__execute_vision__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__execute_vision__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__execute_vision__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExecuteVision_Response(metaclass=Metaclass_ExecuteVision_Response):
    """Message class 'ExecuteVision_Response'."""

    __slots__ = [
        '_success',
        '_results_dict',
        '_results_path',
        '_points',
        '_process_uid',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'results_dict': 'string',
        'results_path': 'string',
        'points': 'sequence<float>',
        'process_uid': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.results_dict = kwargs.get('results_dict', str())
        self.results_path = kwargs.get('results_path', str())
        self.points = array.array('f', kwargs.get('points', []))
        self.process_uid = kwargs.get('process_uid', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.success != other.success:
            return False
        if self.results_dict != other.results_dict:
            return False
        if self.results_path != other.results_path:
            return False
        if self.points != other.points:
            return False
        if self.process_uid != other.process_uid:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def results_dict(self):
        """Message field 'results_dict'."""
        return self._results_dict

    @results_dict.setter
    def results_dict(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'results_dict' field must be of type 'str'"
        self._results_dict = value

    @builtins.property
    def results_path(self):
        """Message field 'results_path'."""
        return self._results_path

    @results_path.setter
    def results_path(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'results_path' field must be of type 'str'"
        self._results_path = value

    @builtins.property
    def points(self):
        """Message field 'points'."""
        return self._points

    @points.setter
    def points(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'points' array.array() must have the type code of 'f'"
            self._points = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'points' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._points = array.array('f', value)

    @builtins.property
    def process_uid(self):
        """Message field 'process_uid'."""
        return self._process_uid

    @process_uid.setter
    def process_uid(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'process_uid' field must be of type 'str'"
        self._process_uid = value


class Metaclass_ExecuteVision(type):
    """Metaclass of service 'ExecuteVision'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('pm_skills_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'pm_skills_interfaces.srv.ExecuteVision')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__execute_vision

            from pm_skills_interfaces.srv import _execute_vision
            if _execute_vision.Metaclass_ExecuteVision_Request._TYPE_SUPPORT is None:
                _execute_vision.Metaclass_ExecuteVision_Request.__import_type_support__()
            if _execute_vision.Metaclass_ExecuteVision_Response._TYPE_SUPPORT is None:
                _execute_vision.Metaclass_ExecuteVision_Response.__import_type_support__()


class ExecuteVision(metaclass=Metaclass_ExecuteVision):
    from pm_skills_interfaces.srv._execute_vision import ExecuteVision_Request as Request
    from pm_skills_interfaces.srv._execute_vision import ExecuteVision_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
