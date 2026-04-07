# generated from rosidl_generator_py/resource/_idl.py.em
# with input from pm_skills_interfaces:srv/CorrectFrameLaser.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CorrectFrameLaser_Request(type):
    """Metaclass of message 'CorrectFrameLaser_Request'."""

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
                'pm_skills_interfaces.srv.CorrectFrameLaser_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__correct_frame_laser__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__correct_frame_laser__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__correct_frame_laser__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__correct_frame_laser__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__correct_frame_laser__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'REMEASURE_AFTER_CORRECTION__DEFAULT': False,
        }

    @property
    def REMEASURE_AFTER_CORRECTION__DEFAULT(cls):
        """Return default value for message field 'remeasure_after_correction'."""
        return False


class CorrectFrameLaser_Request(metaclass=Metaclass_CorrectFrameLaser_Request):
    """Message class 'CorrectFrameLaser_Request'."""

    __slots__ = [
        '_frame_name',
        '_remeasure_after_correction',
        '_use_iterative_sensing',
    ]

    _fields_and_field_types = {
        'frame_name': 'string',
        'remeasure_after_correction': 'boolean',
        'use_iterative_sensing': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.frame_name = kwargs.get('frame_name', str())
        self.remeasure_after_correction = kwargs.get(
            'remeasure_after_correction', CorrectFrameLaser_Request.REMEASURE_AFTER_CORRECTION__DEFAULT)
        self.use_iterative_sensing = kwargs.get('use_iterative_sensing', bool())

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
        if self.frame_name != other.frame_name:
            return False
        if self.remeasure_after_correction != other.remeasure_after_correction:
            return False
        if self.use_iterative_sensing != other.use_iterative_sensing:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def frame_name(self):
        """Message field 'frame_name'."""
        return self._frame_name

    @frame_name.setter
    def frame_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'frame_name' field must be of type 'str'"
        self._frame_name = value

    @builtins.property
    def remeasure_after_correction(self):
        """Message field 'remeasure_after_correction'."""
        return self._remeasure_after_correction

    @remeasure_after_correction.setter
    def remeasure_after_correction(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'remeasure_after_correction' field must be of type 'bool'"
        self._remeasure_after_correction = value

    @builtins.property
    def use_iterative_sensing(self):
        """Message field 'use_iterative_sensing'."""
        return self._use_iterative_sensing

    @use_iterative_sensing.setter
    def use_iterative_sensing(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'use_iterative_sensing' field must be of type 'bool'"
        self._use_iterative_sensing = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_CorrectFrameLaser_Response(type):
    """Metaclass of message 'CorrectFrameLaser_Response'."""

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
                'pm_skills_interfaces.srv.CorrectFrameLaser_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__correct_frame_laser__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__correct_frame_laser__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__correct_frame_laser__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__correct_frame_laser__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__correct_frame_laser__response

            from geometry_msgs.msg import Vector3
            if Vector3.__class__._TYPE_SUPPORT is None:
                Vector3.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CorrectFrameLaser_Response(metaclass=Metaclass_CorrectFrameLaser_Response):
    """Message class 'CorrectFrameLaser_Response'."""

    __slots__ = [
        '_success',
        '_correction_values',
        '_message',
        '_component_name',
        '_component_uuid',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'correction_values': 'geometry_msgs/Vector3',
        'message': 'string',
        'component_name': 'string',
        'component_uuid': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        from geometry_msgs.msg import Vector3
        self.correction_values = kwargs.get('correction_values', Vector3())
        self.message = kwargs.get('message', str())
        self.component_name = kwargs.get('component_name', str())
        self.component_uuid = kwargs.get('component_uuid', str())

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
        if self.correction_values != other.correction_values:
            return False
        if self.message != other.message:
            return False
        if self.component_name != other.component_name:
            return False
        if self.component_uuid != other.component_uuid:
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
    def correction_values(self):
        """Message field 'correction_values'."""
        return self._correction_values

    @correction_values.setter
    def correction_values(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'correction_values' field must be a sub message of type 'Vector3'"
        self._correction_values = value

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value

    @builtins.property
    def component_name(self):
        """Message field 'component_name'."""
        return self._component_name

    @component_name.setter
    def component_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'component_name' field must be of type 'str'"
        self._component_name = value

    @builtins.property
    def component_uuid(self):
        """Message field 'component_uuid'."""
        return self._component_uuid

    @component_uuid.setter
    def component_uuid(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'component_uuid' field must be of type 'str'"
        self._component_uuid = value


class Metaclass_CorrectFrameLaser(type):
    """Metaclass of service 'CorrectFrameLaser'."""

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
                'pm_skills_interfaces.srv.CorrectFrameLaser')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__correct_frame_laser

            from pm_skills_interfaces.srv import _correct_frame_laser
            if _correct_frame_laser.Metaclass_CorrectFrameLaser_Request._TYPE_SUPPORT is None:
                _correct_frame_laser.Metaclass_CorrectFrameLaser_Request.__import_type_support__()
            if _correct_frame_laser.Metaclass_CorrectFrameLaser_Response._TYPE_SUPPORT is None:
                _correct_frame_laser.Metaclass_CorrectFrameLaser_Response.__import_type_support__()


class CorrectFrameLaser(metaclass=Metaclass_CorrectFrameLaser):
    from pm_skills_interfaces.srv._correct_frame_laser import CorrectFrameLaser_Request as Request
    from pm_skills_interfaces.srv._correct_frame_laser import CorrectFrameLaser_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
