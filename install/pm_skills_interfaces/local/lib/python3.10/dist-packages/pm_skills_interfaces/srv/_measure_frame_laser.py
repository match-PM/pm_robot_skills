# generated from rosidl_generator_py/resource/_idl.py.em
# with input from pm_skills_interfaces:srv/MeasureFrameLaser.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MeasureFrameLaser_Request(type):
    """Metaclass of message 'MeasureFrameLaser_Request'."""

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
                'pm_skills_interfaces.srv.MeasureFrameLaser_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__measure_frame_laser__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__measure_frame_laser__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__measure_frame_laser__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__measure_frame_laser__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__measure_frame_laser__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MeasureFrameLaser_Request(metaclass=Metaclass_MeasureFrameLaser_Request):
    """Message class 'MeasureFrameLaser_Request'."""

    __slots__ = [
        '_frame_name',
    ]

    _fields_and_field_types = {
        'frame_name': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.frame_name = kwargs.get('frame_name', str())

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


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_MeasureFrameLaser_Response(type):
    """Metaclass of message 'MeasureFrameLaser_Response'."""

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
                'pm_skills_interfaces.srv.MeasureFrameLaser_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__measure_frame_laser__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__measure_frame_laser__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__measure_frame_laser__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__measure_frame_laser__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__measure_frame_laser__response

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


class MeasureFrameLaser_Response(metaclass=Metaclass_MeasureFrameLaser_Response):
    """Message class 'MeasureFrameLaser_Response'."""

    __slots__ = [
        '_success',
        '_result_vector',
        '_message',
        '_component_name',
        '_component_uuid',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'result_vector': 'geometry_msgs/Vector3',
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
        self.result_vector = kwargs.get('result_vector', Vector3())
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
        if self.result_vector != other.result_vector:
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
    def result_vector(self):
        """Message field 'result_vector'."""
        return self._result_vector

    @result_vector.setter
    def result_vector(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'result_vector' field must be a sub message of type 'Vector3'"
        self._result_vector = value

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


class Metaclass_MeasureFrameLaser(type):
    """Metaclass of service 'MeasureFrameLaser'."""

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
                'pm_skills_interfaces.srv.MeasureFrameLaser')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__measure_frame_laser

            from pm_skills_interfaces.srv import _measure_frame_laser
            if _measure_frame_laser.Metaclass_MeasureFrameLaser_Request._TYPE_SUPPORT is None:
                _measure_frame_laser.Metaclass_MeasureFrameLaser_Request.__import_type_support__()
            if _measure_frame_laser.Metaclass_MeasureFrameLaser_Response._TYPE_SUPPORT is None:
                _measure_frame_laser.Metaclass_MeasureFrameLaser_Response.__import_type_support__()


class MeasureFrameLaser(metaclass=Metaclass_MeasureFrameLaser):
    from pm_skills_interfaces.srv._measure_frame_laser import MeasureFrameLaser_Request as Request
    from pm_skills_interfaces.srv._measure_frame_laser import MeasureFrameLaser_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
