# generated from rosidl_generator_py/resource/_idl.py.em
# with input from pm_skills_interfaces:srv/IterativeGonioAlign.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_IterativeGonioAlign_Request(type):
    """Metaclass of message 'IterativeGonioAlign_Request'."""

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
                'pm_skills_interfaces.srv.IterativeGonioAlign_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__iterative_gonio_align__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__iterative_gonio_align__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__iterative_gonio_align__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__iterative_gonio_align__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__iterative_gonio_align__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class IterativeGonioAlign_Request(metaclass=Metaclass_IterativeGonioAlign_Request):
    """Message class 'IterativeGonioAlign_Request'."""

    __slots__ = [
        '_confocal_laser',
        '_target_alignment_frame',
        '_gonio_endeffector_frame',
        '_num_iterations',
        '_frames_to_measure',
    ]

    _fields_and_field_types = {
        'confocal_laser': 'boolean',
        'target_alignment_frame': 'string',
        'gonio_endeffector_frame': 'string',
        'num_iterations': 'int32',
        'frames_to_measure': 'sequence<string>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.confocal_laser = kwargs.get('confocal_laser', bool())
        self.target_alignment_frame = kwargs.get('target_alignment_frame', str())
        self.gonio_endeffector_frame = kwargs.get('gonio_endeffector_frame', str())
        self.num_iterations = kwargs.get('num_iterations', int())
        self.frames_to_measure = kwargs.get('frames_to_measure', [])

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
        if self.confocal_laser != other.confocal_laser:
            return False
        if self.target_alignment_frame != other.target_alignment_frame:
            return False
        if self.gonio_endeffector_frame != other.gonio_endeffector_frame:
            return False
        if self.num_iterations != other.num_iterations:
            return False
        if self.frames_to_measure != other.frames_to_measure:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def confocal_laser(self):
        """Message field 'confocal_laser'."""
        return self._confocal_laser

    @confocal_laser.setter
    def confocal_laser(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'confocal_laser' field must be of type 'bool'"
        self._confocal_laser = value

    @builtins.property
    def target_alignment_frame(self):
        """Message field 'target_alignment_frame'."""
        return self._target_alignment_frame

    @target_alignment_frame.setter
    def target_alignment_frame(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'target_alignment_frame' field must be of type 'str'"
        self._target_alignment_frame = value

    @builtins.property
    def gonio_endeffector_frame(self):
        """Message field 'gonio_endeffector_frame'."""
        return self._gonio_endeffector_frame

    @gonio_endeffector_frame.setter
    def gonio_endeffector_frame(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'gonio_endeffector_frame' field must be of type 'str'"
        self._gonio_endeffector_frame = value

    @builtins.property
    def num_iterations(self):
        """Message field 'num_iterations'."""
        return self._num_iterations

    @num_iterations.setter
    def num_iterations(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_iterations' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'num_iterations' field must be an integer in [-2147483648, 2147483647]"
        self._num_iterations = value

    @builtins.property
    def frames_to_measure(self):
        """Message field 'frames_to_measure'."""
        return self._frames_to_measure

    @frames_to_measure.setter
    def frames_to_measure(self, value):
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'frames_to_measure' field must be a set or sequence and each value of type 'str'"
        self._frames_to_measure = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_IterativeGonioAlign_Response(type):
    """Metaclass of message 'IterativeGonioAlign_Response'."""

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
                'pm_skills_interfaces.srv.IterativeGonioAlign_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__iterative_gonio_align__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__iterative_gonio_align__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__iterative_gonio_align__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__iterative_gonio_align__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__iterative_gonio_align__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class IterativeGonioAlign_Response(metaclass=Metaclass_IterativeGonioAlign_Response):
    """Message class 'IterativeGonioAlign_Response'."""

    __slots__ = [
        '_success',
        '_message',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())

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
        if self.message != other.message:
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


class Metaclass_IterativeGonioAlign(type):
    """Metaclass of service 'IterativeGonioAlign'."""

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
                'pm_skills_interfaces.srv.IterativeGonioAlign')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__iterative_gonio_align

            from pm_skills_interfaces.srv import _iterative_gonio_align
            if _iterative_gonio_align.Metaclass_IterativeGonioAlign_Request._TYPE_SUPPORT is None:
                _iterative_gonio_align.Metaclass_IterativeGonioAlign_Request.__import_type_support__()
            if _iterative_gonio_align.Metaclass_IterativeGonioAlign_Response._TYPE_SUPPORT is None:
                _iterative_gonio_align.Metaclass_IterativeGonioAlign_Response.__import_type_support__()


class IterativeGonioAlign(metaclass=Metaclass_IterativeGonioAlign):
    from pm_skills_interfaces.srv._iterative_gonio_align import IterativeGonioAlign_Request as Request
    from pm_skills_interfaces.srv._iterative_gonio_align import IterativeGonioAlign_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
