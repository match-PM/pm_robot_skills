# generated from rosidl_generator_py/resource/_idl.py.em
# with input from pm_skills_interfaces:srv/PlaceComponent.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PlaceComponent_Request(type):
    """Metaclass of message 'PlaceComponent_Request'."""

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
                'pm_skills_interfaces.srv.PlaceComponent_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__place_component__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__place_component__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__place_component__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__place_component__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__place_component__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PlaceComponent_Request(metaclass=Metaclass_PlaceComponent_Request):
    """Message class 'PlaceComponent_Request'."""

    __slots__ = [
        '_align_orientation',
        '_x_offset_um',
        '_y_offset_um',
        '_z_offset_um',
        '_rx_offset_deg',
        '_ry_offset_deg',
        '_rz_offset_deg',
    ]

    _fields_and_field_types = {
        'align_orientation': 'boolean',
        'x_offset_um': 'float',
        'y_offset_um': 'float',
        'z_offset_um': 'float',
        'rx_offset_deg': 'float',
        'ry_offset_deg': 'float',
        'rz_offset_deg': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.align_orientation = kwargs.get('align_orientation', bool())
        self.x_offset_um = kwargs.get('x_offset_um', float())
        self.y_offset_um = kwargs.get('y_offset_um', float())
        self.z_offset_um = kwargs.get('z_offset_um', float())
        self.rx_offset_deg = kwargs.get('rx_offset_deg', float())
        self.ry_offset_deg = kwargs.get('ry_offset_deg', float())
        self.rz_offset_deg = kwargs.get('rz_offset_deg', float())

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
        if self.align_orientation != other.align_orientation:
            return False
        if self.x_offset_um != other.x_offset_um:
            return False
        if self.y_offset_um != other.y_offset_um:
            return False
        if self.z_offset_um != other.z_offset_um:
            return False
        if self.rx_offset_deg != other.rx_offset_deg:
            return False
        if self.ry_offset_deg != other.ry_offset_deg:
            return False
        if self.rz_offset_deg != other.rz_offset_deg:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def align_orientation(self):
        """Message field 'align_orientation'."""
        return self._align_orientation

    @align_orientation.setter
    def align_orientation(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'align_orientation' field must be of type 'bool'"
        self._align_orientation = value

    @builtins.property
    def x_offset_um(self):
        """Message field 'x_offset_um'."""
        return self._x_offset_um

    @x_offset_um.setter
    def x_offset_um(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x_offset_um' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'x_offset_um' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._x_offset_um = value

    @builtins.property
    def y_offset_um(self):
        """Message field 'y_offset_um'."""
        return self._y_offset_um

    @y_offset_um.setter
    def y_offset_um(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y_offset_um' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'y_offset_um' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._y_offset_um = value

    @builtins.property
    def z_offset_um(self):
        """Message field 'z_offset_um'."""
        return self._z_offset_um

    @z_offset_um.setter
    def z_offset_um(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z_offset_um' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'z_offset_um' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._z_offset_um = value

    @builtins.property
    def rx_offset_deg(self):
        """Message field 'rx_offset_deg'."""
        return self._rx_offset_deg

    @rx_offset_deg.setter
    def rx_offset_deg(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rx_offset_deg' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'rx_offset_deg' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._rx_offset_deg = value

    @builtins.property
    def ry_offset_deg(self):
        """Message field 'ry_offset_deg'."""
        return self._ry_offset_deg

    @ry_offset_deg.setter
    def ry_offset_deg(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ry_offset_deg' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'ry_offset_deg' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._ry_offset_deg = value

    @builtins.property
    def rz_offset_deg(self):
        """Message field 'rz_offset_deg'."""
        return self._rz_offset_deg

    @rz_offset_deg.setter
    def rz_offset_deg(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rz_offset_deg' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'rz_offset_deg' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._rz_offset_deg = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_PlaceComponent_Response(type):
    """Metaclass of message 'PlaceComponent_Response'."""

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
                'pm_skills_interfaces.srv.PlaceComponent_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__place_component__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__place_component__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__place_component__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__place_component__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__place_component__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PlaceComponent_Response(metaclass=Metaclass_PlaceComponent_Response):
    """Message class 'PlaceComponent_Response'."""

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


class Metaclass_PlaceComponent(type):
    """Metaclass of service 'PlaceComponent'."""

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
                'pm_skills_interfaces.srv.PlaceComponent')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__place_component

            from pm_skills_interfaces.srv import _place_component
            if _place_component.Metaclass_PlaceComponent_Request._TYPE_SUPPORT is None:
                _place_component.Metaclass_PlaceComponent_Request.__import_type_support__()
            if _place_component.Metaclass_PlaceComponent_Response._TYPE_SUPPORT is None:
                _place_component.Metaclass_PlaceComponent_Response.__import_type_support__()


class PlaceComponent(metaclass=Metaclass_PlaceComponent):
    from pm_skills_interfaces.srv._place_component import PlaceComponent_Request as Request
    from pm_skills_interfaces.srv._place_component import PlaceComponent_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
