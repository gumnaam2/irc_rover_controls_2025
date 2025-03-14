# generated from rosidl_generator_py/resource/_idl.py.em
# with input from msg_interfaces:msg/MotorSix.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MotorSix(type):
    """Metaclass of message 'MotorSix'."""

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
            module = import_type_support('msg_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'msg_interfaces.msg.MotorSix')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__motor_six
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__motor_six
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__motor_six
            cls._TYPE_SUPPORT = module.type_support_msg__msg__motor_six
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__motor_six

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MotorSix(metaclass=Metaclass_MotorSix):
    """Message class 'MotorSix'."""

    __slots__ = [
        '_motor1_speed',
        '_motor1_direction',
        '_motor2_speed',
        '_motor2_direction',
        '_motor3_speed',
        '_motor3_direction',
        '_motor4_speed',
        '_motor4_direction',
        '_motor5_speed',
        '_motor5_direction',
        '_motor6_speed',
        '_motor6_direction',
    ]

    _fields_and_field_types = {
        'motor1_speed': 'double',
        'motor1_direction': 'string',
        'motor2_speed': 'double',
        'motor2_direction': 'string',
        'motor3_speed': 'double',
        'motor3_direction': 'string',
        'motor4_speed': 'double',
        'motor4_direction': 'string',
        'motor5_speed': 'double',
        'motor5_direction': 'string',
        'motor6_speed': 'double',
        'motor6_direction': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.motor1_speed = kwargs.get('motor1_speed', float())
        self.motor1_direction = kwargs.get('motor1_direction', str())
        self.motor2_speed = kwargs.get('motor2_speed', float())
        self.motor2_direction = kwargs.get('motor2_direction', str())
        self.motor3_speed = kwargs.get('motor3_speed', float())
        self.motor3_direction = kwargs.get('motor3_direction', str())
        self.motor4_speed = kwargs.get('motor4_speed', float())
        self.motor4_direction = kwargs.get('motor4_direction', str())
        self.motor5_speed = kwargs.get('motor5_speed', float())
        self.motor5_direction = kwargs.get('motor5_direction', str())
        self.motor6_speed = kwargs.get('motor6_speed', float())
        self.motor6_direction = kwargs.get('motor6_direction', str())

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
        if self.motor1_speed != other.motor1_speed:
            return False
        if self.motor1_direction != other.motor1_direction:
            return False
        if self.motor2_speed != other.motor2_speed:
            return False
        if self.motor2_direction != other.motor2_direction:
            return False
        if self.motor3_speed != other.motor3_speed:
            return False
        if self.motor3_direction != other.motor3_direction:
            return False
        if self.motor4_speed != other.motor4_speed:
            return False
        if self.motor4_direction != other.motor4_direction:
            return False
        if self.motor5_speed != other.motor5_speed:
            return False
        if self.motor5_direction != other.motor5_direction:
            return False
        if self.motor6_speed != other.motor6_speed:
            return False
        if self.motor6_direction != other.motor6_direction:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def motor1_speed(self):
        """Message field 'motor1_speed'."""
        return self._motor1_speed

    @motor1_speed.setter
    def motor1_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'motor1_speed' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'motor1_speed' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._motor1_speed = value

    @builtins.property
    def motor1_direction(self):
        """Message field 'motor1_direction'."""
        return self._motor1_direction

    @motor1_direction.setter
    def motor1_direction(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'motor1_direction' field must be of type 'str'"
        self._motor1_direction = value

    @builtins.property
    def motor2_speed(self):
        """Message field 'motor2_speed'."""
        return self._motor2_speed

    @motor2_speed.setter
    def motor2_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'motor2_speed' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'motor2_speed' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._motor2_speed = value

    @builtins.property
    def motor2_direction(self):
        """Message field 'motor2_direction'."""
        return self._motor2_direction

    @motor2_direction.setter
    def motor2_direction(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'motor2_direction' field must be of type 'str'"
        self._motor2_direction = value

    @builtins.property
    def motor3_speed(self):
        """Message field 'motor3_speed'."""
        return self._motor3_speed

    @motor3_speed.setter
    def motor3_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'motor3_speed' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'motor3_speed' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._motor3_speed = value

    @builtins.property
    def motor3_direction(self):
        """Message field 'motor3_direction'."""
        return self._motor3_direction

    @motor3_direction.setter
    def motor3_direction(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'motor3_direction' field must be of type 'str'"
        self._motor3_direction = value

    @builtins.property
    def motor4_speed(self):
        """Message field 'motor4_speed'."""
        return self._motor4_speed

    @motor4_speed.setter
    def motor4_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'motor4_speed' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'motor4_speed' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._motor4_speed = value

    @builtins.property
    def motor4_direction(self):
        """Message field 'motor4_direction'."""
        return self._motor4_direction

    @motor4_direction.setter
    def motor4_direction(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'motor4_direction' field must be of type 'str'"
        self._motor4_direction = value

    @builtins.property
    def motor5_speed(self):
        """Message field 'motor5_speed'."""
        return self._motor5_speed

    @motor5_speed.setter
    def motor5_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'motor5_speed' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'motor5_speed' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._motor5_speed = value

    @builtins.property
    def motor5_direction(self):
        """Message field 'motor5_direction'."""
        return self._motor5_direction

    @motor5_direction.setter
    def motor5_direction(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'motor5_direction' field must be of type 'str'"
        self._motor5_direction = value

    @builtins.property
    def motor6_speed(self):
        """Message field 'motor6_speed'."""
        return self._motor6_speed

    @motor6_speed.setter
    def motor6_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'motor6_speed' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'motor6_speed' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._motor6_speed = value

    @builtins.property
    def motor6_direction(self):
        """Message field 'motor6_direction'."""
        return self._motor6_direction

    @motor6_direction.setter
    def motor6_direction(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'motor6_direction' field must be of type 'str'"
        self._motor6_direction = value
