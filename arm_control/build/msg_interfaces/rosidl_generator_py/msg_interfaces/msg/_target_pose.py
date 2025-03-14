# generated from rosidl_generator_py/resource/_idl.py.em
# with input from msg_interfaces:msg/TargetPose.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TargetPose(type):
    """Metaclass of message 'TargetPose'."""

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
                'msg_interfaces.msg.TargetPose')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__target_pose
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__target_pose
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__target_pose
            cls._TYPE_SUPPORT = module.type_support_msg__msg__target_pose
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__target_pose

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TargetPose(metaclass=Metaclass_TargetPose):
    """Message class 'TargetPose'."""

    __slots__ = [
        '_base',
        '_shoulder',
        '_elbow',
        '_wrist_up_down',
        '_wrist_rot',
    ]

    _fields_and_field_types = {
        'base': 'double',
        'shoulder': 'double',
        'elbow': 'double',
        'wrist_up_down': 'double',
        'wrist_rot': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.base = kwargs.get('base', float())
        self.shoulder = kwargs.get('shoulder', float())
        self.elbow = kwargs.get('elbow', float())
        self.wrist_up_down = kwargs.get('wrist_up_down', float())
        self.wrist_rot = kwargs.get('wrist_rot', float())

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
        if self.base != other.base:
            return False
        if self.shoulder != other.shoulder:
            return False
        if self.elbow != other.elbow:
            return False
        if self.wrist_up_down != other.wrist_up_down:
            return False
        if self.wrist_rot != other.wrist_rot:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def base(self):
        """Message field 'base'."""
        return self._base

    @base.setter
    def base(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'base' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'base' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._base = value

    @builtins.property
    def shoulder(self):
        """Message field 'shoulder'."""
        return self._shoulder

    @shoulder.setter
    def shoulder(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'shoulder' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'shoulder' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._shoulder = value

    @builtins.property
    def elbow(self):
        """Message field 'elbow'."""
        return self._elbow

    @elbow.setter
    def elbow(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'elbow' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'elbow' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._elbow = value

    @builtins.property
    def wrist_up_down(self):
        """Message field 'wrist_up_down'."""
        return self._wrist_up_down

    @wrist_up_down.setter
    def wrist_up_down(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'wrist_up_down' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'wrist_up_down' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._wrist_up_down = value

    @builtins.property
    def wrist_rot(self):
        """Message field 'wrist_rot'."""
        return self._wrist_rot

    @wrist_rot.setter
    def wrist_rot(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'wrist_rot' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'wrist_rot' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._wrist_rot = value
