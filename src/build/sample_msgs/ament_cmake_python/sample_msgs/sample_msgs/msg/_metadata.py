# generated from rosidl_generator_py/resource/_idl.py.em
# with input from sample_msgs:msg/Metadata.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Metadata(type):
    """Metaclass of message 'Metadata'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'DEFAULT': 0,
        'DICTIONARY': 1,
        'RUN_LENGTH': 2,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('sample_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'sample_msgs.msg.Metadata')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__metadata
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__metadata
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__metadata
            cls._TYPE_SUPPORT = module.type_support_msg__msg__metadata
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__metadata

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'DEFAULT': cls.__constants['DEFAULT'],
            'DICTIONARY': cls.__constants['DICTIONARY'],
            'RUN_LENGTH': cls.__constants['RUN_LENGTH'],
        }

    @property
    def DEFAULT(self):
        """Message constant 'DEFAULT'."""
        return Metaclass_Metadata.__constants['DEFAULT']

    @property
    def DICTIONARY(self):
        """Message constant 'DICTIONARY'."""
        return Metaclass_Metadata.__constants['DICTIONARY']

    @property
    def RUN_LENGTH(self):
        """Message constant 'RUN_LENGTH'."""
        return Metaclass_Metadata.__constants['RUN_LENGTH']


class Metadata(metaclass=Metaclass_Metadata):
    """
    Message class 'Metadata'.

    Constants:
      DEFAULT
      DICTIONARY
      RUN_LENGTH
    """

    __slots__ = [
        '_version',
        '_compression_method',
        '_creation_date',
    ]

    _fields_and_field_types = {
        'version': 'int8',
        'compression_method': 'int8',
        'creation_date': 'int16',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.version = kwargs.get('version', int())
        self.compression_method = kwargs.get('compression_method', int())
        self.creation_date = kwargs.get('creation_date', int())

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
        if self.version != other.version:
            return False
        if self.compression_method != other.compression_method:
            return False
        if self.creation_date != other.creation_date:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def version(self):
        """Message field 'version'."""
        return self._version

    @version.setter
    def version(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'version' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'version' field must be an integer in [-128, 127]"
        self._version = value

    @builtins.property
    def compression_method(self):
        """Message field 'compression_method'."""
        return self._compression_method

    @compression_method.setter
    def compression_method(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'compression_method' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'compression_method' field must be an integer in [-128, 127]"
        self._compression_method = value

    @builtins.property
    def creation_date(self):
        """Message field 'creation_date'."""
        return self._creation_date

    @creation_date.setter
    def creation_date(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'creation_date' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'creation_date' field must be an integer in [-32768, 32767]"
        self._creation_date = value
