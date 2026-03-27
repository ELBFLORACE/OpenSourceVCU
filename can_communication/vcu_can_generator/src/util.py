import re
from typing import (
    TYPE_CHECKING,
    Dict,
    Iterator,
    Optional,
    Tuple,
    TypeVar,
    Union,
    cast,
)

__version__ = "1.0.0"

if TYPE_CHECKING:
    from cantools.database.can import Message, Signal

_T1 = TypeVar("_T1")
_T2 = TypeVar("_T2")


class CodeGenSignal:
    def __init__(self, signal: "Signal") -> None:
        self.signal: "Signal" = signal
        self.snake_name = camel_to_snake_case(signal.name)

    @property
    def unit(self) -> str:
        return _get(self.signal.unit, '-')

    @property
    def type_length(self) -> int:
        if self.signal.length <= 8:
            return 8
        elif self.signal.length <= 16:
            return 16
        elif self.signal.length <= 32:
            return 32
        else:
            return 64

    @property
    def type_name(self) -> str:
        if self.signal.conversion.is_float:
            if self.signal.length == 32:
                type_name = 'float'
            else:
                type_name = 'double'
        else:
            type_name = f'int{self.type_length}_t'

            if not self.signal.is_signed:
                type_name = 'u' + type_name

        return type_name

    @property
    def proto_type_name(self) -> str:
        return (self.ros_type_name
                # Protobuf doesnt support those
                .replace("8", "32")
                .replace("16", "32")
                .replace("float32", "float")
                )

    @property
    def ros_type_name(self) -> str:
        # Make an exception for choice values
        if self.signal.choices:
            return self.type_name.replace("_t", "")
        return 'float32'
        # name = self.proto_type_name
        # return name.replace('uint16', 'float32').replace('int16', 'float32').replace('uint32', 'float32').replace('int32', 'float32')

    @property
    def cpp_ros_type_name(self) -> str:
        return (self.ros_type_name
                .replace("float32", "float")
                )

    @property
    def type_suffix(self) -> str:
        try:
            return {
                'uint8_t': 'u',
                'uint16_t': 'u',
                'uint32_t': 'u',
                'int64_t': 'll',
                'uint64_t': 'ull',
                'float': 'f'
            }[self.type_name]
        except KeyError:
            return ''

    @property
    def conversion_type_suffix(self) -> str:
        try:
            return {
                8: 'u',
                16: 'u',
                32: 'u',
                64: 'ull'
            }[self.type_length]
        except KeyError:
            return ''

    @property
    def unique_choices(self) -> Dict[int, str]:
        """Make duplicated choice names unique by first appending its value
        and then underscores until unique.

        """
        if self.signal.choices is None:
            return {}

        items = {
            value: camel_to_snake_case(str(name)).upper()
            for value, name in self.signal.choices.items()
        }
        names = list(items.values())
        duplicated_names = [
            name
            for name in set(names)
            if names.count(name) > 1
        ]
        unique_choices = {
            value: name
            for value, name in items.items()
            if names.count(name) == 1
        }

        for value, name in items.items():
            if name in duplicated_names:
                name += _canonical(f'_{value}')

                while name in unique_choices.values():
                    name += '_'

                unique_choices[value] = name

        return unique_choices

    @property
    def minimum_ctype_value(self) -> Optional[int]:
        if self.type_name == 'int8_t':
            return -2 ** 7
        elif self.type_name == 'int16_t':
            return -2 ** 15
        elif self.type_name == 'int32_t':
            return -2 ** 31
        elif self.type_name == 'int64_t':
            return -2 ** 63
        elif self.type_name.startswith('u'):
            return 0
        else:
            return None

    @property
    def maximum_ctype_value(self) -> Optional[int]:
        if self.type_name == 'int8_t':
            return 2 ** 7 - 1
        elif self.type_name == 'int16_t':
            return 2 ** 15 - 1
        elif self.type_name == 'int32_t':
            return 2 ** 31 - 1
        elif self.type_name == 'int64_t':
            return 2 ** 63 - 1
        elif self.type_name == 'uint8_t':
            return 2 ** 8 - 1
        elif self.type_name == 'uint16_t':
            return 2 ** 16 - 1
        elif self.type_name == 'uint32_t':
            return 2 ** 32 - 1
        elif self.type_name == 'uint64_t':
            return 2 ** 64 - 1
        else:
            return None

    @property
    def minimum_can_raw_value(self) -> Optional[int]:
        if self.signal.conversion.is_float:
            return None
        elif self.signal.is_signed:
            return cast(int, -(2 ** (self.signal.length - 1)))
        else:
            return 0

    @property
    def maximum_can_raw_value(self) -> Optional[int]:
        if self.signal.conversion.is_float:
            return None
        elif self.signal.is_signed:
            return cast(int, (2 ** (self.signal.length - 1)) - 1)
        else:
            return cast(int, (2 ** self.signal.length) - 1)

    def segments(self, invert_shift: bool) -> Iterator[Tuple[int, int, str, int]]:
        index, pos = divmod(self.signal.start, 8)
        left = self.signal.length

        while left > 0:
            if self.signal.byte_order == 'big_endian':
                if left >= (pos + 1):
                    length = (pos + 1)
                    pos = 7
                    shift = -(left - length)
                    mask = ((1 << length) - 1)
                else:
                    length = left
                    shift = (pos - length + 1)
                    mask = ((1 << length) - 1)
                    mask <<= (pos - length + 1)
            else:
                shift = (left - self.signal.length) + pos

                if left >= (8 - pos):
                    length = (8 - pos)
                    mask = ((1 << length) - 1)
                    mask <<= pos
                    pos = 0
                else:
                    length = left
                    mask = ((1 << length) - 1)
                    mask <<= pos

            if invert_shift:
                if shift < 0:
                    shift = -shift
                    shift_direction = 'left'
                else:
                    shift_direction = 'right'
            else:
                if shift < 0:
                    shift = -shift
                    shift_direction = 'right'
                else:
                    shift_direction = 'left'

            yield index, shift, shift_direction, mask

            left -= length
            index += 1


def _canonical(value: str) -> str:
    """Replace anything but 'a-z', 'A-Z' and '0-9' with '_'.

    """

    return re.sub(r'[^a-zA-Z0-9]', '_', value)


def camel_to_snake_case(value: str) -> str:
    value = re.sub(r'(.)([A-Z][a-z]+)', r'\1_\2', value)
    value = re.sub(r'(_+)', '_', value)
    value = re.sub(r'([a-z0-9])([A-Z])', r'\1_\2', value).lower()
    value = _canonical(value)

    return value


def snake_to_pascal_case(snake_str):
    words = snake_str.split("_")
    pascal_str = "".join([word.capitalize() for word in words])
    return pascal_str


def _get(value: Optional[_T1], default: _T2) -> Union[_T1, _T2]:
    if value is None:
        return default
    return value


class CodeGenMessage:
    def __init__(self, message: "Message") -> None:
        self.message = message

        normal_snake_name = camel_to_snake_case(message.name)
        """
        ROS message generation will apply it's own remappings of names and this tries to replicate that
        If the message name ends with "message_123" this will convert it to "message123"
        """
        self.snake_name =  re.sub(r'(.+)_([0-9]+)', r'\1\2', normal_snake_name)

        self.hex_snake_name = camel_to_snake_case(hex(message.frame_id) + '_' + message.name)
        self.pascal_name = snake_to_pascal_case(camel_to_snake_case(message.name))

        self.cg_signals = [CodeGenSignal(signal) for signal in message.signals]

    def get_signal_by_name(self, name: str) -> "CodeGenSignal":
        for cg_signal in self.cg_signals:
            if cg_signal.signal.name == name:
                return cg_signal
        raise KeyError(f"Signal {name} not found.")
