"""
Двустороннее общение по Serial Master - Slave
"""
from enum import Enum
from struct import Struct
from time import sleep
from typing import Final

from serial import Serial
import struct

class Primitives(Enum):
    """Примитивные типы"""

    i8 = Struct("b")
    """int8_t"""
    u8 = Struct("B")
    """uint8_t"""
    i16 = Struct("h")
    """int16_t"""
    u16 = Struct("H")
    """uint16_t"""
    i32 = Struct("l")
    """int32_t"""
    u32 = Struct("L")
    """uint32_t"""
    i64 = Struct("q")
    """int64_t"""
    I64 = Struct("Q")
    """unt64_t"""
    f32 = Struct("f")
    """float"""
    f64 = Struct("d")  # ! Не поддерживается на Arduino
    """double"""

    def pack(self, value: bool | int | float) -> bytes:
        return self.value.pack(value)

    def unpack(self, buffer: bytes) -> bool | int | float:
        return self.value.unpack(buffer)[0]

class Command:
    """
    Команда по порту

    Имеет свой код (Должен совпадать на slave устройстве)
    Сигнатуру аргументов (Должна совпадать на устройстве)
    """

    def __init__(self, code: int, signature: tuple[Primitives, ...]) -> None:
        self.header: Final[bytes] = Primitives.u8.pack(code)
        self.signature = signature

        

    def pack(self, *args) -> bytes:
        """
        Скомпилировать команду в набор байт
        :param args: аргументы команды. (Их столько же, и такого же типа, что и сигнатура команды)
        :return:
        """
        return self.header + b"".join(primitive.pack(arg) for primitive, arg in zip(self.signature, args))


class ArduinoConnection:
    """Пример подключения к Arduino с dataминимальным набором команд"""

    def __init__(self, serial: Serial) -> None:
        self._serial = serial
        # Команды этого устройства
        self._send_state = Command(0x10, (Primitives.i8,))

    # Обёртки над командами ниже, чтобы сразу компилировать и отправлять их в порт
    def send_state(self, state: int) -> None:
        self._serial.write(self._send_state.pack(state))
    
    def close(self):
        self._serial.close()

if __name__ == '__main__':
    port_name = "/dev/ttyACM0"
    arduino = ArduinoConnection(Serial(port_name, 115200))

    while True:
        arduino.send_state(int(input("Состояние:")))

    arduino.close()
