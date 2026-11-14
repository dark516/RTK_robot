from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.node import Node
import serial


class ConfigureMotorBoard(Node):
    """One-time flash configuration. Ordinary bringup never changes board PID."""

    def __init__(self) -> None:
        super().__init__('configure_motor_board')
        self.declare_parameter('serial_device', '/tmp/rover_devices/motor_controller')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('motor_type', 1)
        self.declare_parameter('encoder_lines', 11)
        self.declare_parameter('reduction_ratio', 45)
        self.declare_parameter('wheel_diameter_mm', 60.0)
        self.declare_parameter('deadzone_pwm', 1600)
        self.declare_parameter('configure_pid', False)
        self.declare_parameter('pid_p', 0.8)
        self.declare_parameter('pid_i', 0.06)
        self.declare_parameter('pid_d', 0.5)

    @staticmethod
    def _read(port: serial.Serial, seconds: float) -> str:
        deadline = time.monotonic() + seconds
        data = bytearray()
        while time.monotonic() < deadline:
            chunk = port.read(256)
            if chunk:
                data.extend(chunk)
        return data.decode('ascii', errors='replace').strip()

    def _send(self, port: serial.Serial, command: str, wait: float = 0.45) -> None:
        port.reset_input_buffer()
        self.get_logger().info(f'Sending {command}')
        port.write(command.encode('ascii'))
        port.flush()
        answer = self._read(port, wait)
        if answer:
            self.get_logger().info(f'Board response: {answer}')

    def run(self) -> None:
        device = str(self.get_parameter('serial_device').value)
        motor_type = int(self.get_parameter('motor_type').value)
        lines = int(self.get_parameter('encoder_lines').value)
        ratio = int(self.get_parameter('reduction_ratio').value)
        diameter = float(self.get_parameter('wheel_diameter_mm').value)
        deadzone = int(self.get_parameter('deadzone_pwm').value)
        configure_pid = bool(self.get_parameter('configure_pid').value)

        if motor_type not in (1, 2, 3, 4):
            raise ValueError('motor_type must be 1..4')
        if lines <= 0 or ratio <= 0 or diameter <= 0.0:
            raise ValueError('encoder and wheel scale values must be positive')
        if not 0 <= deadzone <= 3600:
            raise ValueError('deadzone_pwm must be in 0..3600')

        self.get_logger().warning(
            'Writing motor-board flash. Stop the normal base driver before continuing.'
        )
        with serial.Serial(
            device, baudrate=int(self.get_parameter('baudrate').value),
            timeout=0.05, write_timeout=1.0,
        ) as port:
            time.sleep(0.5)
            port.write(b'$upload:0,0,0#$pwm:0,0,0,0#')
            port.flush()
            time.sleep(0.2)
            for command in (
                f'$mtype:{motor_type}#',
                f'$mline:{lines}#',
                f'$mphase:{ratio}#',
                f'$wdiameter:{diameter:.3f}#',
                f'$deadzone:{deadzone}#',
            ):
                self._send(port, command)
            self._send(port, '$read_flash#', 0.8)
            if configure_pid:
                p = float(self.get_parameter('pid_p').value)
                i = float(self.get_parameter('pid_i').value)
                d = float(self.get_parameter('pid_d').value)
                self._send(port, f'$MPID:{p:.4f},{i:.4f},{d:.4f}#', 1.5)

        self.get_logger().info('Configuration finished. Power-cycle the motor board.')


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ConfigureMotorBoard()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
