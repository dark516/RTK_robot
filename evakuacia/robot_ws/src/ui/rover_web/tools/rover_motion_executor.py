#!/usr/bin/env python3
"""Closed-loop motion primitives for a mecanum rover using ROS 2 odometry.

Commands are completed from /odom feedback, not by assuming that a fixed
number of seconds corresponds to a fixed distance or angle.
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
import yaml


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_odometry(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class MotionDefaults:
    linear_speed: float = 0.18
    approach_speed: float = 0.10
    position_tolerance: float = 0.07
    angular_speed: float = 0.32
    minimum_angular_speed: float = 0.10
    angle_tolerance_deg: float = 3.0
    turn_braking_decel_radps2: float = 0.60
    turn_brake_margin_deg: float = 1.0
    turn_correction_speed: float = 0.14
    turn_settle_rate_radps: float = 0.03
    turn_settle_time: float = 0.40
    turn_max_corrections: int = 3

    # Linear stopping and settled-position verification.
    move_braking_decel_mps2: float = 0.22
    move_brake_margin_m: float = 0.010
    move_correction_speed: float = 0.10
    move_correction_approach_speed: float = 0.10
    move_settle_speed_mps: float = 0.015
    move_settle_rate_radps: float = 0.03
    move_settle_time: float = 0.50
    move_max_corrections: int = 1

    odom_timeout: float = 0.50
    command_rate_hz: float = 50.0
    maximum_step_time: float = 45.0


class MotionController(Node):
    def __init__(
        self,
        odom_topic: str,
        cmd_vel_topic: str,
        defaults: MotionDefaults,
    ) -> None:
        super().__init__("rover_motion_executor")

        self.defaults = defaults
        self.publisher = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10,
        )
        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            20,
        )

        self.latest_message: Odometry | None = None
        self.latest_odom_monotonic = 0.0

        self._last_raw_yaw: float | None = None
        self._unwrapped_yaw = 0.0
        self.origin: Pose2D | None = None

    def _odom_callback(self, message: Odometry) -> None:
        raw_yaw = yaw_from_odometry(message)

        if self._last_raw_yaw is None:
            self._unwrapped_yaw = raw_yaw
        else:
            delta = normalize_angle(raw_yaw - self._last_raw_yaw)
            self._unwrapped_yaw += delta

        self._last_raw_yaw = raw_yaw
        self.latest_message = message
        self.latest_odom_monotonic = time.monotonic()

    def wait_until_ready(self, timeout: float = 10.0) -> None:
        print("Ожидание одометрии...")
        deadline = time.monotonic() + timeout

        while self.latest_message is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "Одометрия не появилась за 10 секунд"
                )

        print("Ожидание подписчика /cmd_vel...")
        deadline = time.monotonic() + timeout

        while self.publisher.get_subscription_count() < 1:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "Подписчик команд скорости не найден"
                )

        self.origin = self.current_pose()
        print(
            "Начальная точка: "
            f"x={self.origin.x:+.3f} м, "
            f"y={self.origin.y:+.3f} м, "
            f"yaw={math.degrees(self.origin.yaw):+.2f}°"
        )

    def current_pose(self) -> Pose2D:
        if self.latest_message is None:
            raise RuntimeError("Одометрия ещё не получена")

        position = self.latest_message.pose.pose.position
        return Pose2D(
            x=position.x,
            y=position.y,
            yaw=self._unwrapped_yaw,
        )

    def current_angular_velocity(self) -> float:
        if self.latest_message is None:
            raise RuntimeError("Одометрия ещё не получена")
        return float(
            self.latest_message.twist.twist.angular.z
        )

    def current_body_velocity(self) -> tuple[float, float]:
        if self.latest_message is None:
            raise RuntimeError("Одометрия ещё не получена")
        velocity = self.latest_message.twist.twist.linear
        return float(velocity.x), float(velocity.y)

    def current_linear_speed(self) -> float:
        velocity_x, velocity_y = self.current_body_velocity()
        return math.hypot(velocity_x, velocity_y)

    def _check_odom_freshness(self) -> None:
        age = time.monotonic() - self.latest_odom_monotonic
        if age > self.defaults.odom_timeout:
            raise RuntimeError(
                f"Одометрия не обновляется {age:.2f} с"
            )

    def publish_stop(self, duration: float = 0.8) -> None:
        stop = Twist()
        period = 1.0 / self.defaults.command_rate_hz
        deadline = time.monotonic() + duration

        while time.monotonic() < deadline and rclpy.ok():
            self.publisher.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def _position_error(
        self,
        target: Pose2D,
    ) -> tuple[float, float, float]:
        pose = self.current_pose()
        error_x = target.x - pose.x
        error_y = target.y - pose.y
        return error_x, error_y, math.hypot(error_x, error_y)

    def _settle_after_move(
        self,
        target: Pose2D,
        *,
        timeout: float = 5.0,
    ) -> tuple[float, float, float, float]:
        """Command zero and wait until odometry reports a real stop."""
        period = 1.0 / self.defaults.command_rate_hz
        deadline = time.monotonic() + timeout
        stable_since: float | None = None
        last_report = 0.0

        while rclpy.ok():
            self.publisher.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.0)
            self._check_odom_freshness()

            pose = self.current_pose()
            _, _, distance = self._position_error(target)
            yaw_error = target.yaw - pose.yaw
            linear_speed = self.current_linear_speed()
            angular_rate = self.current_angular_velocity()
            now = time.monotonic()

            stopped = (
                linear_speed
                <= self.defaults.move_settle_speed_mps
                and abs(angular_rate)
                <= self.defaults.move_settle_rate_radps
            )

            if stopped:
                if stable_since is None:
                    stable_since = now
                elif (
                    now - stable_since
                    >= self.defaults.move_settle_time
                ):
                    return (
                        distance,
                        yaw_error,
                        linear_speed,
                        angular_rate,
                    )
            else:
                stable_since = None

            if now - last_report >= 0.20:
                print(
                    f"\rТорможение перемещения: "
                    f"ошибка {distance:.3f} м, "
                    f"курс {math.degrees(yaw_error):+.1f}°, "
                    f"v={linear_speed:.3f} м/с, "
                    f"ω={angular_rate:+.3f}",
                    end="",
                    flush=True,
                )
                last_report = now

            if now >= deadline:
                return (
                    distance,
                    yaw_error,
                    linear_speed,
                    angular_rate,
                )

            time.sleep(period)

        raise RuntimeError("ROS был остановлен")

    def move_to_pose(
        self,
        target: Pose2D,
        *,
        linear_speed: float | None = None,
        approach_speed: float | None = None,
        position_tolerance: float | None = None,
        maximum_time: float | None = None,
        label: str = "перемещение",
    ) -> None:
        requested_max_speed = (
            self.defaults.linear_speed
            if linear_speed is None
            else float(linear_speed)
        )
        requested_min_speed = (
            self.defaults.approach_speed
            if approach_speed is None
            else float(approach_speed)
        )
        tolerance = (
            self.defaults.position_tolerance
            if position_tolerance is None
            else float(position_tolerance)
        )
        time_limit = (
            self.defaults.maximum_step_time
            if maximum_time is None
            else float(maximum_time)
        )

        if not 0.0 < requested_min_speed <= requested_max_speed:
            raise ValueError(
                "approach_speed должен быть > 0 и <= linear_speed"
            )
        if tolerance <= 0.0:
            raise ValueError("position_tolerance должен быть > 0")
        if self.defaults.move_braking_decel_mps2 <= 0.0:
            raise ValueError(
                "move_braking_decel_mps2 должен быть > 0"
            )

        total_start_time = time.monotonic()
        period = 1.0 / self.defaults.command_rate_hz

        print(
            f"{label}: цель x={target.x:+.3f}, "
            f"y={target.y:+.3f}, "
            f"yaw={math.degrees(target.yaw):+.2f}°"
        )

        for correction_index in range(
            self.defaults.move_max_corrections + 1
        ):
            pose = self.current_pose()
            _, _, initial_distance = self._position_error(target)

            if correction_index == 0:
                pass_max_speed = requested_max_speed
                pass_min_speed = requested_min_speed
                phase_name = label
            else:
                pass_max_speed = min(
                    requested_max_speed,
                    self.defaults.move_correction_speed,
                )
                pass_min_speed = min(
                    pass_max_speed,
                    self.defaults.move_correction_approach_speed,
                )
                phase_name = (
                    f"{label}: коррекция позиции "
                    f"{correction_index}/"
                    f"{self.defaults.move_max_corrections}"
                )
                print(
                    f"\n{phase_name}, остаток "
                    f"{initial_distance:.3f} м"
                )

            best_distance = initial_distance
            last_improvement_time = time.monotonic()
            last_report = 0.0

            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                self._check_odom_freshness()

                pose = self.current_pose()
                error_world_x = target.x - pose.x
                error_world_y = target.y - pose.y
                distance = math.hypot(
                    error_world_x,
                    error_world_y,
                )
                yaw_error = target.yaw - pose.yaw

                cosine = math.cos(pose.yaw)
                sine = math.sin(pose.yaw)
                error_body_x = (
                    cosine * error_world_x
                    + sine * error_world_y
                )
                error_body_y = (
                    -sine * error_world_x
                    + cosine * error_world_y
                )

                velocity_x, velocity_y = (
                    self.current_body_velocity()
                )
                linear_speed_now = math.hypot(
                    velocity_x,
                    velocity_y,
                )

                if distance > 1e-9:
                    direction_body_x = error_body_x / distance
                    direction_body_y = error_body_y / distance
                    speed_toward_target = (
                        velocity_x * direction_body_x
                        + velocity_y * direction_body_y
                    )
                else:
                    direction_body_x = 0.0
                    direction_body_y = 0.0
                    speed_toward_target = 0.0

                stopping_distance = (
                    max(0.0, speed_toward_target) ** 2
                    / (
                        2.0
                        * self.defaults.move_braking_decel_mps2
                    )
                )
                braking_threshold = (
                    stopping_distance
                    + self.defaults.move_brake_margin_m
                )

                # Stop before the target based on measured odometry
                # velocity, then assess the final settled pose.
                if (
                    distance <= tolerance
                    or (
                        speed_toward_target > 0.0
                        and distance <= braking_threshold
                    )
                ):
                    self.publisher.publish(Twist())
                    break

                speed_command = clamp(
                    1.0 * distance,
                    pass_min_speed,
                    pass_max_speed,
                )
                command_x = (
                    speed_command * direction_body_x
                )
                command_y = (
                    speed_command * direction_body_y
                )
                angular_command = clamp(
                    1.5 * yaw_error,
                    -self.defaults.angular_speed,
                    self.defaults.angular_speed,
                )

                command = Twist()
                command.linear.x = command_x
                command.linear.y = command_y
                command.angular.z = angular_command
                self.publisher.publish(command)

                now = time.monotonic()

                if distance < best_distance - 0.003:
                    best_distance = distance
                    last_improvement_time = now

                if now - last_improvement_time > 3.0:
                    raise RuntimeError(
                        f"{phase_name}: расстояние до цели "
                        "не уменьшается"
                    )

                if now - total_start_time > time_limit:
                    raise RuntimeError(
                        f"{label}: превышен лимит "
                        f"{time_limit:.1f} с"
                    )

                if now - last_report >= 0.20:
                    print(
                        f"\r{phase_name}: осталось "
                        f"{distance:.3f} м, "
                        f"курс "
                        f"{math.degrees(yaw_error):+.1f}°, "
                        f"v={linear_speed_now:.3f}, "
                        f"стоп≈{stopping_distance:.3f} м, "
                        f"cmd x={command_x:+.2f}, "
                        f"y={command_y:+.2f}",
                        end="",
                        flush=True,
                    )
                    last_report = now

                time.sleep(period)

            (
                settled_distance,
                settled_yaw_error,
                settled_speed,
                settled_rate,
            ) = self._settle_after_move(target)
            print(
                f"\nПосле полной остановки: "
                f"ошибка позиции {settled_distance:.3f} м, "
                f"курс "
                f"{math.degrees(settled_yaw_error):+.2f}°, "
                f"v={settled_speed:.3f} м/с, "
                f"ω={settled_rate:+.3f}"
            )

            # Correct heading separately when position is already good.
            if (
                settled_distance <= tolerance
                and abs(settled_yaw_error)
                > math.radians(
                    self.defaults.angle_tolerance_deg
                )
            ):
                remaining_time = max(
                    3.0,
                    time_limit
                    - (time.monotonic() - total_start_time),
                )
                self.turn_relative(
                    degrees=math.degrees(settled_yaw_error),
                    angular_speed=min(
                        self.defaults.angular_speed,
                        self.defaults.turn_correction_speed,
                    ),
                    minimum_angular_speed=min(
                        self.defaults.minimum_angular_speed,
                        self.defaults.turn_correction_speed,
                    ),
                    angle_tolerance_deg=(
                        self.defaults.angle_tolerance_deg
                    ),
                    maximum_time=remaining_time,
                )
                (
                    settled_distance,
                    settled_yaw_error,
                    settled_speed,
                    settled_rate,
                ) = self._settle_after_move(target)
                print(
                    "После коррекции курса: "
                    f"ошибка позиции "
                    f"{settled_distance:.3f} м, "
                    f"курс "
                    f"{math.degrees(settled_yaw_error):+.2f}°"
                )

            if (
                settled_distance <= tolerance
                and abs(settled_yaw_error)
                <= math.radians(
                    self.defaults.angle_tolerance_deg
                )
                and settled_speed
                <= self.defaults.move_settle_speed_mps
                and abs(settled_rate)
                <= self.defaults.move_settle_rate_radps
            ):
                print(
                    f"{label} завершено после полной остановки:\n"
                    f"  итоговая ошибка позиции: "
                    f"{settled_distance:.3f} м\n"
                    f"  итоговая ошибка курса: "
                    f"{math.degrees(settled_yaw_error):+.2f}°\n"
                    f"  итоговая скорость: "
                    f"{settled_speed:.3f} м/с"
                )
                self.publish_stop(0.5)
                return

        _, _, final_distance = self._position_error(target)
        final_yaw_error = target.yaw - self.current_pose().yaw
        raise RuntimeError(
            "Не удалось стабилизировать позицию после "
            f"{self.defaults.move_max_corrections} коррекций; "
            f"ошибка позиции {final_distance:.3f} м, "
            f"ошибка курса "
            f"{math.degrees(final_yaw_error):+.2f}°"
        )

    def move_relative(
        self,
        forward: float,
        left: float,
        **kwargs: Any,
    ) -> None:
        start = self.current_pose()
        cosine = math.cos(start.yaw)
        sine = math.sin(start.yaw)

        target = Pose2D(
            x=start.x + cosine * forward - sine * left,
            y=start.y + sine * forward + cosine * left,
            yaw=start.yaw,
        )

        self.move_to_pose(
            target,
            label=(
                f"движение forward={forward:+.3f} м, "
                f"left={left:+.3f} м"
            ),
            **kwargs,
        )

    def _settle_after_turn(
        self,
        target_yaw: float,
        tolerance_rad: float,
        *,
        timeout: float = 4.0,
    ) -> tuple[float, float]:
        """Command zero until the rover and EKF have actually settled."""
        period = 1.0 / self.defaults.command_rate_hz
        deadline = time.monotonic() + timeout
        stable_since: float | None = None
        last_report = 0.0

        while rclpy.ok():
            self.publisher.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.0)
            self._check_odom_freshness()

            pose = self.current_pose()
            angular_velocity = self.current_angular_velocity()
            error = target_yaw - pose.yaw
            now = time.monotonic()

            if (
                abs(angular_velocity)
                <= self.defaults.turn_settle_rate_radps
            ):
                if stable_since is None:
                    stable_since = now
                elif (
                    now - stable_since
                    >= self.defaults.turn_settle_time
                ):
                    return error, angular_velocity
            else:
                stable_since = None

            if now - last_report >= 0.20:
                print(
                    f"\rТорможение: ошибка "
                    f"{math.degrees(error):+.2f}°, "
                    f"ω={angular_velocity:+.3f} рад/с",
                    end="",
                    flush=True,
                )
                last_report = now

            if now >= deadline:
                return error, angular_velocity

            time.sleep(period)

        raise RuntimeError("ROS был остановлен")

    def turn_relative(
        self,
        degrees: float,
        *,
        angular_speed: float | None = None,
        minimum_angular_speed: float | None = None,
        angle_tolerance_deg: float | None = None,
        maximum_time: float | None = None,
    ) -> None:
        requested_max_speed = (
            self.defaults.angular_speed
            if angular_speed is None
            else abs(float(angular_speed))
        )
        requested_min_speed = (
            self.defaults.minimum_angular_speed
            if minimum_angular_speed is None
            else abs(float(minimum_angular_speed))
        )
        tolerance_deg = (
            self.defaults.angle_tolerance_deg
            if angle_tolerance_deg is None
            else abs(float(angle_tolerance_deg))
        )
        time_limit = (
            self.defaults.maximum_step_time
            if maximum_time is None
            else float(maximum_time)
        )

        if not 0.0 < requested_min_speed <= requested_max_speed:
            raise ValueError(
                "minimum_angular_speed должен быть > 0 "
                "и <= angular_speed"
            )

        start = self.current_pose()
        target_yaw = start.yaw + math.radians(degrees)
        tolerance_rad = math.radians(tolerance_deg)
        total_start_time = time.monotonic()
        period = 1.0 / self.defaults.command_rate_hz

        print(
            f"Поворот на {degrees:+.2f}°: "
            f"цель yaw={math.degrees(target_yaw):+.2f}°"
        )

        for correction_index in range(
            self.defaults.turn_max_corrections + 1
        ):
            pose = self.current_pose()
            initial_error = target_yaw - pose.yaw

            if abs(initial_error) <= tolerance_rad:
                settled_error, settled_rate = self._settle_after_turn(
                    target_yaw,
                    tolerance_rad,
                )
                print()
                if (
                    abs(settled_error) <= tolerance_rad
                    and abs(settled_rate)
                    <= self.defaults.turn_settle_rate_radps
                ):
                    actual_turn = pose.yaw - start.yaw
                    print(
                        "Поворот завершён после полной остановки:\n"
                        f"  итоговая ошибка: "
                        f"{math.degrees(settled_error):+.2f}°\n"
                        f"  фактический угол по /odom: "
                        f"{math.degrees(actual_turn):+.2f}°\n"
                        f"  итоговая ω: "
                        f"{settled_rate:+.3f} рад/с"
                    )
                    return

            if correction_index == 0:
                pass_max_speed = requested_max_speed
                pass_min_speed = requested_min_speed
                phase_name = "основной поворот"
            else:
                pass_max_speed = min(
                    requested_max_speed,
                    self.defaults.turn_correction_speed,
                )
                pass_min_speed = min(
                    requested_min_speed,
                    pass_max_speed,
                )
                phase_name = (
                    f"коррекция {correction_index}/"
                    f"{self.defaults.turn_max_corrections}"
                )
                print(
                    f"\n{phase_name}: остаточная ошибка "
                    f"{math.degrees(initial_error):+.2f}°"
                )

            best_error = abs(initial_error)
            last_improvement_time = time.monotonic()
            last_report = 0.0

            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                self._check_odom_freshness()

                pose = self.current_pose()
                error = target_yaw - pose.yaw
                absolute_error = abs(error)
                angular_velocity = self.current_angular_velocity()

                # Estimate how much angle is still required to stop.
                stopping_angle = (
                    angular_velocity * angular_velocity
                    / (
                        2.0
                        * self.defaults.turn_braking_decel_radps2
                    )
                )
                braking_threshold = (
                    stopping_angle
                    + math.radians(
                        self.defaults.turn_brake_margin_deg
                    )
                )

                moving_toward_target = (
                    angular_velocity == 0.0
                    or math.copysign(1.0, angular_velocity)
                    == math.copysign(1.0, error)
                )

                # Stop before the target, then observe the actual coast-down.
                if (
                    absolute_error <= tolerance_rad
                    or (
                        moving_toward_target
                        and absolute_error <= braking_threshold
                    )
                    or (
                        angular_velocity != 0.0
                        and math.copysign(1.0, error)
                        != math.copysign(1.0, initial_error)
                    )
                ):
                    self.publisher.publish(Twist())
                    break

                speed = clamp(
                    1.6 * absolute_error,
                    pass_min_speed,
                    pass_max_speed,
                )

                command = Twist()
                command.angular.z = math.copysign(speed, error)
                self.publisher.publish(command)

                now = time.monotonic()
                if absolute_error < best_error - math.radians(0.25):
                    best_error = absolute_error
                    last_improvement_time = now

                if now - last_improvement_time > 3.0:
                    raise RuntimeError(
                        f"{phase_name}: ошибка угла не уменьшается"
                    )

                if now - total_start_time > time_limit:
                    raise RuntimeError(
                        f"Поворот: превышен лимит "
                        f"{time_limit:.1f} с"
                    )

                if now - last_report >= 0.15:
                    print(
                        f"\r{phase_name}: осталось "
                        f"{math.degrees(error):+.2f}°, "
                        f"ω={angular_velocity:+.3f}, "
                        f"стоп≈"
                        f"{math.degrees(stopping_angle):.1f}°",
                        end="",
                        flush=True,
                    )
                    last_report = now

                time.sleep(period)

            settled_error, settled_rate = self._settle_after_turn(
                target_yaw,
                tolerance_rad,
            )
            print(
                f"\nПосле торможения: ошибка "
                f"{math.degrees(settled_error):+.2f}°, "
                f"ω={settled_rate:+.3f} рад/с"
            )

            if (
                abs(settled_error) <= tolerance_rad
                and abs(settled_rate)
                <= self.defaults.turn_settle_rate_radps
            ):
                final_pose = self.current_pose()
                actual_turn = final_pose.yaw - start.yaw
                print(
                    "Поворот завершён после полной остановки:\n"
                    f"  итоговая ошибка: "
                    f"{math.degrees(settled_error):+.2f}°\n"
                    f"  фактический угол по /odom: "
                    f"{math.degrees(actual_turn):+.2f}°\n"
                    f"  итоговая ω: "
                    f"{settled_rate:+.3f} рад/с"
                )
                self.publish_stop(0.5)
                return

        final_error = target_yaw - self.current_pose().yaw
        raise RuntimeError(
            "Не удалось стабилизировать поворот после "
            f"{self.defaults.turn_max_corrections} коррекций; "
            f"остаточная ошибка "
            f"{math.degrees(final_error):+.2f}°"
        )

    def return_to_origin(
        self,
        *,
        restore_heading: bool = True,
        **kwargs: Any,
    ) -> None:
        if self.origin is None:
            raise RuntimeError("Начальная точка не сохранена")

        current = self.current_pose()
        target = Pose2D(
            x=self.origin.x,
            y=self.origin.y,
            yaw=(
                self.origin.yaw
                if restore_heading
                else current.yaw
            ),
        )
        self.move_to_pose(
            target,
            label="возврат в начальную точку",
            **kwargs,
        )

    def print_origin_error(self) -> None:
        if self.origin is None:
            return

        pose = self.current_pose()
        dx = pose.x - self.origin.x
        dy = pose.y - self.origin.y
        distance = math.hypot(dx, dy)
        yaw_error = pose.yaw - self.origin.yaw

        print(
            "\nИтоговая ошибка возврата по одометрии:\n"
            f"  dx: {dx:+.3f} м\n"
            f"  dy: {dy:+.3f} м\n"
            f"  расстояние: {distance:.3f} м\n"
            f"  yaw: {math.degrees(yaw_error):+.2f}°"
        )


def load_defaults(plan: dict[str, Any]) -> MotionDefaults:
    values = plan.get("defaults", {})
    return MotionDefaults(
        linear_speed=float(values.get("linear_speed", 0.18)),
        approach_speed=float(values.get("approach_speed", 0.10)),
        position_tolerance=float(
            values.get("position_tolerance", 0.07)
        ),
        angular_speed=float(values.get("angular_speed", 0.45)),
        minimum_angular_speed=float(
            values.get("minimum_angular_speed", 0.12)
        ),
        angle_tolerance_deg=float(
            values.get("angle_tolerance_deg", 3.0)
        ),
        turn_braking_decel_radps2=float(
            values.get("turn_braking_decel_radps2", 0.60)
        ),
        turn_brake_margin_deg=float(
            values.get("turn_brake_margin_deg", 1.0)
        ),
        turn_correction_speed=float(
            values.get("turn_correction_speed", 0.14)
        ),
        turn_settle_rate_radps=float(
            values.get("turn_settle_rate_radps", 0.03)
        ),
        turn_settle_time=float(
            values.get("turn_settle_time", 0.40)
        ),
        turn_max_corrections=int(
            values.get("turn_max_corrections", 3)
        ),
        move_braking_decel_mps2=float(
            values.get("move_braking_decel_mps2", 0.22)
        ),
        move_brake_margin_m=float(
            values.get("move_brake_margin_m", 0.010)
        ),
        move_correction_speed=float(
            values.get("move_correction_speed", 0.10)
        ),
        move_correction_approach_speed=float(
            values.get(
                "move_correction_approach_speed",
                0.10,
            )
        ),
        move_settle_speed_mps=float(
            values.get("move_settle_speed_mps", 0.015)
        ),
        move_settle_rate_radps=float(
            values.get("move_settle_rate_radps", 0.03)
        ),
        move_settle_time=float(
            values.get("move_settle_time", 0.50)
        ),
        move_max_corrections=int(
            values.get("move_max_corrections", 1)
        ),
        odom_timeout=float(values.get("odom_timeout", 0.50)),
        command_rate_hz=float(
            values.get("command_rate_hz", 50.0)
        ),
        maximum_step_time=float(
            values.get("maximum_step_time", 45.0)
        ),
    )


def execute_plan(
    controller: MotionController,
    plan: dict[str, Any],
) -> None:
    steps = plan.get("steps")
    if not isinstance(steps, list) or not steps:
        raise ValueError("В плане отсутствует непустой список steps")

    for index, step in enumerate(steps, start=1):
        if not isinstance(step, dict):
            raise ValueError(f"Шаг {index} должен быть объектом")

        step_type = str(step.get("type", "")).strip()
        print(f"\n--- Шаг {index}/{len(steps)}: {step_type} ---")

        common: dict[str, Any] = {}
        if "linear_speed" in step:
            common["linear_speed"] = float(step["linear_speed"])
        if "approach_speed" in step:
            common["approach_speed"] = float(
                step["approach_speed"]
            )
        if "position_tolerance" in step:
            common["position_tolerance"] = float(
                step["position_tolerance"]
            )
        if "maximum_time" in step:
            common["maximum_time"] = float(step["maximum_time"])

        if step_type == "move":
            controller.move_relative(
                forward=float(step.get("forward", 0.0)),
                left=float(step.get("left", 0.0)),
                **common,
            )

        elif step_type == "move_polar":
            distance = float(step["distance"])
            direction = math.radians(
                float(step.get("direction_deg", 0.0))
            )
            controller.move_relative(
                forward=distance * math.cos(direction),
                left=distance * math.sin(direction),
                **common,
            )

        elif step_type == "turn":
            controller.turn_relative(
                degrees=float(step["degrees"]),
                angular_speed=(
                    float(step["angular_speed"])
                    if "angular_speed" in step
                    else None
                ),
                minimum_angular_speed=(
                    float(step["minimum_angular_speed"])
                    if "minimum_angular_speed" in step
                    else None
                ),
                angle_tolerance_deg=(
                    float(step["angle_tolerance_deg"])
                    if "angle_tolerance_deg" in step
                    else None
                ),
                maximum_time=(
                    float(step["maximum_time"])
                    if "maximum_time" in step
                    else None
                ),
            )

        elif step_type == "return_to_start":
            controller.return_to_origin(
                restore_heading=bool(
                    step.get("restore_heading", True)
                ),
                **common,
            )

        elif step_type == "pause":
            duration = float(step.get("seconds", 1.0))
            print(f"Пауза {duration:.2f} с")
            controller.publish_stop(duration)

        else:
            raise ValueError(
                f"Неизвестный тип шага {step_type!r}"
            )


def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Замкнутое управление перемещением и поворотом "
            "ровера по одометрии"
        )
    )
    parser.add_argument(
        "--odom-topic",
        default="/odom",
        help="источник одометрии, по умолчанию /odom",
    )
    parser.add_argument(
        "--cmd-vel-topic",
        default="/cmd_vel",
        help="топик команд скорости",
    )

    subparsers = parser.add_subparsers(
        dest="command",
        required=True,
    )

    move = subparsers.add_parser(
        "move",
        help="переместиться относительно текущей ориентации",
    )
    move.add_argument("--forward", type=float, default=0.0)
    move.add_argument("--left", type=float, default=0.0)
    move.add_argument("--speed", type=float, default=0.18)

    turn = subparsers.add_parser(
        "turn",
        help="повернуться на заданное число градусов",
    )
    turn.add_argument("degrees", type=float)
    turn.add_argument("--speed", type=float, default=0.32)
    turn.add_argument(
        "--tolerance-deg",
        type=float,
        default=3.0,
    )

    run = subparsers.add_parser(
        "run",
        help="выполнить последовательность из YAML",
    )
    run.add_argument("plan", type=Path)

    return parser


def main() -> None:
    args = create_parser().parse_args()

    if args.command == "run":
        plan = yaml.safe_load(
            args.plan.read_text(encoding="utf-8")
        )
        if not isinstance(plan, dict):
            raise SystemExit("YAML-план должен содержать объект")
        defaults = load_defaults(plan)
    else:
        plan = None
        defaults = MotionDefaults()

    rclpy.init()
    controller = MotionController(
        odom_topic=args.odom_topic,
        cmd_vel_topic=args.cmd_vel_topic,
        defaults=defaults,
    )

    try:
        controller.wait_until_ready()
        controller.publish_stop(0.5)

        if args.command == "move":
            controller.move_relative(
                forward=args.forward,
                left=args.left,
                linear_speed=args.speed,
            )

        elif args.command == "turn":
            controller.turn_relative(
                degrees=args.degrees,
                angular_speed=args.speed,
                angle_tolerance_deg=args.tolerance_deg,
            )

        elif args.command == "run":
            execute_plan(controller, plan)
            controller.print_origin_error()

    except KeyboardInterrupt:
        print("\nОстановлено пользователем")

    except Exception as error:
        print(f"\nАварийная остановка: {error}")

    finally:
        print("\nОстановка моторов...")
        controller.publish_stop(1.0)
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
