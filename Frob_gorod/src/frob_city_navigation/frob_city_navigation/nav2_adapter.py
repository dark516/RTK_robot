"""Nav2 FollowPath action boundary for the Frob City navigator.

This module owns action-transport details only.  Mission decisions stay in
mission.py and route legality stays in map_planner.  Every goal carries an
immutable execution token so late results can be rejected by the navigator.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Callable

from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient


@dataclass(frozen=True, slots=True)
class ExecutionToken:
    run_id: str
    map_instance_id: str
    navigator_instance_id: str
    route_id: str
    segment_id: str
    goal_generation: int

    def __post_init__(self) -> None:
        if not all((
            self.run_id,
            self.map_instance_id,
            self.navigator_instance_id,
            self.route_id,
            self.segment_id,
        )):
            raise ValueError('execution token identity must be non-empty')
        if self.goal_generation <= 0:
            raise ValueError('goal_generation must be positive')


class Nav2Completion(IntEnum):
    SUCCEEDED = 0
    ABORTED = 1
    CANCELED = 2
    REJECTED = 3
    ERROR = 4


@dataclass(frozen=True, slots=True)
class Nav2Result:
    token: ExecutionToken
    completion: Nav2Completion
    action_status: int
    error_code: int = 0
    error_msg: str = ''


class Nav2Adapter:
    """Single-active-goal FollowPath client with token-preserving callbacks."""

    def __init__(
        self,
        node,
        *,
        action_name: str = '/follow_path',
        callback_group=None,
    ) -> None:
        self._node = node
        self._client = ActionClient(
            node,
            FollowPath,
            action_name,
            callback_group=callback_group,
        )
        self._active_token: ExecutionToken | None = None
        self._goal_handle = None
        self._cancel_future = None

    @property
    def active_token(self) -> ExecutionToken | None:
        return self._active_token

    @property
    def has_active_goal(self) -> bool:
        return self._active_token is not None

    def server_ready(self) -> bool:
        return bool(self._client.server_is_ready())

    def send_path(
        self,
        *,
        path,
        token: ExecutionToken,
        controller_id: str,
        goal_checker_id: str,
        progress_checker_id: str,
        on_accepted: Callable[[ExecutionToken, bool, str], None],
        on_result: Callable[[Nav2Result], None],
    ) -> bool:
        """Send one FollowPath goal; returns False if another goal is active."""
        if self._active_token is not None:
            return False
        self._active_token = token
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = str(controller_id)
        goal.goal_checker_id = str(goal_checker_id)
        # Jazzy FollowPath contains progress_checker_id.  Keep compatibility
        # with older generated interfaces without weakening the contract.
        if hasattr(goal, 'progress_checker_id'):
            goal.progress_checker_id = str(progress_checker_id)

        try:
            future = self._client.send_goal_async(goal)
            future.add_done_callback(
                lambda f: self._on_goal_response(f, token, on_accepted, on_result)
            )
        except Exception as exc:  # transport/setup failure: fail closed
            self._active_token = None
            on_accepted(token, False, f'send_goal_exception:{exc}')
            return True
        return True

    def cancel_active(self) -> bool:
        """Request cancellation once.  Safe and idempotent."""
        if self._active_token is None:
            return False
        if self._goal_handle is None:
            # Goal response is still pending.  Navigator has already disabled
            # motion; result/acceptance will be rejected by token/state checks.
            return True
        if self._cancel_future is not None:
            return True
        try:
            self._cancel_future = self._goal_handle.cancel_goal_async()
        except Exception:
            self._cancel_future = None
        return True

    def abandon(self, token: ExecutionToken | None = None) -> None:
        """Forget local ownership after fail-closed shutdown or stale result."""
        if token is not None and self._active_token != token:
            return
        self._active_token = None
        self._goal_handle = None
        self._cancel_future = None

    def _on_goal_response(self, future, token, on_accepted, on_result) -> None:
        if self._active_token != token:
            return
        try:
            handle = future.result()
        except Exception as exc:
            self.abandon(token)
            on_accepted(token, False, f'goal_response_exception:{exc}')
            return
        if handle is None or not handle.accepted:
            self.abandon(token)
            on_accepted(token, False, 'nav2_goal_rejected')
            return

        self._goal_handle = handle
        on_accepted(token, True, 'nav2_goal_accepted')
        try:
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda f: self._on_result(f, token, on_result)
            )
        except Exception as exc:
            self.abandon(token)
            on_result(
                Nav2Result(
                    token=token,
                    completion=Nav2Completion.ERROR,
                    action_status=GoalStatus.STATUS_UNKNOWN,
                    error_msg=f'get_result_exception:{exc}',
                )
            )

    def _on_result(self, future, token, on_result) -> None:
        if self._active_token != token:
            # Late result from a locally abandoned generation.
            return
        try:
            wrapped = future.result()
            status = int(wrapped.status)
            result = wrapped.result
            error_code = int(getattr(result, 'error_code', 0))
            error_msg = str(getattr(result, 'error_msg', ''))
            if status == GoalStatus.STATUS_SUCCEEDED:
                completion = Nav2Completion.SUCCEEDED
            elif status == GoalStatus.STATUS_CANCELED:
                completion = Nav2Completion.CANCELED
            elif status == GoalStatus.STATUS_ABORTED:
                completion = Nav2Completion.ABORTED
            else:
                completion = Nav2Completion.ERROR
            nav_result = Nav2Result(
                token=token,
                completion=completion,
                action_status=status,
                error_code=error_code,
                error_msg=error_msg,
            )
        except Exception as exc:
            nav_result = Nav2Result(
                token=token,
                completion=Nav2Completion.ERROR,
                action_status=GoalStatus.STATUS_UNKNOWN,
                error_msg=f'result_exception:{exc}',
            )
        self.abandon(token)
        on_result(nav_result)
