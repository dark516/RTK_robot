from pathlib import Path
import inspect
import pytest

from frob_city_navigation.nav2_adapter import ExecutionToken, Nav2Adapter


def token(gen=1):
    return ExecutionToken('run', 'map-instance', 'navigator', 'route', 'segment', gen)


def test_execution_token_rejects_incomplete_or_zero_generation():
    assert token(1).goal_generation == 1
    with pytest.raises(ValueError):
        ExecutionToken('', 'm', 'n', 'r', 's', 1)
    with pytest.raises(ValueError):
        ExecutionToken('r', 'm', 'n', 'route', 's', 0)


def test_nav2_adapter_is_single_goal_and_preserves_token_in_callbacks():
    src = inspect.getsource(Nav2Adapter)
    assert 'if self._active_token is not None:' in src
    assert 'self._active_token = token' in src
    assert 'if self._active_token != token:' in src
    assert 'cancel_goal_async()' in src
    assert 'FollowPath.Goal()' in src
    assert 'goal.path = path' in src
