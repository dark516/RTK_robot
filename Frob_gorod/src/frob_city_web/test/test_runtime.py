import threading
import time

import pytest

from frob_city_web.runtime import ControlMailbox, ImageThrottle, source_health


def test_control_mailbox_round_trip_and_timeout():
    mailbox = ControlMailbox(limit=2)
    returned = []
    thread = threading.Thread(
        target=lambda: returned.append(mailbox.submit("START", "operator", 0.5))
    )
    thread.start()
    pending = mailbox.take()
    assert pending.command == "START"
    mailbox.complete(pending, {"accepted": True})
    thread.join(timeout=1.0)
    assert returned == [{"accepted": True}]

    with pytest.raises(TimeoutError):
        mailbox.submit("PAUSE", "operator", 0.01)
    assert mailbox.take() is None


def test_image_throttle_requires_viewer_and_rate_limit():
    throttle = ImageThrottle(max_fps=5.0, jpeg_quality=70)
    assert throttle.should_encode(now_s=1.0, viewers=0) is False
    assert throttle.should_encode(now_s=1.0, viewers=1) is True
    assert throttle.should_encode(now_s=1.1, viewers=1) is False
    assert throttle.should_encode(now_s=1.2, viewers=1) is True
    assert throttle.jpeg_quality == 70
    with pytest.raises(ValueError):
        ImageThrottle(max_fps=0, jpeg_quality=70)
    with pytest.raises(ValueError):
        ImageThrottle(max_fps=5, jpeg_quality=101)


def test_source_health_reports_missing_fresh_and_stale():
    assert source_health(None, max_age_s=1.0, now_s=10.0)["state"] == "missing"
    assert source_health(9.5, max_age_s=1.0, now_s=10.0) == {
        "state": "fresh", "age_s": 0.5, "max_age_s": 1.0,
    }
    assert source_health(8.0, max_age_s=1.0, now_s=10.0)["state"] == "stale"
