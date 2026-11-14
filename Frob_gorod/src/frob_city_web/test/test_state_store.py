import threading
import time

from frob_city_web.state_store import StateStore


def test_update_is_change_sensitive_and_delta_is_section_scoped():
    store = StateStore(delta_limit=4)
    first = store.update("mission", {"phase": "IDLE"})
    assert first == 1
    assert store.update("mission", {"phase": "IDLE"}) == first

    second = store.update("guard", {"allowed": False})
    delta = store.delta_after(first, timeout_s=0.0)

    assert second == 2
    assert delta == {
        "revision": 2,
        "full": False,
        "sections": {"guard": {"allowed": False}},
    }


def test_delta_overflow_returns_full_snapshot():
    store = StateStore(delta_limit=2)
    store.update("one", {"value": 1})
    store.update("two", {"value": 2})
    store.update("three", {"value": 3})

    delta = store.delta_after(0, timeout_s=0.0)

    assert delta["full"] is True
    assert delta["revision"] == 3
    assert delta["sections"]["three"] == {"value": 3}


def test_waiting_delta_wakes_on_update():
    store = StateStore()
    result = []

    waiter = threading.Thread(
        target=lambda: result.append(store.delta_after(0, timeout_s=0.5))
    )
    waiter.start()
    time.sleep(0.02)
    store.update("pose", {"x": 1.0})
    waiter.join(timeout=1.0)

    assert result[0]["sections"] == {"pose": {"x": 1.0}}


def test_events_and_trail_are_bounded():
    store = StateStore(event_limit=2, trail_limit=2)
    for index in range(3):
        store.append_event("info", "test", f"event-{index}", {"i": index})
        store.append_trail(float(index), float(index + 1), float(index + 2))

    snapshot = store.snapshot()
    assert [item["message"] for item in snapshot["sections"]["events"]] == [
        "event-1",
        "event-2",
    ]
    assert snapshot["sections"]["trail"] == [
        {"x_m": 1.0, "y_m": 2.0, "stamp_s": 3.0},
        {"x_m": 2.0, "y_m": 3.0, "stamp_s": 4.0},
    ]


def test_camera_is_separate_from_json_and_waits_for_new_sequence():
    store = StateStore()
    assert store.camera_after(0, timeout_s=0.0) is None
    sequence = store.set_camera(b"jpeg-data", stamp_s=12.5)

    assert sequence == 1
    assert store.camera_after(0, timeout_s=0.0) == (1, 12.5, b"jpeg-data")
    assert store.camera_after(1, timeout_s=0.0) is None
    assert b"jpeg-data" not in repr(store.snapshot()).encode()


def test_snapshot_is_a_defensive_copy():
    store = StateStore()
    value = {"nested": {"items": [1]}}
    store.update("data", value)
    value["nested"]["items"].append(2)
    snapshot = store.snapshot()
    snapshot["sections"]["data"]["nested"]["items"].append(3)

    assert store.snapshot()["sections"]["data"] == {"nested": {"items": [1]}}
