from collections import deque


class DuplicateCache:
    def __init__(self, max_size: int = 100) -> None:
        self._max_size = max(1, max_size)
        self._order: deque[str] = deque()
        self._items: set[str] = set()

    def seen(self, message_id: str) -> bool:
        if message_id in self._items:
            return True
        self._items.add(message_id)
        self._order.append(message_id)
        while len(self._order) > self._max_size:
            old = self._order.popleft()
            self._items.discard(old)
        return False
