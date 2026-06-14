from __future__ import annotations

import json
import queue
import threading
import urllib.error
import urllib.request
from typing import Any


class DashboardPublisher:
    """Best-effort async publisher for the browser dashboard HTTP endpoint."""

    def __init__(self, url: str, timeout_sec: float = 0.03, max_queue: int = 2):
        self.url = url
        self.timeout_sec = timeout_sec
        self._queue: queue.Queue[dict[str, Any] | None] = queue.Queue(maxsize=max_queue)
        self._last_error: str | None = None
        self._thread = threading.Thread(target=self._run, name="dashboard-publisher", daemon=True)
        self._thread.start()

    @property
    def last_error(self) -> str | None:
        return self._last_error

    def publish(self, payload: dict[str, Any]) -> None:
        try:
            self._queue.put_nowait(payload)
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(payload)
            except queue.Full:
                pass

    def close(self) -> None:
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            pass
        self._thread.join(timeout=0.2)

    def _run(self) -> None:
        while True:
            payload = self._queue.get()
            if payload is None:
                return
            data = json.dumps(payload, allow_nan=False).encode("utf-8")
            request = urllib.request.Request(
                self.url,
                data=data,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            try:
                with urllib.request.urlopen(request, timeout=self.timeout_sec):
                    self._last_error = None
            except (OSError, urllib.error.URLError, ValueError) as exc:
                self._last_error = str(exc)
