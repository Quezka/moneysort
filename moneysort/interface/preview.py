"""Shared preview: one producer feeds every viewer of the annotated camera stream.

Coin detection is the heavy part of the preview (about a core of the Pi's CPU). Computing it per
connected browser made each extra viewer slow every other one down, and fed the detector's
"hold a missed coin" history with calls it did not expect. Here a single worker thread produces
the frames, only while somebody is watching, and every viewer gets the latest one.
"""
import threading
import time


class SharedPreview:
    def __init__(self, produce, period):
        self._produce = produce                  # () -> jpeg bytes or None
        self._period = period                    # seconds between frames
        self._cond = threading.Condition()
        self._jpg = None
        self._seq = 0
        self._watchers = 0
        self._worker = False

    def frames(self):
        """Yield each new frame as it is produced; stops producing once the last viewer leaves."""
        with self._cond:
            self._watchers += 1
            if not self._worker:
                self._worker = True
                threading.Thread(target=self._run, daemon=True).start()
            seen = self._seq
        try:
            while True:
                with self._cond:
                    while self._seq == seen:
                        self._cond.wait(2.0)
                    seen, jpg = self._seq, self._jpg
                if jpg:
                    yield jpg
        finally:
            with self._cond:
                self._watchers -= 1

    def _run(self):
        while True:
            start = time.monotonic()
            try:
                jpg = self._produce()
            except Exception:
                jpg = None                       # a failed frame must not kill the worker
            with self._cond:
                self._jpg, self._seq = jpg, self._seq + 1
                self._cond.notify_all()
                if self._watchers == 0:
                    self._worker = False
                    return
            time.sleep(max(0.0, self._period - (time.monotonic() - start)))
