"""SharedPreview: one producer for many viewers, idle when nobody watches (no hardware)."""
import threading
import time
import unittest

from moneysort.interface.preview import SharedPreview


class SharedPreviewTest(unittest.TestCase):
    def test_viewers_share_one_producer(self):
        calls = []

        def produce():
            calls.append(1)
            return b"frame%d" % len(calls)

        hub = SharedPreview(produce, 0.02)
        got = [[], []]

        def watch(i):
            for jpg in hub.frames():
                got[i].append(jpg)
                if len(got[i]) == 5:
                    return

        threads = [threading.Thread(target=watch, args=(i,)) for i in range(2)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(5)
        self.assertEqual([len(g) for g in got], [5, 5])
        self.assertLess(len(calls), 9)           # ~5 shared frames; one producer per viewer would be 10+
        time.sleep(0.1)
        settled = len(calls)
        time.sleep(0.2)
        self.assertEqual(len(calls), settled)    # nobody watching -> no more detection

    def test_failed_frame_keeps_worker_alive(self):
        n = []

        def produce():
            n.append(1)
            if len(n) == 1:
                raise RuntimeError("boom")
            return b"ok"

        hub = SharedPreview(produce, 0.01)
        gen = hub.frames()
        self.assertEqual(next(gen), b"ok")
        gen.close()


if __name__ == "__main__":
    unittest.main()
