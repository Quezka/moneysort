"""A model of lgpio 0.2.2's pulse thread (lgPthTx.c), and the proof that TxAnchor keeps it from wedging.

The model copies the C logic that matters, line for line:
  * records are kept in a list, newest first; tx_pulse on an idle GPIO creates one at the head;
  * the timing thread walks the list; deleting a finished HEAD ends the walk (the bug), so the records
    behind it are skipped for that pass;
  * the next sleep delay is the smallest next_micros; a negative delay is added to tv_nsec, which can go
    negative -> clock_nanosleep() fails with EINVAL and the C code spins on it for ever (`wedged`).
Reference: lgpio-0.2.2.0/src/lgPthTx.c and xSetAsPwm in lgGpio.c.
"""
import random
import sys
import unittest

import test_stepper_tx as base      # installs the shared fake lgpio module
from moneysort.hardware import stepper as st

lg = sys.modules["lgpio"]


class Rec:
    def __init__(self, gpio, on, off, cycles, next_micros):
        self.gpio, self.active = gpio, True
        self.entries = [[on, off, cycles or -1]]
        self.next_micros = next_micros
        self.next_level = 1 if on else 0


class Model:
    BUF = 10

    def __init__(self, tv_nsec, unlucky=False):
        self.unlucky = unlucky          # after the early exit, pretend tv_nsec is in the 2% that goes negative
        self.min_delay = 0
        self.recs = []                  # head first
        self.delay = 0
        self.tv_nsec = tv_nsec
        self.wedged = False
        self.skipped_active = 0         # times the early exit skipped an ACTIVE record

    # --- lgpio API (xSetAsPwm) ---
    def find(self, gpio):
        return next((r for r in self.recs if r.gpio == gpio), None)

    def pulse(self, gpio, on, off, cycles):
        r = self.find(gpio)
        if r and r.active:
            if on or off:
                if len(r.entries) > 1 and r.entries[-1][2] == -1:
                    r.entries.pop()
                if len(r.entries) < self.BUF:
                    r.entries.append([on, off, cycles or -1])
                    return self.BUF - len(r.entries)
                raise lg.error("TX queue full")
            r.active = False
            return 0
        ct = on + off
        usec = self.tv_nsec // 1000
        frac = usec - ct * (usec // ct)
        self.recs.insert(0, Rec(gpio, on, off, cycles, ct - frac + self.delay))
        return self.BUF - 1

    def room(self, gpio):
        r = self.find(gpio)
        return self.BUF - len(r.entries) if r and r.active else self.BUF

    def busy(self, gpio):
        r = self.find(gpio)
        return 1 if r and r.active else 0

    # --- the thread (one pass of the while(1) loop) ---
    def run_pass(self):
        if self.wedged:
            return
        for p in self.recs:
            p.next_micros -= self.delay
        self.delay = 20000
        i = 0
        while i < len(self.recs):
            p = self.recs[i]
            if p.active:
                if p.next_micros <= 0:
                    on, off, cyc = p.entries[0]
                    if p.next_level or on == 0:
                        if cyc <= 0 and len(p.entries) > 1:
                            p.entries.pop(0)
                            on, off, cyc = p.entries[0]
                        if cyc == 0:
                            p.active = False
                        elif on:
                            p.next_micros += on
                            if off:
                                p.next_level = 0
                        else:
                            p.next_micros += off
                            p.next_level = 1
                        p.entries[0][2] = max(cyc - 1, -1) if cyc != 0 else 0
                    else:
                        p.next_micros += p.entries[0][1]
                        p.next_level = 1
                if p.next_micros < self.delay:
                    self.delay = p.next_micros
                i += 1
            else:
                head = i == 0
                self.recs.pop(i)
                if head:                                # the lgpio bug: the walk ends here
                    skipped = sum(1 for q in self.recs if q.active)
                    self.skipped_active += skipped
                    if skipped and self.unlucky:
                        self.tv_nsec = 990_000_000      # the +20 ms step then wraps past the second
                    break
        self.min_delay = min(self.min_delay, self.delay)
        self.tv_nsec += self.delay * 1000
        if self.tv_nsec >= 1_000_000_000:
            self.tv_nsec -= 1_000_000_000
        if self.tv_nsec < 0:                            # EINVAL from clock_nanosleep -> infinite spin
            self.wedged = True

    def advance(self, passes=40):
        for _ in range(passes):
            self.run_pass()


class TestLgpioModel(unittest.TestCase):
    def setUp(self):
        self._saved = {n: getattr(lg, n) for n in ("tx_pulse", "tx_room", "tx_busy", "gpio_claim_output")}
        self.model = None

        def pin_of(h, pin):
            return pin
        lg.tx_pulse = lambda h, pin, on, off, offs, cyc: self.model.pulse(pin, on, off, cyc)
        lg.tx_room = lambda h, pin, kind: (self.model.advance(), self.model.room(pin))[1]
        lg.tx_busy = lambda h, pin, kind: (self.model.advance(), self.model.busy(pin))[1]
        lg.gpio_claim_output = lambda *a: None
        self.fresh_anchor()

    def fresh_anchor(self):
        st.ANCHOR = st.TxAnchor()
        st.ANCHOR._thread = object()                 # pretend the watcher runs: no real thread in tests

    def tearDown(self):
        for n, f in self._saved.items():
            setattr(lg, n, f)
        st.ANCHOR = st.TxAnchor()

    def run_move(self, with_anchor, lengths, tv_nsec, unlucky=True):
        """Three axes (x, y, z pins 5, 17, 23) get trains of different lengths, like move_many."""
        self.model = Model(tv_nsec, unlucky)
        steppers = [st.Stepper(1, p, p + 1) for p in (5, 17, 23)]
        self.fresh_anchor()
        if with_anchor:
            st.ANCHOR.setup(1, 21, steppers)            # (we release it by hand at the end, like the watcher would)
        for s, n in zip(steppers, lengths):
            while n > 0:
                s._tx(6000, min(n, 500))              # bursts of up to 500 cycles at 6000 pps (~83 ms)
                n -= 500
        for _ in range(4000):
            self.model.advance(10)
            if all(not self.model.busy(p) for p in (5, 17, 23)):
                break
        if with_anchor:
            st.ANCHOR.release()
            self.model.advance(50)
        return self.model

    def test_without_the_anchor_a_short_last_axis_wedges_the_thread(self):
        # z (created last = head) is the shortest and finishes while x and y still run behind it
        m = self.run_move(False, (1000, 2000, 300), tv_nsec=5_000_000)
        self.assertGreater(m.skipped_active, 0)
        self.assertLess(m.min_delay, -10000, "skipped records fall ~20 ms behind -> a huge negative delay")
        self.assertTrue(m.wedged, "the model must reproduce the wedge the real thread showed")

    def test_the_bug_alone_is_not_always_fatal_but_it_is_always_a_20ms_stall(self):
        m = self.run_move(False, (1000, 2000, 300), tv_nsec=5_000_000, unlucky=False)
        self.assertLess(m.min_delay, -10000)          # fatal only when tv_nsec happens to be < 20 ms (~2%)

    def test_with_the_anchor_it_never_wedges(self):
        for lengths in ((1000, 2000, 300), (300, 2000, 1000), (2000, 300, 1000), (500, 500, 500), (100, 1500, 700)):
            for tv in (1_000_000, 5_000_000, 10_000_000, 19_000_000, 400_000_000):
                m = self.run_move(True, lengths, tv_nsec=tv)
                self.assertFalse(m.wedged, (lengths, tv))
                self.assertEqual(m.skipped_active, 0, (lengths, tv))
                self.assertGreater(m.min_delay, -1000, (lengths, tv))

    def test_random_moves_never_wedge_with_the_anchor(self):
        rng = random.Random(7)
        for _ in range(60):
            lengths = tuple(rng.choice((0, 100, 200, 500, 1000, 3000)) for _ in range(3))
            m = self.run_move(True, lengths, tv_nsec=rng.randrange(0, 20_000_000))
            self.assertFalse(m.wedged, lengths)

    def test_without_the_anchor_random_moves_do_wedge_sometimes(self):
        rng = random.Random(7)
        wedges = 0
        for _ in range(60):
            lengths = tuple(rng.choice((0, 100, 200, 500, 1000, 3000)) for _ in range(3))
            wedges += self.run_move(False, lengths, tv_nsec=rng.randrange(0, 20_000_000)).wedged  # unlucky tv
        self.assertGreater(wedges, 0)


if __name__ == "__main__":
    unittest.main()
