"""Trapezoidal motion planning -- pure math, no hardware.

A move is split into a ramp-up, a cruise, and a ramp-down, each emitted as one or
more ``(pps, cycles)`` bursts. Ramping avoids stalling the motor by asking it to
jump straight to full speed from a standstill. The hardware layer (Stepper) turns
these bursts into ``lgpio.tx_pulse`` calls; keeping the planning pure means it can
be unit-tested without a Pi attached.
"""

# The cruise is emitted as many short bursts instead of one long one. lgpio plays
# a finite burst to the end no matter what (only an *infinite* burst can be cut
# short), so a single 20 s cruise would ignore an e-stop until it finished. With
# ~this-many-seconds bursts, disable() stops the feed and the few already-queued
# bursts drain in a fraction of a second (torque is already off regardless).
CRUISE_CHUNK_S = 0.04


def ramp_segs(ramp_steps, max_pps, min_pps, up, k=16):
    """Return ``[(pps, cycles), ...]`` for a rising (``up``) or falling ramp."""
    segs = []
    if ramp_steps <= 0:
        return segs
    base, rem = divmod(ramp_steps, k)
    for i in range(k):
        cyc = base + (1 if i < rem else 0)
        if cyc <= 0:
            continue
        frac = (i + 1) / k
        if up:
            pps = min_pps + (max_pps - min_pps) * frac
        else:
            pps = max_pps - (max_pps - min_pps) * frac
        segs.append((max(pps, min_pps), cyc))
    return segs


# Bursts of one or two cycles last well under a millisecond, so the next tx_pulse is queued just as
# the previous one ends -- right in lgpio's idle/wake-up window, where its queue has been seen to
# wedge (every axis "busy, room 0", only a restart clears it; always on tiny moves). Merging tiny
# neighbours into bursts of at least this many cycles keeps the queue feed away from that window.
MIN_BURST_CYCLES = 8


def merge_short(segs, min_pps, min_cycles=MIN_BURST_CYCLES):
    """Merge adjacent bursts so none has fewer than ``min_cycles`` cycles (keeps total cycles and time)."""
    out = []
    cyc_acc, time_acc = 0, 0.0
    for pps, cyc in segs:
        cyc_acc += cyc
        time_acc += cyc / pps
        if cyc_acc >= min_cycles:
            out.append([cyc_acc / time_acc, cyc_acc])
            cyc_acc, time_acc = 0, 0.0
    if cyc_acc:
        if out:                                     # fold the remainder into the last burst
            c = out[-1][1] + cyc_acc
            t = out[-1][1] / out[-1][0] + time_acc
            out[-1] = [c / t, c]
        else:                                       # the whole move is tiny: one gentle burst
            out.append([min(cyc_acc / time_acc, 2 * min_pps), cyc_acc])
    return [(pps, cyc) for pps, cyc in out]


def plan_segments(steps, max_pps, min_pps, accel_steps):
    """Trapezoid for ``abs(steps)`` pulses: ``[(pps, cycles), ...]``.

    Direction is the caller's concern (sign of ``steps`` + wiring invert); this
    returns only the magnitude profile. The cruise is chunked (see
    ``CRUISE_CHUNK_S``) so an e-stop can interrupt it promptly.
    """
    n = abs(steps)
    ramp = min(accel_steps, n // 2)
    cruise = n - 2 * ramp
    segs = ramp_segs(ramp, max_pps, min_pps, up=True)
    if cruise > 0:
        chunk = max(1, int(max_pps * CRUISE_CHUNK_S))   # bound e-stop latency
        full, rem = divmod(cruise, chunk)
        segs.extend([(max_pps, chunk)] * full)
        if rem:
            segs.append((max_pps, rem))
    segs += ramp_segs(ramp, max_pps, min_pps, up=False)
    return merge_short(segs, min_pps)
