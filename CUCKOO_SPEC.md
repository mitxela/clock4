# CUCKOO — scheduled display animations for the Precision Clock Mk IV

Status: v2 · supersedes v1 · branch `cuckoo-v2` (off rollup)

A cuckoo is a short, scheduled display flourish. It lives entirely on the TIME board; the
date row is never addressed. Every animation modulates the true display — the clock never
shows wrong digits, and every animation's final frame is the plain live face. The v1
primitives carry forward with one integration change: per-segment brightness rides the
seg_balance 80-slot bit-reversed dither interleave already shipped on this branch (one
interleave engine, two clients) instead of the v1 branch's own scan. The 10 ms
terminating-countdown tick and the second-edge scheduler (never preempted, colliding
trigger skipped) are unchanged. PCC inherits everything through the byte-faithful WASM
build.

## The cadence

Cadence is intrinsic, not scheduled. Two tiers, after the Westminster pattern: each
quarter-hour (:15/:30/:45) gets one fixed small gesture — the nod; the hour (:00) gets the
full configured piece. **Cadence law: the clock nods on each quarter edge and performs its
piece on the hour edge — one gesture per quarter, one piece per hour, nothing configurable
between them.**

## Config vocabulary

```
# off (default) | trust
cuckoo = off
```

One key, enable-and-select fused. The value names the piece — `cuckoo = heartbeat` reads
"the cuckoo is heartbeat" — so `cuckoo_effect` / `cuckoo_transition` would merely restate
the value; the noun *is* the feature. `off` disables both tiers. `rain` is not in the enum
(date board, deferred).

`cuckoo_interval` is omitted from this spec because a cuckoo's cadence is intrinsic — the
quarter gesture and the hourly piece are anchored to the face's own structure, not a user
knob — and an interval would only license flourishes that no longer mark the hour.

## The nod — the quarter gesture

One fixed gesture, identical on every :15/:30/:45 second edge; never the selected piece in
miniature.

- **Elements:** the six big time digits (HH MM SS) only. Colons untouched (they are the
  live pulse); sub-second and ms digits untouched (fast elements get no blooms).
- **Envelope:** a single shallow brightness trough sweeps left to right. Digit *k*
  (k = 0 leftmost) begins at `k·30 ms`; each digit dips nominal → 8/16 and back on a
  symmetric integer trough, 200 ms down, 200 ms up. Floor 8/16 — never below half, so
  every digit stays fully legible. The wave clears the row and settles by ~550 ms.
  Brightness only: no glyph, colour, or data change.
- **Mechanism:** the dither interleave at digit resolution on 10 ms countdowns; the
  envelope terminates at nominal, so the final frame *is* the plain live face.
- **Numbers provisional:** depth (8/16) and sweep rate (30 ms/digit) are implemented as
  written but held for a bench check at across-the-room distance — respiration, not a
  power sag — before the PR locks them.

**Perceptual gamma (bench-tuned 2026-07-20):** pieces write INTENT levels; the render maps
them through `duty = 16·(L/16)^2.2` (floored at one lit cycle for any lit intent). Light
output is linear in dither duty but the eye is not — the first bench check found `carry`
and `pendulum` nearly invisible (their action lives at the top of the duty range, where
perceived differences compress) while `trust` (a 2→16 swing) read fine. Under the gamma an
intent difference reads evenly anywhere in the range. The tuning knob is this one curve;
piece bodies stay in intent space and never re-tune per piece.

Why it survives 96 plays/day: shallow, brief, monochrome, carrying no data — nothing in it
can be wrong and there is no novelty to exhaust. Identical every quarter, it reads as the
clock's respiration, not a message. In holdover it is unchanged (pure brightness, no PPS
or GPS dependency).

## The hourly flourish

At :00 the configured piece performs. :00 is also a quarter edge; the nod is suppressed —
`if minute == 0 → piece; elif minute ∈ {15,30,45} → nod`. One performance per hour
boundary. If the configured piece cannot run honestly (`pendulum` in holdover), **the nod
stands in** — the hour boundary is never wholly silent, and the honest minimum gesture
never forges the absent performance.

There is no artificial hour-count: the hour digits are lit and exact; counted blinks would
duplicate legible data and read as a fault. (`trust` is honest in every state, so with the
shipped catalogue the stand-in never fires; the law remains for the parked pieces' return.)

## Pose and the date row

Cuckoo is **time-board-only**; the date row is never touched. Both constraints the client
raised dissolve at this boundary, for two hardware reasons:

1. **Pose is automatic.** The TIME row reads left-to-right in both poses (stacked: date
   above time; extended: date left of time), and the time board cannot sense orientation —
   only the date board senses pose (GPIOC bit 0) and self-corrects its glyphs, decimal
   points and button codes; the date→time UART carries pre-corrected button bytes only. A
   time-board-local animation is therefore pose-invariant by construction. No pose flag.
2. **The date row is unreachable, honestly.** Its UART carries rendered text plus a latch
   byte and no brightness — per-segment intensity on the date row is impossible today (the
   wall v1 deferred rain against). So whatever it holds — civil date (also under
   LST/SOLAR), ADEV ladder, star transit, COUNTDOWN, TEXT — it keeps reading truthfully
   *through* the flourish. A countdown that keeps counting is correct; freezing it to
   match the time row would be the lie.

**v2 hook:** date-board participation (rain's canvas, trust's date sweep, showcase names)
waits on one UART extension — a per-element intensity frame atop the existing text+latch
framing.

## Preview — hearing a piece without waiting for it

Two ways to play a piece immediately, outside the schedule:

- **The menu.** The on-device CUCKOO row (DISP section, values OFF / TRUST) **performs each
  value as it is selected** — the menu's own colon-preview idiom applied to the flourishes.
  The menu owns the date row while open, the piece plays on the time row; the two never
  collide. Cancel restores the old value and stops the piece; commit persists the value and
  stops the piece (it then waits for its hour); the idle auto-exit stops it too. (When the
  parked pieces return: a preview must not forge what the schedule refuses — a PPS-dependent
  piece previews the NOD in holdover.)
- **Serial.** `cuckoo_preview = trust|nod|on` (serial-only, never valid in config.txt —
  a file must not perform) plays once without touching the `cuckoo` config; `on` previews
  the configured piece.

A preview preempts a running preview — a tap-through of the menu ring must respond per
tap. That is a deliberate act at the clock, not a scheduler exception; the scheduler itself
still never preempts.

## The catalogue

**Shipping: `trust` only** (first bench verdict, 2026-07-20: it was the one piece that read
well on hardware). The others are **parked, not deleted** — parked means absent from the
firmware, per this project's own rule:

- **`carry`, `heartbeat`, `pendulum`** — bodies preserved in git (59fc649); they return
  through the same enum when their visual rework earns a second bench verdict. Candidate
  direction on record: even under the perceptual gamma their action may need structural
  changes (bigger level strides, slower beats), not just brighter rendering.
- **`rain`** stays on the date board — deferred, not degraded.

The nod-stand-in law stands as design law (an hour whose piece cannot run honestly gets the
nod); with `trust` — honest in every state — it is currently dormant.

## What was rejected, and why it stays rejected

- **`cuckoo_interval`** — cadence is intrinsic; an interval would desync the flourish from
  the hour it marks.
- **Per-piece or scaled quarter gestures** — one fixed nod only; a shrunk piece would let
  a data piece (`trust`, `pendulum`) fire un-earned four times an hour.
- **Westminster phrase-growth across the quarters** — it would encode the quarter number
  the digits already state; redundant decoration.
- **An artificial hour-strike count** — duplicates the legible, exact hour digits and
  reads as a fault; `carry`'s real rollover chain is the only honest "count."
- **A substitute piece for an unrunnable hour flourish** — the nod stands in; a swapped-in
  *piece* would forge "it played."
- **`MODE_CUCKOO_SHOWCASE`** — deferred: it prints piece names on the date row, the one
  place cuckoo must not touch; it returns with the v2 date extension.
- v1's standing rejections (three-timescale tour, moon dial, equation-of-time walk,
  multiplex-scan reveal, count-based rain) remain rejected for their original reasons.

## Emulator / PCC integration

- The shim exports `emu_seg_bright(cell, seg)` mirroring `emu_digit_fade`; PCC renders it
  through the existing per-element `setSegField` face surface. The web-side prototype
  engine (demo7.js) is retired when this branch reaches the rollup.
- Conformance: the golden suite must show byte-identical display behaviour with
  `cuckoo = off` (the feature costs nothing when disabled); a fixture drives one nod and
  one `carry` and asserts every envelope terminates at nominal.
- Standby-gated: the scheduler refuses while the display is dark (MODE_STANDBY) — a dark
  clock stays dark.
