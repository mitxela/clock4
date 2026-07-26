# Mk IV firmware additions

This section documents the features added by the proposed firmware PR series (#5–#10 plus the three
follow-ons: the Allan deviation display, the star transit predictor and the on-device menu). It
describes the **rollup firmware** — the tested union of the whole series, which is exactly what the
ready-to-flash images linked from each PR contain. That build currently carries version
`0.0.5+<build>` on the time board and `0.0.2+<build>` on the date board; nothing here should be
read as describing any other 0.0.5 build. Everything is
opt-in: with none of the new config keys set, and a stock date board, behaviour is identical to the
stock firmware.

## New modes

As before, modes are enabled from `config.txt` and the buttons cycle through the enabled set. The new
modes fall into two groups: date-row read-outs, where the payload shows on the 10-character date row
while the clock keeps running on the time row (the same pattern as `MODE_SATVIEW`), and alternate
timebases, which take over the time row itself.

Most of these need to know where the clock is. A GPS fix provides that automatically; indoors you can
set `fake_latitude` / `fake_longitude` (decimal degrees, north and east positive) instead. With
neither, the mode shows dashes. Note that setting a fake position also pins the clock's position —
GPS position updates are ignored.

### Astro modes

```
MODE_SUN = enabled
MODE_SUN_AZEL = enabled
MODE_MOON = enabled
MODE_GRID = enabled
MODE_LATLON = enabled
```

`MODE_SUN` shows sunrise, sunset and solar noon in local time, paging `RISE 06.32` → `SET  21.14` →
`SOL  13.02` (see `page_ms` below for the dwell). During polar day or night, when the sun doesn't rise
or set on the current date, the rise and set pages show `----`; solar noon is still shown.

`MODE_SUN_AZEL` shows the sun's azimuth and elevation right now, in whole degrees, e.g. `AZ142EL38`.
Elevation can be negative (`AZ302EL-12`).

`MODE_MOON` shows the moon phase index and illuminated percentage, e.g. `MOON 4  99`. The index runs
0 new, 1 waxing crescent, 2 first quarter, 3 waxing gibbous, 4 full, 5 waning gibbous, 6 last
quarter, 7 waning crescent. This is the one astro mode that needs no position.

`MODE_GRID` shows the Maidenhead grid locator, e.g. `IO91xl`.

`MODE_LATLON` pages latitude and longitude in decimal degrees: `LAT  51.48` → `LON  -0.01` (a 3-digit longitude drops the gap: `LON-179.99`).

`MODE_DARK` is the observing-session twilight ladder: five pages on the `page_ms` dwell, laid out in
`MODE_SUN`'s columns so the digits stay put as it pages. The first page is a live countdown in hours
and minutes to the start of astronomical darkness (`DARK 05.57`), flipping once dark to the time left
in it, counting down to astronomical dawn (`DAWN 03.20`). Behind it are the evening tiers and the end
of the night, in local time: civil dusk at -6 degrees (`CIV  21.42`), nautical at -12 (`NAU  22.31`),
astronomical at -18 (`AST  23.40`), and the dawn that ends the dark (`End  02.16`, taken as the
mirror of dusk about solar noon). Where the sun sets but never reaches -18 the headline reads
`NO DARK` and the `AST` and `End` pages read `----` rather than a fabricated time, while `CIV` and
`NAU` still show. During polar day or night, when the sun doesn't rise or set on the current date,
all four tier pages read `----` and the headline reads `NO DARK`, or `DARK  NOW` if the sun is below
-18 right now. With no position the mode shows `DARK ----`.

### Sidereal and solar time

```
MODE_LST = enabled
MODE_SOLAR = enabled
```

These replace the time row itself: the big digits tick an alternate timebase while the date row keeps
the civil date, so the bottom row always remains an unambiguous civil anchor.

`MODE_LST` is Local Sidereal Time as a live ticking clock. Sidereal time runs 1.00273791 times faster
than civil time, so the seconds display double-steps about once every six minutes — that's real
sidereal behaviour, not a glitch.

`MODE_SOLAR` is apparent solar time — what a sundial reads: UTC shifted by your longitude plus the
equation of time. It reads exactly 12:00:00 at local solar noon.

Both modes inherit the GPS discipline and the accuracy-tolerance behaviour of the civil display, and
both need a position (dashes without one). So that solar time — which can sit within minutes of civil
time — can never be mistaken for it at a glance, these modes use their own colon animation, set by
`colon_alt_mode` (see below). By default this is kept automatically distinct from your civil colon.

### Temperature compensation display

```
MODE_TEMPCOMP = enabled
```

A diagnostic read-out for the temperature compensation system (see its own section below), paging
four sub-screens on the date row: the die temperature (`tC  32C`), the active timebase steering
correction in ppm (`HSE -0.25` — this reads `0.00` while GPS-locked, since the PPS owns the phase
then, and `----` with no usable model), the RTC crystal's absolute model value in ppm (`rtC  18.68`),
and the sample count with a state letter (`n   159 L`). The state letter is `A` applying (steering
engaged), `F` frozen from config, `S` running on a warm-start seed, `L` learning, `-` idle.

### Allan deviation display

```
MODE_ADEV = enabled
```

A live Allan deviation of the free-running crystal — its true stability, undisciplined. The firmware
integrates the raw oscillator phase against the GPS second (independently of the disciplined display)
and computes the overlapping Allan deviation on the fly. The date row pages σ<sub>y</sub>(τ) across
octave averaging times τ = 1, 2, 4, … up to 1024 seconds, one octave per page: the τ label in
seconds, then the value, e.g. `1 3.2e-11`, `64 3e-11`. (There's deliberately no `s` unit — the
7-segment S reads as a 5.)

Each octave only appears once enough contiguous locked seconds have accumulated to estimate it
honestly (four times the averaging factor), so the display starts as `Adev ----` and grows one octave
at a time; the full 1024 s octave needs a bit over an hour of lock. During holdover it shows
`Adev HOLd` rather than replaying a frozen curve. A gap in PPS restarts the measurement (a single
missed pulse is tolerated).

The companion serial commands `adev_dump` and `hdev_dump` print the whole curve — see USB serial
output below.

### Star transits

```
MODE_STAR = enabled
```

A bright-star meridian-transit predictor. A star culminates — crosses your local meridian at its
highest point — exactly when the Local Sidereal Time equals its right ascension. This mode pages
countdowns to the soonest bright stars to culminate, one star per page: under an hour it reads
minutes and seconds (`SIRI  1 35`), an hour or more switches to hours and minutes (`SIRI  2h15`).
When the soonest star reaches culmination the row holds `SIRI  NOW` for eight seconds. With no
position it shows `STAr ----`.

The catalogue is loaded at boot from `STARS.BIN` on the flash drive (93 naked-eye stars — see
Flash memory files below), trimmed by `star_max_mag`. The mode **requires** that file: without a
valid `STARS.BIN` the catalogue is empty and the row shows `STAr ----` — there is deliberately no
baked-in substitute. (The menu's ASTRO section shows the loaded count, so a bad file is obvious at a
glance.) Catalogue positions are J2000, corrected for proper motion and rigorously precessed to the
current date, so the shown minute stays right for decades — including for Polaris.

The companion serial command `star_dump` prints the whole list — see USB serial output below.

### Second timezone

```
MODE_ZONE2 = enabled
zone2 = America/New_York
```

A second civil timezone on the date row, while the time row keeps local time. The mode alternates on
a fixed eight-second cycle: two seconds of label, then six seconds of the remote clock. It is not one
of the paged modes, so `page_ms` has no effect on it.

The label page shows the tail of the IANA name after the last `/` (`America/New_York` reads
`New_York`), or a fixed offset verbatim, truncated to the row's ten characters. The clock page reads
`HH:MM:SS` and ticks with the time row, with a `+1` or `-1` marker appended when the remote calendar
day is ahead of or behind the local one (`19:04:37-1` is the previous day there). The reading is
computed from GPS-disciplined UTC and the zone's own transition table on every repaint, so it follows
that zone's daylight-saving changes on their own dates; no offset is stored.

`zone2` takes the same format as `ZONE_OVERRIDE`, and sits beside it in `config.txt`. An IANA name is
resolved from the same `tzrules.bin` the main zone uses, into a second transition table held
alongside it. A literal `UTC`, `GMT`, `+HH:MM` or `-HH:MM` (`+05:30`, or `-8` with the minutes left
off) resolves immediately without reading the drive, and has no daylight saving; offsets beyond 14
hours are rejected. The mode needs no position.

Leave `zone2` unset, or name a zone `tzrules.bin` does not carry, and the row shows a single dash.
`MODE_ZONE2` is also on the menu (CAL > ZONE 2); the zone target is config-only.

## New configuration parameters

All of these go in `config.txt` like any other parameter, and can also be typed over the USB serial
port. Several can additionally be changed from the on-device menu — those are marked, and the
precedence between the two is explained in the menu section.

### page_ms

```
page_ms = 5500
```

The dwell time per sub-screen, in milliseconds. Every date-row mode that pages honours it, including
`MODE_SUN`, `MODE_LATLON`, `MODE_TEMPCOMP`, `MODE_ADEV` and `MODE_STAR`. Default 5500, minimum 250.
Also settable from the menu (DISP > PAGE, shown in seconds).

### significance_fade

```
significance_fade = on
```

Replaces the fixed `Tolerance_time_*` ladder with measured uncertainty. When enabled, the clock
continuously computes a 3-sigma bound on its holdover time error — from the age and quality of the
last calibration, the measured residual of the learned temperature model, and aging — and each
sub-second digit fades out smoothly as the uncertainty passes through its place value, rather than
being dashed at a fixed time. A digit that is still significant keeps ticking at partial brightness
on its way out. At power-on the sub-second digits start dark and only light once the clock can prove
they're significant. Default off. Also settable from the menu (DISP > SIG FADE).

### seg_balance

```
seg_balance = on
```

Equalises per-segment brightness. The display is voltage-driven and all lit segments of a digit share
a common return path, so digits with fewer lit segments glow brighter — a `1` outshines an `8`,
worst at low brightness. `seg_balance` counters this in software by giving sparse digits
proportionally less duty, on both boards, with no change to the scan timing.

`on` (or `1`) applies a calibrated automatic curve that tracks the display brightness — this is the
intended setting. A number from 2 to 300 pins a fixed manual strength instead, for experiments:
values up to 100 are a linear blend (100 = exactly duty-proportional), and 101 to 300 overdrive it
with a power law, useful at the dimmest settings where the LED response is exponential and a linear
map under-corrects. Default off (stock behaviour). Requires a matrix frequency of at least ~4 kHz;
below that the balancing dither would visibly strobe and is disabled.

Balancing the date row requires the updated date-board firmware (`fwd.bin`); with a stock date board
only the time row is balanced.

### colon_balance

```
colon_balance = on
```

The colons are driven by their own PWM rail, not the segment scan, so out of the box they hold full
brightness while the digits dim around them. `colon_balance` ties the colon brightness to the display
brightness: `on` (or `1`) applies a calibrated curve — full brightness over the bright half of the
range, tapering linearly to a dim floor at the darkest — and a number from 2 to 256 pins a fixed
scale (of 256) for calibration sweeps. Default off. Leave it off until the digit brightness is
dialled in, then switch on.

Two anchors of the automatic curve are overridable per-hardware, exactly like the `BS` brightness
stops:

```
colon_full_at = 2048
colon_floor = 20
```

`colon_full_at` is the internal brightness level (DAC code, 0–4095, 0 = brightest) at or below which
the colons run at full scale; `colon_floor` is the minimum scale (of 256) reached at the dimmest
setting. The defaults are measured values from a production clock.

The menu's DISP > BALANCE toggle arms `seg_balance` and `colon_balance` together at their automatic
curves; the individual keys remain for calibration.

### colon_alt_mode

```
colon_alt_mode = alt_sawtooth
```

The colon animation used while an alternate-timebase mode (`MODE_LST` / `MODE_SOLAR`) is displayed,
so those can never be mistaken for civil time at a glance. Takes the same names as `colon_mode`. If
you don't set it, the firmware guarantees it differs from your civil colon automatically (defaulting
to alt_sawtooth, or toggle if that's your civil choice); setting it explicitly disables that
guarantee, including setting the two equal if you really want to. Also settable from the menu
(DISP > ACOLON).

### cuckoo

```
cuckoo = trust
```

Scheduled display flourishes on the time row. The cadence is intrinsic, after the Westminster
pattern: a fixed small gesture on each quarter-hour, and the piece named by this key on the hour.
There is no interval knob. Default `off`, which disables both tiers. `trust` is the only piece in
the shipped catalogue; every other value, including the parked piece names in `CUCKOO_SPEC.md`,
parses to `off`, so a misspelt name cannot leave a stale piece performing. Also settable from the
menu (DISP > CUCKOO), where each value performs on the real digits as you select it.

Nothing is ever written to the date row, so whatever it holds — the date, a star countdown, an ADEV
octave — keeps reading straight through a flourish, and the feature asks nothing of the date board.

The quarter gesture (:15, :30, :45) is the same every time and carries no data: a shallow brightness
trough sweeps the six big digits left to right, each digit starting 30 ms after the one to its left
and dipping to half brightness and back over 400 ms. It is finished about 0.6 s after the boundary.
The colons and the sub-second digits are untouched, and holdover changes nothing about it.

`trust`, the hour piece, shows how much of the displayed time the clock can currently stand behind.
The whole row dips to a dim x-ray for 0.3 s, then a relight wave walks HH → MM → SS → ds → cs → ms
at 80 ms per digit, each digit coming back to the confidence the `Tolerance_time_*` ladder holds for
it at that moment:

| digit | relights on |
|---|---|
| the six whole-second digits | full once the clock has seen a PPS pulse or came up on a good battery-backed RTC; a dim trace if it has had neither |
| deciseconds | the better of PPS age against `Tolerance_time_10ms` and RTC-calibration age against `Tolerance_time_100ms` |
| centiseconds | PPS age against `Tolerance_time_10ms` |
| milliseconds | PPS age against `Tolerance_time_1ms` |

Within a window the digit is full while the age is under half the tolerance, then rolls off linearly
to dark at the tolerance itself, which is where the ladder would dash it. A clock that has had
neither anchor leaves all three sub-second digits dark for the piece. With every digit full the row
closes on one unified pulse and the piece ends about 1.5 s after the hour; otherwise the wave dies
at the honest position, holds that picture, and the dark digits fade back up to the live face, about
1.9 s in all. Both endings finish on the plain live face, and a digit that `significance_fade` has
already extinguished is never relit by a flourish.

Flourishes are refused, and one in flight is abandoned, in `MODE_STANDBY`, in the countdown and text
displays, and while an alternate timebase drives the time row (`MODE_LST` / `MODE_SOLAR`). They also
need the same dither canvas as `seg_balance`, so a matrix frequency below ~4 kHz skips them rather
than showing them badly; `seg_balance` itself need not be on, since the mirror runs at identity duty
when it is off. With `cuckoo = off` the display path is byte-identical to stock.

```
cuckoo_preview = trust
```

Serial-only, and ignored in `config.txt` so that loading a file never makes the clock perform. Plays
one flourish immediately, without changing `cuckoo`: `nod` plays the quarter gesture, `trust` plays
the hour piece, `on` plays whatever `cuckoo` is currently set to (nothing, if that is `off`). A
second preview replaces one already running.

### star_max_mag

```
star_max_mag = 1.5
```

Trims the star transit catalogue to stars brighter than this apparent magnitude, applied at boot when
`STARS.BIN` is loaded. Smaller numbers mean fewer, more famous stars: 1.5 keeps roughly the two dozen
first-magnitude stars, 2.5 admits the whole supplied file. Default 6.0 (no trim).

### pps

```
pps = on
```

Outputs one `$PMTXTS` timing sentence over USB serial on each GPS PPS pulse, carrying the
sub-millisecond phase measurement made at the edge, for host-side jitter and drift analysis. When on,
the sentence also carries a USB SOF-correlation tail that lets a host place the edge on its own clock
to around a microsecond despite USB delivery jitter; a host that only needs the phase can ignore the
extra fields. Enabling this turns on the USB start-of-frame interrupt (a cheap 1 kHz latch). Default
off. Also settable from the menu (SYS > PPS MSG). The sentence format is documented under USB serial
output below.

### Temperature compensation

The clock's short-term timekeeping between PPS pulses, and its holdover when GPS is lost, are limited
by how the two crystals drift with temperature. While GPS-locked the clock can measure this for
itself: it has a die temperature sensor, a per-second phase measurement against the PPS, and the
existing 63-second RTC calibration. The temperature compensation system learns a ppm-versus-
temperature model for both oscillators, then uses it during a GPS outage to steer the display
timebase and to keep the battery RTC trimmed for the next power-up.

Everything defaults off; with none of the keys set the behaviour is stock.

```
tc_learn = on
tc_apply = on
tc_rtc = on
```

`tc_learn` accumulates (temperature, ppm) samples while locked and periodically refits the models.
`tc_apply` steers the display timebase from the model during holdover — the correction applied is the
temperature-driven *change* since the moment GPS was lost, so it engages glitch-free at zero and
grows only as the temperature moves. `tc_rtc` additionally trims the battery RTC's calibration
register from the model while GPS is absent, so the RTC hands over better time across a power loss.
The menu's DIAG > TEMPCOMP toggle arms `tc_learn`, `tc_apply` and `tc_persist` together — switch the compensator on from the front panel and its learned model persists across power-off.

```
tc_t0 = 40
tc_engage_s = 2
tc_max_ppm = 100
```

`tc_t0` is the model's centre temperature in degrees C — the polynomial's reference origin, not a
temperature the die necessarily reaches (default 40; a unit whose die runs 26–36 °C still uses 40 as
its origin). `tc_engage_s` is how many seconds of PPS absence before steering engages (minimum and
default 2). `tc_max_ppm` is a hard clamp on the applied correction (default 100).

```
tc_hse_b = 0.00227
tc_hse_c = -0.000412
tc_lse_a = 15.3113
tc_lse_b = -0.67777
tc_lse_c = -0.031479
```

(The values above are a real unit's learned model, exactly as its `tc_dump` printed them.)

Pasting coefficients freezes the model — the values are asserted and learning no longer changes them.
The main oscillator (HSE) model is used through temperature differences only, so it has no `a` term:
just slope (`ppm/°C`) and curvature (`ppm/°C²`) about `tc_t0`. The RTC crystal (LSE) model is
absolute ppm. You don't write these by hand: the `tc_dump` serial command prints the learned model in
exactly this form, ready to paste. Setting a coefficient to `nan` unfreezes that slot.

```
tc_seed = on
tc_seed_lo = 26
tc_seed_hi = 36
```

Warm start: with `tc_seed = on`, pasted coefficients load as an *evolving* starting point rather than
a freeze — the clock is compensated from the first second, keeps learning, and hands over to its own
data once that data is at least as rich as the seed. `tc_seed_lo` / `tc_seed_hi` record the seed's
temperature coverage (the model is never extrapolated beyond it); `tc_dump` prints them so a paste
round-trips. Over serial, send the coefficients first and `tc_seed = on` last — the order `tc_dump`
prints.

With `tc_persist = on` the learned model is also saved automatically to the settings store (internal
flash on 1 MB parts, `SETTINGS.BIN` on 256 KB parts) and warm-starts itself on every boot — so a
pasted seed block is no longer required for day-to-day power cycles. It is still worth keeping in
`config.txt` as the recovery point: a factory reset (or losing `SETTINGS.BIN`) wipes the stored
model, and the file's block is what re-seeds the clock immediately afterwards.

```
tc_persist = on
```

Automatically persists the learned model to a dedicated retained store (separate from `config.txt`
and the menu settings), so the very first holdover after a power-up is already compensated instead of
waiting to re-learn. Only a well-supported model is saved — enough samples, real temperature
coverage, a believable fit — at most once every 30 minutes and only when it has moved meaningfully,
so the flash lasts decades. On boot the stored model loads as an evolving seed; `config.txt` still
wins per-key, so pasted coefficients override it. On 1M-flash clocks the store lives in internal
flash; on 256K-flash clocks it lives inside `SETTINGS.BIN` (see Flash memory files below). Default
off.

## USB serial output

As before, the serial port accepts the same `key = value` lines as `config.txt`. The following
commands are serial-only — they're ignored if left in `config.txt`, so a pasted dump can't re-trigger
itself on every config load.

```
tc_dump = on
```
Prints the learned temperature model as ready-to-paste config lines (a comment header, `tc_t0`, the
`tc_hse_*` / `tc_lse_*` coefficients, and `tc_seed_lo` / `tc_seed_hi`), plus two checksummed
machine-readable sentences:

```
$PMTXTC,H,<n>,<tmin>,<tmax>,<b>,<c>,<V|->*CC
$PMTXTC,L,<n>,<a>,<b>,<c>,<V|->*CC
```

| field | meaning |
|---|---|
| `H` / `L` | which oscillator: H = main (HSE), L = RTC crystal (LSE) |
| `n` | lifetime sample count for that oscillator |
| `tmin`, `tmax` | (H only) learned die-temperature coverage, °C |
| `a` | (L only) absolute offset at `tc_t0`, ppm |
| `b`, `c` | slope ppm/°C and curvature ppm/°C² about `tc_t0` |
| `V` / `-` | model valid / no usable fit |

```
tc_reset = on
tc_forget = on
```
`tc_reset` clears the live learned data (a cold restart; with `tc_persist` on it also wipes the
retained copy). `tc_forget` erases only the retained flash copy, leaving live learning untouched.

```
adev_dump = on
hdev_dump = on
```
Each prints the current stability curve as one checksummed sentence. `$PMADEV` is the overlapping
Allan deviation; `$PMHDEV` is its Hadamard twin, whose third-difference kernel cancels linear
frequency drift (a temperature ramp or aging), so the long-τ octaves report the oscillator rather
than the ramp. Both share the shape:

```
$PMADEV,<epoch>,<tau0>,<valid>,<noct>,<sigma_0>,...,<sigma_noct-1>*CC
```

| field | meaning |
|---|---|
| `epoch` | Unix time the sentence was generated |
| `tau0` | base averaging time in seconds (currently always 1); τ<sub>k</sub> = tau0 × 2<sup>k</sup> |
| `valid` | contiguous 1 Hz phase samples behind the estimate (confidence) |
| `noct` | number of octaves that follow |
| `sigma_k` | σ<sub>y</sub>(τ<sub>k</sub>), fractional frequency, scientific notation |

```
star_dump = on
```
Prints the current soonest-transit list as one checksummed sentence:

```
$PMSTAR,<n>,<name>,<sec>,<alt>,<dir>,...*CC
```

| field | meaning |
|---|---|
| `n` | number of entries that follow (up to 8) |
| `name` | 4-character star name, per entry |
| `sec` | seconds until that star's next upper culmination |
| `alt` | altitude at culmination, whole degrees |
| `dir` | `S` or `N` — whether it culminates due south or due north of you |

```
menu_dump = on
```
Prints one line reporting where the on-device menu settings live: internal-flash-backed (settings
persist), QSPI `SETTINGS.BIN` (settings persist), or RAM-only with the reason (no `SETTINGS.BIN`, or
the file is fragmented or too small). Useful for checking whether your settings will survive a power
cycle without guessing which silicon your clock was built with.

```
factory_reset = on
```
Factory-resets the on-device menu: erases every setting changed with the two-button menu and
immediately re-reads `config.txt`, returning the clock to exactly what the file says. No reboot
needed. Note the on-device REBOOT chord only reboots — it keeps your menu settings; the on-device
equivalent of this command is SYS > RESET in the menu.

### The $PMTXTS timing sentence

With `pps = on`, one sentence per PPS pulse:

```
$PMTXTS,<seq>,<epoch>,<subms>,<systick>,<load>,<calerr>,<sincecal>,<temp>,<flags>[,<dwt_pps>,<sof_frame>,<dwt_sof>]*CC
```

| field | meaning |
|---|---|
| `seq` | increments every PPS edge; gaps reveal missed pulses |
| `epoch` | Unix time (UTC) at the edge |
| `subms` | the firmware's modelled millisecond-of-second at the edge, 0–999 |
| `systick` | SysTick counter value captured at the edge (a down-counter) |
| `load` | SysTick reload value (constant, e.g. 79999 at 80 MHz), sent so the host needn't assume the core clock |
| `calerr` | signed RTC crystal cycle error over the 63 s calibration period; ppm = calerr × 10⁶ / (32768 × 63) |
| `sincecal` | seconds since the last successful RTC calibration (holdover age) |
| `temp` | die temperature, °C |
| `flags` | hex: bit0 GPS data valid, bit1 PPS seen, bit2 RTC good |

The modelled sub-second position at the edge — the phase error against GPS — is
`subms + (load − systick)/(load + 1)` milliseconds.

The last three fields are the SOF-correlation tail, present once USB has enumerated: `dwt_pps` is a
free-running 80 MHz cycle count latched at the PPS edge, `sof_frame` the 11-bit USB frame number of
the most recent start-of-frame, and `dwt_sof` the cycle count at that SOF. A host that can read each
USB frame's own arrival time in hardware can place the PPS edge on its clock as
`hostTime(sof_frame) + (dwt_pps − dwt_sof)/f_dwt`, immune to the several milliseconds of USB read
jitter; successive `dwt_pps` deltas self-calibrate `f_dwt`. Parsers that only want the phase can
treat the sentence as 9 fields and ignore any extras.

## The on-device menu

With this firmware the two buttons also give you a settings menu, so the common adjustments no
longer need a computer. It needs the updated date-board firmware (`fwd.bin`) as well as the time-side
update: the date board is what reads the buttons, and the menu is driven by a small protocol it emits
when both buttons are held. With a stock date board the menu stays dormant and holding both buttons
keeps its old meaning (reset).

The menu never touches the time row — the big digits keep showing live time throughout. All menu text
appears on the 10-character date row.

### Entering the menu, and the chord

Hold both buttons together. After about 0.8 s the date row shows `SETUP`; release both buttons while
it's showing and you're in the menu.

Keep holding instead, and the label steps every 0.7 s through a short rolling cycle. Releasing fires
whatever is currently shown — so you read the label, and let go on the one you want. A stage showing
`----` is a harmless no-op buffer. What the stages offer depends on where you are:

- At the clock: `SETUP`, then `----`; on the *second* time around the cycle the third stage becomes
  `REBOOT` (about 4.3 s of holding) — this resets the clock, exactly like the old both-buttons hold,
  and deliberately can't be reached by a brief over-hold of the menu gesture.
- In the section list: `ENTER`, `EXIT`, `CLOCK`.
- In an item list: `EDIT`, `BACK` (to the sections), `CLOCK`.
- In a value editor: `APPLY`, `----`, `CANCEL` — commit and discard are separated by the buffer
  stage, so overshooting an APPLY can never silently discard the edit. On/off items show `DONE`
  instead (they save on any shallow release); the deep stage is still `CANCEL`.

The deep (third) stage is a consistent bail-out: from anywhere inside the menu it returns to the
clock.

Everywhere in the menu, the two buttons individually mean next / previous. Fifteen seconds without a
press abandons the menu and restores the clock row; about three seconds before that strikes, the row
flickers once as a warning. An abandoned edit is reverted, exactly like CANCEL.

### Layout

The menu is five sections; ENTER drops into the item list of the shown section, and re-entering the
menu resumes where you left off.

**CAL** — the calendar/date modes, each an on/off row: `ISO 8601`, `ISO ORD`, `ISO WEEK`, `UNIX`,
`JULIAN`, `MOD JD`, `UTC OFFS`, `TZ NAME`, `WEEKDAY`, `WKDAY DD`, `WDY MMDD`.

**ASTRO** — the astronomy modes: `SUN`, `SUN AZEL`, `MOON`, `GRID`, `LAT LON`, `LST`, `SOLAR`,
`STAR`, plus a read-only `STARS` row (below).

**DISP** — display settings: `BRIGHT` (fixed brightness override; scrub the raw 0–4095 level with
live preview, or step below zero for `AUTO` to return to the ambient sensor), `BALANCE` (the
segment + colon brightness equalisers, both at their automatic curves), `COLON` and `ACOLON` (the
civil and alternate-timebase colon animations: `SLOWFADE`, `HEARTBt`, `1PPS SAW`, `ALT SAW`,
`TOGGLE`, `FULL`), `PAGE` (the paged-mode dwell, shown in seconds), `SIG FADE`, and the `STANDBY`
mode row.

**DIAG** — diagnostics: `TEMPCOMP` (arms the temperature-compensation learn + apply + persist trio — this is the
enable; the read-out is the row beside it), `SATVIEW`, `ADEV`, `TC DATA` (the `MODE_TEMPCOMP`
read-out), `FW CRC`, `VBAT`, plus a read-only `PPS` row.

**SYS** — system: `PPS MSG` (the `$PMTXTS` timestamp sentence — not a hardware 1PPS output), `NMEA`
(`ALL` / `RMC` / `NONE`), `MATRIX` (the matrix frequency in kHz, floored at 8 kHz here to keep the
display flicker-free while you scrub; `config.txt` can still reach the 1 kHz hardware floor), and
`RESET` (factory reset, below).

### Editing values

EDIT opens the value editor. Every change applies live — brightness scrubs the real display, colon
choices preview on the real colons (even the alternate colon, whichever mode you're in), the matrix
frequency retunes as you step. APPLY keeps the value; CANCEL (or the idle timeout) puts everything
back.

Number editors accelerate: tap for single steps, hold for an auto-repeating run that doubles its
increment as it goes, snapping to round values so you can't sail past a target. Pause or reverse and
it's back to single steps. A resting value blinks slowly to say "you are setting this"; it stays
solid while you're actively scrubbing.

Trying to switch off the last enabled mode is refused with `LASt` — the clock never lets you strip
the mode ring bare.

### Info rows

Two rows are read-only live read-outs (they offer no EDIT):

- **DIAG > PPS** — `PPS LOCK` while disciplined, `HOLD 47` with the holdover age in seconds once
  pulses stop, `PPS ----` if no pulse has ever been seen.
- **ASTRO > STARS** — `STARS 93` reports how many catalogue entries loaded from `STARS.BIN`.
  `STARS 0` means no valid file — the transit mode shows `STAr ----` until one is provided.

### Factory reset

SYS > RESET erases every setting you've changed with the menu **and everything the clock has
learned** — the stored and live temperature-compensation model included — then immediately re-reads
`config.txt`, returning the clock to exactly what the file says (a `tc_*` seed block in the file
re-seeds compensation on the spot). It is the same thing as the `factory_reset` serial command. EDIT
opens a `DELETE ALL` confirm screen: APPLY fires it (the row flashes `DONE`), CANCEL backs out.
This is deliberately a different thing from the REBOOT chord, which restarts the clock but keeps
your settings.

### Where settings live

Menu changes take effect immediately and are written to persistent storage shortly after you return
to the clock face, so they survive power-off. Where they're stored depends on the silicon your clock
was built with:

- **1M-flash parts (STM32L476RG):** two spare pages of internal flash, above the region covered by
  the firmware CRC. Nothing on the USB drive is involved.
- **256K-flash parts (STM32L476RC):** there is no spare internal flash, so the store lives inside
  `SETTINGS.BIN` on the USB drive (see below). If that file is missing or unusable, settings still
  work but are RAM-only — lost at power-off.

`menu_dump = on` over serial reports which of these your clock is doing.

Only the settings you actually changed are recorded, so `config.txt` remains the source of truth for
everything else. When both have an opinion about the same key, the rule is: a key defined in
`config.txt` wins, *unless* the menu change was made against this exact version of the file — each
menu edit is stamped with the config file's modification time, and a stored override only beats the
file while the stamps still match. In practice: edit something on the device and it sticks; re-save
`config.txt` from your computer and the file reasserts whatever it defines. (If your host writes zero
FAT timestamps, the file always wins after a re-save.)

## Flash memory files

Two new files join `config.txt`, `tzrules.bin` and `tzmap.bin` on the USB drive.

### SETTINGS.BIN

On clocks built with 256K-flash silicon the firmware keeps its settings store (on-device menu changes
plus the retained temperature-compensation model) inside `SETTINGS.BIN`: a 16 KiB file whose four
4 KiB clusters the firmware rewrites in place through the flash driver, without ever touching the FAT
metadata. Rules:

- Create it **first** on a freshly formatted volume, so it is contiguous — the firmware verifies
  contiguity at boot and falls back to RAM-only settings (lost at power-off) if the file is
  fragmented, missing, or under 16 KiB.
- Fill it with `0xFF` (the erased state of the flash):
  `tr '\0' '\377' < /dev/zero | dd bs=4096 count=4 of=SETTINGS.BIN`. A zero-filled file also works —
  the firmware erases it on first use.
- Treat it as **opaque**: don't edit, copy over, or defragment it. If a host tool rewrites or moves
  it, the firmware detects this before its next write, re-resolves the file's location and
  re-initialises it (stored settings reset to the live values; nothing else on the volume is
  touched).

1M-flash clocks ignore the file entirely and use internal flash. `menu_dump = on` over serial tells
you which store is in use and, if it's RAM-only, why.

### STARS.BIN

The star-transit catalogue, generated by `generate-stars.py` in the qspi folder from the HYG v4.1
database (CC0): the naked-eye stars people actually recognise, default magnitude ≤ 2.5 — 93
stars, magnitude-sorted. Little-endian format:

| offset | size | field |
|---|---|---|
| 0 | 4 | magic `MST1` |
| 4 | 2 | record count (u16) |
| 6 | 2 | record length (u16, = 14) |
| 8 | 2 | magnitude scale (u16, = 100) |
| 10 | 6 | zero padding |

then `count` 14-byte records:

| offset | size | field |
|---|---|---|
| 0 | 2 | RA (u16): round(ra_hours / 24 × 65536) |
| 2 | 2 | Dec (i16): round(dec_degrees × 100) |
| 4 | 2 | magnitude (i16): round(mag × 100) — used only for the `star_max_mag` cut |
| 6 | 4 | name, 4 chars uppercase, space-padded, unique |
| 10 | 2 | proper motion in RA (i16, mas/yr, μ<sub>α</sub>* including cos δ) |
| 12 | 2 | proper motion in Dec (i16, mas/yr) |

Positions are J2000; the firmware applies proper motion and precession itself. Legacy 10-byte records
(the same layout without the two proper-motion fields) are also accepted, with proper motion treated
as zero. The loader validates the header and every record; a missing or malformed file — or a valid
one whose every record is trimmed away by `star_max_mag` — yields an empty catalogue, and the mode
shows `STAr ----`. There is no baked-in substitute.

To regenerate: `python3 generate-stars.py` writes `output/stars.bin` (set `STAR_MAG_CUT` in the
environment for a different magnitude cut); copy it to the drive as `STARS.BIN`. Names are chosen for
the 7-segment font — every letter renders, though I, O, S, Z read as 1, 0, 5, 2.

## Version numbers

The version train for this series: the time board is **0.0.5** (upstream's latest release being
0.0.4) and the date board is **0.0.2** — those are the versions the linked rollup images carry and
the versions this documentation describes.

Both boards' reported versions carry a per-build tag: `0.0.5+1783851034` / `0.0.2+1783835902`, where
the suffix is the build time in Unix seconds (a bare source checkout without the pre-build step
reports `+dev`). This exists because of how updates work: the bootloader only flashes an image whose
CRC differs from what's already loaded, so two builds of identical source made the same day could
otherwise produce byte-identical images and the "update" would be silently skipped. The tag makes
every build byte-unique — a reflash always takes — and lets you tell same-day builds apart from the
version string.
