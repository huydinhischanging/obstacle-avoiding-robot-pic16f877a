# Obstacle-Avoiding Robot — PIC16F877A

Bare-metal firmware in C for an autonomous obstacle-avoiding robot, built for
*Embedded Real-time Systems* at International University, VNU-HCM.

An HC-SR04 ultrasonic sensor measures the distance ahead; its echo pulse
width is captured by **edge interrupts on RB0/INT** against a free-running
Timer1 — nothing spins on the pin. Two digital IR sensors detect obstacles
left and right. A DRV8833 dual H-bridge drives the motors under PWM speed
control (Timer2 + CCP1/CCP2). A non-blocking state machine clocked by a 1 ms
Timer0 interrupt picks an avoidance manoeuvre when something comes within
20 cm, and a **USART telemetry stream** (9600 8N1) reports state, distance and
wheel speeds to a PC, accepting single-character commands back.

Every peripheral is interrupt-driven; the main loop only decides. The one
remaining busy-wait is the 10 µs HC-SR04 trigger pulse. No HAL, no framework —
the firmware writes directly to the SFRs.

## Status

| Version | Verification |
|---------|--------------|
| `src/main_original.c` | Original course submission — **ran on the physical robot**. Kept for reference; carries the defects listed under *Debugging notes*. |
| `src/main.c` (v3) | Compiles clean on XC8 v3.00; Timer0 tick and the state machine checked in the **MPLAB X simulator**. **Not yet run on hardware** — the v2→v3 pin moves need rewiring and the DRV8833 PWM polarity needs a bench check first. |

---

## Hardware

| Part | Role | Key specs |
|------|------|-----------|
| PIC16F877A | MCU | 8-bit, 20 MHz, 8 KB flash, 368 B RAM |
| HC-SR04 | Ultrasonic ranging | 2–400 cm, 40 kHz, ±3 mm |
| 2× IR sensor | Left / right obstacle detection | Digital out, 2–30 cm |
| DRV8833 | Dual H-bridge motor driver | 1.5 A per channel, thermal protection |

Full pin table: [`docs/pin-mapping.md`](docs/pin-mapping.md)

## Layout

```
Makefile             command-line build (Microchip XC8)
src/main.c           firmware (v3)
src/main_original.c  original submission, reference only (excluded from the build)
tools/telemetry.py   PC-side UART reader / logger / plotter
docs/pin-mapping.md  pin table and DRV8833 truth table
docs/report.pdf      full course report (not committed — see docs/README.md)
LICENSE              MIT
```

## Building

Needs the Microchip XC8 compiler (`xc8-cc`) and `make`:

```
winget install -e --id Microchip.MPLABXC8CCompiler
winget install -e --id ezwinports.make
```

Then, with both on `PATH`:

```
make            # -> build/obstacle-avoiding-robot.hex
make clean
```

Builds clean (0 warnings) on **XC8 v3.00** for `PIC16F877A` — ~1.9 k words of
flash (23 %), ~155 bytes of RAM (42 %). `-O2` in the `Makefile`; drop to `-O1`
if a licence mode rejects it.

Or in MPLAB X IDE: new **Application** project, device `PIC16F877A`, 20 MHz HS,
tool **Simulator**. Add **only** `src/main.c`. `src/main_original.c` defines
its own `main()` and is compiled out unless `BUILD_ORIGINAL` is defined
(`xc8-cc -DBUILD_ORIGINAL src/main_original.c`).

Telemetry viewer:

```
pip install pyserial
python tools/telemetry.py --port COM5            # live table
python tools/telemetry.py --port COM5 --csv run.csv --plot
```

Flashed over ICSP.

> **Compiles clean; verified in the MPLAB X simulator (Timer0 tick measured at
> 1.02 ms). Not yet run on real hardware.** Before trusting it: wire per
> `docs/pin-mapping.md` (note the v2→v3 pin moves), confirm DRV8833 PWM
> polarity, and bench-test with the wheels off the ground.

---

## How ranging works

The HC-SR04 returns distance as a pulse width on its echo line, so the
measurement is a timer-capture problem — and in v3 it is done the way a real
driver would, entirely in interrupts:

1. The main loop drives `TRIG` high for 10 µs, then arms the `INT` interrupt
   on RB0 for a **rising** edge.
2. Rising edge → ISR records the free-running Timer1 count, flips `INT` to
   **falling**.
3. Falling edge → ISR computes `now − start` (16-bit, exact across one wrap),
   publishes it, disarms `INT`.
4. The main loop converts the tick count to centimetres when it sees the
   published flag. It never waits.

The conversion, worked out from the clock tree rather than copied:

```
Fosc = 20 MHz  →  instruction clock = Fosc/4 = 5 MHz
Timer1 prescaler 1:4  →  1.25 MHz  →  0.8 µs per tick
Speed of sound  →  0.0343 cm/µs
Round trip, so:  distance_cm = ticks × 0.8 × 0.0343 / 2
                             = ticks × 0.01372
```

Timer1 is 16 bits — 65535 ticks ≈ 52.4 ms ≈ 449 cm round trip. A valid echo
(≤ 400 cm) is ~23 ms, well under that. The falling-edge handler classifies
each pulse: shorter than `ECHO_MIN_TICKS` is electrical noise (ignored);
longer than `ECHO_MAX_MS` (~25 ms) means **nothing is within range** — a
sentinel "far" distance, the robot drives on; in between is a real reading,
and because it is under one wrap `(uint16_t)(now − start)` is exact. A pulse
that never falls, or no edge at all, is a *miss*; only `FAULT_MISSES`
consecutive misses trip **FAULT**. See [FIX 7].

## Avoidance logic

A fresh ultrasonic reading is taken every ~60 ms (the HC-SR04's rated cycle);
the two IR lines are sampled every loop pass. The state machine has four
states — **CRUISE**, **BACK**, **TURN**, **FAULT**.

In **CRUISE**, the ultrasonic distance and the two IR flags pick the drive:

| Distance | Left IR | Right IR | Action |
|----------|---------|----------|--------|
| > 20 cm | clear | clear | Forward, both wheels 90 % |
| > 20 cm | blocked | clear | Steer right — left wheel 90 %, right 50 % |
| > 20 cm | clear | blocked | Steer left — right wheel 90 %, left 50 % |
| > 20 cm | blocked | blocked | Creep forward, both wheels 45 % |
| ≤ 20 cm | blocked | clear | → BACK, then pivot right |
| ≤ 20 cm | clear | blocked | → BACK, then pivot left |
| ≤ 20 cm | clear | clear | → BACK, then pivot left |
| ≤ 20 cm | blocked | blocked | → BACK, then **sharp** pivot right (dead end) |

**BACK** reverses at 80 % for 600 ms, or ends early (after a 200 ms minimum)
once the path ahead is clear by more than a 10 cm margin. **TURN** point-turns
at 85 % for 450 ms (900 ms for a dead end), then returns to CRUISE. None of
these manoeuvres blocks the loop — the robot keeps ranging throughout.

Only after several consecutive cycles with no echo edge at all does the robot
enter **FAULT**: motors braked, LED blinking at ~4 Hz, auto-recovering to
CRUISE as soon as any usable reading (near *or* far) comes back. A one-off
dropout, or an honest "nothing ahead", does not stop it.

A **`p`** command over UART toggles a paused hold (motors stopped, sensing and
telemetry continue, and the current manoeuvre's timer restarts on resume);
**`?`** forces an immediate status frame.

---

## Debugging notes

Reviewing the submitted firmware afterwards turned up seven defects. They are
listed here because working through them was the most useful part of the
project. Each is marked `[FIX n]` in `src/main.c`. `src/main_original.c` keeps the
original code for comparison — the only change to it is a `#ifdef
BUILD_ORIGINAL` guard so its `main()` does not collide with `src/main.c`
at link time.

### 1. Unbounded busy-wait on the echo line — the serious one

The original ranging routine was:

```c
while (Echo == 0);   /* wait for the echo to start */
TMR1ON = 1;
while (Echo == 1);   /* wait for the echo to end   */
```

Neither loop can ever exit if the pulse does not arrive. A disconnected
sensor, a loose jumper, or a surface that scatters the ping instead of
reflecting it will lock the main loop permanently — the robot freezes with
the motors still latched in whatever state they were last set to.

**Symptom that led to it:** the robot occasionally stopped dead and stayed
stopped, with no obvious pattern. Power-cycling always recovered it, which
pointed at a software lock-up rather than a hardware failure.

**Fix (v2):** both loops bounded by a millisecond deadline plus a Timer1
overflow check; the routine returns a status flag and the state machine goes
to FAULT on timeout.
**Fix (v3):** the wait is gone entirely. The echo is measured by rising/
falling edge interrupts on `RB0/INT` against a free-running Timer1, so the
"missing pulse" case is just an edge that never arrives — the main loop bounds
it with a millisecond deadline and only declares a fault after several
consecutive misses (a one-off dropout or an out-of-range "nothing ahead" does
not stop the robot).

*Lesson:* any wait on an external signal needs an exit condition. This is the
same class of bug as an I²C transfer with no NACK handling — and the better
fix is usually to not busy-wait at all.

### 2. Left IR input never configured

`docs` and code disagreed about the pin map. The firmware read `RB0` as the
left IR sensor but configured `TRISB1` as an *output*, leaving `TRISB0`
untouched. The input worked only because the port happens to reset to input
mode — the behaviour was correct by accident, not by configuration.

**Fix:** every input's `TRIS` bit is set explicitly in `system_init()`, and
the pin table in `docs/pin-mapping.md` is kept in step with the code. (In v3
the left IR sensor is on RB3 — RB0 became the `INT` echo-capture pin.)

### 3. Signed 16-bit timer read

```c
int time_taken = (TMR1L | (TMR1H << 8));
```

On XC8, `int` is a signed 16-bit type, so a high byte above 0x7F shifts into
the sign bit and yields a negative pulse width. In practice that only occurs
past ~360 cm, beyond the threshold, so it never showed up in testing — but it
is wrong.

**Fix:** `uint16_t` throughout, with `TMR1L` read before `TMR1H`.

### 4. Floating-point maths on an MCU with no FPU

`distance = (0.0272 * time_taken) / 2;` pulls the XC8 software floating-point
library into an 8 KB part and costs hundreds of instruction cycles per call.

**Fix:** integer arithmetic — `(ticks * 137 + 5000) / 10000` — using a
`uint32_t` intermediate so the multiply cannot overflow. The constant is
derived from the clock tree above rather than taken on faith, and the `+ 5000`
rounds to the nearest centimetre instead of truncating (a true 20 cm now
reads 20, not 19).

### 5. Five measurements per loop iteration

The original called `calculate_distance()` five times per pass, once before
each `if`. Every call blocks for up to ~24 ms waiting on the echo, so the
robot spent well over 100 ms per iteration re-measuring the same distance and
reacted late to anything that appeared quickly.

**Fix:** at most one measurement per 60 ms cycle, cached in `dist` and reused
by every state of the machine instead of re-triggered before each test.

### 6. No spacing between ultrasonic measurements

Nothing paced the HC-SR04. Re-triggering it immediately after the previous
echo violates its ~60 ms measurement cycle: the tail of the previous 40 kHz
burst, or a late echo bouncing off the room, lands in the next measurement
window and reads as a spuriously short distance — a phantom obstacle.

**Fix:** measurements are gated to one per `MEAS_PERIOD_MS` (60 ms) against
the millisecond time base — a non-blocking replacement for the interim
`__delay_ms(60)`.

### 7. Timer1 overflow never checked

`calculate_distance()` read `TMR1L | (TMR1H << 8)` unconditionally. Timer1 is
16-bit at 0.8 µs/tick, so it rolls over after 52.4 ms. Any echo longer than
that — a target out of range, a stuck-high line, a clone module that holds
the echo — wraps the counter and yields a *small* number, i.e. a phantom
obstacle right in front of the robot. The original argued this "can't happen
for a valid measurement", which is true but only covers the valid case.

**Fix:** v2 polled `TMR1IF` in the echo-low wait. v3 has no wait — the
falling-edge ISR gates on a coarse millisecond deadline (`ECHO_MAX_MS`,
~25 ms ≈ 430 cm). A longer pulse is reported as "nothing in range" (the robot
keeps driving), not a bogus short distance; every *accepted* pulse is under
one Timer1 wrap, so the 16-bit subtraction is exact and long echoes never
reach the tick-to-centimetre macro. Reviewer catch: an earlier cut of v3
treated "beyond range" as a fault, which parked the robot in open space —
now split from the real (repeated-miss) fault path.

---

## Upgrades (v2)

The three items that were left open after the debugging pass are now done.
`src/main.c` is a fuller rewrite; `src/main_original.c` remains the reference.

### Non-blocking state machine

Every `__delay_ms()` in the control path is gone. Timer0 raises a 1 ms
interrupt that advances `g_ms`, and that is the firmware's only clock. Reverse
and turn manoeuvres are `CRUISE → BACK → TURN → CRUISE` state transitions
timed against `g_ms` instead of `__delay_ms(500..1000)`, so the loop keeps
ranging while the robot moves and a reverse can be cut short the moment the
way is clear. The only remaining blocking waits are the 800 ms power-up blink
and the echo-line polls inside a single measurement.

### PWM speed control

Timer2 with CCP1 (RC2) and CCP2 (RC1) generates a ~5 kHz PWM on each motor's
`IN1` line; `IN2` stays a plain direction bit. `motor_drive(side, dir, pct)`
takes a 0–100 % speed. This is what makes the *steer* and *creep* rows in the
table above possible — the robot now curves around a side obstacle instead of
stopping and pivoting for every one. Reverse uses slow-decay (brake/reverse)
chopping, so its duty is inverted internally; see the comment in the pin map.

**Wiring change from v1:** motor control moved from four plain lines
(RC4–RC7) to two PWM + two direction lines (RC1/RC2 + RC4/RC5). Rewire per
`docs/pin-mapping.md` before flashing v2.

### One interrupt, and an honest report

The firmware now genuinely uses interrupts, so the course report's
"interrupts: enabled" line is finally accurate.

---

## Upgrades (v3)

v2 removed the blocking manoeuvre delays but a measurement still spun on the
echo pin. v3 finishes the job and adds a host-side interface.

### Interrupt-driven echo capture

`ECHO` moved from RB3 to **RB0/INT**. The pulse is timed by a rising-edge
interrupt (latch Timer1, switch to falling) and a falling-edge interrupt
(classify, publish, disarm) against a **free-running Timer1**. The
falling-edge handler splits the pulse three ways — noise / in-range reading /
beyond range — and the main loop counts consecutive missing edges, faulting
only after several. The ISR is a leaf function (no calls) so it costs exactly
one hardware-stack level. Zero busy-wait: the main loop fires a trigger and
reads a flag ~1 ms later.

> CCP capture is the textbook method for pulse width, but both CCP modules are
> committed to motor PWM. The `INT` pin with edge-polarity swapping gives the
> same result — non-blocking, hardware-timestamped — without giving up a PWM
> channel. Same technique you'd use for an unbuffered UART break or a tacho
> input when the capture peripherals are spoken for.

### UART telemetry + PC tool

USART on RC6/RC7, 9600 8N1. TX is an **interrupt-driven ring buffer** (the
main loop never blocks on `TXIF`); RX is a one-character command interrupt
with `OERR`/`FERR` recovery. A frame every 100 ms:

```
t=12840 st=1 pz=0 d=18 L=1 R=0 vL=80 vR=80
```

`tools/telemetry.py` (pyserial) prints a live table, logs CSV, optionally
plots distance and state, and forwards `p` / `?` commands typed at the
console.

### v2→v3 pin moves

| Signal | v2 | v3 |
|--------|----|----|
| HC-SR04 ECHO | RB3 | **RB0 / INT** |
| Left IR | RB0 | **RB3** |
| UART TX / RX | — | **RC6 / RC7** |

Motor pins (RC1/RC2 PWM, RC4/RC5 direction), TRIG (RB2), right IR (RB4) and
the LED (RD2) are unchanged from v2.

### Still open

- **Reverse speed control is coarse.** Slow-decay chopping makes low reverse
  duties non-linear; fine for a fixed 80 % escape move, not for fine control.
- **No wheel feedback.** Turn and reverse amounts are open-loop time. Encoders
  or an IMU would let manoeuvres be specified in degrees / centimetres.
- **No framed protocol.** Telemetry is plain text with a checksum-free line
  terminator — fine for a bench view, not for a noisy link.

---

## Team

Course project, four members. My contribution: firmware — HC-SR04 timing
routine, Timer1 configuration, motor control, and the avoidance decision
logic.

## License

MIT — see [`LICENSE`](LICENSE).
