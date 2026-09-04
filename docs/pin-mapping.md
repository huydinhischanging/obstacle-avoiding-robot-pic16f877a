# Pin mapping — PIC16F877A

Oscillator: 20 MHz crystal, HS mode. Instruction clock: 5 MHz.

Timer / peripheral use:

| Block | Role | Setup |
|-------|------|-------|
| Timer0 | 1 ms system tick (interrupt) | Fosc/4, prescaler 1:32, reload 100 |
| Timer1 | free-running time base for echo capture | Fosc/4, prescaler 1:4 → 0.8 µs/tick, no IRQ |
| Timer2 | PWM time base for CCP1/CCP2 | Fosc/4, prescaler 1:4, PR2 = 249 → ~5 kHz |
| INT (RB0) | HC-SR04 echo edge capture | edge polarity flipped rising↔falling in the ISR |
| USART | telemetry / commands | 9600 8N1 async, SPBRG = 129, BRGH = 1 |

## Sensors

| Pin | Direction | Connected to | Notes |
|-----|-----------|--------------|-------|
| RB2 | Output | HC-SR04 `TRIG` | 10 µs pulse starts a measurement |
| RB0 / INT | Input | HC-SR04 `ECHO` | pulse width timed by INT rising/falling edge IRQs |
| RB3 | Input | Left IR sensor | Digital, active low |
| RB4 | Input | Right IR sensor | Digital, active low |

> **Changed in v3.** ECHO moved from RB3 to RB0/INT so the pulse can be
> measured by edge interrupts instead of a busy-wait; the left IR sensor took
> the freed RB3. (v1 also had a real bug here — it configured `TRISB1` while
> reading `RB0`.)

## Motor driver (DRV8833, dual H-bridge)

One PWM line + one direction line per motor.

| Pin | Function | Direction | Connected to | Motor |
|-----|----------|-----------|--------------|-------|
| RC2 | CCP1 PWM | Output | `AIN1` | Left |
| RC4 | direction | Output | `AIN2` | Left |
| RC1 | CCP2 PWM | Output | `BIN1` | Right |
| RC5 | direction | Output | `BIN2` | Right |

> Unchanged since v2. v1 used four plain direction lines on RC4–RC7 with no
> speed control.

DRV8833 per-channel truth table:

| IN1 | IN2 | Result |
|-----|-----|--------|
| 0 | 0 | Coast |
| 1 | 0 | Forward |
| 0 | 1 | Reverse |
| 1 | 1 | Brake |

The firmware chops `IN1` with PWM and holds `IN2` as the direction bit:

| IN2 | IN1 (PWM duty d) | Effect |
|-----|------------------|--------|
| 0 | d | Forward at `d` (fast decay: forward ↔ coast) |
| 1 | d | Reverse at `100 − d` (slow decay: brake ↔ reverse) |
| 1 | 100 % | Brake |
| 0 | 0 % | Coast |

So `motor_drive()` inverts the duty for reverse. Forward speed is linear in
`d`; reverse speed is not, which is why reverse is only ever used at a fixed
escape speed.

## Telemetry (USART)

| Pin | Direction | Connected to |
|-----|-----------|--------------|
| RC6 / TX | Output (peripheral) | to PC RX (via USB-serial / level shifter) |
| RC7 / RX | Input (peripheral) | from PC TX |

`TRISC6` / `TRISC7` are left as inputs — the USART drives the pins once
`SPEN = 1`. 9600 8N1. Output: one `t=… st=… pz=… d=… L=… R=… vL=… vR=…` line
every 100 ms. Input: `p` toggles pause, `?` forces a frame.

## Indicator

| Pin | Direction | Connected to |
|-----|-----------|--------------|
| RD2 | Output | Status LED |

LED behaviour:
- One ~800 ms blink at power-up (system initialised)
- Solid on in CRUISE (forward / steering / creeping)
- Off during a BACK or TURN manoeuvre
- ~4 Hz blink in FAULT (echo sensor not responding)
- ~2 Hz blink while paused (`p` command)

## Programming

ICSP header: `MCLR`, `PGD` (RB7), `PGC` (RB6), `VDD`, `VSS`.
