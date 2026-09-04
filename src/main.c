/*
 * main.c
 *
 * Obstacle-avoiding robot firmware — v3.
 *
 * Target    : PIC16F877A @ 20 MHz (HS oscillator)
 * Toolchain : MPLAB X IDE + XC8 v3.x
 * Programmer: ICSP
 *
 * Sensing   : HC-SR04 ultrasonic — echo pulse width measured by edge
 *             interrupts on RB0/INT against a free-running Timer1.
 *             No blocking wait: the main loop fires a trigger and reads the
 *             result the capture ISR leaves behind.
 *             2x digital IR sensors (left / right).
 * Actuation : DRV8833 dual H-bridge, PWM speed control (Timer2 + CCP1/CCP2).
 * Telemetry : USART 9600 8N1 on RC6/RC7 — status frames out (interrupt-driven
 *             TX ring buffer), single-character commands in.
 * Timebase  : Timer0, ~1 ms tick interrupt — the firmware's only clock.
 *
 * Everything is interrupt-driven; the main loop only decides. The one
 * remaining busy-wait is the 10 us HC-SR04 trigger pulse.
 *
 * [FIX n] notes refer to defects in src/main_original.c. The v2/v3 rework
 * is described in README.md under "Upgrades".
 */

#include <xc.h>
#include <stdint.h>

/* ---------------- Configuration bits ---------------- */

#pragma config FOSC  = HS       // High-speed crystal oscillator
#pragma config WDTE  = OFF      // Watchdog timer disabled
#pragma config PWRTE = ON       // Power-up timer enabled
#pragma config BOREN = ON       // Brown-out reset enabled
#pragma config LVP   = OFF      // Low-voltage programming disabled.
                               // Required: with LVP on, RB3 is the PGM pin
                               // and cannot be used as the IR-left input.
#pragma config CPD   = OFF      // Data EEPROM protection off
#pragma config WRT   = OFF      // Flash write protection off
#pragma config CP    = OFF      // Code protection off

#define _XTAL_FREQ 20000000UL

/* ---------------- Pin map ----------------
 * See docs/pin-mapping.md for the full table.
 *
 * [v3] ECHO moved to RB0/INT (edge-interrupt capture). IR-left moved off
 * RB0 to RB3 (freed when ECHO left it). UART added on RC6/RC7.
 */

#define ECHO_PIN        RB0     // HC-SR04 echo  -> INT external interrupt
#define TRIG_PIN        RB2     // HC-SR04 trigger (output)
#define IR_LEFT         RB3     // Left  IR sensor (input, active low)
#define IR_RIGHT        RB4     // Right IR sensor (input, active low)
#define STATUS_LED      RD2     // Status LED (output)

/* Motor driver — DRV8833, one PWM line + one direction line per motor.
 *   Left  motor : IN1 = RC2 / CCP1 (PWM),  IN2 = RC4 (direction)
 *   Right motor : IN1 = RC1 / CCP2 (PWM),  IN2 = RC5 (direction)
 *
 * IN1 is chopped by PWM, IN2 is the direction bit:
 *   IN2=0, duty d -> forward at d          (fast decay: forward / coast)
 *   IN2=1, duty d -> reverse at (100 - d)  (slow decay: brake / reverse)
 * so motor_drive() inverts the duty for reverse.
 */
#define MOTOR_L_IN2     RC4
#define MOTOR_R_IN2     RC5

/* UART: RC6 = TX, RC7 = RX (peripheral drives the pins; TRIS set to input). */

/* ---------------- Tuning constants ---------------- */

#define THRESHOLD_CM        20u     // Obstacle distance threshold
#define CLEAR_HYST_CM       10u     // Extra clearance before a reverse ends early

/* Motor speeds, percent of full scale (0..100). */
#define SPEED_CRUISE       90u
#define SPEED_VEER_OUT     90u
#define SPEED_VEER_IN      50u
#define SPEED_CREEP        45u
#define SPEED_REVERSE      80u
#define SPEED_TURN         85u

/* Manoeuvre durations, milliseconds. */
#define T_REVERSE_MS      600u
#define T_REVERSE_MIN_MS  200u
#define T_TURN_MS         450u
#define T_SHARP_TURN_MS   900u

#define MEAS_PERIOD_MS    60u       // HC-SR04 rated measurement cycle
#define ECHO_TIMEOUT_MS   60u       // no edge at all within this -> sensor fault
#define TELEMETRY_MS      100u      // status frame interval

/* Distance conversion. Timer1 @ Fosc/4 with 1:4 prescaler = 0.8 us/tick.
 * distance_cm = ticks * 0.8 * 0.0343 / 2 = ticks * 0.01372  ~=  ticks * 137/10000
 * [FIX 4] Integer maths, no float library. The + 5000 rounds to nearest cm.
 */
#define TICKS_TO_CM(t)  ((uint16_t)(((uint32_t)(t) * 137UL + 5000UL) / 10000UL))

/* ~1 ms tick: 5 MHz / 32 / (256 - 100) ~= 1001 Hz. Writing TMR0 clears the
 * prescaler, so the partial count is lost on every reload -> the tick runs
 * ~1-2 % long (measured 1.02 ms in the simulator). Fine for manoeuvre timing;
 * do NOT integrate this tick for odometry without correcting for it.
 */
#define TMR0_RELOAD      100u
#define PWM_FULL         1000u      // 4 * (PR2 + 1) = 4 * 250

/* SPBRG for 9600 baud, BRGH = 1, Fosc = 20 MHz:
 *   SPBRG = Fosc / (16 * baud) - 1 = 20e6 / 153600 - 1 = 129  (actual 9615, +0.16%)
 */
#define UART_BAUD        9600UL
#define SPBRG_VALUE      ((uint8_t)(_XTAL_FREQ / (16UL * UART_BAUD) - 1UL))

/* ================================================================= *
 *  Millisecond time base (Timer0)
 * ================================================================= */

static volatile uint16_t g_ms;     // wraps every ~65 s; used only in deltas

/* ================================================================= *
 *  HC-SR04 capture (RB0/INT edge interrupts, free-running Timer1)
 * ================================================================= */

typedef enum { ECHO_IDLE, ECHO_WAIT_RISE, ECHO_WAIT_FALL } echo_state_t;

/* A valid 400 cm echo is ~23 ms. ECHO_MAX_MS is a coarse (ms-resolution)
 * upper gate that also keeps every accepted pulse well under one Timer1 wrap
 * (52 ms), so (uint16_t)(now - start) is always exact.
 *
 * A pulse LONGER than the gate means nothing is within range — that is the
 * good case for an obstacle-avoider, reported as echo_far (drive on), NOT a
 * fault. A pulse SHORTER than ECHO_MIN_TICKS is electrical noise, ignored.
 * A real fault (dead sensor / stuck line) shows up as repeated "no edge at
 * all" and is only declared after FAULT_MISSES consecutive misses.
 */
#define ECHO_MAX_MS      25u        // > this ms high  -> nothing in range (far)
#define ECHO_MIN_TICKS   60u        // < this (~0.5 cm) -> boundary noise, ignore
#define DIST_FAR_CM      999u       // sentinel distance for "clear ahead"
#define FAULT_MISSES     4u         // consecutive no-edge cycles before FAULT

static volatile echo_state_t echo_state = ECHO_IDLE;
static volatile uint16_t     echo_t0;       // Timer1 at the rising edge
static volatile uint16_t     echo_rise_ms;  // g_ms at the rising edge
static volatile uint16_t     echo_width;    // last good pulse width, in ticks
static volatile uint8_t      echo_ready;    // 1 = echo_width holds a fresh value
static volatile uint8_t      echo_far;      // 1 = pulse longer than range (clear)
static volatile uint8_t      echo_noise;    // 1 = pulse shorter than minimum (ignore)

/* ================================================================= *
 *  UART telemetry — interrupt-driven TX ring buffer, 1-char RX
 * ================================================================= */

#define TXBUF_SZ  64u              /* power of two -> mask instead of modulo */
static volatile uint8_t  txbuf[TXBUF_SZ];
static volatile uint8_t  tx_head, tx_tail;
static volatile uint8_t  rx_cmd;           // last received command char, 0 = none

static void uart_putc(uint8_t c) {
    uint8_t next = (uint8_t)((tx_head + 1u) & (TXBUF_SZ - 1u));
    if (next == tx_tail) return;           // full: drop (telemetry is non-critical)
    txbuf[tx_head] = c;
    tx_head = next;
    TXIE = 1;                              // make sure the drain ISR is running
}

static void uart_puts(const char *s) {
    while (*s) uart_putc((uint8_t)*s++);
}

static void uart_put_u16(uint16_t v) {
    uint8_t d[5];
    int8_t  n = 0;
    if (v == 0) { uart_putc('0'); return; }
    while (v) { d[n++] = (uint8_t)('0' + (v % 10u)); v /= 10u; }
    while (n) uart_putc(d[--n]);
}

/* ================================================================= *
 *  Interrupt service — single vector, leaf function (no calls, so it
 *  costs exactly one hardware-stack level on a part with no overflow trap)
 * ================================================================= */

void __interrupt() isr(void) {

    /* --- HC-SR04 echo edge on RB0/INT (most timing-sensitive) --- */
    if (INTE && INTF) {
        /* coherent 16-bit read of the free-running Timer1 (no read latch) */
        uint8_t  h, l, h2;
        uint16_t t1;
        do { h = TMR1H; l = TMR1L; h2 = TMR1H; } while (h != h2);
        t1 = (uint16_t)(((uint16_t)h << 8) | l);

        if (echo_state == ECHO_WAIT_RISE) {
            echo_t0      = t1;
            echo_rise_ms = g_ms;
            echo_state   = ECHO_WAIT_FALL;
            INTEDG       = 0;            // next edge: falling
        }
        else if (echo_state == ECHO_WAIT_FALL) {
            uint16_t w    = (uint16_t)(t1 - echo_t0);           // exact (pulse < 1 wrap)
            uint16_t held = (uint16_t)(g_ms - echo_rise_ms);    // coarse ms, range gate
            if (w < ECHO_MIN_TICKS)        echo_noise = 1;      // sub-minimum: noise
            else if (held > ECHO_MAX_MS)   echo_far   = 1;      // beyond range: clear
            else { echo_width = w;         echo_ready = 1; }
            echo_state = ECHO_IDLE;
            INTE       = 0;              // disarm until the next trigger
            INTEDG     = 1;              // back to rising
        }
        INTF = 0;                        // clear after any INTEDG change
    }

    /* --- 1 ms system tick --- */
    if (T0IE && T0IF) {
        TMR0 = TMR0_RELOAD;
        T0IF = 0;
        g_ms++;
    }

    /* --- UART TX: drain the ring buffer --- */
    if (TXIE && TXIF) {
        if (tx_head == tx_tail) {
            TXIE = 0;                    // empty: stop asking for TXIF
        } else {
            TXREG   = txbuf[tx_tail];
            tx_tail = (uint8_t)((tx_tail + 1u) & (TXBUF_SZ - 1u));
        }
    }

    /* --- UART RX: latch one command char --- */
    if (RCIE && RCIF) {
        if (OERR)      { CREN = 0; CREN = 1; }   // clear + flush on overrun
        else if (FERR) { (void)RCREG; }          // framing error: discard
        else           { rx_cmd = RCREG; }
    }
}

/* Consistent 16-bit read of a value the ISR updates byte-by-byte. */
static uint16_t millis(void) {
    uint16_t a, b;
    do { a = g_ms; b = g_ms; } while (a != b);
    return a;
}

/* ================================================================= *
 *  Motor control
 * ================================================================= */

typedef enum { M_COAST, M_FWD, M_REV, M_BRAKE } mdir_t;

static uint8_t g_speed_l, g_speed_r;      // last commanded speeds, for telemetry

static void left_pwm(uint16_t duty) {
    CCPR1L  = (uint8_t)(duty >> 2);
    CCP1CON = (uint8_t)(0x0C | ((duty & 0x03u) << 4));
}

static void right_pwm(uint16_t duty) {
    CCPR2L  = (uint8_t)(duty >> 2);
    CCP2CON = (uint8_t)(0x0C | ((duty & 0x03u) << 4));
}

static void motor_drive(uint8_t left, mdir_t dir, uint8_t pct) {
    uint16_t duty;
    uint8_t  in2;

    if (pct > 100u) pct = 100u;

    switch (dir) {
        case M_FWD:   in2 = 0; duty = (uint16_t)pct * 10u;             break;
        case M_REV:   in2 = 1; duty = PWM_FULL - (uint16_t)pct * 10u;  break;
        case M_BRAKE: in2 = 1; duty = PWM_FULL;                        break;
        case M_COAST:
        default:      in2 = 0; duty = 0u;                              break;
    }

    if (left) {
        MOTOR_L_IN2 = (in2 != 0u);
        left_pwm(duty);
        g_speed_l = (dir == M_FWD || dir == M_REV) ? pct : 0u;
    } else {
        MOTOR_R_IN2 = (in2 != 0u);
        right_pwm(duty);
        g_speed_r = (dir == M_FWD || dir == M_REV) ? pct : 0u;
    }
}

static void drive_forward(uint8_t l_pct, uint8_t r_pct) {
    motor_drive(1, M_FWD, l_pct);
    motor_drive(0, M_FWD, r_pct);
}
static void drive_reverse(uint8_t pct) {
    motor_drive(1, M_REV, pct);
    motor_drive(0, M_REV, pct);
}
static void drive_stop(void) {
    motor_drive(1, M_BRAKE, 0);
    motor_drive(0, M_BRAKE, 0);
}
static void pivot_right(uint8_t pct) {     // left forward, right reverse
    motor_drive(1, M_FWD, pct);
    motor_drive(0, M_REV, pct);
}
static void pivot_left(uint8_t pct) {      // left reverse, right forward
    motor_drive(1, M_REV, pct);
    motor_drive(0, M_FWD, pct);
}

/* ================================================================= *
 *  Ultrasonic — non-blocking
 * ================================================================= */

/* Fire a 10 us trigger and arm the INT capture. The rising/falling edges
 * are timed by the ISR; the result appears in echo_ready / echo_far / echo_noise.
 * [FIX 1] Replaces the original unbounded `while (Echo == ...)` busy-waits.
 */
static void hcsr04_fire(void) {
    echo_ready = 0;
    echo_far   = 0;
    echo_noise = 0;
    echo_state = ECHO_WAIT_RISE;

    TRIG_PIN = 1;
    __delay_us(10);                        // the only remaining busy-wait
    TRIG_PIN = 0;

    /* Arm AFTER the pulse: the real echo can't rise for ~0.5 ms, so a noise
     * edge in the arming window can't be mistaken for the echo start. */
    INTEDG = 1;                            // rising edge first
    INTF   = 0;
    INTE   = 1;
}

/* ================================================================= *
 *  Telemetry frame
 * ================================================================= */

static void telemetry_send(uint8_t st, uint8_t paused, uint16_t d,
                           uint8_t L, uint8_t R) {
    uart_puts("t=");   uart_put_u16(millis());
    uart_puts(" st="); uart_put_u16(st);
    uart_puts(" pz="); uart_putc((uint8_t)('0' + paused));
    uart_puts(" d=");  uart_put_u16(d);
    uart_puts(" L=");  uart_putc((uint8_t)('0' + L));
    uart_puts(" R=");  uart_putc((uint8_t)('0' + R));
    uart_puts(" vL="); uart_put_u16(g_speed_l);
    uart_puts(" vR="); uart_put_u16(g_speed_r);
    uart_putc('\r');
    uart_putc('\n');
}

/* ================================================================= *
 *  Initialisation
 * ================================================================= */

static void system_init(void) {
    /* Port B: INT + IR inputs, TRIG output.
     * [FIX 2] Every input pin is configured explicitly.
     */
    TRISB0 = 1;     // ECHO -> INT
    TRISB2 = 0;     // TRIG
    TRISB3 = 1;     // IR left
    TRISB4 = 1;     // IR right

    /* Port C: PWM (RC1/RC2), direction (RC4/RC5), UART (RC6/RC7 as inputs). */
    TRISC1 = 0;
    TRISC2 = 0;
    TRISC4 = 0;
    TRISC5 = 0;
    TRISC6 = 1;
    TRISC7 = 1;
    MOTOR_L_IN2 = 0;
    MOTOR_R_IN2 = 0;

    TRISD2 = 0;     // status LED

    /* PORTA/PORTE reset to all-analog; force digital so a future RA/RE pin
     * (mode switch, 2nd LED) doesn't silently read 0. No pins used there now. */
    ADCON1 = 0x06;  // PCFG = 0110 -> all digital

    /* Timer1: free-running time base for the echo capture.
     * Internal clock (Fosc/4), 1:4 prescaler -> 0.8 us/tick, running.
     * No overflow interrupt: the ms gate in the ISR keeps every accepted
     * pulse under one wrap, and the main loop bounds the "no edge" case.
     */
    T1CON = 0x21;   // T1CKPS=10 (1:4), TMR1CS=0 (Fosc/4), TMR1ON=1

    /* Timer2 + CCP1/CCP2: ~5 kHz PWM. */
    PR2     = 249;
    T2CON   = 0x05;   // TMR2 on, prescale 1:4
    CCPR1L  = 0;
    CCPR2L  = 0;
    CCP1CON = 0x0C;   // PWM mode, 0 % duty
    CCP2CON = 0x0C;

    /* Timer0: ~1 ms tick.
     * OPTION_REG: nRBPU=1, INTEDG=1 (rising), T0CS=0 (Fosc/4), PSA=0, PS=1:32
     */
    OPTION_REG = 0xC4;
    TMR0 = TMR0_RELOAD;

    /* USART: 9600 8N1, async. */
    SPBRG = SPBRG_VALUE;
    BRGH  = 1;
    SYNC  = 0;
    SPEN  = 1;
    TXEN  = 1;
    CREN  = 1;

    /* Interrupts: Timer0 tick, INT pin (echo), UART RX. TX enabled on demand. */
    T0IF = 0; T0IE = 1;
    INTF = 0; INTE = 0;         // armed per-measurement by hcsr04_fire()
    RCIF = 0; RCIE = 1;
    PEIE = 1;                   // RCIE / TXIE are peripheral interrupts
    GIE  = 1;

    drive_stop();

    /* One long blink at power-up — the only blocking delay in the firmware. */
    STATUS_LED = 1;
    __delay_ms(800);
    STATUS_LED = 0;

    uart_puts("obstacle-robot v3 ready\r\n");
}

/* ================================================================= *
 *  Main loop — CRUISE / BACK / TURN / FAULT state machine
 * ================================================================= */

typedef enum { ST_CRUISE, ST_BACK, ST_TURN, ST_FAULT } state_t;

void main(void) {
    state_t  state    = ST_CRUISE;
    uint16_t t_state  = 0;         // when the current state began
    uint16_t t_led    = 0;         // LED blink timer (separate from t_state)
    uint16_t t_ping   = 0;
    uint16_t t_tlm    = 0;
    uint16_t t_fired  = 0;
    uint16_t dist     = DIST_FAR_CM;   // assume clear until the first reading
    uint8_t  dist_ok  = 0;         // last reading produced a usable distance
    uint8_t  miss     = 0;         // consecutive "no echo edge" cycles
    uint8_t  fresh;
    uint8_t  L, R;
    uint8_t  paused   = 0;
    uint8_t  was_paused = 0;
    uint8_t  pivot_is_left = 0;
    uint16_t turn_len = T_TURN_MS;

    system_init();
    t_state = t_led = millis();
    t_ping  = (uint16_t)(millis() - MEAS_PERIOD_MS);   // ping on pass 1
    t_tlm   = millis();

    for (;;) {
        uint16_t now = millis();

        /* ---- commands ---- */
        if (rx_cmd) {
            uint8_t c = rx_cmd;
            rx_cmd = 0;
            if      (c == 'p' || c == 'P') paused ^= 1u;
            else if (c == '?')             t_tlm = (uint16_t)(now - TELEMETRY_MS);
        }

        /* ---- ranging: fire a ping, collect whatever the ISR left ---- */
        if (echo_state == ECHO_IDLE &&
            (uint16_t)(now - t_ping) >= MEAS_PERIOD_MS) {
            t_ping  = now;
            t_fired = now;
            hcsr04_fire();
        }

        /* Latch the ISR's result with interrupts off so a falling edge that
         * lands mid-check can't make us both time out AND report a reading.
         * `do { GIE = 0; } while (GIE);` closes the 1-instruction window a
         * bare di() leaves if an interrupt is already syncing.
         */
        fresh = 0;
        {
            uint8_t got_ready, got_far, got_noise, timed_out;
            do { GIE = 0; } while (GIE);
            got_ready = echo_ready;
            got_far   = echo_far;
            got_noise = echo_noise;
            timed_out = (!got_ready && !got_far && !got_noise &&
                         echo_state != ECHO_IDLE &&
                         (uint16_t)(now - t_fired) > ECHO_TIMEOUT_MS);
            if (timed_out) { echo_state = ECHO_IDLE; INTE = 0; }
            echo_ready = echo_far = echo_noise = 0;
            GIE = 1;

            if (got_ready) {
                dist = TICKS_TO_CM(echo_width);   // ISR won't touch echo_width until next fire
                dist_ok = 1; miss = 0; fresh = 1;
            } else if (got_far) {
                dist = DIST_FAR_CM;               // nothing within range -> path clear
                dist_ok = 1; miss = 0; fresh = 1;
            } else if (timed_out) {
                if (miss < 255u) miss++;          // no edge at all this cycle
                dist_ok = 0; fresh = 1;
            }
            /* got_noise: sub-minimum pulse — ignore, keep the last distance */
        }

        L = (IR_LEFT  == 0);
        R = (IR_RIGHT == 0);

        /* ---- telemetry ---- */
        if ((uint16_t)(now - t_tlm) >= TELEMETRY_MS) {
            t_tlm = now;
            telemetry_send((uint8_t)state, paused, dist_ok ? dist : 0u, L, R);
        }

        /* ---- paused: hold, keep sensing + reporting ---- */
        if (paused) {
            drive_stop();
            if ((uint16_t)(now - t_led) >= 250u) { STATUS_LED ^= 1; t_led = now; }
            was_paused = 1;
            continue;
        }
        if (was_paused) {                 // just resumed: restart the current
            was_paused = 0;               // manoeuvre's clock so a stale delta
            t_state = now;                // doesn't make it exit instantly
        }

        /* ---- confirmed sensor fault: only after FAULT_MISSES misses in a
         *      row, so open space (echo_far) and one-off dropouts don't stop
         *      the robot. ---- */
        if (fresh && !dist_ok && miss >= FAULT_MISSES && state != ST_FAULT) {
            drive_stop();
            STATUS_LED = 0;
            state   = ST_FAULT;
            t_state = now;
        }

        /* ---- act ---- */
        switch (state) {

        case ST_CRUISE:
            if (dist <= THRESHOLD_CM) {
                if      ( L && !R) { pivot_is_left = 0; turn_len = T_TURN_MS; }
                else if (!L &&  R) { pivot_is_left = 1; turn_len = T_TURN_MS; }
                else if ( L &&  R) { pivot_is_left = 0; turn_len = T_SHARP_TURN_MS; }
                else               { pivot_is_left = 1; turn_len = T_TURN_MS; }
                drive_reverse(SPEED_REVERSE);
                STATUS_LED = 0;
                state   = ST_BACK;
                t_state = now;
            }
            else if (L && !R) { drive_forward(SPEED_VEER_OUT, SPEED_VEER_IN); STATUS_LED = 1; }
            else if (R && !L) { drive_forward(SPEED_VEER_IN, SPEED_VEER_OUT); STATUS_LED = 1; }
            else if (L && R)  { drive_forward(SPEED_CREEP, SPEED_CREEP);      STATUS_LED = 1; }
            else              { drive_forward(SPEED_CRUISE, SPEED_CRUISE);    STATUS_LED = 1; }
            break;

        case ST_BACK: {
            uint16_t el = (uint16_t)(now - t_state);
            if (el >= T_REVERSE_MS ||
                (el >= T_REVERSE_MIN_MS &&
                 dist > (uint16_t)(THRESHOLD_CM + CLEAR_HYST_CM))) {
                if (pivot_is_left) pivot_left(SPEED_TURN);
                else               pivot_right(SPEED_TURN);
                state   = ST_TURN;
                t_state = now;
            }
            break;
        }

        case ST_TURN:
            if ((uint16_t)(now - t_state) >= turn_len) {
                drive_stop();
                state   = ST_CRUISE;
                t_state = now;
            }
            break;

        case ST_FAULT:
            if ((uint16_t)(now - t_led) >= 120u) { STATUS_LED ^= 1; t_led = now; }
            if (fresh && dist_ok) {        // any usable reading (near OR far)
                STATUS_LED = 0;
                state   = ST_CRUISE;
                t_state = now;
            }
            break;
        }
    }
}
