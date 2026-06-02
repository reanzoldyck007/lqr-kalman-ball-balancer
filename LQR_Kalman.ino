/*
 * lqr_balance.ino
 * LQR Ball-Balancing Controller
 * Pure C — AVR register access only, zero Arduino library calls.
 *
 * Compiled with Arduino IDE (board: Arduino Nano / Uno, ATmega328P @ 16MHz)
 *
 * Pin mapping:
 *   Trig  -> PB0  (Arduino D8)
 *   Echo  -> PB2  (Arduino D10)
 *   Servo -> PB1  (Arduino D9)  — OC1A hardware PWM
 *
 * Measured range : 3.90 cm (closest) to 12.94 cm (farthest)
 * Center/target  : (3.90 + 12.94) / 2 = 8.42 cm
 *
 * Timer usage:
 *   Timer0 — LEFT ALONE (Arduino IDE owns it via wiring.c)
 *   Timer1 — Servo PWM 50 Hz on OC1A
 *   Timer2 — Our millisecond counter
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

/* ------------------------------------------------------------------ */
/*  CPU FREQUENCY                                                       */
/* ------------------------------------------------------------------ */
#undef  F_CPU
#define F_CPU 16000000UL

/* ------------------------------------------------------------------ */
/*  PIN DEFINITIONS  (all Port B)                                      */
/* ------------------------------------------------------------------ */
#define TRIG_PIN   PB0   /* D8  — output */
#define ECHO_PIN   PB2   /* D10 — input  */
#define SERVO_PIN  PB1   /* D9  — OC1A   */

/* ------------------------------------------------------------------ */
/*  UART  — U2X0 double-speed, exact 115200 at 16 MHz                 */
/*  UBRR = F_CPU / (8 * BAUD) - 1 = 16                                */
/* ------------------------------------------------------------------ */
#define BAUD       115200UL
#define UBRR_U2X   ((F_CPU / (8UL * BAUD)) - 1UL)   /* = 16 */

/* ------------------------------------------------------------------ */
/*  SERVO  — Timer1 Fast PWM 50 Hz on OC1A                            */
/*  Prescaler 8  ->  tick = 0.5 us                                     */
/*  ICR1 = 40000 ->  period = 20 ms                                    */
/* ------------------------------------------------------------------ */
#define ICR1_TOP      40000U
#define SERVO_US_MIN  544U    /* 0 deg   */
#define SERVO_US_MAX  2400U   /* 180 deg */

/* ------------------------------------------------------------------ */
/*  MEASURED RANGE & DERIVED CONSTANTS                                  */
/*                                                                      */
/*  Closest  : 3.90 cm                                                  */
/*  Farthest : 12.94 cm                                                 */
/*  Center   : (3.90 + 12.94) / 2 = 8.42 cm  <- balance target        */
/* ------------------------------------------------------------------ */
#define DIST_MIN       3.90f
#define DIST_MAX       12.94f
#define TARGET_POS     7.5f   /* centre of measured range multiplier (variable)*/
#define TARGET_POSITION     9.2f /* ideal centre of measured range */

#define X_EST_INIT     7.5f   /* Kalman starts at target  */
#define LAST_POS_INIT  7.5f

/* ------------------------------------------------------------------ */
/*  SERVO LIMITS                                                        */
/* ------------------------------------------------------------------ */
#define SERVO_CENTER   68
#define SERVO_MIN_DEG  30
#define SERVO_MAX_DEG  140
#define MAX_RATE       3

/* ------------------------------------------------------------------ */
/*  LQR GAINS                                                           */
/* ------------------------------------------------------------------ */
#define K1   3.0f   /* position gain */
#define K2   2.0f   /* velocity gain */

/* ------------------------------------------------------------------ */
/*  KALMAN FILTER                                                       */
/* ------------------------------------------------------------------ */
#define P_INIT   1.0f
#define Q_NOISE  0.1f
#define R_NOISE  0.8f

/* ------------------------------------------------------------------ */
/*  CONTROL / VELOCITY LIMITS                                           */
/* ------------------------------------------------------------------ */
#define CTRL_MIN  -15.0f
#define CTRL_MAX   15.0f
#define VEL_MIN   -50.0f
#define VEL_MAX    50.0f

/* ================================================================== */
/*  TIMER2 — MILLISECOND COUNTER                                       */
/*  Prescaler 128: overflow every 2.048 ms                             */
/*  Fractional accumulator corrects the 0.048 ms remainder.            */
/* ================================================================== */
static volatile uint32_t g_ms      = 0;
static volatile uint8_t  g_ms_frac = 0;

ISR(TIMER2_OVF_vect)
{
    g_ms += 2;
    g_ms_frac += 6;
    if (g_ms_frac >= 128) {
        g_ms_frac -= 128;
        g_ms++;
    }
}

static void timer2_init(void)
{
    TCCR2A = 0x00;
    TCCR2B = (1 << CS22) | (1 << CS20);   /* Normal mode, prescaler 128 */
    TIMSK2 = (1 << TOIE2);
}

static uint32_t my_millis(void)
{
    uint32_t t;
    uint8_t sreg = SREG;
    cli();
    t = g_ms;
    SREG = sreg;
    return t;
}

/* ================================================================== */
/*  SERVO                                                               */
/* ================================================================== */
static void servo_init(void)
{
    DDRB |= (1 << SERVO_PIN);
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    ICR1   = ICR1_TOP;
    uint16_t us = (uint16_t)(SERVO_US_MIN +
                  (uint32_t)SERVO_CENTER * (SERVO_US_MAX - SERVO_US_MIN) / 180U);
    OCR1A = us * 2U;
}

static void servo_write(uint8_t angle)
{
    if (angle > 180U) angle = 180U;
    uint16_t us = (uint16_t)(SERVO_US_MIN +
                  (uint32_t)angle * (SERVO_US_MAX - SERVO_US_MIN) / 180U);
    OCR1A = us * 2U;
}

/* ================================================================== */
/*  UART                                                                */
/* ================================================================== */
static void uart_init(void)
{
    UCSR0B = 0;
    UBRR0H = (uint8_t)(UBRR_U2X >> 8);
    UBRR0L = (uint8_t)(UBRR_U2X);
    UCSR0A = (1 << U2X0);
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static void uart_putc(char c)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = (uint8_t)c;
}

static void uart_puts(const char *s)
{
    while (*s) uart_putc(*s++);
}

static void uart_print_float(float v)
{
    char buf[12];
    int  whole, frac, i, a, b;
    char tmp;

    if (v < 0.0f) { uart_putc('-'); v = -v; }

    whole = (int)v;
    frac  = (int)((v - (float)whole) * 100.0f + 0.5f);
    if (frac >= 100) { whole++; frac -= 100; }

    if (whole == 0) {
        buf[0] = '0'; buf[1] = '\0';
    } else {
        int w = whole; i = 0;
        while (w > 0) { buf[i++] = (char)('0' + w % 10); w /= 10; }
        buf[i] = '\0';
        a = 0; b = i - 1;
        while (a < b) { tmp=buf[a]; buf[a]=buf[b]; buf[b]=tmp; a++; b--; }
    }
    uart_puts(buf);
    uart_putc('.');
    uart_putc((char)('0' + frac / 10));
    uart_putc((char)('0' + frac % 10));
}

/* ================================================================== */
/*  HELPERS                                                             */
/* ================================================================== */
static float fclamp(float v, float lo, float hi)
{
    return (v < lo) ? lo : (v > hi) ? hi : v;
}
static int iclamp(int v, int lo, int hi)
{
    return (v < lo) ? lo : (v > hi) ? hi : v;
}
static int iabs_val(int v) { return (v < 0) ? -v : v; }

/* ================================================================== */
/*  STATE VARIABLES                                                     */
/* ================================================================== */
static float    x_est      = X_EST_INIT;
static float    P_kalman   = P_INIT;
static float    velocity   = 0.0f;
static float    last_pos   = LAST_POS_INIT;
static float    servo_filt = (float)SERVO_CENTER;
static int      last_servo = SERVO_CENTER;
static uint32_t last_time  = 0;

/* ================================================================== */
/*  HC-SR04  — valid range clamped to measured DIST_MIN / DIST_MAX    */
/* ================================================================== */
static float read_distance(void)
{
    uint32_t timeout, duration = 0;

    PORTB &= ~(1 << TRIG_PIN);
    _delay_us(2);
    PORTB |=  (1 << TRIG_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIG_PIN);

    timeout = 20000UL;
    while (!(PINB & (1 << ECHO_PIN))) {
        _delay_us(1);
        if (--timeout == 0) return x_est;
    }

    timeout = 20000UL;
    while (PINB & (1 << ECHO_PIN)) {
        _delay_us(1);
        duration++;
        if (--timeout == 0) return x_est;
    }

    float dist = (float)duration * 0.034f / 2.0f;

    /* Reject readings outside the measured physical range */
    if (dist < DIST_MIN || dist > DIST_MAX) return x_est;
    return dist;
}

/* ================================================================== */
/*  SETUP                                                               */
/* ================================================================== */
void setup(void)
{
    DDRB |=  (1 << TRIG_PIN);
    DDRB &= ~(1 << ECHO_PIN);

    timer2_init();
    servo_init();
    uart_init();
    sei();

    servo_write(SERVO_CENTER);
    _delay_ms(500);

    x_est     = read_distance();
    last_pos  = x_est;
    last_time = my_millis();

    uart_puts("LQR Ready | Target: 8.42 cm\r\n");
}

/* ================================================================== */
/*  LOOP                                                                */
/* ================================================================== */
void loop(void)
{
    uint32_t now = my_millis();
    float dt = (float)(now - last_time) / 1000.0f;

    if (dt < 0.01f) return;
    last_time = now;

    /* Sensor */
    float pos = read_distance();

    /* Kalman filter */
    P_kalman += Q_NOISE;
    float K_k = P_kalman / (P_kalman + R_NOISE);
    x_est    += K_k * (pos - x_est);
    P_kalman  = (1.0f - K_k) * P_kalman;

    /* Velocity estimate */
    velocity = 0.4f * velocity + 0.6f * ((x_est - last_pos) / dt);
    velocity = fclamp(velocity, VEL_MIN, VEL_MAX);
    last_pos = x_est;

  /* LQR control with out-of-band boost and extra nudge if too far */
  float error = x_est - TARGET_POS;

  /* If ball is outside a small tolerance window, increase corrective gain */
  float gain_mul = 1.0f;
  if (x_est < 8.2f || x_est > 8.5f) {
      gain_mul = 1.5f;
  }

  float control = gain_mul * (K1 * error) + K2 * velocity;

  /* Add +2 extra control when ball is at/above 9.0 cm to nudge more */
  if (x_est >= 9.0f) {
      control += 2.0f;
  }

  /* Optionally relax control clamps when far from target */
  float abs_err = (error < 0.0f) ? -error : error;
  float ctrl_min = CTRL_MIN, ctrl_max = CTRL_MAX;
  if (abs_err > 2.0f) {
      ctrl_min = CTRL_MIN * 1.5f;
      ctrl_max = CTRL_MAX * 1.5f;
  }

  control = fclamp(control, ctrl_min, ctrl_max);

  /* Servo command */
  int servo_cmd = SERVO_CENTER + (int)control;
  servo_cmd = iclamp(servo_cmd, SERVO_MIN_DEG, SERVO_MAX_DEG);

  /* Rate limiting */
  int delta = servo_cmd - last_servo;
  if (iabs_val(delta) > MAX_RATE) {
      delta = iclamp(delta, -MAX_RATE, MAX_RATE);
      servo_cmd = last_servo + delta;
  }
  last_servo = servo_cmd;

    /* Servo filtering */
    servo_filt = 0.7f * servo_filt + 0.3f * (float)servo_cmd;
    servo_write((uint8_t)servo_filt);

    /* Telemetry */
    uart_puts("Dist: ");
    uart_print_float(x_est);
    uart_puts(" | Tgt: ");
    uart_print_float(TARGET_POSITION);
    uart_puts(" | Err: ");
    uart_print_float(error);
    uart_puts(" | Ctrl: ");
    uart_print_float(control);
    uart_puts("\r\n");

    _delay_ms(10);
}
