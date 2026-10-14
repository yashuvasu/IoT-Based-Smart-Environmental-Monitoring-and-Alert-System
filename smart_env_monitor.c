/* =============================================================================
 *  smart_env_monitor.c
 *  IoT-Based Smart Environmental Monitoring and Alert System
 *  Week 2: Firmware Development and Implementation (simulated embedded system)
 *
 *  Author : Pilla Yashwanth (Yuva Intern)
 *  Target : ESP32 firmware architecture, executed here on a PC through a
 *           simulated Hardware Abstraction Layer (HAL).
 *  Language: C99, single file, no external libraries.
 * =============================================================================
 *
 *  README (embedded)
 *  -----------------
 *  WHAT THIS PROGRAM IS
 *    The firmware of the Week 1 design: it samples a DHT22 (temperature and
 *    humidity) and an MQ-135 (relative air quality), validates and filters the
 *    data, evaluates configurable thresholds with hysteresis, drives an alert
 *    LED and buzzer, updates a 128x64 OLED, reports telemetry over Wi-Fi, and
 *    accepts commands over UART. All hardware is SIMULATED in section 9 so the
 *    firmware logic can be compiled and tested on a PC without a board.
 *
 *  KEY FIRMWARE FEATURES
 *    - 1 ms TIMER INTERRUPT (timer_isr_1ms) that only keeps time and raises
 *      flags; all real work is done in the main loop (fw_loop).
 *    - UART RECEIVE INTERRUPT (uart_rx_isr) feeding a ring buffer; a command
 *      line interpreter (HELP, STATUS, THRESH, SET, MUTE) runs in fw_loop.
 *    - Non-blocking cooperative scheduler, no delay() calls anywhere.
 *    - Input validation, moving average, hysteresis, confirmation counting.
 *    - Fault handling for sensors, OLED, Wi-Fi and transmission, plus a
 *      (simulated) watchdog and reset-cause logging.
 *    - Self-test suite of 20 tests and a scripted demo (see "HOW TO RUN").
 *
 *  HOW TO COMPILE
 *      gcc -std=c99 -Wall -Wextra -Wpedantic -O2 -o smart_env_monitor \
 *          smart_env_monitor.c
 *    (Windows/MinGW: the same command; output file is smart_env_monitor.exe)
 *
 *  HOW TO RUN
 *      ./smart_env_monitor            run the 20-test self-test suite
 *      ./smart_env_monitor verbose    same, and print the firmware UART log
 *      ./smart_env_monitor demo       scripted scenario with OLED frames
 *      ./smart_env_monitor help       show these options
 *    Exit code is 0 when all tests pass and 1 otherwise.
 *
 *  UART COMMANDS (sent to the firmware, case-insensitive, end with newline)
 *      HELP | STATUS | THRESH | SET <TEMP_HI|TEMP_LO|HUM_HI|HUM_LO> <value>
 *      MUTE <ON|OFF>
 *
 *  FILE MAP
 *      1 Configuration      2 Types and state     3 HAL interface (port line)
 *      4 Logging            5 Interrupt routines  6 Firmware modules
 *      7 Entry points (fw_setup / fw_loop)
 *      8 (reserved)         9 Simulated HAL       10 Self-test suite
 *      11 Demo and main()
 *
 *  PORTING TO REAL ESP32 HARDWARE (not done or tested in Week 2)
 *    Sections 1 to 7 are hardware independent. Only section 9 must be replaced
 *    by real drivers (Arduino/ESP-IDF): hal_dht_read -> DHT library,
 *    hal_adc_read_mq135 -> analogRead on ADC1 GPIO34, hal_oled_* -> SSD1306
 *    driver on I2C GPIO21/22, hal_wifi_* -> WiFi/HTTP client, hal_uart_* ->
 *    Serial, hal_wdt_* -> task watchdog, timer_isr_1ms -> hardware timer ISR.
 * ===========================================================================*/

#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SIMULATION 1 /* 1 = compile with the simulated HAL of section 9 */

/* =============================================================================
 * 1. CONFIGURATION  (all tunable values in one place, see NFR-03 / FR-23)
 * ===========================================================================*/
#define FW_VERSION "2.0.0-sim"
#define DEVICE_ID "env-node-01"

/* Task periods driven by the timer interrupt (milliseconds). */
#define DHT_PERIOD_MS 2000UL     /* DHT22 minimum sampling period is ~2 s    */
#define MQ_PERIOD_MS 1000UL      /* MQ-135 sampling                          */
#define DISPLAY_PERIOD_MS 1000UL /* display task                             */
#define TX_PERIOD_MS 15000UL     /* Wi-Fi telemetry interval                 */
#define BLINK_FAST_HALF_MS 250U  /* alert pattern (LED steady, buzzer beeps) */
#define BLINK_SLOW_HALF_MS 1000U /* fault pattern (LED slow blink)           */

/* MQ-135 handling. */
#define MQ_WARMUP_MS 10000UL /* SIMULATION value. On hardware use minutes.   */
#define MQ_AVG_LEN 8         /* moving-average window (samples)              */
#define ADC_MAX 4095U        /* 12-bit ADC full scale                        */
#define AQ_MODERATE_PCT 125U /* ratio to baseline: >= 125 % is MODERATE      */
#define AQ_POOR_ON_PCT 175U  /* >= 175 % raises the air alert                */
#define AQ_POOR_OFF_PCT 160U /* < 160 % clears it (hysteresis)               */

/* Validation limits (typical DHT22 range) and spike filters. */
#define DHT_TEMP_MIN (-40.0f)
#define DHT_TEMP_MAX 80.0f
#define DHT_HUM_MIN 0.0f
#define DHT_HUM_MAX 100.0f
#define MAX_TEMP_STEP 10.0f /* max plausible change between samples (C)      */
#define MAX_HUM_STEP 30.0f  /* max plausible change between samples (%RH)    */

/* Alert logic. */
#define CONFIRM_COUNT 2U  /* consecutive samples needed to raise an alert    */
#define FAULT_LIMIT 3U    /* consecutive bad samples before a fault          */
#define RECOVER_COUNT 2U  /* consecutive good samples before a fault clears  */
#define DEF_TEMP_HI 35.0f /* default thresholds (proposed, Week 1 section 7) */
#define DEF_TEMP_LO 10.0f
#define DEF_HUM_HI 80.0f
#define DEF_HUM_LO 20.0f
#define HYST_TEMP 1.0f /* clear point is this far inside the trip point      */
#define HYST_HUM 2.0f

/* Wi-Fi and display behaviour. */
#define WIFI_CONNECT_TIMEOUT_MS 5000UL
#define WIFI_BACKOFF_MIN_MS 5000UL
#define WIFI_BACKOFF_MAX_MS 60000UL
#define OLED_RETRY_MS 30000UL        /* retry a failed OLED every 30 s       */
#define OLED_FULL_REFRESH_MS 5000UL  /* rewrite frame even if unchanged      */
#define OLED_LINES 6
#define OLED_COLS 22 /* 21 characters + terminator                           */

/* System. */
#define WDT_TIMEOUT_MS 10000UL
#define UART_RX_BUF_SIZE 64U /* ring buffer size                             */
#define UART_LINE_MAX 48U    /* longest accepted command line                */
#define PAYLOAD_MAX 256U

/* =============================================================================
 * 2. TYPES AND STATE
 * ===========================================================================*/
typedef enum { ST_INIT = 0, ST_WARMUP, ST_NORMAL, ST_ALERT, ST_DEGRADED } sys_state_t;
static const char *const STATE_NAME[] = {"INIT", "WARMUP", "NORMAL", "ALERT", "DEGRADED"};

typedef enum { WIFI_DOWN_WAIT = 0, WIFI_CONNECTING, WIFI_UP } wifi_state_t;
static const char *const WIFI_NAME[] = {"DOWN", "CONNECTING", "UP"};

typedef enum { AQ_UNKNOWN = 0, AQ_WARMING, AQ_GOOD, AQ_MODERATE, AQ_POOR } aq_status_t;
static const char *const AQ_NAME[] = {"---", "WARMING UP", "GOOD", "MODERATE", "POOR"};

/* Alert channels. The bit position in 'alerts' equals the channel number. */
typedef enum { CH_TEMP_HI = 0, CH_TEMP_LO, CH_HUM_HI, CH_HUM_LO, CH_AIR, CH_COUNT } channel_t;
static const char *const CH_NAME[CH_COUNT] = {"TEMP_HI", "TEMP_LO", "HUM_HI", "HUM_LO", "AIR"};

/* Fault bits. FAULT_WIFI and FAULT_TX are informational, not "hardware faults". */
#define FAULT_DHT (1u << 0)
#define FAULT_MQ (1u << 1)
#define FAULT_OLED (1u << 2)
#define FAULT_WIFI (1u << 3)
#define FAULT_TX (1u << 4)
#define FAULT_HW_MASK (FAULT_DHT | FAULT_MQ | FAULT_OLED)

typedef enum { RESET_POWERON = 0, RESET_BROWNOUT = 1, RESET_WDT = 2 } reset_cause_t;
static const char *const RESET_NAME[] = {"POWER-ON", "BROWN-OUT", "WATCHDOG"};

/* One threshold channel: trips after CONFIRM_COUNT samples, clears with hysteresis. */
typedef struct {
    bool high;     /* true: alert when value >= trip; false: when value <= trip */
    float trip;    /* alert limit                                               */
    float clear;   /* value must pass this point (inside trip) to clear         */
    uint8_t count; /* consecutive samples beyond trip                           */
    bool active;
} limit_ch_t;

/* Latest processed data shared between modules (Week 1, section 15.3). */
typedef struct {
    float temp, hum;
    bool temp_valid, hum_valid;
    uint16_t aq_raw;   /* moving average of ADC counts                         */
    uint16_t aq_ratio; /* percent of clean-air baseline                        */
    bool aq_valid;
    aq_status_t aq_status;
} snapshot_t;

typedef struct {
    uint32_t dht_ok, dht_bad, mq_samples, tx_ok, tx_fail, frames;
} stats_t;

/* All firmware state in one object, so a reset is a single memset. */
typedef struct {
    /* ---- shared with interrupts (volatile) ---- */
    volatile uint32_t ms; /* milliseconds since boot (system tick)             */
    volatile uint8_t flag_dht, flag_mq, flag_disp, flag_tx;
    volatile uint8_t blink_fast, blink_slow;
    volatile uint8_t rx_buf[UART_RX_BUF_SIZE];
    volatile uint8_t rx_head, rx_tail;
    volatile uint16_t rx_overflow;
    /* ---- interrupt-private divider counters ---- */
    uint16_t cnt_dht, cnt_mq, cnt_disp, cnt_bf, cnt_bs;
    uint32_t cnt_tx;
    /* ---- main-loop state ---- */
    sys_state_t state;
    snapshot_t snap;
    limit_ch_t ch[CH_COUNT];
    uint32_t alerts, faults;
    bool mute;
    bool led, buzzer;
    /* DHT22 */
    float last_t, last_h;
    bool have_ref;
    uint8_t dht_bad_run, dht_good_run;
    /* MQ-135 */
    uint16_t mq_ring[MQ_AVG_LEN];
    uint8_t mq_n, mq_idx, mq_bad_run, mq_good_run;
    uint16_t baseline;
    bool baseline_valid;
    /* display */
    bool oled_ok;
    uint8_t oled_bad_run;
    uint32_t oled_retry_at, oled_refresh_at;
    char frame[OLED_LINES][OLED_COLS];
    bool frame_valid;
    /* Wi-Fi */
    wifi_state_t wifi;
    uint32_t wifi_t0, wifi_wait_until, backoff;
    uint32_t tx_seq;
    /* CLI */
    char line[UART_LINE_MAX];
    uint8_t line_len;
    bool line_overflow;
    stats_t stat;
} fw_t;

static fw_t g; /* the one and only firmware state object */

/* =============================================================================
 * 3. HAL INTERFACE  (the "port line": implement these for real hardware)
 * ===========================================================================*/
static bool hal_dht_read(float *temp_c, float *hum_pct); /* false = no response */
static uint16_t hal_adc_read_mq135(void);                /* 12-bit counts       */
static bool hal_oled_init(void);
static bool hal_oled_write_frame(const char lines[][OLED_COLS], int n);
static void hal_led_set(bool on);
static void hal_buzzer_set(bool on);
static void hal_wifi_begin(void);
static bool hal_wifi_connected(void);
static void hal_wifi_disconnect(void);
static bool hal_wifi_send(const char *payload);
static int hal_wifi_rssi(void);
static void hal_uart_write(const char *text);
static void hal_wdt_init(uint32_t timeout_ms);
static void hal_wdt_feed(void);
static reset_cause_t hal_reset_cause(void);

/* =============================================================================
 * 4. LOGGING AND SMALL HELPERS
 * ===========================================================================*/
static void uart_print(const char *fmt, ...) {
    char buf[200];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    hal_uart_write(buf);
}

/* Timestamped diagnostic line (FR-24). */
static void log_msg(const char *fmt, ...) {
    char msg[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof msg, fmt, ap);
    va_end(ap);
    uart_print("[%7lu ms] %s\n", (unsigned long)g.ms, msg);
}

/* Read-and-clear an interrupt flag. On real hardware wrap this in a critical
 * section (portENTER_CRITICAL) or use an atomic exchange. */
static bool take_flag(volatile uint8_t *flag) {
    if (*flag) {
        *flag = 0;
        return true;
    }
    return false;
}

static bool warm_done(void) { return g.ms >= MQ_WARMUP_MS; }

static void fault_set(uint32_t bit, const char *name) {
    if (!(g.faults & bit)) {
        g.faults |= bit;
        log_msg("FAULT SET   : %s", name);
    }
}
static void fault_clear(uint32_t bit, const char *name) {
    if (g.faults & bit) {
        g.faults &= ~bit;
        log_msg("FAULT CLEAR : %s", name);
    }
}

/* Format a float as "27.4" or "null"/"--.-" style text. */
static void fmt_value(char *out, size_t n, bool valid, float v, const char *invalid_text) {
    if (valid) snprintf(out, n, "%.1f", (double)v);
    else snprintf(out, n, "%s", invalid_text);
}

/* =============================================================================
 * 5. INTERRUPT SERVICE ROUTINES
 * ===========================================================================*/

/* 5.1 Timer interrupt, 1 ms period (FR-02..FR-04 timing, NFR-07).
 * Rules for this ISR: keep it short, no printing, no sensor access, no
 * blocking. It only advances the clock and raises flags for fw_loop(). */
static void timer_isr_1ms(void) {
    g.ms++;
    if (++g.cnt_dht >= DHT_PERIOD_MS) { g.cnt_dht = 0; g.flag_dht = 1; }
    if (++g.cnt_mq >= MQ_PERIOD_MS) { g.cnt_mq = 0; g.flag_mq = 1; }
    if (++g.cnt_disp >= DISPLAY_PERIOD_MS) { g.cnt_disp = 0; g.flag_disp = 1; }
    if (++g.cnt_tx >= TX_PERIOD_MS) { g.cnt_tx = 0; g.flag_tx = 1; }
    if (++g.cnt_bf >= BLINK_FAST_HALF_MS) { g.cnt_bf = 0; g.blink_fast = !g.blink_fast; }
    if (++g.cnt_bs >= BLINK_SLOW_HALF_MS) { g.cnt_bs = 0; g.blink_slow = !g.blink_slow; }
}

/* 5.2 UART receive interrupt: one byte in, stored in a ring buffer.
 * If the buffer is full the byte is dropped and counted, never overwritten. */
static void uart_rx_isr(uint8_t byte) {
    uint8_t next = (uint8_t)((g.rx_head + 1U) % UART_RX_BUF_SIZE);
    if (next == g.rx_tail) {
        g.rx_overflow++;
        return;
    }
    g.rx_buf[g.rx_head] = byte;
    g.rx_head = next;
}

/* =============================================================================
 * 6. FIRMWARE MODULES (Week 1, section 15)
 * ===========================================================================*/

/* ---- 6.1 Threshold monitoring: hysteresis and confirmation (FR-11, FR-12) -- */

static void ch_setup(channel_t id, bool high, float trip, float hyst) {
    limit_ch_t *c = &g.ch[id];
    c->high = high;
    c->trip = trip;
    c->clear = high ? trip - hyst : trip + hyst;
    c->count = 0;
    c->active = false;
}

static void ch_reset(channel_t id) {
    g.ch[id].count = 0;
    g.ch[id].active = false;
}

/* Call once for every FRESH, VALID sample of the monitored quantity. */
static void ch_eval(channel_t id, float v) {
    limit_ch_t *c = &g.ch[id];
    if (!c->active) {
        bool beyond = c->high ? (v >= c->trip) : (v <= c->trip);
        c->count = beyond ? (uint8_t)(c->count + 1U) : 0U;
        if (c->count >= CONFIRM_COUNT) {
            c->active = true;
            log_msg("ALERT ON    : %s (value %.1f, limit %.1f)", CH_NAME[id], (double)v, (double)c->trip);
        }
    } else {
        bool back = c->high ? (v < c->clear) : (v > c->clear);
        if (back) {
            c->active = false;
            c->count = 0;
            log_msg("ALERT OFF   : %s (value %.1f, cleared at %.1f)", CH_NAME[id], (double)v, (double)c->clear);
        }
    }
}

/* ---- 6.2 Sensor acquisition + data processing: DHT22 (FR-02, 03, 05, 06, 21) */

static void dht_task(void) {
    float t = 0.0f, h = 0.0f;
    bool responded = hal_dht_read(&t, &h);
    bool valid = responded && !isnan(t) && !isnan(h) && t >= DHT_TEMP_MIN && t <= DHT_TEMP_MAX &&
                 h >= DHT_HUM_MIN && h <= DHT_HUM_MAX;
    const char *why = responded ? "out of range" : "no response";

    /* Rate-of-change check: rejects physically implausible jumps (spikes).
     * It is skipped while a fault is active so that the sensor can recover. */
    if (valid && g.have_ref && !(g.faults & FAULT_DHT) &&
        (fabsf(t - g.last_t) > MAX_TEMP_STEP || fabsf(h - g.last_h) > MAX_HUM_STEP)) {
        valid = false;
        why = "spike";
    }

    if (!valid) {
        g.dht_good_run = 0;
        g.stat.dht_bad++;
        if (g.dht_bad_run < 255U) g.dht_bad_run++;
        log_msg("DHT22 sample rejected (%s), consecutive=%u", why, (unsigned)g.dht_bad_run);
        if (g.dht_bad_run >= FAULT_LIMIT && !(g.faults & FAULT_DHT)) {
            fault_set(FAULT_DHT, "DHT22 (sensor)");
            g.snap.temp_valid = false;
            g.snap.hum_valid = false;
            /* Alerts based on data we no longer trust are cleared; the fault
             * indication (LED pattern, display, telemetry) replaces them. */
            ch_reset(CH_TEMP_HI); ch_reset(CH_TEMP_LO); ch_reset(CH_HUM_HI); ch_reset(CH_HUM_LO);
        }
        return; /* otherwise keep the last valid snapshot (hold) */
    }

    g.dht_bad_run = 0;
    g.last_t = t;
    g.last_h = h;
    g.have_ref = true;

    if (g.faults & FAULT_DHT) { /* recovery samples are counted, not evaluated */
        if (++g.dht_good_run >= RECOVER_COUNT) {
            g.dht_good_run = 0;
            fault_clear(FAULT_DHT, "DHT22 (sensor)");
        }
        return;
    }

    g.snap.temp = t; g.snap.hum = h;
    g.snap.temp_valid = true; g.snap.hum_valid = true;
    g.stat.dht_ok++;
    ch_eval(CH_TEMP_HI, t); ch_eval(CH_TEMP_LO, t);
    ch_eval(CH_HUM_HI, h);  ch_eval(CH_HUM_LO, h);
}

/* ---- 6.3 Sensor acquisition + data processing: MQ-135 (FR-04, 05, 06, 07) -- */

static uint16_t mq_average(void) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < g.mq_n; i++) sum += g.mq_ring[i];
    return g.mq_n ? (uint16_t)(sum / g.mq_n) : 0U;
}

static void aq_update_status(void) {
    if (!warm_done()) g.snap.aq_status = AQ_WARMING;
    else if (!g.snap.aq_valid) g.snap.aq_status = AQ_UNKNOWN;
    else if (g.ch[CH_AIR].active) g.snap.aq_status = AQ_POOR;
    else if (g.snap.aq_ratio >= AQ_MODERATE_PCT) g.snap.aq_status = AQ_MODERATE;
    else g.snap.aq_status = AQ_GOOD;
}

static void mq_task(void) {
    uint16_t raw = hal_adc_read_mq135();
    /* 0 and full-scale are the signature of a floating or shorted input. */
    bool valid = (raw > 0U) && (raw < ADC_MAX);

    if (!valid) {
        g.mq_good_run = 0;
        if (g.mq_bad_run < 255U) g.mq_bad_run++;
        log_msg("MQ-135 sample rejected (raw=%u), consecutive=%u", (unsigned)raw, (unsigned)g.mq_bad_run);
        if (g.mq_bad_run >= FAULT_LIMIT && !(g.faults & FAULT_MQ)) {
            fault_set(FAULT_MQ, "MQ-135 (sensor)");
            g.snap.aq_valid = false;
            ch_reset(CH_AIR);
        }
        aq_update_status();
        return;
    }

    g.mq_bad_run = 0;
    g.mq_ring[g.mq_idx] = raw;
    g.mq_idx = (uint8_t)((g.mq_idx + 1U) % MQ_AVG_LEN);
    if (g.mq_n < MQ_AVG_LEN) g.mq_n++;
    g.stat.mq_samples++;

    if (g.faults & FAULT_MQ) {
        if (++g.mq_good_run >= RECOVER_COUNT) {
            g.mq_good_run = 0;
            g.mq_n = 0; g.mq_idx = 0; /* flush stale samples from before the fault */
            fault_clear(FAULT_MQ, "MQ-135 (sensor)");
        }
        aq_update_status();
        return;
    }

    g.snap.aq_raw = mq_average();
    if (warm_done()) {
        /* The clean-air baseline is captured once, right after warm-up, from a
         * full averaging window. Air quality is a RELATIVE index (A-08). */
        if (!g.baseline_valid && g.mq_n >= MQ_AVG_LEN) {
            g.baseline = g.snap.aq_raw ? g.snap.aq_raw : 1U;
            g.baseline_valid = true;
            log_msg("MQ-135 baseline captured: %u counts", (unsigned)g.baseline);
        }
        if (g.baseline_valid) {
            g.snap.aq_ratio = (uint16_t)(((uint32_t)g.snap.aq_raw * 100UL) / g.baseline);
            g.snap.aq_valid = true;
            ch_eval(CH_AIR, (float)g.snap.aq_ratio);
        }
    }
    aq_update_status();
}

/* ---- 6.4 System state machine (Week 1, section 15.2) ----------------------- */

static void state_update(void) {
    g.alerts = 0;
    for (int i = 0; i < CH_COUNT; i++)
        if (g.ch[i].active) g.alerts |= (1u << i);

    sys_state_t s;
    if (g.alerts) s = ST_ALERT;
    else if (!warm_done()) s = ST_WARMUP;
    else if (g.faults & FAULT_HW_MASK) s = ST_DEGRADED;
    else s = ST_NORMAL;

    if (s != g.state) {
        log_msg("STATE       : %s -> %s", STATE_NAME[g.state], STATE_NAME[s]);
        g.state = s;
    }
}

/* ---- 6.5 Alert manager (FR-13, 14, 15, 26) ---------------------------------- */

static void alert_task(void) {
    bool led = false, bz = false;
    if (g.alerts) {                      /* ALERT : LED steady, buzzer beeping */
        led = true;
        bz = g.blink_fast != 0;
    } else if (g.faults & FAULT_HW_MASK) { /* FAULT : LED slow blink, no buzzer  */
        led = g.blink_slow != 0;
    }
    if (g.mute) bz = false;              /* mute silences the buzzer only      */
    if (led != g.led) { g.led = led; hal_led_set(led); }
    if (bz != g.buzzer) { g.buzzer = bz; hal_buzzer_set(bz); }
}

/* ---- 6.6 Display manager (FR-08, 09, 10, 22) -------------------------------- */

static void faults_text(char *out, size_t n) {
    out[0] = '\0';
    if (g.faults & FAULT_DHT) strncat(out, "DHT ", n - strlen(out) - 1);
    if (g.faults & FAULT_MQ) strncat(out, "MQ ", n - strlen(out) - 1);
    if (g.faults & FAULT_OLED) strncat(out, "OLED ", n - strlen(out) - 1);
    if (g.faults & FAULT_TX) strncat(out, "TX ", n - strlen(out) - 1);
}

static void build_frame(char f[][OLED_COLS]) {
    char tv[16], hv[16], al[32] = "", ft[32];
    const char *wf = (g.wifi == WIFI_UP) ? "OK" : (g.wifi == WIFI_CONNECTING ? "..." : "OFF");
    fmt_value(tv, sizeof tv, g.snap.temp_valid, g.snap.temp, "--.-");
    fmt_value(hv, sizeof hv, g.snap.hum_valid, g.snap.hum, "--.-");
    for (int i = 0; i < CH_COUNT; i++) {
        if (g.alerts & (1u << i)) {
            if (al[0]) strncat(al, "+", sizeof al - strlen(al) - 1);
            strncat(al, CH_NAME[i], sizeof al - strlen(al) - 1);
        }
    }
    faults_text(ft, sizeof ft);
    snprintf(f[0], OLED_COLS, "ENV MONITOR WiFi:%s", wf);
    snprintf(f[1], OLED_COLS, "Temp : %.8s C", tv);
    snprintf(f[2], OLED_COLS, "Hum  : %.8s %%", hv);
    snprintf(f[3], OLED_COLS, "Air  : %.14s", AQ_NAME[g.snap.aq_status]);
    snprintf(f[4], OLED_COLS, "Alert: %.14s", al[0] ? al : "NONE"); /* long lists are cut to fit */
    if (ft[0]) snprintf(f[5], OLED_COLS, "FAULT: %.14s", ft);
    else snprintf(f[5], OLED_COLS, "State: %.14s", STATE_NAME[g.state]);
}

static void display_task(void) {
    if (!g.oled_ok) { /* display lost: retry now and then (FR-22) */
        if (g.ms >= g.oled_retry_at) {
            g.oled_retry_at = g.ms + OLED_RETRY_MS;
            if (hal_oled_init()) {
                g.oled_ok = true;
                g.oled_bad_run = 0;
                g.frame_valid = false; /* force redraw */
                fault_clear(FAULT_OLED, "OLED (display)");
            }
        }
        if (!g.oled_ok) return;
    }

    char f[OLED_LINES][OLED_COLS];
    build_frame(f);
    bool changed = !g.frame_valid || memcmp(f, g.frame, sizeof f) != 0;
    bool refresh = g.ms >= g.oled_refresh_at; /* periodic rewrite, detects a dead bus */
    if (!changed && !refresh) return;         /* avoid needless I2C traffic */

    g.oled_refresh_at = g.ms + OLED_FULL_REFRESH_MS;
    if (hal_oled_write_frame((const char(*)[OLED_COLS])f, OLED_LINES)) {
        memcpy(g.frame, f, sizeof f);
        g.frame_valid = true;
        g.oled_bad_run = 0;
        g.stat.frames++;
    } else if (++g.oled_bad_run >= FAULT_LIMIT) {
        g.oled_ok = false;
        g.oled_retry_at = g.ms + OLED_RETRY_MS;
        fault_set(FAULT_OLED, "OLED (display)");
    }
}

/* ---- 6.7 Wi-Fi communication (FR-09, 10, 16..20) ---------------------------- */

static void build_and_send(void) {
    char tv[16], hv[16], payload[PAYLOAD_MAX];
    fmt_value(tv, sizeof tv, g.snap.temp_valid, g.snap.temp, "null");
    fmt_value(hv, sizeof hv, g.snap.hum_valid, g.snap.hum, "null");
    int n = snprintf(payload, sizeof payload,
                     "{\"device_id\":\"%s\",\"seq\":%lu,\"uptime_s\":%lu,\"temp_c\":%s,\"hum_pct\":%s,"
                     "\"aq_raw\":%u,\"aq_status\":\"%s\",\"alert_flags\":%lu,\"fault_flags\":%lu,"
                     "\"wifi_rssi_dbm\":%d}",
                     DEVICE_ID, (unsigned long)g.tx_seq, (unsigned long)(g.ms / 1000UL), tv, hv,
                     (unsigned)g.snap.aq_raw, AQ_NAME[g.snap.aq_status], (unsigned long)g.alerts,
                     (unsigned long)(g.faults & ~FAULT_TX), hal_wifi_rssi());
    if (n < 0 || (size_t)n >= sizeof payload) { /* never send a truncated message */
        log_msg("TX payload too long, not sent");
        return;
    }
    g.tx_seq++;
    if (hal_wifi_send(payload)) {
        g.stat.tx_ok++;
        fault_clear(FAULT_TX, "TX (transmission)");
    } else {
        g.stat.tx_fail++;
        fault_set(FAULT_TX, "TX (transmission)");
    }
}

static void wifi_task(void) {
    switch (g.wifi) {
    case WIFI_CONNECTING:
        if (hal_wifi_connected()) {
            g.wifi = WIFI_UP;
            g.backoff = WIFI_BACKOFF_MIN_MS;
            fault_clear(FAULT_WIFI, "Wi-Fi (offline)");
            log_msg("WIFI        : connected (RSSI %d dBm)", hal_wifi_rssi());
        } else if (g.ms - g.wifi_t0 >= WIFI_CONNECT_TIMEOUT_MS) {
            hal_wifi_disconnect();
            g.wifi = WIFI_DOWN_WAIT;
            g.wifi_wait_until = g.ms + g.backoff;
            log_msg("WIFI        : connect timeout, retry in %lu s", (unsigned long)(g.backoff / 1000UL));
            g.backoff = (g.backoff * 2UL > WIFI_BACKOFF_MAX_MS) ? WIFI_BACKOFF_MAX_MS : g.backoff * 2UL;
        }
        break;
    case WIFI_DOWN_WAIT:
        if (g.ms >= g.wifi_wait_until) {
            hal_wifi_begin();
            g.wifi = WIFI_CONNECTING;
            g.wifi_t0 = g.ms;
        }
        break;
    case WIFI_UP:
        if (!hal_wifi_connected()) { /* link lost: monitoring continues (FR-19) */
            hal_wifi_disconnect();
            g.wifi = WIFI_DOWN_WAIT;
            g.wifi_wait_until = g.ms + WIFI_BACKOFF_MIN_MS;
            fault_set(FAULT_WIFI, "Wi-Fi (offline)");
            log_msg("WIFI        : link lost, offline mode");
        }
        break;
    }
    /* Telemetry is periodic (FR-17); when not connected the report is skipped. */
    if (take_flag(&g.flag_tx) && g.wifi == WIFI_UP) build_and_send();
}

/* ---- 6.8 UART command line interpreter (FR-24, FR-23) ----------------------- */

static bool parse_float(const char *s, float *out) {
    char *end = NULL;
    if (!s) return false;
    float v = strtof(s, &end);
    if (end == s || *end != '\0' || isnan(v) || isinf(v)) return false;
    *out = v;
    return true;
}

static bool set_threshold(const char *name, float v) {
    float tmax = DHT_TEMP_MAX, tmin = DHT_TEMP_MIN;
    if (!strcmp(name, "TEMP_HI")) {
        if (v > tmax || v < g.ch[CH_TEMP_LO].trip + 2.0f * HYST_TEMP + 0.01f) return false;
        ch_setup(CH_TEMP_HI, true, v, HYST_TEMP);
    } else if (!strcmp(name, "TEMP_LO")) {
        if (v < tmin || v > g.ch[CH_TEMP_HI].trip - 2.0f * HYST_TEMP - 0.01f) return false;
        ch_setup(CH_TEMP_LO, false, v, HYST_TEMP);
    } else if (!strcmp(name, "HUM_HI")) {
        if (v > DHT_HUM_MAX || v < g.ch[CH_HUM_LO].trip + 2.0f * HYST_HUM + 0.01f) return false;
        ch_setup(CH_HUM_HI, true, v, HYST_HUM);
    } else if (!strcmp(name, "HUM_LO")) {
        if (v < DHT_HUM_MIN || v > g.ch[CH_HUM_HI].trip - 2.0f * HYST_HUM - 0.01f) return false;
        ch_setup(CH_HUM_LO, false, v, HYST_HUM);
    } else {
        return false;
    }
    return true;
}

static void cli_exec(char *line) {
    for (char *p = line; *p; ++p) *p = (char)toupper((unsigned char)*p);
    char *cmd = strtok(line, " \t");
    if (!cmd) return;
    char *a1 = strtok(NULL, " \t");
    char *a2 = strtok(NULL, " \t");

    if (!strcmp(cmd, "HELP")) {
        uart_print("OK commands: HELP STATUS THRESH SET <TEMP_HI|TEMP_LO|HUM_HI|HUM_LO> <v> MUTE <ON|OFF>\n");
    } else if (!strcmp(cmd, "STATUS")) {
        char tv[16], hv[16];
        fmt_value(tv, sizeof tv, g.snap.temp_valid, g.snap.temp, "--.-");
        fmt_value(hv, sizeof hv, g.snap.hum_valid, g.snap.hum, "--.-");
        uart_print("OK STATE=%s T=%s H=%s AQ=%s(%u%%) ALERT=0x%02lX FAULT=0x%02lX WIFI=%s UP=%lus TX=%lu/%lu\n",
                   STATE_NAME[g.state], tv, hv, AQ_NAME[g.snap.aq_status], (unsigned)g.snap.aq_ratio,
                   (unsigned long)g.alerts, (unsigned long)g.faults, WIFI_NAME[g.wifi],
                   (unsigned long)(g.ms / 1000UL), (unsigned long)g.stat.tx_ok,
                   (unsigned long)(g.stat.tx_ok + g.stat.tx_fail));
    } else if (!strcmp(cmd, "THRESH")) {
        uart_print("OK TEMP_HI=%.1f TEMP_LO=%.1f HUM_HI=%.1f HUM_LO=%.1f AIR=%u%%\n",
                   (double)g.ch[CH_TEMP_HI].trip, (double)g.ch[CH_TEMP_LO].trip,
                   (double)g.ch[CH_HUM_HI].trip, (double)g.ch[CH_HUM_LO].trip, (unsigned)AQ_POOR_ON_PCT);
    } else if (!strcmp(cmd, "SET")) {
        float v;
        if (!a1 || !a2 || !parse_float(a2, &v)) uart_print("ERR usage: SET <name> <number>\n");
        else if (!set_threshold(a1, v)) uart_print("ERR unknown name or value out of range\n");
        else {
            uart_print("OK %s=%.1f\n", a1, (double)v);
            log_msg("CONFIG      : %s set to %.1f", a1, (double)v);
        }
    } else if (!strcmp(cmd, "MUTE")) {
        if (a1 && !strcmp(a1, "ON")) { g.mute = true; uart_print("OK MUTE ON\n"); }
        else if (a1 && !strcmp(a1, "OFF")) { g.mute = false; uart_print("OK MUTE OFF\n"); }
        else uart_print("ERR usage: MUTE <ON|OFF>\n");
    } else {
        uart_print("ERR unknown command\n");
    }
}

/* Drain the ring buffer, assemble lines, execute complete lines. */
static void cli_poll(void) {
    while (g.rx_tail != g.rx_head) {
        uint8_t c = g.rx_buf[g.rx_tail];
        g.rx_tail = (uint8_t)((g.rx_tail + 1U) % UART_RX_BUF_SIZE);
        if (c == '\r') continue;
        if (c == '\n') {
            if (g.line_overflow) uart_print("ERR line too long\n");
            else { g.line[g.line_len] = '\0'; cli_exec(g.line); }
            g.line_len = 0;
            g.line_overflow = false;
        } else if (g.line_len < UART_LINE_MAX - 1U) {
            g.line[g.line_len++] = (char)c;
        } else {
            g.line_overflow = true;
        }
    }
}

/* =============================================================================
 * 7. ENTRY POINTS (map to Arduino setup() and loop())
 * ===========================================================================*/

/* Start-up sequence from Week 1, section 4.4. Failures are recorded, not fatal. */
static void fw_setup(void) {
    memset(&g, 0, sizeof g);
    g.state = ST_INIT;

    hal_led_set(false);    /* FR-26: alert outputs OFF before anything else   */
    hal_buzzer_set(false);

    log_msg("BOOT        : %s v%s, reset cause %s", DEVICE_ID, FW_VERSION, RESET_NAME[hal_reset_cause()]);

    ch_setup(CH_TEMP_HI, true, DEF_TEMP_HI, HYST_TEMP);
    ch_setup(CH_TEMP_LO, false, DEF_TEMP_LO, HYST_TEMP);
    ch_setup(CH_HUM_HI, true, DEF_HUM_HI, HYST_HUM);
    ch_setup(CH_HUM_LO, false, DEF_HUM_LO, HYST_HUM);
    ch_setup(CH_AIR, true, (float)AQ_POOR_ON_PCT, (float)(AQ_POOR_ON_PCT - AQ_POOR_OFF_PCT));

    g.snap.aq_status = AQ_WARMING;
    g.oled_ok = hal_oled_init();
    if (!g.oled_ok) fault_set(FAULT_OLED, "OLED (display)");
    g.oled_retry_at = OLED_RETRY_MS;

    hal_wifi_begin();               /* non-blocking: progress handled in wifi_task */
    g.wifi = WIFI_CONNECTING;
    g.wifi_t0 = 0;
    g.backoff = WIFI_BACKOFF_MIN_MS;
    fault_set(FAULT_WIFI, "Wi-Fi (offline)");

    hal_wdt_init(WDT_TIMEOUT_MS);   /* timer interrupt is started by the HAL/port */
    g.state = ST_WARMUP;
    log_msg("BOOT        : initialisation complete, MQ-135 warm-up started");
}

/* One pass of the cooperative main loop. Every call returns quickly. */
static void fw_loop(void) {
    hal_wdt_feed();                                 /* FR-25                  */
    cli_poll();                                     /* UART commands          */
    if (take_flag(&g.flag_mq)) mq_task();           /* acquire + process      */
    if (take_flag(&g.flag_dht)) dht_task();
    state_update();                                 /* thresholds -> state    */
    alert_task();                                   /* LED and buzzer         */
    if (take_flag(&g.flag_disp)) display_task();    /* OLED                   */
    wifi_task();                                    /* link + telemetry       */
}

#if SIMULATION
/* =============================================================================
 * 9. SIMULATED HAL  (replace this whole section when porting to hardware)
 *    The simulator lets a test inject sensor values and failures, and observe
 *    the LED, buzzer, OLED, Wi-Fi traffic and UART output.
 * ===========================================================================*/
#define SIM_UART_CAP 24576
#define SIM_MAX_EVENTS 128

typedef struct {
    /* ---- injected conditions (set by tests) ---- */
    float temp, hum;
    bool dht_fail;
    int adc_base, adc_noise, adc_force; /* adc_force >= 0 overrides the reading */
    bool oled_present, wifi_ap_up, send_ok;
    uint32_t wifi_connect_delay;
    int rssi;
    reset_cause_t reset_cause;
    bool hang; /* true: main loop does not run (simulates a firmware lock-up) */
    /* ---- observed outputs ---- */
    bool led, buzzer;
    uint32_t led_toggles, led_ms, buzzer_ms;
    char oled[OLED_LINES][OLED_COLS];
    uint32_t oled_writes;
    bool print_oled;
    char uart[SIM_UART_CAP];
    size_t uart_len;
    bool echo_uart;
    bool wifi_session;
    uint32_t wifi_begin_ms, attempts, attempt_ms[SIM_MAX_EVENTS];
    uint32_t tx_count, send_ms[SIM_MAX_EVENTS];
    char last_payload[PAYLOAD_MAX];
    /* ---- clock, watchdog, rng ---- */
    uint32_t total_ms, last_feed, max_feed_gap, reboots;
    bool wdt_armed;
    uint32_t rng;
} sim_t;

static sim_t sim;

static bool hal_dht_read(float *t, float *h) {
    if (sim.dht_fail) return false;
    *t = sim.temp;
    *h = sim.hum;
    return true;
}

static uint16_t hal_adc_read_mq135(void) {
    if (sim.adc_force >= 0) return (uint16_t)sim.adc_force;
    sim.rng = sim.rng * 1103515245u + 12345u; /* small deterministic noise */
    int noise = (int)((sim.rng >> 16) % (uint32_t)(2 * sim.adc_noise + 1)) - sim.adc_noise;
    int v = sim.adc_base + noise;
    if (v < 1) v = 1;
    if (v > (int)ADC_MAX - 1) v = (int)ADC_MAX - 1;
    return (uint16_t)v;
}

static bool hal_oled_init(void) { return sim.oled_present; }

static bool hal_oled_write_frame(const char lines[][OLED_COLS], int n) {
    if (!sim.oled_present) return false;
    for (int i = 0; i < n; i++) memcpy(sim.oled[i], lines[i], OLED_COLS);
    sim.oled_writes++;
    if (sim.print_oled) {
        printf("      +----------------------+\n");
        for (int i = 0; i < n; i++) printf("      | %-21s|\n", lines[i]);
        printf("      +----------------------+\n");
    }
    return true;
}

static void hal_led_set(bool on) {
    if (on != sim.led) sim.led_toggles++;
    sim.led = on;
}
static void hal_buzzer_set(bool on) { sim.buzzer = on; }

static void hal_wifi_begin(void) {
    sim.wifi_session = true;
    sim.wifi_begin_ms = sim.total_ms;
    if (sim.attempts < SIM_MAX_EVENTS) sim.attempt_ms[sim.attempts] = sim.total_ms;
    sim.attempts++;
}
static bool hal_wifi_connected(void) {
    return sim.wifi_session && sim.wifi_ap_up && (sim.total_ms - sim.wifi_begin_ms) >= sim.wifi_connect_delay;
}
static void hal_wifi_disconnect(void) { sim.wifi_session = false; }
static bool hal_wifi_send(const char *payload) {
    if (!hal_wifi_connected() || !sim.send_ok) return false;
    if (sim.tx_count < SIM_MAX_EVENTS) sim.send_ms[sim.tx_count] = sim.total_ms;
    sim.tx_count++;
    snprintf(sim.last_payload, sizeof sim.last_payload, "%s", payload);
    return true;
}
static int hal_wifi_rssi(void) { return sim.rssi; }

static void hal_uart_write(const char *text) {
    size_t n = strlen(text);
    if (sim.uart_len + n + 1 >= SIM_UART_CAP) { /* keep the most recent half */
        size_t keep = SIM_UART_CAP / 2;
        memmove(sim.uart, sim.uart + sim.uart_len - keep, keep);
        sim.uart_len = keep;
    }
    memcpy(sim.uart + sim.uart_len, text, n);
    sim.uart_len += n;
    sim.uart[sim.uart_len] = '\0';
    if (sim.echo_uart) fputs(text, stdout);
}

static void hal_wdt_init(uint32_t timeout_ms) {
    (void)timeout_ms;
    sim.wdt_armed = true;
    sim.last_feed = sim.total_ms;
}
static void hal_wdt_feed(void) {
    uint32_t gap = sim.total_ms - sim.last_feed;
    if (gap > sim.max_feed_gap) sim.max_feed_gap = gap;
    sim.last_feed = sim.total_ms;
}
static reset_cause_t hal_reset_cause(void) { return sim.reset_cause; }

/* ---- simulator control ---- */
static void sim_defaults(void) {
    memset(&sim, 0, sizeof sim);
    sim.temp = 25.0f;
    sim.hum = 50.0f;
    sim.adc_base = 800;
    sim.adc_noise = 3;
    sim.adc_force = -1;
    sim.oled_present = true;
    sim.wifi_ap_up = true;
    sim.send_ok = true;
    sim.wifi_connect_delay = 1500;
    sim.rssi = -61;
    sim.rng = 12345u;
}

static void sim_boot(reset_cause_t cause) {
    sim.reset_cause = cause;
    sim.hang = false;
    fw_setup();
}

static void sim_tick(void) {
    sim.total_ms++;
    timer_isr_1ms(); /* the interrupt always fires ... */
    if (!sim.hang) fw_loop(); /* ... the main loop may be stuck */
    if (sim.led) sim.led_ms++;
    if (sim.buzzer) sim.buzzer_ms++;
    if (sim.wdt_armed && (sim.total_ms - sim.last_feed) > WDT_TIMEOUT_MS) {
        sim.reboots++; /* watchdog expiry resets the MCU */
        sim_boot(RESET_WDT);
    }
}
static void sim_run_ms(uint32_t n) { while (n--) sim_tick(); }
static void sim_run_s(uint32_t s) { sim_run_ms(s * 1000UL); }

static void sim_uart_send(const char *text) { /* host -> firmware via the RX interrupt */
    for (; *text; ++text) {
        uart_rx_isr((uint8_t)*text);
        sim_tick(); /* about one byte per millisecond, roughly 9600 baud */
    }
}
static void sim_uart_clear(void) { sim.uart_len = 0; sim.uart[0] = '\0'; }
static bool uart_has(const char *s) { return strstr(sim.uart, s) != NULL; }
#else
#error "Provide a hardware HAL (see PORTING TO REAL ESP32 HARDWARE in the header)."
#endif /* SIMULATION */

/* =============================================================================
 * 10. SELF-TEST SUITE (host-side verification of the firmware logic)
 * ===========================================================================*/
static int t_checks, t_fail;
static bool t_verbose;

#define CHECK(cond, msg)                                                              \
    do {                                                                              \
        t_checks++;                                                                   \
        if (!(cond)) {                                                                \
            t_fail++;                                                                 \
            printf("      FAIL line %d: %s\n", __LINE__, msg);                        \
        }                                                                             \
    } while (0)

static void start(void) { /* fresh simulated board, powered on */
    sim_defaults();
    sim.echo_uart = t_verbose;
}
static void settle(void) { /* power on and run through warm-up into NORMAL */
    start();
    sim_boot(RESET_POWERON);
    sim_run_ms(MQ_WARMUP_MS + 3000UL);
}
static void run_until_alert(uint32_t bit, uint32_t limit_ms, uint32_t *elapsed) {
    uint32_t t0 = sim.total_ms;
    while (!(g.alerts & bit) && (sim.total_ms - t0) < limit_ms) sim_tick();
    *elapsed = sim.total_ms - t0;
}
static void run_until_dht_sample(void) {
    uint32_t n = g.stat.dht_ok + g.stat.dht_bad;
    while ((g.stat.dht_ok + g.stat.dht_bad) == n) sim_tick();
}

/* Move the simulated room towards a target in physically plausible steps, so
 * that the firmware's spike filter (MAX_TEMP_STEP / MAX_HUM_STEP) is not hit. */
static void ramp_temp(float target) {
    while (fabsf(target - sim.temp) > 4.0f) {
        sim.temp += (target > sim.temp) ? 4.0f : -4.0f;
        sim_run_ms(DHT_PERIOD_MS);
    }
    sim.temp = target;
}
static void ramp_hum(float target) {
    while (fabsf(target - sim.hum) > 20.0f) {
        sim.hum += (target > sim.hum) ? 20.0f : -20.0f;
        sim_run_ms(DHT_PERIOD_MS);
    }
    sim.hum = target;
}

static void t01_startup(void) {
    start();
    sim_boot(RESET_POWERON);
    CHECK(!sim.led && !sim.buzzer, "outputs must be OFF right after boot (FR-26)");
    CHECK(g.state == ST_WARMUP, "state must be WARMUP after initialisation");
    sim_run_s(5);
    CHECK(g.state == ST_WARMUP, "still WARMUP at 5 s");
    CHECK(g.snap.aq_status == AQ_WARMING, "air status must be WARMING during warm-up (FR-07)");
    CHECK(strstr(sim.oled[3], "WARMING") != NULL, "OLED must show WARMING UP");
    sim_run_s(8);
    CHECK(g.state == ST_NORMAL, "state must be NORMAL after warm-up");
    CHECK(g.baseline_valid, "baseline captured after warm-up");
}
static void t02_normal_display(void) {
    settle();
    CHECK(g.snap.temp_valid && fabsf(g.snap.temp - 25.0f) < 0.01f, "temperature 25.0 acquired (FR-02)");
    CHECK(g.snap.hum_valid && fabsf(g.snap.hum - 50.0f) < 0.01f, "humidity 50.0 acquired (FR-03)");
    CHECK(g.snap.aq_status == AQ_GOOD, "air quality GOOD at baseline (FR-04)");
    CHECK(strstr(sim.oled[1], "25.0") != NULL, "OLED temperature line (FR-08)");
    CHECK(strstr(sim.oled[2], "50.0") != NULL, "OLED humidity line (FR-08)");
    CHECK(strstr(sim.oled[3], "GOOD") != NULL, "OLED air status (FR-09)");
    CHECK(strstr(sim.oled[4], "NONE") != NULL, "OLED alert status (FR-10)");
    CHECK(strstr(sim.oled[0], "WiFi:OK") != NULL, "OLED Wi-Fi status (FR-10)");
    CHECK(!sim.led && !sim.buzzer, "no alert output in normal state");
}
static void t03_temp_alert_latency(void) {
    settle();
    ramp_temp(34.0f);
    sim_run_s(4);
    sim.temp = 36.0f;
    uint32_t lat;
    run_until_alert(1u << CH_TEMP_HI, 8000, &lat);
    printf("      measured alert latency (simulated time): %lu ms\n", (unsigned long)lat);
    CHECK(g.alerts & (1u << CH_TEMP_HI), "TEMP_HI alert raised (FR-11, FR-13)");
    CHECK(lat <= 2U * DHT_PERIOD_MS, "latency within two sampling periods (NFR-07)");
    sim_run_ms(1000);
    CHECK(sim.led, "LED ON during alert");
    CHECK(sim.buzzer_ms > 0, "buzzer sounded during alert");
    CHECK(g.state == ST_ALERT, "state ALERT");
    CHECK(strstr(sim.oled[4], "TEMP_HI") != NULL, "OLED shows alert source");
}
static void t04_hysteresis(void) {
    settle();
    ramp_temp(36.0f); sim_run_s(6);
    CHECK(g.alerts & (1u << CH_TEMP_HI), "alert active");
    sim.temp = 34.5f; sim_run_s(8); /* between clear (34.0) and trip (35.0) */
    CHECK(g.alerts & (1u << CH_TEMP_HI), "alert must STAY active inside hysteresis band (FR-12)");
    sim.temp = 33.5f; sim_run_s(6);
    CHECK(!(g.alerts & (1u << CH_TEMP_HI)), "alert cleared below the clear point (FR-14)");
    CHECK(!sim.led && !sim.buzzer, "outputs OFF after clear");
}
static void t05_isolated_sample_ignored(void) {
    settle();
    sim.temp = 32.0f; sim_run_s(6);
    sim.temp = 36.0f; run_until_dht_sample(); /* exactly one over-limit sample */
    sim.temp = 32.0f; sim_run_s(10);
    CHECK(g.alerts == 0, "single over-limit sample must not raise an alert (confirmation)");
    CHECK(g.faults == 0, "valid isolated sample is not a fault");
}
static void t06_humidity(void) {
    settle();
    ramp_hum(85.0f); sim_run_s(6);
    CHECK(g.alerts & (1u << CH_HUM_HI), "HUM_HI alert raised");
    sim.hum = 79.0f; sim_run_s(6);
    CHECK(g.alerts & (1u << CH_HUM_HI), "still active at 79 % (clear point is 78 %)");
    sim.hum = 77.0f; sim_run_s(6);
    CHECK(!(g.alerts & (1u << CH_HUM_HI)), "cleared at 77 %");
    ramp_temp(8.0f); sim_run_s(6);
    CHECK(g.alerts & (1u << CH_TEMP_LO), "TEMP_LO alert raised at 8 C");
}
static void t07_air_quality(void) {
    settle();
    sim.adc_base = 1600; /* twice the clean-air baseline */
    uint32_t lat;
    run_until_alert(1u << CH_AIR, 20000, &lat);
    printf("      air-quality alert delay incl. 8-sample averaging: %lu ms\n", (unsigned long)lat);
    CHECK(g.alerts & (1u << CH_AIR), "AIR alert raised for poor air");
    CHECK(g.snap.aq_status == AQ_POOR, "status POOR");
    CHECK(strstr(sim.oled[3], "POOR") != NULL, "OLED shows POOR");
    sim.adc_base = 800; sim_run_s(15);
    CHECK(!(g.alerts & (1u << CH_AIR)), "AIR alert cleared when air recovers");
    CHECK(g.snap.aq_status == AQ_GOOD, "status back to GOOD");
}
static void t08_warmup_suppresses_air_alert(void) {
    start();
    sim.adc_base = 3000; /* high reading during warm-up */
    sim_boot(RESET_POWERON);
    sim_run_ms(MQ_WARMUP_MS - 500UL);
    CHECK(!(g.alerts & (1u << CH_AIR)), "no air alert during warm-up (FR-07)");
    CHECK(g.snap.aq_status == AQ_WARMING, "status WARMING UP");
    sim_run_s(5);
    CHECK(g.baseline_valid, "baseline taken at end of warm-up");
    CHECK(!(g.alerts & (1u << CH_AIR)), "no false alert after warm-up");
}
static void t09_dht_disconnected(void) {
    settle();
    sim.dht_fail = true;
    uint32_t mq0 = g.stat.mq_samples;
    sim_run_s(8);
    CHECK(g.faults & FAULT_DHT, "DHT fault declared after 3 failed reads (FR-21)");
    CHECK(g.state == ST_DEGRADED, "state DEGRADED");
    CHECK(strstr(sim.oled[5], "DHT") != NULL, "fault shown on OLED");
    CHECK(strstr(sim.oled[1], "--.-") != NULL, "temperature shows --.- when invalid");
    CHECK(g.alerts == 0, "no alert from invalid data");
    CHECK(g.stat.mq_samples > mq0 + 5, "MQ-135 monitoring continues during DHT fault (NFR-02)");
    uint32_t tg0 = sim.led_toggles;
    sim_run_s(6);
    CHECK(sim.led_toggles >= tg0 + 4, "LED slow-blink fault pattern (FR-15)");
    CHECK(!sim.buzzer, "buzzer silent for a sensor fault");
    sim.dht_fail = false; sim_run_s(8);
    CHECK(!(g.faults & FAULT_DHT), "fault clears automatically after recovery");
    CHECK(g.state == ST_NORMAL, "back to NORMAL");
}
static void t10_mq_disconnected(void) {
    settle();
    sim.adc_force = 0; /* signal stuck at ground */
    sim_run_s(5);
    CHECK(g.faults & FAULT_MQ, "MQ-135 fault declared");
    CHECK(g.snap.aq_status == AQ_UNKNOWN, "air status unknown");
    ramp_temp(36.0f); sim_run_s(6);
    CHECK(g.alerts & (1u << CH_TEMP_HI), "temperature alert still works with MQ fault (NFR-02)");
    ramp_temp(25.0f); sim.adc_force = -1; sim_run_s(14);
    CHECK(!(g.faults & FAULT_MQ), "MQ fault clears after recovery");
    CHECK(g.snap.aq_status == AQ_GOOD, "air status GOOD again");
}
static void t11_oled_failure(void) {
    start();
    sim.oled_present = false;
    sim_boot(RESET_POWERON);
    CHECK(g.faults & FAULT_OLED, "OLED fault recorded at start-up (FR-22)");
    sim_run_ms(MQ_WARMUP_MS + 3000UL);
    ramp_temp(36.0f); sim_run_s(6);
    CHECK(g.alerts & (1u << CH_TEMP_HI), "alert works without OLED");
    CHECK(sim.led, "LED ON without OLED");
    CHECK(sim.tx_count > 0, "telemetry still sent without OLED");
    ramp_temp(25.0f); sim.oled_present = true; sim_run_s(40);
    CHECK(!(g.faults & FAULT_OLED), "OLED recovered by periodic retry");
    CHECK(sim.oled_writes > 0, "frames written after recovery");
}
static void t12_wifi_never_available(void) {
    start();
    sim.wifi_ap_up = false;
    sim_boot(RESET_POWERON);
    sim_run_s(120);
    CHECK(g.wifi != WIFI_UP, "node stays offline");
    CHECK(sim.attempts >= 4, "reconnect attempts continue");
    uint32_t gap1 = sim.attempt_ms[1] - sim.attempt_ms[0];
    uint32_t gap2 = sim.attempt_ms[2] - sim.attempt_ms[1];
    uint32_t gap3 = sim.attempt_ms[3] - sim.attempt_ms[2];
    printf("      gaps between connection attempts: %lu, %lu, %lu ms\n", (unsigned long)gap1,
           (unsigned long)gap2, (unsigned long)gap3);
    CHECK(gap1 < gap2 && gap2 < gap3, "back-off grows between attempts (FR-20)");
    CHECK(g.state != ST_INIT && g.stat.dht_ok > 40, "sensing continues while offline (FR-19)");
    ramp_temp(36.0f); sim_run_s(6);
    CHECK(g.alerts & (1u << CH_TEMP_HI), "alerting works offline (FR-19)");
    sim.wifi_ap_up = true; sim_run_s(70);
    CHECK(g.wifi == WIFI_UP, "reconnects within the back-off limit once the network returns");
}
static void t13_wifi_drop(void) {
    settle();
    sim_run_s(5);
    CHECK(g.wifi == WIFI_UP, "connected before the drop");
    sim.wifi_ap_up = false;
    uint32_t d0 = g.stat.dht_ok;
    sim_run_s(10);
    CHECK(g.wifi != WIFI_UP, "link loss detected");
    CHECK(strstr(sim.oled[0], "WiFi:OFF") != NULL || strstr(sim.oled[0], "WiFi:...") != NULL, "OLED shows Wi-Fi not OK");
    CHECK(g.stat.dht_ok >= d0 + 4, "sampling continues during outage");
    sim.wifi_ap_up = true; sim_run_s(12);
    CHECK(g.wifi == WIFI_UP, "automatic reconnection");
}
static void t14_telemetry(void) {
    settle();
    sim_run_s(75);
    CHECK(sim.tx_count >= 4, "periodic transmissions (FR-17)");
    bool intervals_ok = true;
    for (uint32_t i = 1; i < sim.tx_count && i < SIM_MAX_EVENTS; i++) {
        uint32_t d = sim.send_ms[i] - sim.send_ms[i - 1];
        if (d < TX_PERIOD_MS - 5 || d > TX_PERIOD_MS + 5) intervals_ok = false;
    }
    CHECK(intervals_ok, "transmission interval equals 15 s");
    unsigned long seq = 0;
    const char *p = strstr(sim.last_payload, "\"seq\":");
    CHECK(p && sscanf(p + 6, "%lu", &seq) == 1 && seq == sim.tx_count - 1, "sequence number increments by 1 per message (FR-18)");
    CHECK(strstr(sim.last_payload, "\"device_id\":\"env-node-01\"") != NULL, "device_id field");
    CHECK(strstr(sim.last_payload, "\"temp_c\":25.0") != NULL, "temperature field");
    CHECK(strstr(sim.last_payload, "\"aq_status\":\"GOOD\"") != NULL, "air status field");
    printf("      last payload: %s\n", sim.last_payload);
    sim.dht_fail = true; sim_run_s(35);
    CHECK(strstr(sim.last_payload, "\"temp_c\":null") != NULL, "invalid temperature sent as null");
}
static void t15_send_failure(void) {
    settle();
    sim.send_ok = false;
    sim_run_s(35);
    CHECK(g.stat.tx_fail >= 2, "failed sends counted, loop keeps running");
    CHECK(g.faults & FAULT_TX, "TX fault flagged");
    CHECK(g.wifi == WIFI_UP, "link itself still up");
    sim.send_ok = true; sim_run_s(20);
    CHECK(!(g.faults & FAULT_TX), "TX fault cleared after a successful send");
}
static void t16_uart_cli(void) {
    settle();
    sim_uart_clear(); sim_uart_send("status\n"); sim_run_ms(5);
    CHECK(uart_has("OK STATE=NORMAL"), "STATUS reports state (lower-case accepted)");
    sim_uart_clear(); sim_uart_send("THRESH\n"); sim_run_ms(5);
    CHECK(uart_has("TEMP_HI=35.0"), "THRESH lists thresholds");
    sim_uart_clear(); sim_uart_send("SET TEMP_HI 30\n"); sim_run_ms(5);
    CHECK(uart_has("OK TEMP_HI=30.0"), "SET accepted");
    sim.temp = 31.0f; sim_run_s(6);
    CHECK(g.alerts & (1u << CH_TEMP_HI), "new threshold is applied (FR-23)");
    sim_uart_clear(); sim_uart_send("MUTE ON\n"); sim_run_s(2);
    uint32_t bz = sim.buzzer_ms; sim_run_s(3);
    CHECK(uart_has("OK MUTE ON") && sim.buzzer_ms == bz && sim.led, "MUTE silences buzzer, LED stays on");
    sim_uart_clear(); sim_uart_send("SET TEMP_HI abc\n"); sim_run_ms(5);
    CHECK(uart_has("ERR usage"), "non-numeric value rejected");
    sim_uart_clear(); sim_uart_send("SET TEMP_HI 5\n"); sim_run_ms(5);
    CHECK(uart_has("ERR unknown name or value out of range"), "value below TEMP_LO rejected");
    sim_uart_clear(); sim_uart_send("FOO\n"); sim_run_ms(5);
    CHECK(uart_has("ERR unknown command"), "unknown command rejected");
    sim_uart_clear();
    sim_uart_send("SET TEMP_HI 3333333333333333333333333333333333333333333333333333\n"); sim_run_ms(5);
    CHECK(uart_has("ERR line too long"), "over-long line rejected safely");
    sim_uart_clear();
    for (int i = 0; i < 100; i++) uart_rx_isr('X'); /* flood without servicing */
    CHECK(g.rx_overflow > 0, "ring buffer overflow detected and counted, no corruption");
    sim_run_ms(5);             /* main loop drains the buffer */
    sim_uart_send("\n"); sim_run_ms(5); /* resynchronise: junk line is discarded */
    sim_uart_clear(); sim_uart_send("STATUS\n"); sim_run_ms(5);
    CHECK(uart_has("OK STATE="), "interpreter works normally after overflow");
}
static void t17_watchdog(void) {
    settle();
    sim_run_s(60);
    CHECK(sim.max_feed_gap < WDT_TIMEOUT_MS / 2, "watchdog fed regularly (max gap well below timeout)");
    ramp_temp(36.0f); sim_run_s(6);
    CHECK(sim.led, "alert active before simulated lock-up");
    uint32_t r0 = sim.reboots;
    sim.hang = true; /* main loop stops, timer interrupt keeps running */
    sim_run_ms(WDT_TIMEOUT_MS + 500UL);
    CHECK(sim.reboots == r0 + 1, "watchdog reset the firmware after the lock-up (FR-25)");
    CHECK(uart_has("reset cause WATCHDOG"), "reset cause logged as WATCHDOG");
    CHECK(!sim.led && !sim.buzzer, "outputs OFF after watchdog reset");
}
static void t18_power_interruption(void) {
    settle();
    ramp_temp(36.0f); sim_run_s(6);
    CHECK(sim.led, "alert active before power loss");
    sim_boot(RESET_BROWNOUT); /* supply dips, MCU restarts */
    CHECK(!sim.led && !sim.buzzer, "outputs OFF immediately after restart (FR-26)");
    CHECK(g.state == ST_WARMUP && g.ms == 0, "warm-up restarted, uptime restarted");
    CHECK(uart_has("reset cause BROWN-OUT"), "reset cause logged");
    sim_run_s(6);
    CHECK(g.alerts & (1u << CH_TEMP_HI), "alert re-evaluated from fresh data after restart");
}
static void t19_invalid_values(void) {
    settle();
    sim.temp = 32.0f; sim_run_s(6);
    sim.temp = 45.0f; run_until_dht_sample(); /* one 13 C jump: physically implausible */
    sim.temp = 32.0f; sim_run_s(6);
    CHECK(g.alerts == 0 && fabsf(g.snap.temp - 32.0f) < 0.01f, "single spike rejected, last valid value kept");
    CHECK(g.faults == 0, "isolated spike is not a fault");
    sim.temp = 120.0f; sim_run_s(8); /* persistent out-of-range output */
    CHECK(g.faults & FAULT_DHT, "persistent out-of-range data becomes a fault");
    CHECK(g.alerts == 0, "no alert from out-of-range data");
    sim.temp = 25.0f; sim_run_s(8);
    CHECK(!(g.faults & FAULT_DHT) && fabsf(g.snap.temp - 25.0f) < 0.01f, "recovers when data is valid again");
}
static void t20_display_efficiency(void) {
    settle();
    uint32_t w0 = sim.oled_writes;
    sim_run_s(30);
    uint32_t w = sim.oled_writes - w0;
    printf("      OLED frame writes in 30 s with constant data: %lu\n", (unsigned long)w);
    CHECK(w <= 30UL / (OLED_FULL_REFRESH_MS / 1000UL) + 1UL, "frame rewritten only on change or periodic refresh");
    CHECK(w >= 1, "periodic refresh still occurs");
}

typedef struct { const char *id, *name; void (*fn)(void); } test_t;
static const test_t TESTS[] = {
    {"T01", "Start-up sequence, safe outputs, warm-up", t01_startup},
    {"T02", "Normal operation and OLED content", t02_normal_display},
    {"T03", "Temperature alert and response time", t03_temp_alert_latency},
    {"T04", "Hysteresis (no chatter near threshold)", t04_hysteresis},
    {"T05", "Isolated over-limit sample ignored", t05_isolated_sample_ignored},
    {"T06", "Humidity and low-temperature alerts", t06_humidity},
    {"T07", "Air-quality alert, recovery", t07_air_quality},
    {"T08", "Warm-up suppresses air-quality alert", t08_warmup_suppresses_air_alert},
    {"T09", "DHT22 disconnected, recovery", t09_dht_disconnected},
    {"T10", "MQ-135 disconnected, recovery", t10_mq_disconnected},
    {"T11", "OLED absent at start, later recovery", t11_oled_failure},
    {"T12", "Wi-Fi unavailable, back-off, reconnect", t12_wifi_never_available},
    {"T13", "Wi-Fi drops during operation", t13_wifi_drop},
    {"T14", "Telemetry interval and payload", t14_telemetry},
    {"T15", "Transmission failure handling", t15_send_failure},
    {"T16", "UART commands and ring buffer", t16_uart_cli},
    {"T17", "Watchdog feeding and lock-up reset", t17_watchdog},
    {"T18", "Power interruption (brown-out) restart", t18_power_interruption},
    {"T19", "Spike and out-of-range data handling", t19_invalid_values},
    {"T20", "Display update efficiency", t20_display_efficiency},
};
#define N_TESTS ((int)(sizeof TESTS / sizeof TESTS[0]))

static int run_tests(bool verbose) {
    t_verbose = verbose;
    int passed = 0;
    int results[N_TESTS];
    printf("Smart Environmental Monitor v%s : self-test suite (simulated hardware)\n", FW_VERSION);
    printf("==============================================================\n");
    for (int i = 0; i < N_TESTS; i++) {
        int before = t_fail;
        printf("%s  %s\n", TESTS[i].id, TESTS[i].name);
        TESTS[i].fn();
        results[i] = (t_fail == before);
        if (results[i]) passed++;
        printf("      -> %s\n", results[i] ? "PASS" : "FAIL");
    }
    printf("==============================================================\n");
    for (int i = 0; i < N_TESTS; i++)
        printf("%s  %-46s %s\n", TESTS[i].id, TESTS[i].name, results[i] ? "PASS" : "FAIL");
    printf("--------------------------------------------------------------\n");
    printf("Tests passed: %d/%d   Checks executed: %d   Checks failed: %d\n", passed, N_TESTS, t_checks, t_fail);
    return t_fail == 0 ? 0 : 1;
}

/* =============================================================================
 * 11. DEMO AND main()
 * ===========================================================================*/
static void demo_note(const char *text) { printf("\n>>> [%6lu ms] SCENARIO: %s\n", (unsigned long)sim.total_ms, text); }

static void run_demo(void) {
    sim_defaults();
    sim.echo_uart = true;
    sim.print_oled = true;
    printf("Smart Environmental Monitor v%s : scripted demo (simulated hardware)\n", FW_VERSION);
    demo_note("power on, warm-up, normal operation");
    sim_boot(RESET_POWERON);
    sim_run_ms(MQ_WARMUP_MS + 3000UL);
    demo_note("temperature rises to 36 C (gradual ramp)");
    ramp_temp(36.0f); sim_run_s(6);
    demo_note("temperature falls to 34.5 C (inside hysteresis band): alert must remain");
    sim.temp = 34.5f; sim_run_s(6);
    demo_note("temperature falls to 33.5 C: alert clears");
    sim.temp = 33.5f; sim_run_s(6);
    demo_note("DHT22 unplugged");
    sim.dht_fail = true; sim_run_s(8);
    demo_note("DHT22 reconnected");
    sim.dht_fail = false; sim_run_s(6);
    demo_note("Wi-Fi access point lost, then restored");
    sim.wifi_ap_up = false; sim_run_s(8);
    sim.wifi_ap_up = true; sim_run_s(12);
    demo_note("UART commands");
    sim_uart_send("STATUS\n"); sim_run_ms(5);
    sim_uart_send("SET TEMP_HI 30\n"); sim_run_ms(5);
    sim_uart_send("THRESH\n"); sim_run_ms(5);
    sim_uart_send("SET TEMP_HI abc\n"); sim_run_ms(5);
    demo_note("end of demo");
    printf("Last telemetry message: %s\n", sim.last_payload);
}

int main(int argc, char **argv) {
    const char *mode = argc > 1 ? argv[1] : "";
    if (!strcmp(mode, "demo")) { run_demo(); return 0; }
    if (!strcmp(mode, "help") || !strcmp(mode, "--help")) {
        printf("usage: %s [verbose|demo|help]\n  (no argument) run the self-test suite\n  verbose       tests plus firmware UART log\n  demo          scripted scenario with OLED frames\n", argv[0]);
        return 0;
    }
    return run_tests(!strcmp(mode, "verbose"));
}
