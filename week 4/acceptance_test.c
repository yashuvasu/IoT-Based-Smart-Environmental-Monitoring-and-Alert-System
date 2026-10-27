/* Week 4 system-level acceptance test: one continuous "day in the life" scenario.
 * State carries over from step to step, unlike the isolated unit tests.
 * Every check is tagged with the Week 1 requirement it demonstrates. */
#define main fw_original_main
#include "smart_env_monitor.c"   /* the firmware under test, same folder */
#undef main
typedef struct { char id[10]; int pass, total; } req_t;
static req_t R[96]; static int nR;
static void REQ(const char *id, bool ok, const char *what) {
    int i; for (i = 0; i < nR; i++) if (!strcmp(R[i].id, id)) break;
    if (i == nR) { snprintf(R[nR].id, sizeof R[nR].id, "%s", id); R[nR].pass = R[nR].total = 0; nR++; }
    R[i].total++; if (ok) R[i].pass++; else printf("      FAIL %s: %s\n", id, what);
}
static void step(int n, const char *t) { printf("STEP %02d  %-62s [t=%lu s]\n", n, t, (unsigned long)(sim.total_ms / 1000)); }
static int cmp(const void *a, const void *b) { const req_t *x = a, *y = b; int fx = x->id[0]=='F', fy = y->id[0]=='F'; if (fx != fy) return fy - fx; return strcmp(x->id, y->id); }
int main(void) {
    sim_defaults(); sim_boot(RESET_POWERON);
    step(1, "Power on: safe outputs, initialisation, warm-up starts");
    REQ("FR-01", g.state == ST_WARMUP && g.oled_ok, "init");
    REQ("FR-26", !sim.led && !sim.buzzer, "outputs off at boot");
    REQ("FR-24", uart_has("BOOT"), "boot log");
    sim_run_s(5);
    step(2, "Warm-up, then normal operation, display and Wi-Fi");
    REQ("FR-07", g.snap.aq_status == AQ_WARMING && !g.alerts, "warm-up");
    sim_run_ms(MQ_WARMUP_MS + 3000UL - 5000UL);
    REQ("FR-07", g.baseline_valid && g.state == ST_NORMAL, "baseline after warm-up");
    REQ("FR-02", g.snap.temp_valid && fabsf(g.snap.temp - 25.0f) < 0.01f, "temp");
    REQ("FR-03", g.snap.hum_valid && fabsf(g.snap.hum - 50.0f) < 0.01f, "hum");
    REQ("FR-04", g.snap.aq_status == AQ_GOOD, "air");
    REQ("FR-05", g.snap.aq_raw > 700 && g.snap.aq_raw < 900, "averaged ADC value");
    REQ("FR-08", strstr(sim.oled[1], "25.0 C") && strstr(sim.oled[2], "50.0 %"), "oled t/h with units");
    REQ("FR-09", strstr(sim.oled[3], "GOOD") != NULL, "oled air");
    REQ("FR-10", strstr(sim.oled[4], "NONE") && strstr(sim.oled[0], "WiFi:OK"), "oled alert + wifi");
    REQ("FR-16", g.wifi == WIFI_UP, "wifi connected");
    REQ("NFR-05", strstr(sim.oled[4], "NONE") && strstr(sim.oled[3], "GOOD") && strstr(sim.oled[1], " C"), "plain-language status");
    step(3, "Periodic telemetry");
    sim_run_s(35);
    bool ok = sim.tx_count >= 3;
    for (uint32_t i = 1; i < sim.tx_count && i < SIM_MAX_EVENTS; i++) if (sim.send_ms[i] - sim.send_ms[i-1] != TX_PERIOD_MS) ok = false;
    REQ("FR-17", ok, "15 s telemetry interval");
    REQ("FR-18", strstr(sim.last_payload, "\"device_id\"") && strstr(sim.last_payload, "\"seq\"") && strstr(sim.last_payload, "\"temp_c\"") && strstr(sim.last_payload, "\"fault_flags\""), "payload fields");
    REQ("NFR-04", strstr(sim.last_payload, "\"device_id\":\"env-node-01\"") != NULL, "device id for multiple nodes");
    step(4, "Sensor glitches: one over-limit sample and one implausible spike");
    ramp_temp(32.0f); sim_run_s(6);
    uint32_t led0 = sim.led_ms, bz0 = sim.buzzer_ms, bad0 = g.stat.dht_bad;
    sim.temp = 36.0f; run_until_dht_sample(); sim.temp = 32.0f; sim_run_s(6);
    REQ("FR-12", g.alerts == 0 && sim.led_ms == led0 && sim.buzzer_ms == bz0, "single over-limit sample ignored");
    sim.temp = 45.0f; run_until_dht_sample();
    REQ("FR-06", g.stat.dht_bad == bad0 + 1U && fabsf(g.snap.temp - 32.0f) < 0.01f, "implausible spike rejected");
    sim.temp = 32.0f; sim_run_s(6); ramp_temp(25.0f); sim_run_s(4);
    step(5, "Temperature alert (36 C), hysteresis, clear");
    ramp_temp(34.0f); sim_run_s(4); sim.temp = 36.0f;
    uint32_t lat; run_until_alert(1u << CH_TEMP_HI, 8000, &lat); sim_run_s(1);
    REQ("FR-11", (g.alerts & (1u << CH_TEMP_HI)) != 0, "threshold compare");
    REQ("FR-13", sim.led && sim.buzzer_ms > 0, "led and buzzer");
    REQ("FR-15", sim.led && g.state == ST_ALERT, "alert pattern/state");
    REQ("NFR-07", lat <= 2U * DHT_PERIOD_MS, "alert latency");
    REQ("FR-10", strstr(sim.oled[4], "TEMP_HI") != NULL, "oled alert source");
    sim.temp = 34.5f; sim_run_s(8);
    REQ("FR-12", (g.alerts & (1u << CH_TEMP_HI)) != 0, "hysteresis");
    sim.temp = 33.5f; sim_run_s(6);
    REQ("FR-14", !g.alerts && !sim.led && !sim.buzzer, "alert cleared");
    ramp_temp(25.0f); sim_run_s(4);
    step(6, "Humidity alert and clear");
    ramp_hum(85.0f); sim_run_s(6);
    REQ("FR-11", (g.alerts & (1u << CH_HUM_HI)) != 0, "humidity alert");
    ramp_hum(50.0f); sim_run_s(8);
    REQ("FR-14", !g.alerts, "humidity cleared");
    step(7, "DHT22 unplugged during an alert, then reconnected");
    ramp_temp(36.0f); sim_run_s(6);
    REQ("FR-11", (g.alerts & (1u << CH_TEMP_HI)) != 0, "alert active before unplugging");
    sim.dht_fail = true; sim_run_s(10);
    uint32_t mq0 = g.stat.mq_samples; sim_run_s(5);
    REQ("FR-21", (g.faults & FAULT_DHT) && g.state == ST_DEGRADED, "dht fault");
    REQ("FR-06", g.alerts == 0 && strstr(sim.oled[1], "--.-"), "no use of invalid data");
    REQ("NFR-02", g.stat.mq_samples >= mq0 + 4, "other sensor continues");
    sim.dht_fail = false; sim_run_s(14);
    REQ("NFR-01", !(g.faults & FAULT_DHT) && (g.alerts & (1u << CH_TEMP_HI)), "automatic recovery, alert returns while 36 C persists");
    ramp_temp(25.0f); sim_run_s(10);
    REQ("NFR-01", g.state == ST_NORMAL && !sim.led, "back to normal");
    step(8, "Wi-Fi lost during an alert, then restored");
    sim.wifi_ap_up = false; sim_run_s(3); ramp_temp(36.0f); sim_run_s(6);
    REQ("FR-19", g.wifi != WIFI_UP && (g.alerts & (1u << CH_TEMP_HI)) && sim.led, "alert works offline");
    REQ("FR-10", strstr(sim.oled[0], "WiFi:OFF") || strstr(sim.oled[0], "WiFi:..."), "oled wifi status");
    sim.wifi_ap_up = true; sim_run_s(20);
    REQ("FR-20", g.wifi == WIFI_UP, "automatic reconnect");
    ramp_temp(25.0f); sim_run_s(10);
    step(9, "OLED removed and restored");
    sim.oled_present = false; sim_run_s(20); uint32_t tx0 = sim.tx_count; ramp_temp(36.0f); sim_run_s(20);
    REQ("FR-22", (g.faults & FAULT_OLED) && (g.alerts & (1u << CH_TEMP_HI)) && sim.led && sim.tx_count > tx0, "alert and telemetry without display");
    ramp_temp(25.0f); sim.oled_present = true; sim_run_s(40);
    REQ("NFR-01", !(g.faults & FAULT_OLED) && g.state == ST_NORMAL, "display recovered");
    step(10, "MQ-135 stuck, recovery, poor air alert");
    sim.adc_force = 0; sim_run_s(6);
    REQ("FR-21", (g.faults & FAULT_MQ) != 0 && g.snap.aq_status == AQ_UNKNOWN, "mq fault");
    sim.adc_force = -1; sim_run_s(15); sim.adc_base = 1600; sim_run_s(20);
    REQ("FR-04", (g.alerts & (1u << CH_AIR)) && g.snap.aq_status == AQ_POOR, "poor air");
    sim.adc_base = 800; sim_run_s(20);
    REQ("FR-14", !(g.alerts & (1u << CH_AIR)), "air alert cleared");
    step(11, "UART interface: STATUS, SET, MUTE");
    sim_uart_clear(); sim_uart_send("STATUS\n"); sim_run_ms(5);
    REQ("FR-24", uart_has("OK STATE=NORMAL"), "status");
    sim_uart_clear(); sim_uart_send("SET TEMP_HI 30\n"); sim_run_ms(5); sim.temp = 31.0f; sim_run_s(6);
    REQ("FR-23", uart_has("OK TEMP_HI=30.0") && (g.alerts & (1u << CH_TEMP_HI)), "runtime threshold");
    sim_uart_send("MUTE ON\n"); sim_run_ms(5); uint32_t bz = sim.buzzer_ms; sim_run_s(3);
    REQ("FR-13", sim.buzzer_ms == bz && sim.led, "mute keeps led");
    sim_uart_send("MUTE OFF\n"); sim_uart_send("SET TEMP_HI 35\n"); sim.temp = 25.0f; sim_run_ms(100); sim_run_s(8);
    step(12, "Transmission failure and recovery");
    sim.send_ok = false; sim_run_s(35);
    REQ("FR-17", (g.faults & FAULT_TX) && g.wifi == WIFI_UP, "tx failure handled");
    sim.send_ok = true; sim_run_s(20);
    REQ("NFR-01", !(g.faults & FAULT_TX), "tx recovered");
    step(13, "Power interruption during an alert");
    ramp_temp(36.0f); sim_run_s(6);
    bool had = sim.led; sim_boot(RESET_BROWNOUT);
    REQ("FR-26", had && !sim.led && !sim.buzzer && g.state == ST_WARMUP, "safe restart");
    REQ("FR-01", uart_has("reset cause BROWN-OUT"), "reset cause logged");
    sim_run_s(8);
    REQ("FR-11", (g.alerts & (1u << CH_TEMP_HI)) != 0, "alert re-evaluated after restart");
    ramp_temp(25.0f); sim_run_s(10);
    step(14, "Firmware lock-up recovered by the watchdog");
    uint32_t r0 = sim.reboots; sim.hang = true; sim_run_ms(WDT_TIMEOUT_MS + 500UL);
    REQ("FR-25", sim.reboots == r0 + 1 && uart_has("reset cause WATCHDOG") && !sim.led, "watchdog reset");
    sim_run_s(20);
    REQ("NFR-01", g.state != ST_INIT && g.wifi != WIFI_DOWN_WAIT + 99, "running again");
    printf("\nREQUIREMENT COMPLIANCE (system-level acceptance run)\n");
    qsort(R, (size_t)nR, sizeof R[0], cmp);
    int bad = 0;
    for (int i = 0; i < nR; i++) { printf("%-8s %-5s (%d/%d checks)\n", R[i].id, R[i].pass == R[i].total ? "PASS" : "FAIL", R[i].pass, R[i].total); if (R[i].pass != R[i].total) bad++; }
    printf("\nRequirements exercised: %d   failed: %d   simulated time: %lu s\n", nR, bad, (unsigned long)(sim.total_ms / 1000));
    return bad ? 1 : 0;
}
