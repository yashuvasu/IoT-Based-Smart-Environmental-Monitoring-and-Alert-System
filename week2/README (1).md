# IoT-Based Smart Environmental Monitoring and Alert System: Week 2

**Firmware Development and Implementation (simulated embedded system)**
Author: Pilla Yashwanth (Yuva Intern)

This repository holds the Week 2 deliverables of a four-week Embedded Systems Internship. The firmware of the ESP32-based environmental monitor designed in Week 1 is implemented in a **single C file** and tested on a PC using a **simulated hardware layer**. No physical hardware was used.

## What the firmware does

- Samples a **DHT22** (temperature, humidity) every 2 s and an **MQ-135** (relative air quality) every 1 s.
- **Validates** data (range, spike, missing), **filters** the MQ-135 with an 8-sample average and converts it to a baseline-relative index.
- Raises alerts with **configurable thresholds, hysteresis and 2-sample confirmation**, and drives an **LED and buzzer**.
- Updates a 128x64 **OLED** (temperature, humidity, air status, alert, Wi-Fi status).
- Sends **telemetry** (JSON) every 15 s over Wi-Fi, with **non-blocking reconnection and back-off**. Local monitoring continues when Wi-Fi is down.
- Accepts **UART commands** (`STATUS`, `THRESH`, `SET`, `MUTE`, `HELP`).
- Handles faults: sensor disconnected, invalid values, OLED failure, Wi-Fi loss, send failure, brown-out and watchdog reset.

## Embedded techniques demonstrated

| Technique | Where |
|---|---|
| **Timer interrupt** (1 ms tick, raises flags only) | `timer_isr_1ms()` |
| **UART receive interrupt** with ring buffer and overflow counting | `uart_rx_isr()`, `cli_poll()` |
| Cooperative, non-blocking scheduler (no `delay()`) | `fw_loop()` |
| Hardware abstraction layer (HAL) | section 3 and section 9 of the file |
| State machine (INIT, WARMUP, NORMAL, ALERT, DEGRADED) | `state_update()` |
| Hysteresis and confirmation counting | `ch_eval()` |
| Watchdog and reset-cause handling | `hal_wdt_*`, `fw_setup()` |

## Files

| File | Purpose |
|---|---|
| `smart_env_monitor.c` | The complete firmware, simulated HAL and self-test suite (one file, with inline comments and an embedded README) |
| `README.md` | This file |
| `test_results.txt` | Captured output of the self-test suite |
| `demo_output.txt` | Captured output of the scripted demo (log plus OLED frames) |
| `Week2_Technical_Report_*.docx` | Full Week 2 report |

## Requirements

A C99 compiler (GCC or Clang). No other libraries.

## Compile

```bash
gcc -std=c99 -Wall -Wextra -Wpedantic -O2 -o smart_env_monitor smart_env_monitor.c -lm
```

Windows (MinGW): the same command; the output is `smart_env_monitor.exe`.

## Run

```bash
./smart_env_monitor            # run the 20-test self-test suite
./smart_env_monitor verbose    # same, plus the firmware's UART log
./smart_env_monitor demo       # scripted scenario with OLED frames
./smart_env_monitor help
```

The exit code is `0` when all tests pass, `1` otherwise.

Expected end of the test output:

```
Tests passed: 20/20   Checks executed: 112   Checks failed: 0
```

## UART commands

Case-insensitive, end each command with a newline.

| Command | Action |
|---|---|
| `HELP` | List commands |
| `STATUS` | State, readings, flags, Wi-Fi, uptime |
| `THRESH` | Show thresholds |
| `SET <TEMP_HI\|TEMP_LO\|HUM_HI\|HUM_LO> <value>` | Change a threshold (range and ordering are checked) |
| `MUTE <ON\|OFF>` | Silence the buzzer (LED stays on) |

## Key functions

| Function | Role |
|---|---|
| `fw_setup()` | Start-up sequence; outputs OFF first; failures are recorded, not fatal |
| `fw_loop()` | One pass of the main loop |
| `timer_isr_1ms()` | System tick and task flags |
| `uart_rx_isr()` | Receive byte into ring buffer |
| `dht_task()`, `mq_task()` | Acquire, validate, filter, evaluate thresholds |
| `ch_eval()` | Hysteresis and confirmation for one alert channel |
| `state_update()` | System state from alerts, warm-up and faults |
| `alert_task()` | LED and buzzer patterns |
| `display_task()` | OLED frame, redraw on change plus periodic refresh |
| `wifi_task()` | Connection state machine, back-off, telemetry |
| `cli_poll()`, `cli_exec()` | UART command interpreter |

## Simulation notes

The simulated HAL (section 9 of the C file) lets tests inject sensor values and failures and observe the LED, buzzer, OLED, Wi-Fi traffic and UART output. Time is a virtual millisecond clock, so tests are fast and repeatable.

Values chosen for simulation, to be revisited on real hardware: the MQ-135 warm-up is 10 s (`MQ_WARMUP_MS`), whereas real sensors need several minutes; the thresholds and air-quality percentages are initial proposals, not calibrated values.

## Porting to ESP32

Only section 9 must be replaced with real drivers (DHT library, `analogRead` on GPIO34, SSD1306 on I2C GPIO21/22, WiFi client, Serial, task watchdog, hardware timer). **This port has not been done or tested in Week 2.**

## Status and roadmap

- **Week 1:** Requirements and architecture (done)
- **Week 2:** Firmware and simulated testing (this repository)
- **Week 3:** Debugging and performance optimization
- **Week 4:** System integration and final testing

All test results in this repository come from the PC simulation. No measurements on physical hardware exist yet.
