 IoT-Based Smart Environmental Monitoring and Alert System: Week 3

**Debugging and Performance Optimization (simulated embedded system)**
Author: Pilla Yashwanth (Yuva Intern)

Week 3 takes the Week 2 firmware and asks two questions: what is still wrong with it, and what does it cost to run? It is still one C file, tested on a PC with a simulated hardware layer. No physical hardware was used.

## What was done

**Defects found and fixed (all with regression tests)**

| ID | Problem | Found with |
|---|---|---|
| D1 | Display change check compared uninitialised memory (OLED rewritten every second at `-O0`) | Valgrind |
| D2 | 32-bit millisecond counter wraps after 49.7 days: state fell back to WARMUP | Time-warp test |
| D3 | `SET` silenced an active alert | Targeted probing |
| D4 | Static-analysis and type-conversion findings | cppcheck, `-Wconversion` |
| D5 | A fault changed late in a loop pass took effect 1 ms late | 7-day soak test |

**Performance (measured in the simulation)**

| Metric | Week 2 | Week 3 |
|---|---|---|
| Firmware instructions, 10 min | 101.3 M | 52.6 M (-48%) |
| Timer ISR instructions | 20.4 M | 2.4 M (-88%) |

| I2C bytes per hour | 1,753,088 | 376,704 (-78.5%) |

The behaviour fingerprint of the benchmark (every alert, LED and state change with its timestamp) is identical in both versions.

## Verification

32 tests (163 checks), zero warnings at `-O0/-O2/-O3/-Os`, no AddressSanitizer/UBSan/Valgrind errors, 99.3% line coverage of the firmware, 16 of 16 injected defects caught by the tests (mutation testing), and a 7-simulated-day randomised fault-injection soak test with 0 violations.

## Compile and run

```bash
gcc -std=c99 -Wall -Wextra -Wpedantic -O2 -o smart_env_monitor smart_env_monitor.c -lm
./smart_env_monitor            # 32 self-tests
./smart_env_monitor demo       # scripted scenario with OLED frames
./smart_env_monitor bench 600  # fixed benchmark scenario
./smart_env_monitor soak 24    # randomised soak test (24 simulated hours)
```

## Files

| File | Purpose |
|---|---|
| `smart_env_monitor.c` | Firmware v3.0.0 (single file) |
| `Week3_Technical_Report_*.docx` | Full report |
| `test_results_week3.txt`, `demo_output_week3.txt` | Captured output |
| `week3_analysis_tools.zip` | Week 2 baseline, bug-hunt harness, benchmark harness, analysis logs |

## Limits

All results come from a PC simulation. Instruction counts are x86 host counts (relative work, not ESP32 cycles) and I2C times are analytical. Hardware timing, interrupt races and current draw are for Week 4./
