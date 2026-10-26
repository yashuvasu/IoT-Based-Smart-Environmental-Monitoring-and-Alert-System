# IoT-Based Smart Environmental Monitoring and Alert System

![Target](https://img.shields.io/badge/target-ESP32-blue)
![Language](https://img.shields.io/badge/language-C99-informational)
![Progress](https://img.shields.io/badge/progress-4%20of%204%20weeks-brightgreen)
![Tests](https://img.shields.io/badge/tests-32%20passing-brightgreen)
![Testing](https://img.shields.io/badge/testing-simulation%20only-orange)

> A low-cost embedded system that **measures temperature, humidity and air quality, warns people locally when something is wrong, and reports to a remote dashboard**. It was built step by step during a four-week Embedded Systems Internship: **plan, build, debug and optimise, then integrate and verify**.

**Author:** Pilla Yashwanth (Yuva Intern)

![System at a glance](docs/images/system_overview.png)

---

## Contents

1. [What is this project?](#what-is-this-project)
2. [How it works](#how-it-works)
3. [How the project evolved](#how-the-project-evolved)
4. [Week 1: Plan](#week-1-plan-requirements-and-design)
5. [Week 2: Build](#week-2-build-firmware-in-simulation)
6. [Week 3: Debug and speed up](#week-3-debug-and-speed-up)
7. [Week 4: Integrate and verify](#week-4-integrate-and-verify-final-release)
8. [Final results](#final-results)
9. [Quick start](#quick-start)
10. [Repository structure](#repository-structure)
11. [Honest status and limitations](#honest-status-and-limitations)
12. [What is next](#what-is-next-hardware-validation)
13. [For reviewers: where to look](#for-reviewers-where-to-look)

---

## What is this project?

Rooms such as labs, workshops, storerooms and classrooms are rarely monitored. Heat, damp or stale air are noticed only after damage or discomfort has happened.

This project is the **software and design of a small monitoring node** that:

- measures **temperature, humidity and a relative air-quality indication**;
- shows the readings on a **small screen**;
- switches on an **LED and buzzer** when a limit is exceeded, even if the network is down;
- sends the data over **Wi-Fi** to a remote dashboard (a concept, not yet built).

It is designed around an **ESP32** microcontroller. The internship took it from an idea to tested firmware, with one documented step per week.

## How it works

The node follows three steps, shown in the picture above: **sense, think, act**.

| Step | What happens | Parts |
|---|---|---|
| **Sense** | Sensors are read every 1 to 2 seconds | DHT22 (temperature, humidity), MQ-135 (air-quality indication) |
| **Think** | The ESP32 checks that readings are plausible, smooths noise, compares them with limits and decides whether to raise an alert | ESP32 firmware (C) |
| **Act** | Shows values, sounds the alert, reports over Wi-Fi | 0.96" OLED, LED, buzzer, Wi-Fi |

### How an alert avoids false alarms

A sensor reading can jump for a moment. Two simple rules stop the buzzer from going off for no reason:

1. **Confirmation:** an alert needs **2 samples in a row** beyond the limit.
2. **Hysteresis:** once on, it only switches off after the value falls clearly **below** the limit, so it does not flicker around the threshold.

![How an alert works](docs/images/alert_logic.png)

*The curve illustrates the rule; it is not measured data.*

### Hardware (proposed)

| Part | Role |
|---|---|
| ESP32 development board | Brain: processing, Wi-Fi, inputs and outputs |
| DHT22 | Temperature and humidity |
| MQ-135 | Gas sensor used as a **relative** air-quality indicator (not a lab-grade analyser) |
| 0.96" OLED (I2C) | Local display |
| LED and buzzer | Local alert |
| 5 V USB supply, regulated to 3.3 V | Power |

---

## How the project evolved

Each week asked a harder question than the one before. The project did not just grow bigger; it grew **more trustworthy**.

![Project journey](docs/images/project_timeline.png)

| Week | Topic | Question asked | Status | Folder |
|---|---|---|---|---|
| 1 | Requirements and block diagram design | *What exactly must it do?* | Done | [`week1/`](week1/) |
| 2 | Firmware development and implementation | *Does it work?* | Done (simulated) | [`week2/`](week2/) |
| 3 | Debugging and performance optimization | *What is still wrong, and what does it cost?* | Done (simulated) | [`week3/`](week3/) |
| 4 | System integration and final testing | *Does the whole system meet every requirement?* | Done (simulated) | [`week4/`](week4/) |

The charts below show what grew between weeks: the number of tests and checks, how much of the code the tests reach, and how long the system survives random faults.

![How the project grew](docs/images/project_growth.png)

---

## Week 1: Plan (requirements and design)

**Goal:** decide exactly what the system must do before writing any code.

**What was produced**

- **26 functional** and **13 non-functional** requirements, each numbered so it can be traced.
- Input, output, interface and power specifications.
- A hardware **block diagram**, a software **flowchart**, a data-flow description and a fault analysis.
- A **requirement traceability matrix** linking every requirement to a component and a software module.

![Block diagram](docs/images/week1_block_diagram.png)

Key design decisions made here: the MQ-135 output is scaled down before it reaches the ESP32 (it can exceed 3.3 V), the air sensor is read on a channel that still works while Wi-Fi is on, and local alerts must work without any network.

**State after Week 1:** a complete specification and design. No code yet.

Details: [`week1/`](week1/)

---

## Week 2: Build (firmware in simulation)

**Goal:** turn the design into working firmware and test it, without physical hardware.

**What was produced**

- The whole firmware in **one C file**, plus a **simulated board** so it can be compiled and tested on a normal PC.
- A **timer interrupt** and a **UART interrupt** with a ring buffer, a non-blocking scheduler, validation and filtering of sensor data, a state machine, an OLED manager, Wi-Fi handling with back-off, a command interface and fault handling.
- A **self-test suite**: 20 tests (112 checks) that simulate unplugged sensors, a dead display, Wi-Fi loss, power interruption and a locked-up program.

![Firmware architecture](docs/images/week2_firmware_architecture.png)

The firmware keeps **hardware access in one layer**, so moving to real hardware means replacing only that layer.

**State after Week 2:** working firmware, all 20 tests passing.

Details: [`week2/`](week2/)

---

## Week 3: Debug and speed up

**Goal:** find what was still wrong with the Week 2 firmware, and make it cheaper to run.

Week 2's tests all passed, so Week 3 used **tools that look for problems the tests do not**: static analysis, Valgrind, code coverage, fuzzing, a 7-day randomised fault test and *mutation testing* (deliberately injecting bugs to see whether the tests notice).

**Defects found and fixed**

| ID | Problem | Found by |
|---|---|---|
| D1 | The display check compared uninitialised memory, so the screen was rewritten every second in one build mode | Valgrind |
| D2 | After 49.7 days the millisecond counter wraps; the device fell back into warm-up and muted the air alert | Time-warp test |
| D3 | Changing a threshold silenced an active alert | Targeted probing |
| D4 | Static-analysis and type-conversion findings | cppcheck |
| D5 | A fault cleared late in a loop pass took effect 1 ms late | 7-day soak test |

**Design changes, driven by measurement**

![How the firmware design evolved](docs/images/design_evolution.png)

**Performance (measured in the simulation)**

| Measure | Week 2 | Week 3 |
|---|---|---|
| Firmware instructions (10 min) | 101.3 M | 52.6 M (**-48%**) |
| Timer interrupt instructions | 20.4 M | 2.4 M (**-88%**) |
| Display (I2C) traffic per hour | 1.75 MB | 0.38 MB (**-78.5%**) |

![Week 3 results](docs/images/week3_results.png)

Every alert, LED and state change happens at exactly the same moment as before, checked with a fingerprint of the whole benchmark scenario.

**How well is it tested now?** 32 tests (163 checks), no compiler warnings, no memory errors, 99.3% line coverage, and **16 of 16 deliberately injected bugs caught**. That last result only reached 16 after six of my own tests were found too weak and strengthened.

**State after Week 3:** five defects fixed, firmware about half as costly to run, tests that were themselves tested.

Details: [`week3/`](week3/)

---

## Week 4: Integrate and verify (final release)

**Goal:** test the complete system as one, against every requirement from Week 1, and close the project.

Unit tests start each test from a clean state. Week 4 added something different: **one continuous scenario** in which events follow each other and the system must recover from each without help.

![How the testing evolved](docs/images/testing_evolution.png)

**What was produced**

- A **14-step acceptance scenario** (about 8.5 simulated minutes): power on, warm-up, alerts, an unplugged sensor, Wi-Fi loss, a removed display, a stuck gas sensor, a power cut and a locked-up program.
- **All 26 functional and 5 non-functional requirements demonstrated, 0 failures.** The acceptance test itself was checked by injecting 7 deliberate bugs. Three slipped through at first, so the scenario was extended twice until all 7 were caught.
- A **30-simulated-day** random-fault run: 12,847 events, 223 injected resets, 0 violations.
- A **final report** with a requirement compliance matrix, a **15-step hardware validation checklist**, limitations, and a submission checklist.

**State after Week 4:** a frozen, verified release (firmware v3.0.0) and a clear list of what still needs real hardware.

Details: [`week4/`](week4/)

---

## Final results

| Area | Result |
|---|---|
| Requirements | 39 written (26 functional, 13 non-functional); all 26 functional demonstrated in the acceptance run |
| Tests | 32 tests, 163 checks, all passing at four compiler optimisation levels |
| Acceptance run | 14 steps, 31 requirement identifiers, 0 failed |
| Code quality | 0 warnings, no sanitizer or Valgrind errors, 99.3% line coverage |
| Test quality | 16 of 16 (unit suite) and 7 of 7 (acceptance run) injected bugs caught |
| Long-run | 30 simulated days of random faults, 0 violations, 0 watchdog resets |
| Efficiency vs Week 2 | 48% less firmware work, 88% less timer-interrupt work, 78.5% less display traffic |
| Defects found and fixed | 5, each with a regression test |

---

## Quick start

You only need a C compiler such as GCC. No other software is required.

```bash
# get the code
git clone https://github.com/<your-username>/<repository-name>.git
cd <repository-name>/week4

# compile
gcc -std=c99 -Wall -Wextra -Wpedantic -O2 -o smart_env_monitor smart_env_monitor.c -lm

# run the self-tests (should end with "Tests passed: 32/32")
./smart_env_monitor

# watch a scripted scenario, including the simulated OLED screen
./smart_env_monitor demo
```

Other modes: `./smart_env_monitor bench 600` (benchmark) and `./smart_env_monitor soak 24` (24 simulated hours of random faults).

To run the **end-to-end acceptance test** (it needs `smart_env_monitor.c` in the same folder):

```bash
gcc -std=gnu99 -O2 -o acceptance_test acceptance_test.c -lm
./acceptance_test        # ends with: Requirements exercised: 31   failed: 0
```

Earlier versions are in [`week2/`](week2/) and [`week3/`](week3/) and build the same way.

## Repository structure

```
.
├── README.md                  this page
├── docs/images/               diagrams used in the READMEs
├── week1/                     requirements, block diagram, flowchart, report
├── week2/                     firmware v2, tests, demo output, report
├── week3/                     firmware v3, analysis tools, results, report
└── week4/                     final release, acceptance test, final report
```

---

## Honest status and limitations

- **Everything was tested in simulation.** No physical ESP32, sensor, display or buzzer has been built or measured.
- **Numbers are simulation numbers.** Instruction counts are from a PC, so they show relative improvement, not ESP32 timing. Display bus time is calculated from byte counts.
- **Not verified, because it needs real hardware:** power draw, electrical behaviour, environmental operating range and real network security.
- **The MQ-135 is a prototype air-quality indicator.** It is not selective, needs warm-up and calibration, and cannot give reliable gas concentrations. The system must not be used as a safety or life-protection detector.
- **The dashboard is a concept.** No cloud service has been deployed.
- Thresholds, air-quality percentages and the warm-up time are **initial proposals** and still need tuning with real data.

## What is next (hardware validation)

The four internship weeks are complete. The remaining step is to run the firmware on real hardware. The final report contains a 15-step checklist with pass criteria:

- Replace the simulated hardware layer with real drivers (the only part of the code that changes).
- Measure supply current, loop timing, interrupt duration and I2C behaviour.
- Calibrate the MQ-135 warm-up and baseline, and compare the DHT22 with a reference.
- Run a 24-hour unattended test.

Later ideas (not implemented): a cloud dashboard, a mobile app, a better air-quality sensor, battery operation and secure over-the-air updates.

## For reviewers: where to look

| If you have | Look at |
|---|---|
| **3 minutes** | The pictures on this page and the [Final results](#final-results) table |
| **10 minutes** | [`week4/Week4_Final_Technical_Report.docx`](week4/): executive summary, compliance matrix, hardware validation plan |
| **30 minutes** | Read `week4/smart_env_monitor.c` from the top: the header explains the file map, and the code is commented section by section |

---

*Author: Pilla Yashwanth, Yuva Intern. License: [add a license of your choice].*
