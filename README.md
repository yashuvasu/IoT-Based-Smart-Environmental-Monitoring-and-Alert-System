 IoT-Based Smart Environmental Monitoring and Alert System 

Week 2 – Firmware Development and Implementation

An embedded C firmware project developed as part of the **Yuva Intern Embedded Systems Internship**. This project implements the control logic for an ESP32-based environmental monitoring system and validates its behaviour through a PC-based hardware simulation.

Project Overview

The IoT-Based Smart Environmental Monitoring and Alert System is designed to monitor temperature, humidity, and relative air-quality indications. It processes sensor readings, evaluates configurable thresholds, manages alerts, and prepares telemetry for remote reporting.

During Week 2, the focus was on firmware development, interrupt-driven scheduling, UART communication, fault handling, and systematic software testing.

 Week 2 was implemented and tested using a PC-based simulation with GCC. No physical ESP32 board or sensors were used.

 Objectives

- Implement modular embedded firmware using C99.
- Develop timer-driven task scheduling using a simulated 1 ms timer interrupt.
- Implement UART receive handling using an interrupt-fed ring buffer.
- Validate and filter temperature, humidity, and simulated air-quality readings.
- Implement threshold monitoring, hysteresis, and alert confirmation.
- Develop LED and buzzer alert logic.
- Implement a non-blocking Wi-Fi connection state machine and simulated telemetry.
- Provide a UART command-line interface for system status and threshold configuration.
- Verify firmware behaviour using automated tests and fault injection.

 Technologies and Tools

| Technology | Purpose |
|---|---|
| C99 | Firmware implementation |
| GCC | Compilation and execution |
| PC-based simulation | Simulated hardware and peripheral behaviour |
| Timer interrupt model | Periodic task scheduling |
| UART and ring buffer | Command reception and diagnostics |
| Hardware Abstraction Layer (HAL) | Separation of firmware logic from hardware |
| AddressSanitizer | Memory-error checking |
| UndefinedBehaviorSanitizer | Undefined-behaviour checking |
| Git and GitHub | Source-code version control and project documentation |

 System Architecture

The firmware follows a modular architecture in which the main control loop manages the application tasks, while the Hardware Abstraction Layer separates application logic from simulated peripherals.

System flow:

Sensor Inputs → Data Acquisition → Validation and Filtering → Threshold Monitoring → System State Management → Alert and Display Management → Simulated Wi-Fi Telemetry

UART commands provide an additional interface for inspecting status and changing supported thresholds.

Key Features Implemented

 1. Sensor acquisition and processing
- Simulated DHT22 temperature and humidity readings.
- Simulated 12-bit ADC readings for the MQ-135.
- Data validation, spike rejection, and moving-average filtering.
- Relative air-quality index based on a simulated baseline.

 2. Interrupt-driven scheduling
- A simulated timer interrupt generates periodic task flags.
- DHT22 acquisition is scheduled every 2 seconds.
- MQ-135 acquisition and display evaluation are scheduled every 1 second.
- Telemetry transmission is scheduled every 15 seconds.

 3. Alert management
- Configurable temperature and humidity thresholds.
- Confirmation of threshold violations across consecutive samples.
- Hysteresis to reduce repeated switching near threshold limits.
- Simulated LED and buzzer alert patterns.
- Optional buzzer muting.

 4. UART command interface

The firmware supports the following commands:

| Command | Function |
|---|---|
| `HELP` | Display available commands |
| `STATUS` | Show system state, readings, alerts, faults, and Wi-Fi status |
| `THRESH` | Display configured thresholds |
| `SET TEMP_HI 30` | Change the high-temperature threshold |
| `MUTE ON` | Mute the buzzer |
| `MUTE OFF` | Enable the buzzer again |

Threshold values are validated before being applied.

5. Fault handling and simulated connectivity
- Sensor fault detection and recovery.
- Display-fault detection and retry logic.
- Non-blocking Wi-Fi connection attempts and retry back-off.
- Continued local monitoring when simulated Wi-Fi is unavailable.
- Watchdog and reset behaviour represented in the simulation.

 Testing and Verification

The firmware includes **20 self-tests** covering normal operation, threshold alerts, hysteresis, sensor faults, Wi-Fi failures, UART commands, and other error scenarios.

The Week 2 report records:
- Self-tests for implemented firmware requirements.
- Fault-injection scenarios.
- Compiler warning checks using `-Wall -Wextra -Wpedantic`.
- AddressSanitizer and UndefinedBehaviorSanitizer runs.
- Simulated demonstration output.

The report records successful simulated test execution. These results validate the tested software model, not physical hardware performance.

 How to Compile and Run

Prerequisites
- GCC or Clang with C99 support.
- The `smart_env_monitor.c` source file.
- No external libraries are required beyond the standard C library and math library.

 Step 1: Compile the firmware

```bash
gcc -std=c99 -Wall -Wextra -Wpedantic -O2 -o smart_env_monitor smart_env_monitor.c -lm
```

 Step 2: Run the self-test suite

Linux / Ubuntu / macOS:

```bash
./smart_env_monitor
```

Windows with MinGW:

```bash
smart_env_monitor.exe
```

Step 3: Run additional modes

Run tests with detailed firmware UART logs:

```bash
./smart_env_monitor verbose
```

Run the scripted demonstration:

```bash
./smart_env_monitor demo
```

Display available program help:

```bash
./smart_env_monitor help
```

Save test output to a file:

```bash
./smart_env_monitor > test_results.txt
```

The program returns an exit code of `0` when all self-tests pass.

Project Files

```text
IoT-Environmental-Monitoring/
├── smart_env_monitor.c
├── README.md
├── test_results.txt
├── demo_output.txt
└── Week2_Technical_Report_Firmware_Development.pdf
```

The filenames above describe the intended project structure. Include each file in your repository only when it is actually available.

 Limitations

- All Week 2 testing was performed on a PC simulation.
- Real ESP32 interrupt timing and concurrency have not been verified.
- Physical sensor accuracy, MQ-135 calibration, and actual warm-up behaviour have not been tested.
- Wi-Fi communication and telemetry delivery are simulated; no live cloud dashboard is implemented.
- Actual power consumption and hardware timing have not been measured.
- Proposed thresholds and air-quality percentages require further validation.

 Future Improvements — Weeks 3 and 4

- Port the simulated HAL to actual ESP32 hardware.
- Integrate the DHT22, MQ-135, OLED, LED, and buzzer.
- Verify voltage levels and ADC signal conditioning.
- Test real UART and timer interrupt behaviour.
- Measure actual execution timing and current consumption.
- Validate fault recovery and Wi-Fi reconnection on hardware.
- Integrate a remote dashboard using an appropriate communication protocol.
- Perform system-level testing and document measured results.

 Internship Details

- Internship Organization: Yuva Intern
- Training Institute: Ignite Embedded Systems
- Project: IoT-Based Smart Environmental Monitoring and Alert System
- Week 1: System Requirements Analysis and Block Diagram Design
- Week 2: Firmware Development and Implementation
- Development Language: C99
- Testing Platform: PC-based simulation using GCC

 Conclusion

Week 2 focused on converting the system requirements defined in Week 1 into a modular, testable firmware implementation. The project demonstrates embedded C programming concepts, interrupt-driven scheduling, UART communication, sensor-data validation, threshold monitoring, fault handling, and automated software testing.

The next stage is to port the firmware to an ESP32 development board and verify the system under real hardware conditions.

---

Developed by Pilla Yashwanth  
Automobile Engineering Student | Embedded Systems Enthusiast
