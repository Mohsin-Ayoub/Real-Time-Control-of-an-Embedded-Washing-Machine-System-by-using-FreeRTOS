# Real-Time Control of an Embedded Washing Machine System using FreeRTOS

This repository contains the source code and hardware design for an industrial-style washing machine control system. Built on the **ESP32-C6**, the project demonstrates the use of a unicore FreeRTOS environment to manage high-power thermal loads and bidirectional motion control with real-time feedback.

## 📌 Project Overview
The system simulates a washing machine cycle by managing two primary subsystems:
1. **Thermal Subsystem:** Maintains "water" temperature using a 40W heater and NTC thermistor.
2. **Motion Subsystem:** Drives a drum (TT Motor) with bidirectional cycles and RPM monitoring.
3. **Safety Supervisor:** A dedicated high-priority watchdog task that monitors system health and handles emergency stops.

---

## 🛠 Hardware Architecture

### 1. Thermal Subsystem Implementation

* **MCU Pin:** GPIO 0 (Digital Out) -> Relay IN1
* **MCU Pin:** GPIO 2 (ADC In) -> NTC Thermistor Junction
* **Power:** 12V External Supply for a 40W Heating Element.

### 2. Motion Subsystem Implementation

* **MCU Pin:** GPIO 18 & 4 (PWM) -> L298N Motor Driver
* **MCU Pin:** GPIO 3 (Interrupt) -> MH-Series Speed Sensor (D0)
* **Power:** 5V External Supply for TT Gear Motor.

---

## 💻 Software Logic & RTOS Configuration

The system is developed using the **ESP-IDF** framework. It utilizes priority-based preemptive scheduling to ensure safety-critical tasks are never blocked.

### Task Priority Mapping
| Priority | Task Name | Description |
| :--- | :--- | :--- |
| **7** | `supervisor_task` | System watchdog; checks heartbeats every 5s. |
| **6** | `motor_task` | PID-based speed control & directional ramping. |
| **5** | `heating_task` | Time-proportioning PWM for relay-based heating. |

### Key Algorithms
* **Time-Proportioning PWM:** Since mechanical relays cannot switch at high frequencies, the PID output is mapped over a 10-second window (e.g., 20% power = 2s ON, 8s OFF).
* **Stall Detection:** The motor task monitors RPM. If PWM is applied but Hz feedback is missing, the system triggers an `EMERGENCY_STOP`.
* **Heartbeat Monitoring:** Sub-tasks must set a bit in a shared register; if a task hangs, the Supervisor resets the system safely.

---

## 🚀 Getting Started

### Prerequisites
* ESP-IDF v5.1 or later.
* ESP32-C6 DevKit.

### Build and Flash
```bash
# Set target to ESP32-C6
idf.py set-target esp32c6

# Build the project
idf.py build

# Flash and open monitor
idf.py -p [PORT] flash monitor
