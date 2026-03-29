# 🐝 BUMBLEBEE-CP — Semi-Autonomous Custom Drone Flight Controller

<p align="center">
  <img src="https://img.shields.io/badge/Arduino-UNO%20%7C%20Nano-00979D?style=for-the-badge&logo=arduino&logoColor=white" />
  <img src="https://img.shields.io/badge/C++-Arduino%20Sketch-00599C?style=for-the-badge&logo=cplusplus&logoColor=white" />
  <img src="https://img.shields.io/badge/RF24-NRF24L01%20%40%20250kbps-E24329?style=for-the-badge" />
  <img src="https://img.shields.io/badge/IMU-MPU--6050-FF6F00?style=for-the-badge" />
  <img src="https://img.shields.io/badge/Protocol-BB--CP%20%2B%20Reed--Solomon-6A1B9A?style=for-the-badge" />
  <img src="https://img.shields.io/badge/PID-Roll%20%7C%20Pitch%20%7C%20Yaw-2E7D32?style=for-the-badge" />
  <img src="https://img.shields.io/badge/License-Open%20Source-blue?style=for-the-badge&logo=opensourceinitiative&logoColor=white" />
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Platform-AVR%20ATmega328P-9C27B0?style=flat-square" />
  <img src="https://img.shields.io/badge/Baud%20Rate-230400-37474F?style=flat-square" />
  <img src="https://img.shields.io/badge/RF%20Channel-108-1565C0?style=flat-square" />
  <img src="https://img.shields.io/badge/RS%20Correction-2%20byte%20errors-C62828?style=flat-square" />
  <img src="https://img.shields.io/badge/Motors-4×%20Brushless%20(X--frame)-33691E?style=flat-square" />
</p>

---

## 🧠 About This Project

**BUMBLEBEE-CP** is a fully custom-built semi-autonomous quadcopter drone, designed and programmed from scratch on Arduino hardware — no off-the-shelf flight stack like Betaflight or ArduPilot is used. Every layer of the system, from RF communication to PID stabilization, is hand-written in C++.

The drone is operated remotely from a **Ground Station** (a second Arduino Nano), which sends flight commands over a **2.4 GHz RF link** using a custom-designed communication protocol called **BB-CP (BUMBLEBEE Custom Protocol)**. All packets are protected with **Reed-Solomon(12,8) error correction**, implemented entirely in software on the microcontroller — meaning the link can tolerate up to 2 corrupted bytes per packet and still recover the original command perfectly.

On the drone itself, a **dual-board architecture** separates concerns cleanly: one Nano handles all RF communication and packet decoding, while an Arduino UNO acts as the dedicated flight controller — reading the MPU-6050 IMU sensor, running a PID loop for attitude stabilization across roll, pitch, and yaw axes, and driving 4 brushless motors via ESCs.

### What Makes This Project Unique

- **No off-the-shelf flight stack** — everything runs on bare Arduino with fully custom firmware
- **Custom wireless protocol (BB-CP)** — purpose-built packet format with sequence numbering, source/destination addressing, and hardware-tolerant error correction
- **Reed-Solomon error correction in GF(256)** — implemented from scratch in C++ on a microcontroller with only 2KB of RAM, using PROGMEM to store Galois Field tables in flash memory
- **Semi-autonomous flight modes** — the drone can autonomously takeoff, hover, fire a payload, and land; the Ground Station simply issues high-level mode commands
- **Fault-tolerant design** — 50° tilt cutoff safety, emergency stop packet, RF drop resilience, and smooth throttle ramping on takeoff/landing

### System Overview

```
  ┌──────────────────────────┐                        ┌──────────────────────────────────────┐
  │     GROUND STATION       │   2.4 GHz NRF24L01     │              DRONE                   │
  │     Arduino Nano         │ ══════════════════════> │  ┌───────────────────────────────┐   │
  │                          │   BB-CP Protocol        │  │   Arduino Nano  (Drone-CP)    │   │
  │  - Keyboard commands     │   250 kbps / CH 108     │  │   RF receive + RS decode      │   │
  │  - Builds BB-CP packets  │ <══════════════════════ │  │   Forward to UNO via Serial   │   │
  │  - RS encodes payload    │   PONG / Telemetry      │  └──────────────┬────────────────┘   │
  │  - Sends via NRF24L01    │                         │                 │ SoftwareSerial      │
  └──────────────────────────┘                         │                 │ 38400 baud          │
                                                        │  ┌──────────────▼────────────────┐   │
                                                        │  │   Arduino UNO (Flight Ctrl)   │   │
                                                        │  │   MPU-6050 IMU → PID loop     │   │
                                                        │  │   4× ESC PWM output           │   │
                                                        │  │   Flight mode state machine   │   │
                                                        │  └───────────────────────────────┘   │
                                                        └──────────────────────────────────────┘
```

---

## 🗂️ Repository Structure

```
BUMBLEBEE-CP/
├── firmware/
│   ├── BUMBLEBEE_Drone_Nano_Phase6_FINAL/   ← Drone-side Nano (RF receive + RS decode)
│   ├── BUMBLEBEE_GS_Nano_Phase6b/           ← Ground Station Nano (transmitter)
│   └── Land_fix/                            ← Arduino UNO Flight Controller (latest)
│
├── tools/                                   ← Calibration & diagnostic sketches
│   ├── ESC_Calibration/                     ← Full ESC calibration routine
│   ├── ESC_Callibration_Test/               ← ESC test with manual throttle
│   ├── Motor_Direction_Test/                ← Verify CW/CCW per motor position
│   ├── MPU_Angle_Read/                      ← Live MPU-6050 roll/pitch output
│   ├── MPU_Wrining_Verify/                  ← MPU-6050 wiring check
│   └── Trottle_Test/                        ← Throttle sweep test
│
└── docs/
    ├── WIRING.md                            ← Full wiring reference
    ├── NRF_to_Nano.png                      ← Schematic: NRF24L01 ↔ Nano
    ├── Uno_to_ESC_wirring.png               ← Schematic: UNO ↔ ESCs
    ├── Uno_to_MPU6050_wiring.png            ← Schematic: UNO ↔ MPU-6050
    └── Uno_to_Rx_Nano.png                   ← Schematic: Nano ↔ UNO Serial
```

---

## ⚙️ Hardware Requirements

| Component | Qty | Role |
|---|---|---|
| Arduino UNO | 1 | Flight Controller — PID loop + ESC output |
| Arduino Nano | 2 | Drone-CP RF receiver + Ground Station transmitter |
| MPU-6050 IMU | 1 | 6-axis gyro/accelerometer for attitude sensing |
| NRF24L01 Module | 2 | 2.4 GHz RF transceiver (one per Nano) |
| Brushless Motor | 4 | Standard quadcopter X-frame layout |
| ESC | 4 | PWM input, calibrated to 1000–2000 µs range |
| LiPo Battery | 1 | 3S or 4S recommended |
| 100µF Capacitor | 2 | Power stabilisation on NRF24L01 VCC pins |

---

## 📡 BB-CP Protocol

All communication uses the custom **BB-CP (BUMBLEBEE Custom Protocol)** — a lightweight packet protocol designed specifically for this project. Every packet is protected with **Reed-Solomon(12,8)** forward error correction over GF(256), capable of correcting up to **2 corrupted bytes** per 32-byte RF transmission without any retransmission needed.

### Over-the-Air Packet Format (32 bytes)

```
┌──────┬──────┬─────┬─────┬─────┬──────┬─────┬──────────────────┬───────────────┐
│ 0xBB │ 0xEE │ SRC │ DST │ SEQ │ TYPE │ LEN │  DATA (0–8 B)    │ RS Parity ×4  │
└──────┴──────┴─────┴─────┴─────┴──────┴─────┴──────────────────┴───────────────┘
  Start  Start  Addr  Addr  Seq   Cmd    Len    Payload             Error Correction
```

### Packet Types

| Constant | Value | Direction | Description |
|---|---|---|---|
| `PKT_CMD_FLIGHT` | `0x01` | GS → Drone | Throttle, pitch, roll, yaw + flight mode |
| `PKT_CMD_FIRE` | `0x02` | GS → Drone | Trigger payload fire |
| `PKT_EMERGENCY_STOP` | `0x03` | GS → Drone | Immediate motor cutoff |
| `PKT_TLM_STATUS` | `0x04` | Drone → GS | Telemetry status |
| `PKT_PING` | `0x06` | GS → Drone | Heartbeat check |
| `PKT_PONG` | `0x07` | Drone → GS | Heartbeat reply with uptime |

### Internal Serial Format (Nano → UNO)

After RS decoding, the Drone Nano forwards commands to the UNO over SoftwareSerial at **38400 baud** using a simplified framing with XOR checksum:

```
[0xBB][0xEE][TYPE][LEN][DATA 0–8 bytes][XOR_CRC]
```

---

## 🚁 Flight Modes

The UNO runs a state machine with 7 flight modes, all triggerable from the Ground Station:

| Mode | Value | Behaviour |
|---|---|---|
| `MODE_IDLE` | `0x00` | All motors off, waiting for command |
| `MODE_TAKEOFF` | `0x01` | Smooth throttle ramp-up over 4 seconds |
| `MODE_HOVER` | `0x02` | Hold stable altitude with PID active |
| `MODE_TRACK` | `0x03` | Follow / track target |
| `MODE_ALIGN` | `0x04` | Align heading to target |
| `MODE_FIRE` | `0x05` | Fire payload mechanism |
| `MODE_LAND` | `0x06` | Controlled throttle ramp-down over 5 seconds |

---

## 🔧 PID Configuration

```cpp
float Kp = 0.85f;   // Proportional — immediate response to tilt error
float Ki = 0.002f;  // Integral     — corrects steady-state drift
float Kd = 0.18f;   // Derivative   — dampens oscillation

#define THROTTLE_IDLE       1000  // µs — motors off
#define THROTTLE_MIN_SPIN   1150  // µs — just starts spinning
#define THROTTLE_HOVER      1420  // µs — approximate hover point
#define THROTTLE_MAX        1950  // µs — full power

#define TILT_CUTOFF         50.0f // degrees — auto motor-kill if flipped
```

---

## 🔌 Wiring Summary

### Drone Nano (RF Receiver & CP Decoder)
| Signal | Pin |
|---|---|
| NRF24 CE | D7 |
| NRF24 CSN | D8 |
| NRF24 SCK | D13 |
| NRF24 MOSI | D11 |
| NRF24 MISO | D12 |
| TX → UNO D11 | D10 |
| RX ← UNO D12 | D4 |

### Arduino UNO (Flight Controller)
| Signal | Pin |
|---|---|
| ESC1 Front-Left (CW) | D3 |
| ESC2 Front-Right (CCW) | D5 |
| ESC3 Rear-Right (CW) | D6 |
| ESC4 Rear-Left (CCW) | D9 |
| RX ← Nano D10 | D11 |
| TX → Nano D4 | D12 |
| MPU-6050 SDA | A4 |
| MPU-6050 SCL | A5 |

> ⚠️ NRF24L01 **must** be powered from **3.3V**, not 5V — 5V will permanently damage the module.

---

## 🗺️ Schematic Diagrams

> Full pin-by-pin reference: [`docs/WIRING.md`](docs/WIRING.md)

### 1. NRF24L01 ↔ Arduino Nano Interface
![NRF24L01 to Arduino Nano Schematic](docs/NRF_to_Nano.png)

### 2. Arduino UNO ↔ ESC Wiring (Motor PWM Control)
![UNO to ESC Wiring Schematic](docs/Uno_to_ESC_wirring.png)

### 3. MPU-6050 IMU ↔ Arduino UNO (I²C Link)
![UNO to MPU-6050 Wiring Schematic](docs/Uno_to_MPU6050_wiring.png)

### 4. Drone Nano ↔ Arduino UNO (SoftwareSerial Link)
![Drone Nano to UNO SoftwareSerial Schematic](docs/Uno_to_Rx_Nano.png)

---

## 🛠️ Setup & Flashing Order

1. **Calibrate ESCs** — Flash `tools/ESC_Calibration` to UNO, follow Serial Monitor instructions with LiPo connected.
2. **Verify Motor Directions** — Flash `tools/Motor_Direction_Test`, confirm CW/CCW per position (props off!).
3. **Verify MPU-6050** — Flash `tools/MPU_Angle_Read`, tilt the board and confirm roll/pitch respond correctly.
4. **Flash Flight Controller** — Flash `firmware/Custom_Flight_Controller_Code` to Arduino UNO.
5. **Flash Drone Nano** — Flash `firmware/Drone_Rx_(Arduino Nano)` to the Drone Nano.
6. **Flash Ground Station** — Flash `firmware/Ground_Station_Controller_(Arduino Nano)` to the GS Nano.

---

## 📟 Ground Station Commands

Connect the Ground Station Nano to Serial Monitor at **230400 baud** and use keyboard keys:

| Key | Action |
|---|---|
| `1` | Send PING (verify RF link) |
| `2` | HOVER mode |
| `3` | TAKEOFF |
| `4` | FIRE payload |
| `5` | ⛔ EMERGENCY STOP |
| `6` | LAND |
| `7` | Inject 1 RS error (link resilience test) |
| `8` | Inject 2 RS errors (link resilience test) |

---

## 📦 Required Libraries

Install via Arduino IDE → Library Manager:

| Library | Install Name | Used In |
|---|---|---|
| RF24 | `RF24` by TMRh20 | Both Nanos |
| MPU6050 | `MPU6050` by Electronic Cats | UNO |
| Servo | Built-in | UNO |
| Wire | Built-in | UNO |
| SoftwareSerial | Built-in | Nano + UNO |

---

## 🧰 Tech Stack

<table>
<tr>
  <td align="center"><img src="https://img.shields.io/badge/-Arduino-00979D?style=flat-square&logo=arduino&logoColor=white" /><br><b>Arduino</b><br><sub>UNO + Nano (ATmega328P)</sub></td>
  <td align="center"><img src="https://img.shields.io/badge/-C++-00599C?style=flat-square&logo=cplusplus&logoColor=white" /><br><b>C++ / AVR</b><br><sub>Arduino Sketches</sub></td>
  <td align="center"><img src="https://img.shields.io/badge/-NRF24L01-E53935?style=flat-square" /><br><b>NRF24L01</b><br><sub>2.4 GHz RF Transceiver</sub></td>
  <td align="center"><img src="https://img.shields.io/badge/-MPU--6050-FF6F00?style=flat-square" /><br><b>MPU-6050</b><br><sub>6-axis IMU (I²C)</sub></td>
</tr>
<tr>
  <td align="center"><img src="https://img.shields.io/badge/-Reed--Solomon-6A1B9A?style=flat-square" /><br><b>Reed-Solomon(12,8)</b><br><sub>GF(256) Error Correction</sub></td>
  <td align="center"><img src="https://img.shields.io/badge/-PID-2E7D32?style=flat-square" /><br><b>PID Controller</b><br><sub>Roll / Pitch / Yaw</sub></td>
  <td align="center"><img src="https://img.shields.io/badge/-PWM%20ESC-37474F?style=flat-square" /><br><b>ESC PWM</b><br><sub>1000–2000 µs servo signals</sub></td>
  <td align="center"><img src="https://img.shields.io/badge/-SoftwareSerial-0277BD?style=flat-square" /><br><b>SoftwareSerial</b><br><sub>Nano ↔ UNO inter-board link</sub></td>
</tr>
</table>

---

## ⚠️ Safety Notes

- Always bench-test with propellers **removed** first.
- Set `BENCH_TEST_MODE 1` in `Land_fix.ino` during desk testing — relaxes tilt cutoff to 75°.
- Emergency Stop (`PKT_EMERGENCY_STOP`) immediately kills all motors regardless of current mode.
- The 50° tilt cutoff is a hard safety — if the drone flips, motors cut automatically.
- All ESCs must share a **common GND** with the Arduino UNO.

---

## 📄 License

This project is open-source for personal and educational use.
