# LQR-Kalman Ball Balancer

A high-performance LQR (Linear-Quadratic Regulator) ball-balancing controller running on an Arduino Nano / ATmega328P. Uses an HC-SR04 ultrasonic sensor for position feedback, a Kalman filter for state estimation, and hardware PWM servo control — all implemented in pure C with direct AVR register access. No Arduino library dependencies.

![C](https://img.shields.io/badge/C-AVR%20GCC-blue?logo=c)
![Arduino](https://img.shields.io/badge/Arduino-Nano%2FUno-00979D?logo=arduino)
![License](https://img.shields.io/badge/License-MIT-green)

## 🎯 Overview

This project implements a real-time control system that balances a ball on a beam using a servo-actuated platform. The controller combines:

- **LQR State Feedback** — Optimal control law minimizing position error and control effort
- **Kalman Filter** — Sensor fusion for robust position and velocity estimation
- **Hardware PWM** — Timer1 Fast PWM on OC1A for jitter-free servo control
- **Custom Timing** — Timer2 fractional millisecond counter (leaves Timer0 free for Arduino compatibility)

The entire firmware is written in pure C with direct AVR register manipulation, achieving deterministic loop timing and minimal overhead on a 16 MHz ATmega328P.

## 🧠 Control Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     CONTROL LOOP (10 ms)                     │
│                                                              │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐             │
│  │ HC-SR04  │───→│  Kalman  │───→│   LQR    │             │
│  │ Position │    │ Filter   │    │ Control  │             │
│  └──────────┘    └──────────┘    └────┬─────┘             │
│         ↑                             │                     │
│         └─────────────────────────────┘ (velocity feedback)  │
│                                                              │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐             │
│  │  Rate    │───→│  Servo   │───→│   SG90   │             │
│  │ Limiter  │    │  Filter  │    │  Servo   │             │
│  └──────────┘    └──────────┘    └──────────┘             │
└─────────────────────────────────────────────────────────────┘
```

### State-Space Model
The system is modeled as a second-order mass-on-beam:
- **State x₁** — Ball position (cm, measured by HC-SR04)
- **State x₂** — Ball velocity (cm/s, estimated via Kalman filter)
- **Control u** — Servo angle offset (degrees)

### LQR Control Law
```
u = -K₁·x₁ - K₂·x₂
```
Where `K₁ = 3.0` (position gain) and `K₂ = 2.0` (velocity gain) are tuned for stability and responsiveness.

## 🔧 Hardware Requirements

| Component | Pin | Purpose |
|-----------|-----|---------|
| **Arduino Nano / Uno** | — | ATmega328P @ 16 MHz |
| **HC-SR04 Trig** | D8 (PB0) | Ultrasonic trigger output |
| **HC-SR04 Echo** | D10 (PB2) | Ultrasonic echo input |
| **SG90 Servo** | D9 (PB1 / OC1A) | PWM servo control |
| **USB Power** | 5V | Power supply |

### Mechanical Setup
- Servo mounted at beam center, tilting a ~30 cm beam
- Ball rolls freely on the beam surface
- HC-SR04 mounted perpendicular to beam, measuring distance to ball
- **Measured range**: 3.90 cm (closest) to 12.94 cm (farthest)
- **Target position**: 8.42 cm (center of measured range)

## ⚙️ Timer Allocation

| Timer | Mode | Purpose | Notes |
|-------|------|---------|-------|
| **Timer0** | PWM | Arduino millis() / micros() | **Left alone** — wiring.c owns this |
| **Timer1** | Fast PWM 50 Hz | Servo control on OC1A | ICR1 = 40000, prescaler 8 |
| **Timer2** | Normal overflow | Custom millisecond counter | Prescaler 128, fractional correction |

## 📊 Kalman Filter Implementation

```c
// Prediction step
P_kalman += Q_NOISE;           // Process noise: 0.1

// Update step
K_k = P_kalman / (P_kalman + R_NOISE);  // Measurement noise: 0.8
x_est += K_k * (measurement - x_est);
P_kalman = (1 - K_k) * P_kalman;
```

The Kalman filter provides:
- **Smooth position estimates** — Rejects ultrasonic noise and out-of-range readings
- **Velocity estimation** — Numerical derivative with exponential smoothing (α = 0.6)
- **Adaptive uncertainty** — Covariance `P_kalman` auto-adjusts based on measurement trust

## 🎮 Control Features

### Adaptive Gain Scheduling
```c
// Increase corrective gain when ball is far from center
if (x_est < 8.2f || x_est > 8.5f) {
    gain_mul = 1.5f;  // 50% boost
}
```

### Out-of-Band Nudge
```c
// Extra correction when ball is near the far edge
if (x_est >= 9.0f) {
    control += 2.0f;  // Additional push
}
```

### Relaxed Control Limits
```c
// Widen servo authority when ball is >2 cm from target
if (abs(error) > 2.0f) {
    ctrl_min = CTRL_MIN * 1.5f;  // -22.5°
    ctrl_max = CTRL_MAX * 1.5f;  // +22.5°
}
```

### Rate Limiting & Filtering
- **Rate limit**: Max 3° per control cycle (prevents servo jitter)
- **Servo filter**: First-order IIR with α = 0.3 (smooths commanded angles)
- **Servo limits**: 30° to 140° (safe mechanical bounds)

## 📡 Telemetry Output

Serial UART at **115200 baud** (U2X0 double-speed mode, exact timing):

```
Dist: 8.42 | Tgt: 9.20 | Err: 0.00 | Ctrl: 0.00
Dist: 8.35 | Tgt: 9.20 | Err: -0.07 | Ctrl: -0.21
Dist: 8.50 | Tgt: 9.20 | Err: 0.08 | Ctrl: 0.24
```

All floating-point values are printed with 2 decimal places using a custom lightweight `printf` implementation (no `stdio.h` bloat).

## 🚀 Getting Started

### Prerequisites
- Arduino IDE 1.8.x or 2.x
- Arduino Nano or Uno (ATmega328P @ 16 MHz)
- HC-SR04 ultrasonic sensor
- SG90 micro servo
- Jumper wires and breadboard

### Wiring Diagram
```
Arduino Nano       HC-SR04          SG90 Servo
─────────────      ───────          ──────────
5V      ────────── VCC
GND     ────────── GND
D8 (PB0) ───────── Trig
D10 (PB2) ──────── Echo
D9 (PB1/OC1A) ────────────────────── Signal
5V      ──────────────────────────── VCC (red)
GND     ──────────────────────────── GND (brown)
```

### Installation

1. **Open Arduino IDE** → File → Open → `lqr_balance.ino`
2. **Select board**: Tools → Board → Arduino Nano (or Arduino Uno)
3. **Select processor**: Tools → Processor → ATmega328P
4. **Select port**: Tools → Port → your COM port
5. **Upload** → Ctrl+U

### Serial Monitor Setup
- **Baud rate**: 115200
- **Line ending**: Both NL & CR

## 🔬 Tuning Guide

### LQR Gains
| Gain | Default | Effect if Increased | Effect if Decreased |
|------|---------|---------------------|---------------------|
| `K1` | 3.0 | Faster position correction | More stable, slower response |
| `K2` | 2.0 | Stronger damping, less overshoot | Weaker velocity feedback, more oscillation |

### Kalman Parameters
| Parameter | Default | Purpose |
|-----------|---------|---------|
| `Q_NOISE` | 0.1 | Process noise — increase if model is uncertain |
| `R_NOISE` | 0.8 | Measurement noise — increase if sensor is noisy |
| `P_INIT` | 1.0 | Initial covariance — higher = less trust in initial estimate |

### Servo Calibration
```c
#define SERVO_US_MIN  544   // 0° pulse width (μs)
#define SERVO_US_MAX  2400  // 180° pulse width (μs)
```
Adjust these if your servo has non-standard pulse width requirements.

## 📁 Project Structure

```
lqr-kalman-ball-balancer/
├── lqr_balance.ino          # Main firmware (single file)
├── README.md                # This file
├── LICENSE                  # MIT License
└── docs/
    ├── wiring_diagram.png   # Visual connection guide
    ├── tuning_guide.md      # Advanced tuning procedures
    └── control_theory.md    # LQR derivation and math
```

## 🛠️ Technical Highlights

### Pure C / AVR Register Access
- **No `analogWrite()`** — Direct Timer1 register manipulation
- **No `digitalWrite()`** — Direct PORTB bit manipulation
- **No `millis()`** — Custom Timer2 counter with fractional correction
- **No `Serial.print()`** — Custom UART driver with lightweight float formatter
- **No Arduino libraries** — Zero dependency on `Arduino.h` runtime

### Deterministic Timing
- Control loop runs every **10 ms** (100 Hz)
- Ultrasonic pulse timeout: **20 ms** (prevents blocking)
- Servo PWM period: **20 ms** (50 Hz standard)
- UART baud rate: **115200** with U2X0 (0.2% error, well within tolerance)

### Memory Efficiency
- **Flash**: ~2.5 KB (minimal C code, no Arduino overhead)
- **RAM**: ~200 bytes (static variables, no heap allocation)
- **Stack**: Minimal (no recursion, no large local arrays)

## 🗺️ Roadmap

- [ ] **IMU Integration** — Add MPU6050 for beam angle feedback (full state feedback)
- [ ] **Second Ball** — Multi-agent balancing with collision avoidance
- [ ] **Wireless Telemetry** — ESP8266/ESP32 bridge for Wi-Fi dashboard
- [ ] **Auto-Tuning** — Online LQR gain adaptation via gradient descent
- [ ] **MATLAB Simulation** — Simulink model for offline controller design
- [ ] **PCB Design** — Custom shield with integrated sensor mounts
- [ ] **Python Visualization** — Real-time plotting of position/velocity/control

## 🤝 Contributing

Contributions welcome! Priority areas:
- **Control theory improvements** — MPC, PID comparison, robust control
- **Sensor fusion** — Add encoder or IR sensor for redundancy
- **Documentation** — Video tutorials, Fritzing diagrams, 3D printed mounts
- **Performance benchmarks** — Settling time, overshoot, disturbance rejection tests

## 📄 License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **LQR Theory** — Based on optimal control principles by Kalman (1960)
- **AVR Register Reference** — Atmega328P datasheet for timer configurations
- **Servo PWM Timing** — Standard RC servo pulse width specifications
- **HC-SR04 Interface** — Standard ultrasonic ranging module protocol

---

<p align="center">
  Balanced with precision ⚖️ — Controlled with math 📐
</p>
