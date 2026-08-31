# CAN-Based Driver Telemetry System, STM32

Electric vehicles depend on distributed embedded systems: independent nodes reading sensors, applying safety-critical logic in hard real time, and communicating state over a shared bus with no single point of failure. This project implements a CAN-networked driver input and state estimation node from first principles on an STM32 microcontroller, targeting realistic accelerator/brake safety and sensor-timing requirements, and evaluates each subsystem against measured hardware behavior rather than assumed correctness.

## Problem Statement

An electric vehicle's accelerator pedal position sensor (APPS) and brake system need strict plausibility checking: if brake and throttle are applied simultaneously past a threshold, or if two redundant APPS signals disagree, motor power must be cut within a bounded time window. Getting this wrong either creates a real safety hazard (power that doesn't cut) or a car that shuts off under normal driving (false positives from noisy analog signals). Beyond the safety interlock, a car benefits from onboard state estimation, using low-cost sensors to flag when a given reading (like wheel speed) is likely unreliable, for example during hard cornering, and from getting that data off the ECU and onto a shared bus where other nodes and a dashboard can use it. This project builds and verifies each of those pieces individually, on real hardware, rather than only in simulation.

## Method

**Hardware.** STM32 Nucleo-F446RE (STM32F446), developed in STM32CubeIDE using HAL drivers and C. An MPU-6500 IMU provides gyro-Z yaw rate over I2C. An A3144 Hall-effect sensor with neodymium magnets was intended to provide wheel-speed pulses via an EXTI interrupt; a push button substitutes for the sensor in final testing (see Results). An MCP2515/TJA1050 CAN controller module communicates over SPI3, terminated with a 120Ω resistor. Potentiometers and push buttons simulate dual APPS and brake inputs on the bench.

**APPS plausibility and brake-throttle interlock.** Implements an accelerator/brake plausibility check: brake engaged with APPS above 25% triggers an immediate power cut, held until APPS drops below 5%. A related redundant-braking safety requirement was deliberately scoped out; it mandates a standalone, nonprogrammable hardware circuit and cannot be satisfied in firmware under any framing, so it is excluded rather than approximated.

**Yaw-rate slip context flag.** Gyro-Z and wheel speed measure different physical quantities (angular rate vs. linear pulse rate), so this is deliberately not framed as a complementary filter fusing two measurements of the same signal. Instead, `get_slip_context()` applies a threshold (30 dps, a placeholder pending real track data) to gyro-Z: above threshold, the car is treated as cornering hard enough that single-wheel speed is a less trustworthy proxy for vehicle speed, and the flag is set accordingly. Track width (approximately 1.2 m) is used as a documented assumption, not a measured constant.

**Wheel speed.** Pulses are captured via a falling-edge EXTI interrupt, windowed over a 100 ms period to compute pulses per second. Sampling runs at 10 ms, a 10x margin against the 100 ms plausibility timing bound used for the interlock, decoupled from the CAN transmission rate by design. The interrupt path, rate calculation, and downstream CAN packing were validated end to end using a push button as the pulse source, after the Hall-effect sensor was diagnosed as faulty (see Results).

**CAN bus telemetry.** A from-scratch MCP2515 driver (`mcp2515.h`/`.c`) implements reset, bit-timing configuration, mode switching, and frame TX/RX over SPI, with retry logic to handle transient signal issues on the breadboard prototyping platform. Three frame IDs carry telemetry: 0x100 (safety, event-driven plus 50 ms heartbeat), 0x110 (driver input, 100 ms), 0x120 (motion, 100 ms). A parallel USART2 link (115200 8N1) streams the same decoded data to a Python/FastAPI/WebSocket dashboard for live visualization.

## Results

| Subsystem | Status | Verification |
|---|---|---|
| APPS plausibility and brake interlock | Implemented | Logic matches spec; bench-tested with potentiometer and button inputs |
| Gyro-Z yaw rate (MPU-6500) | Verified | Approximately 9.74 dps during physical rotation vs. near-zero at rest |
| Yaw-rate slip context flag | Implemented | Threshold logic in place; 30 dps threshold is a placeholder pending real cornering data |
| Wheel speed pipeline (EXTI, rate calc, CAN packing) | Verified | Confirmed end to end using a push button as pulse source, after the Hall-effect sensor was isolated as defective (idled at approximately 0.5V rather than a clean logic high, unresolved even after adding an external pull-up resistor) |
| MCP2515 CAN driver (SPI3) | Verified | Reset, bit-timing configuration, mode switching, and TX/RX confirmed via internal loopback with matching sent and received frames |
| Serial telemetry (USART2) and dashboard | Verified | 115200 8N1 configured; dashboard auto-detects serial port, connects to live hardware, and renders updating gauges and charts |

The Hall-effect sensor itself was diagnosed through direct measurement rather than assumption. A first unit was confirmed unresponsive by multimeter. A second unit produced a real, magnet-responsive signal but at an abnormally low voltage swing, insufficient to reliably trigger the interrupt even after adding an external pull-up resistor. The firmware and interrupt configuration were confirmed correct independently by driving the input pin directly, which fired the interrupt reliably. With the software path validated and no working sensor available, a push button wired to the same interrupt pin was used to complete end-to-end validation of the wheel-speed pipeline.

## Conclusion

Each subsystem was built and evaluated against measured hardware behavior rather than assumed correctness. The accelerator/brake interlock, gyro yaw-rate sensing, and CAN driver were verified directly on hardware. The wheel-speed pipeline, interrupt handling, rate calculation, and CAN packing, was validated end to end using a push button as a substitute pulse source after the intended Hall-effect sensor was found defective, with the substitution and its reasoning documented above.

## Architecture

**On the STM32F446RE:**

| Peripheral | Function |
|---|---|
| ADC1 | Dual APPS potentiometers |
| I2C1 | MPU-6500 gyro-Z |
| EXTI (PC1) | Wheel-speed pulse source (push button, substituting for Hall sensor) |
| SPI3 | MCP2515 CAN controller (loopback validated) |
| USART2 | Decoded telemetry output, 115200 baud |

**Off the board:**

USART2 feeds `tools/dashboard/server.py`, a FastAPI service that auto-detects the serial port and broadcasts parsed telemetry over a WebSocket. `tools/dashboard/index.html` connects to that WebSocket and renders live gauges, indicator lights, and a wheel-speed chart in the browser.

## Running the Dashboard

```bash
cd tools/dashboard
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python3 server.py
```

Then open `http://127.0.0.1:8000` in a browser. The server auto-detects the board's serial port; if it cannot find one, the dashboard falls back to simulated data so the interface can still be reviewed without hardware attached.

## Stack

STM32CubeIDE, C, HAL drivers, MCP2515/TJA1050, MPU-6500, Python (FastAPI, pyserial), HTML/JavaScript (canvas-based gauges, no build step)
