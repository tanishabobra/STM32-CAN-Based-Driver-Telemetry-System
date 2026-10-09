# STM32 CAN-Based Driver Telemetry System

Electric vehicles depend on distributed embedded systems: independent nodes reading sensors, applying safety-critical logic in hard real time, and communicating state over a shared bus with no single point of failure. This project implements a driver input and telemetry node from first principles on an STM32 microcontroller, targeting realistic accelerator/brake safety and sensor-timing requirements, and evaluates each subsystem against hardware behavior where it could be tested, documenting what remains unverified.

## Problem Statement

An electric vehicle's accelerator pedal position sensor (APPS) and brake system need strict plausibility checking: if brake and throttle are applied simultaneously past a threshold, or if two redundant APPS signals disagree for longer than a bounded time window, motor power must be cut. Getting this wrong either creates a real safety hazard (power that doesn't cut) or a car that shuts off under normal driving (false positives from noisy analog signals). Beyond the safety interlock, a car benefits from low-cost plausibility context, using additional sensors to flag when a given reading (like wheel speed) is likely unreliable, for example during hard cornering, and from getting that data off the ECU and onto a shared bus where other nodes and a dashboard can use it. This project builds each of those pieces individually and verifies them on real hardware where possible, rather than only in simulation.

## Method

**Hardware.** STM32 Nucleo-F446RE (STM32F446), developed in STM32CubeIDE using HAL drivers and C. An MPU-6500 IMU provides gyro-Z yaw rate over I2C. An A3144 Hall-effect sensor with neodymium magnets was intended to provide wheel-speed pulses via an EXTI interrupt; a push button substitutes for the sensor in final testing (see Results). An MCP2515/TJA1050 CAN controller module communicates over SPI3. Two identically wired potentiometers simulate the dual APPS inputs and a push button simulates the brake switch on the bench.

**APPS plausibility and brake-throttle interlock.** Both APPS channels are sampled by the ADC every 10 ms and converted to percent pedal travel. If the two readings disagree by more than 10%, a persistence timer starts; if the disagreement lasts longer than 100 ms, the firmware asserts a power-cut signal (PCUT). Separately, brake engaged with APPS above 25% asserts the power-cut signal immediately, with no timer, and it stays latched until APPS drops below 5%. The power cut is a firmware state reported over telemetry; no physical relay or motor is switched in this prototype. Because the two potentiometers are wired identically, the setup does not model sensors with deliberately different transfer functions, and open/short-circuit detection on the APPS inputs is not implemented. Both are out of scope for this prototype. A related redundant-braking safety requirement was deliberately scoped out; it mandates a standalone, nonprogrammable hardware circuit and cannot be satisfied in firmware under any framing, so it is excluded rather than approximated.

**Yaw-rate slip context flag.** Gyro-Z and wheel speed measure different physical quantities (angular rate vs. linear pulse rate), so this is deliberately not framed as a complementary filter fusing two measurements of the same signal. Instead, `get_slip_context()` applies a threshold (30 dps, a placeholder pending real track data) to gyro-Z: above threshold, the car is treated as cornering hard enough that single-wheel speed is a less trustworthy proxy for vehicle speed, and the flag is set accordingly. Track width (approximately 1.2 m) is used as a documented assumption, not a measured constant.

**Wheel speed.** Pulses are captured via a falling-edge EXTI interrupt, windowed over a 100 ms period to compute pulses per second. Sensor sampling runs at 10 ms, a 10x margin against the 100 ms APPS persistence window, and is decoupled from the CAN transmission rate by design. The interrupt path, rate calculation, and downstream CAN packing were validated end to end using a push button as the pulse source, after the Hall-effect sensor was diagnosed as faulty (see Results).

**CAN bus telemetry.** A from-scratch MCP2515 driver (`mcp2515.h`/`.c`) implements reset, bit-timing configuration, mode switching, and frame TX/RX over SPI, with retry logic to handle transient signal issues on the breadboard prototyping platform. Three priority-ordered frame IDs carry telemetry: 0x100 (safety, event-driven plus 50 ms heartbeat), 0x110 (driver input, 100 ms), 0x120 (motion, 100 ms). Bit timing is configured through CNF1/CNF2/CNF3 for 125 kbps from the module's 8 MHz oscillator. The controller runs in loopback mode: frames are transmitted and received internally by the MCP2515, and no second node is on a physical bus. A parallel USART2 link (115200 8N1) streams the same decoded data to a Python/FastAPI/WebSocket dashboard for live visualization.

## Results

| Subsystem | Status | Verification |
|---|---|---|
| Brake-throttle interlock | Verified | Fault trip and reset validated on hardware with potentiometer and push-button inputs |
| APPS disagreement check (10%, 100 ms) | Implemented | Persistence-timer logic in place; 100 ms is the design threshold, not a measured response time |
| Gyro-Z yaw rate (MPU-6500) | Verified | Approximately 9.74 dps during physical rotation vs. near-zero at rest |
| Yaw-rate slip context flag | Implemented | Threshold logic in place; 30 dps threshold is a placeholder pending real cornering data |
| Wheel speed pipeline (EXTI, rate calc, CAN packing) | Verified | Confirmed end to end using a push button as pulse source, after the Hall-effect sensor was isolated as defective (idled at approximately 0.5V rather than a clean logic high, unresolved even after adding an external pull-up resistor) |
| MCP2515 CAN driver (SPI3) | Verified (loopback only) | Reset, bit-timing configuration, mode switching, and TX/RX confirmed in loopback mode with matching sent and received frames; not tested on a live multi-node bus |
| Serial telemetry (USART2) and dashboard | Verified | 115200 8N1 configured; dashboard auto-detects serial port, connects to live hardware, and renders updating gauges and charts |

The Hall-effect sensor itself was diagnosed through direct measurement rather than assumption. A first unit was confirmed unresponsive by multimeter. A second unit produced a real, magnet-responsive signal but at an abnormally low voltage swing, insufficient to reliably trigger the interrupt even after adding an external pull-up resistor. The firmware and interrupt configuration were confirmed correct independently by driving the input pin directly, which fired the interrupt reliably. With the software path validated and no working sensor available, a push button wired to the same interrupt pin was used to complete end-to-end validation of the wheel-speed pipeline.

## Debugging: SPI Bring-Up

Early SPI reads from the MCP2515 came back corrupted. Using the STM32CubeIDE debugger to inspect the receive buffer, three separate root causes were found and fixed:

1. **Missing peripheral initialization.** CubeMX had not generated `HAL_SPI_MspInit`, so the SPI3 peripheral clock was never enabled and PC10/11/12 were never set to alternate-function mode, even though `MX_SPI3_Init()` looked correct. Regenerating the code with SPI3 enabled fixed it, after which CANSTAT read back the expected reset value of 0x80.
2. **RX-buffer overrun.** Transmit-only HAL calls (`HAL_SPI_Transmit`) on the full-duplex peripheral left received bytes unread, corrupting later reads. Switching every transaction to `HAL_SPI_TransmitReceive` fixed it.
3. **SPI clock too fast for breadboard wiring.** At prescaler 8 (about 5.25 MHz), reads failed; at prescaler 16 (about 2.6 MHz), both modules read CANSTAT correctly. Prescaler 16 is used in the final firmware.

A separate intermittent fault with two modules on the shared SPI bus was traced to unreliable breadboard connections and cleared by re-wiring. On the I2C side, ending a debug session mid-transaction left the MPU-6500 bus stuck; a full USB power cycle, not a reset, cleared it.

## Conclusion

Each subsystem was built and evaluated against hardware behavior where it could be tested. The brake-throttle interlock and gyro yaw-rate sensing were verified directly on hardware, and the CAN driver was verified on hardware in loopback mode. The wheel-speed pipeline (interrupt handling, rate calculation, and CAN packing) was validated end to end using a push button as a substitute pulse source after the intended Hall-effect sensor was found defective, with the substitution and its reasoning documented above.

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
