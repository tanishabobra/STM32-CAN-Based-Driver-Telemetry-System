/*
 * can_schema.h
 *
 * Defines the CAN frame layout for the driver input / state estimation
 * telemetry system. Frame IDs are assigned by priority: lower numeric ID
 * wins arbitration on a real CAN bus, so safety-critical data gets the
 * lowest ID.
 *
 * Design notes:
 * - Sampling rate and transmission rate are intentionally decoupled.
 *   Sensors are sampled as fast as their own logic requires (e.g. wheel
 *   speed uses a 100ms window for pulse-count stability); CAN frames are
 *   transmitted at whatever rate a downstream consumer actually needs.
 * - The safety frame (0x100) should be sent immediately on state change
 *   (event-driven), not just polled on a fixed interval, since fault
 *   state needs to reach the bus as fast as possible.
 */

#ifndef INC_CAN_SCHEMA_H_
#define INC_CAN_SCHEMA_H_

#include "stm32f4xx_hal.h"

/* ---- Frame IDs (priority order: lower ID = higher priority) ---- */
#define CAN_ID_SAFETY       0x100   // fault flags, power-cut state — highest priority
#define CAN_ID_DRIVER_INPUT 0x110   // APPS1%, APPS2%, brake state
#define CAN_ID_MOTION       0x120   // wheel speed, gyro-Z, slip-context flag

/* ---- Suggested transmit intervals (ms). Safety frame is event-driven
 *      on top of this — send immediately on any state change, in
 *      addition to (or instead of) the periodic interval. ---- */
#define CAN_INTERVAL_SAFETY_MS       50    // periodic heartbeat; also send on-change
#define CAN_INTERVAL_DRIVER_INPUT_MS 100
#define CAN_INTERVAL_MOTION_MS       100

/* ---- 0x100: Safety frame ----
 * Byte 0: fault status bits
 *   bit 0 = APPS plausibility fault active (fault_was_active)
 *   bit 1 = power cut active
 *   bit 2 = clearing in progress
 *   bits 3-7 = reserved
 * Byte 1: brake state (0 = not pressed, 1 = pressed)
 * Bytes 2-7: reserved
 */
#define CAN_SAFETY_BIT_APPS_FAULT      (1 << 0)
#define CAN_SAFETY_BIT_POWER_CUT       (1 << 1)
#define CAN_SAFETY_BIT_CLEARING        (1 << 2)

/* ---- 0x110: Driver input frame ----
 * Byte 0: APPS1 percent (0-100, direct uint8)
 * Byte 1: APPS2 percent (0-100, direct uint8)
 * Byte 2: brake state (0/1) — duplicated here for convenience alongside
 *         other driver-input signals, even though it's also in safety frame
 * Bytes 3-7: reserved
 */

/* ---- 0x120: Motion frame ----
 * Bytes 0-1: wheel speed, pulses-per-second, uint16 (big-endian: byte0=MSB, byte1=LSB)
 * Bytes 2-3: gyro Z, degrees-per-second * 100 (fixed-point, int16, big-endian)
 *            e.g. 45.67 dps -> 4567 -> {0x11, 0xD7}
 * Byte 4: slip-context flag
 *   0 = wheel speed considered trustworthy
 *   1 = flagged unreliable (high yaw rate suggests cornering/slip)
 * Bytes 5-7: reserved
 */
#define CAN_SLIP_CONTEXT_TRUSTWORTHY  0
#define CAN_SLIP_CONTEXT_UNRELIABLE   1

#endif /* INC_CAN_SCHEMA_H_ */
