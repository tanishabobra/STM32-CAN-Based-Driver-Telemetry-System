/*
 * mcp2515.h
 *
 *  Created on: 27-Aug-2026
 *      Author: rishibobra
 */

#ifndef INC_MCP2515_H_
#define INC_MCP2515_H_

#ifndef MCP2515_H
#define MCP2515_H

#include "stm32f4xx_hal.h"

// MCP2515 SPI commands
#define MCP2515_CMD_RESET      0xC0
#define MCP2515_CMD_READ       0x03
#define MCP2515_CMD_WRITE      0x02
#define MCP2515_CMD_RTS_TXB0   0x81

// MCP2515 register addresses
#define MCP2515_REG_CANSTAT    0x0E
#define MCP2515_REG_CANCTRL    0x0F
#define MCP2515_REG_CNF3       0x28
#define MCP2515_REG_CNF2       0x29
#define MCP2515_REG_CNF1       0x2A
#define MCP2515_REG_TXB0SIDH   0x31
#define MCP2515_REG_RXB0SIDH   0x61

// CANCTRL mode request bits (REQOP[2:0], bits 7:5)
#define MCP2515_MODE_NORMAL    0x00
#define MCP2515_MODE_SLEEP     0x20
#define MCP2515_MODE_LOOPBACK  0x40
#define MCP2515_MODE_LISTEN    0x60
#define MCP2515_MODE_CONFIG    0x80

#define MCP2515_MAX_RETRIES    5

typedef struct {
    GPIO_TypeDef* csPort;
    uint16_t csPin;
} MCP2515_Handle;

typedef struct {
    uint16_t id;      // 11-bit standard identifier
    uint8_t dlc;      // data length, 0-8
    uint8_t data[8];
} CAN_Frame;

uint8_t MCP2515_Reset(SPI_HandleTypeDef* hspi, MCP2515_Handle* dev);
uint8_t MCP2515_ReadRegister(SPI_HandleTypeDef* hspi, MCP2515_Handle* dev, uint8_t addr);
uint8_t MCP2515_SetBitTiming(SPI_HandleTypeDef* hspi, MCP2515_Handle* dev, uint8_t cnf1, uint8_t cnf2, uint8_t cnf3);
uint8_t MCP2515_SetMode(SPI_HandleTypeDef* hspi, MCP2515_Handle* dev, uint8_t mode);
uint8_t MCP2515_SendFrame(SPI_HandleTypeDef* hspi, MCP2515_Handle* dev, CAN_Frame* frame);
uint8_t MCP2515_ReadFrame(SPI_HandleTypeDef* hspi, MCP2515_Handle* dev, CAN_Frame* frame);

#endif

#endif /* INC_MCP2515_H_ */
