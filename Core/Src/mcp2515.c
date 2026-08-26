#include "mcp2515.h"

static void MCP2515_WriteRegister(SPI_HandleTypeDef* hspi, MCP2515_Handle* dev, uint8_t addr, uint8_t data)
{
    uint8_t tx[3] = {MCP2515_CMD_WRITE, addr, data};
    uint8_t dummy_rx[3];

    HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, tx, dummy_rx, 3, 100);
    HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_SET);
}

uint8_t MCP2515_ReadRegister(SPI_HandleTypeDef* hspi, MCP2515_Handle* dev, uint8_t addr)
{
    uint8_t tx[3] = {MCP2515_CMD_READ, addr, 0xFF};
    uint8_t rx[3];

    HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, tx, rx, 3, 100);
    HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_SET);

    return rx[2];
}

uint8_t MCP2515_Reset(SPI_HandleTypeDef* hspi, MCP2515_Handle* dev)
{
    uint8_t cmd = MCP2515_CMD_RESET;
    uint8_t dummy_rx;

    HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, &cmd, &dummy_rx, 1, 100);
    HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_SET);

    HAL_Delay(10);  // MCP2515 needs a short settle time after reset

    // Confirm chip landed in Configuration mode post-reset
    uint8_t canstat = MCP2515_ReadRegister(hspi, dev, MCP2515_REG_CANSTAT);
    return ((canstat & 0xE0) == MCP2515_MODE_CONFIG) ? 1 : 0;
}

uint8_t MCP2515_SetBitTiming(SPI_HandleTypeDef* hspi, MCP2515_Handle* dev, uint8_t cnf1, uint8_t cnf2, uint8_t cnf3)
{
    for (uint8_t attempt = 0; attempt < MCP2515_MAX_RETRIES; attempt++)
    {
        // Burst write CNF3, CNF2, CNF1 (consecutive ascending addresses, auto-increment)
        uint8_t tx[5] = {MCP2515_CMD_WRITE, MCP2515_REG_CNF3, cnf3, cnf2, cnf1};
        uint8_t dummy_rx[5];

        HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(hspi, tx, dummy_rx, 5, 100);
        HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_SET);

        // Verify
        uint8_t tx_read[5] = {MCP2515_CMD_READ, MCP2515_REG_CNF3, 0xFF, 0xFF, 0xFF};
        uint8_t rx_read[5];

        HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(hspi, tx_read, rx_read, 5, 100);
        HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_SET);

        if (rx_read[2] == cnf3 && rx_read[3] == cnf2 && rx_read[4] == cnf1)
        {
            return 1;
        }
    }
    return 0;  // failed after MAX_RETRIES attempts
}

uint8_t MCP2515_SetMode(SPI_HandleTypeDef* hspi, MCP2515_Handle* dev, uint8_t mode)
{
    for (uint8_t attempt = 0; attempt < MCP2515_MAX_RETRIES; attempt++)
    {
        MCP2515_WriteRegister(hspi, dev, MCP2515_REG_CANCTRL, mode);
        HAL_Delay(1);  // brief settle time before checking status

        uint8_t canstat = MCP2515_ReadRegister(hspi, dev, MCP2515_REG_CANSTAT);
        if ((canstat & 0xE0) == mode)
        {
            return 1;
        }
    }
    return 0;
}

uint8_t MCP2515_SendFrame(SPI_HandleTypeDef* hspi, MCP2515_Handle* dev, CAN_Frame* frame)
{
    uint8_t sidh = (frame->id >> 3) & 0xFF;
    uint8_t sidl = (frame->id & 0x07) << 5;

    // Burst write: SIDH, SIDL, EID8, EID0, DLC, then up to 8 data bytes
    uint8_t tx[15] = {MCP2515_CMD_WRITE, MCP2515_REG_TXB0SIDH, sidh, sidl, 0x00, 0x00, frame->dlc};
    for (uint8_t i = 0; i < frame->dlc; i++)
    {
        tx[7 + i] = frame->data[i];
    }
    uint8_t dummy_rx[15];
    uint8_t txLen = 7 + frame->dlc;

    HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, tx, dummy_rx, txLen, 100);
    HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_SET);

    // Request transmission
    uint8_t rts = MCP2515_CMD_RTS_TXB0;
    uint8_t rts_dummy;

    HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, &rts, &rts_dummy, 1, 100);
    HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_SET);

    return 1;  // transmission requested; caller verifies via read-back if needed
}

uint8_t MCP2515_ReadFrame(SPI_HandleTypeDef* hspi, MCP2515_Handle* dev, CAN_Frame* frame)
{
    uint8_t tx[15] = {MCP2515_CMD_READ, MCP2515_REG_RXB0SIDH};
    for (uint8_t i = 2; i < 15; i++) tx[i] = 0xFF;
    uint8_t rx[15];

    HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, tx, rx, 15, 100);
    HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_SET);

    uint8_t sidh = rx[2];
    uint8_t sidl = rx[3];
    frame->id = (sidh << 3) | (sidl >> 5);
    frame->dlc = rx[6] & 0x0F;

    for (uint8_t i = 0; i < frame->dlc && i < 8; i++)
    {
        frame->data[i] = rx[7 + i];
    }

    return 1;
}
