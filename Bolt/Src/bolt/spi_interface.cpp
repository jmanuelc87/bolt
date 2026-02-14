#include "interface/spi_interface.hpp"

bolt::spi::SpiSyncPort::SpiSyncPort(SPI_HandleTypeDef *hspi, bolt::OutputPin *nss) : hspi_(hspi), nss_(nss)
{
    nss_->setHigh();
}

bolt::spi::SpiSyncPort::~SpiSyncPort()
{
    nss_->setHigh();
}

void bolt::spi::SpiSyncPort::transmit(const uint8_t *data, uint16_t size)
{
    nss_->setLow();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(hspi_, data, size, TIMEOUT_MS);
    nss_->setHigh();
    configASSERT(st == HAL_OK);
}

void bolt::spi::SpiSyncPort::receive(uint8_t *buffer, uint16_t size)
{
    nss_->setLow();
    HAL_StatusTypeDef st = HAL_SPI_Receive(hspi_, buffer, size, TIMEOUT_MS);
    nss_->setHigh();
    configASSERT(st == HAL_OK);
}

void bolt::spi::SpiSyncPort::transmitReceive(const uint8_t *txData, uint8_t *rxData, uint16_t size)
{
    nss_->setLow();
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(hspi_, txData, rxData, size, TIMEOUT_MS);
    nss_->setHigh();
    configASSERT(st == HAL_OK);
}

uint8_t bolt::spi::SpiSyncPort::readRegister(uint8_t reg)
{
    uint8_t tx[2] = {static_cast<uint8_t>(reg | 0x80), 0x00};
    uint8_t rx[2] = {0};

    nss_->setLow();
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(hspi_, tx, rx, 2, TIMEOUT_MS);
    nss_->setHigh();
    configASSERT(st == HAL_OK);

    return rx[1];
}

void bolt::spi::SpiSyncPort::writeRegister(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {static_cast<uint8_t>(reg & 0x7F), value};

    nss_->setLow();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(hspi_, tx, 2, TIMEOUT_MS);
    nss_->setHigh();
    configASSERT(st == HAL_OK);
}

void bolt::spi::SpiSyncPort::readRegisters(uint8_t reg, uint8_t *buffer, uint16_t size)
{
    uint8_t cmd = reg | 0x80;

    nss_->setLow();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(hspi_, &cmd, 1, TIMEOUT_MS);
    if (st == HAL_OK)
    {
        st = HAL_SPI_Receive(hspi_, buffer, size, TIMEOUT_MS);
    }
    nss_->setHigh();
    configASSERT(st == HAL_OK);
}
