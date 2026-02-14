#ifndef BOLT_SPI_INTERFACE_HPP
#define BOLT_SPI_INTERFACE_HPP

#include "interface.hpp"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

namespace bolt
{
    namespace spi
    {
        class SpiSyncPort : public bolt::SpiPort
        {
        public:
            SpiSyncPort(SPI_HandleTypeDef *hspi, bolt::OutputPin *nss);
            ~SpiSyncPort();

            void transmit(const uint8_t *data, uint16_t size) override;
            void receive(uint8_t *buffer, uint16_t size) override;
            void transmitReceive(const uint8_t *txData, uint8_t *rxData, uint16_t size) override;

            uint8_t readRegister(uint8_t reg);
            void writeRegister(uint8_t reg, uint8_t value);
            void readRegisters(uint8_t reg, uint8_t *buffer, uint16_t size);

        private:
            SPI_HandleTypeDef *hspi_;
            bolt::OutputPin *nss_;
            static constexpr uint32_t TIMEOUT_MS = 100;
        };
    }
}

#endif /* BOLT_SPI_INTERFACE_HPP */
