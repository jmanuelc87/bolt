#ifndef BOLT_FLASH_INTERFACE_HPP
#define BOLT_FLASH_INTERFACE_HPP

#include "interface.hpp"
#include "stm32f1xx_hal.h"

#include <cstring>

namespace bolt
{
    namespace flash
    {
        class InternalFlash : public bolt::FlashMemory
        {
        public:
            InternalFlash() = default;
            ~InternalFlash() = default;

            void read(uint32_t address, uint8_t *buffer, uint16_t size) override
            {
                std::memcpy(buffer, reinterpret_cast<const void *>(address), size);
            }

            void write(uint32_t address, const uint8_t *data, uint16_t size) override
            {
                HAL_FLASH_Unlock();

                for (uint16_t i = 0; i < size; i += 2)
                {
                    uint16_t halfWord;
                    if (i + 1 < size)
                    {
                        halfWord = static_cast<uint16_t>(data[i]) |
                                   (static_cast<uint16_t>(data[i + 1]) << 8);
                    }
                    else
                    {
                        halfWord = static_cast<uint16_t>(data[i]) | 0xFF00;
                    }
                    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + i, halfWord);
                }

                HAL_FLASH_Lock();
            }

            void eraseSector(uint32_t address) override
            {
                FLASH_EraseInitTypeDef eraseInit{};
                uint32_t pageError = 0;

                eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
                eraseInit.PageAddress = address;
                eraseInit.NbPages = 1;

                HAL_FLASH_Unlock();
                HAL_FLASHEx_Erase(&eraseInit, &pageError);
                HAL_FLASH_Lock();
            }

        private:
        };
    }
}

#endif /* BOLT_FLASH_INTERFACE_HPP */