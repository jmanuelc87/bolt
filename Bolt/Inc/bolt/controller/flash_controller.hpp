#ifndef BOLT_FLASH_CONTROLLER_HPP
#define BOLT_FLASH_CONTROLLER_HPP

#include <cstdint>
#include <cstring>

#include "interface.hpp"

namespace bolt
{
    namespace controller
    {
        enum class FlashKey : uint8_t
        {
            MOTOR1_KP = 0,
            MOTOR1_KI,
            MOTOR1_KD,
            MOTOR2_KP,
            MOTOR2_KI,
            MOTOR2_KD,
            MOTOR3_KP,
            MOTOR3_KI,
            MOTOR3_KD,
            MOTOR4_KP,
            MOTOR4_KI,
            MOTOR4_KD,
            COUNT
        };

        class FlashController
        {
        public:
            FlashController(bolt::FlashMemory *flash, uint32_t baseAddress)
                : flash_(flash), baseAddress_(baseAddress)
            {
                loadAllFromFlash();
            }

            float load(FlashKey key) const
            {
                return cache_[static_cast<uint8_t>(key)];
            }

            void store(FlashKey key, float value)
            {
                cache_[static_cast<uint8_t>(key)] = value;
                writeCache();
            }

            void loadAll(float *values) const
            {
                std::memcpy(values, cache_, sizeof(cache_));
            }

            void storeAll(const float *values)
            {
                std::memcpy(cache_, values, sizeof(cache_));
                writeCache();
            }

        private:
            static constexpr uint8_t KEY_COUNT = static_cast<uint8_t>(FlashKey::COUNT);
            static constexpr uint32_t ERASED_WORD = 0xFFFFFFFF;

            bolt::FlashMemory *flash_;
            uint32_t baseAddress_;
            float cache_[KEY_COUNT]{};

            void loadAllFromFlash()
            {
                flash_->read(baseAddress_,
                             reinterpret_cast<uint8_t *>(cache_),
                             sizeof(cache_));

                for (uint8_t i = 0; i < KEY_COUNT; i++)
                {
                    uint32_t raw;
                    std::memcpy(&raw, &cache_[i], sizeof(uint32_t));
                    if (raw == ERASED_WORD)
                    {
                        cache_[i] = 0.0f;
                    }
                }
            }

            void writeCache()
            {
                flash_->eraseSector(baseAddress_);
                flash_->write(baseAddress_,
                              reinterpret_cast<const uint8_t *>(cache_),
                              sizeof(cache_));
            }
        };
    }
}

#endif /* BOLT_FLASH_CONTROLLER_HPP */
