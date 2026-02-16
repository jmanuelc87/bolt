#include <gtest/gtest.h>
#include <cstring>
#include <cstdint>

#include "bolt/interface.hpp"
#include "bolt/controller/flash_controller.hpp"

using bolt::controller::FlashController;
using bolt::controller::FlashKey;

class FakeFlashMemory : public bolt::FlashMemory
{
public:
    static constexpr uint16_t PAGE_SIZE = 1024;
    uint8_t memory[PAGE_SIZE];

    FakeFlashMemory()
    {
        std::memset(memory, 0xFF, PAGE_SIZE);
    }

    void read(uint32_t address, uint8_t *buffer, uint16_t size) override
    {
        std::memcpy(buffer, &memory[address], size);
    }

    void write(uint32_t address, const uint8_t *data, uint16_t size) override
    {
        std::memcpy(&memory[address], data, size);
    }

    void eraseSector(uint32_t address) override
    {
        (void)address;
        std::memset(memory, 0xFF, PAGE_SIZE);
    }
};

class FlashControllerTest : public ::testing::Test
{
protected:
    FakeFlashMemory flash;
};

TEST_F(FlashControllerTest, DefaultValuesAreZeroOnErasedFlash)
{
    FlashController ctrl(&flash, 0);

    EXPECT_FLOAT_EQ(ctrl.load(FlashKey::MOTOR1_KP), 0.0f);
    EXPECT_FLOAT_EQ(ctrl.load(FlashKey::MOTOR1_KI), 0.0f);
    EXPECT_FLOAT_EQ(ctrl.load(FlashKey::MOTOR1_KD), 0.0f);
    EXPECT_FLOAT_EQ(ctrl.load(FlashKey::MOTOR4_KD), 0.0f);
}

TEST_F(FlashControllerTest, StoreAndLoadSingleKey)
{
    FlashController ctrl(&flash, 0);

    ctrl.store(FlashKey::MOTOR2_KP, 1.5f);
    EXPECT_FLOAT_EQ(ctrl.load(FlashKey::MOTOR2_KP), 1.5f);
    EXPECT_FLOAT_EQ(ctrl.load(FlashKey::MOTOR1_KP), 0.0f);
}

TEST_F(FlashControllerTest, StoreAllAndLoadAll)
{
    FlashController ctrl(&flash, 0);

    float gains[12] = {
        1.0f, 0.1f, 0.01f,
        2.0f, 0.2f, 0.02f,
        3.0f, 0.3f, 0.03f,
        4.0f, 0.4f, 0.04f,
    };
    ctrl.storeAll(gains);

    float loaded[12] = {};
    ctrl.loadAll(loaded);

    for (int i = 0; i < 12; i++)
    {
        EXPECT_FLOAT_EQ(loaded[i], gains[i]);
    }
}

TEST_F(FlashControllerTest, PersistenceAcrossInstances)
{
    {
        FlashController ctrl(&flash, 0);
        ctrl.store(FlashKey::MOTOR3_KD, 3.14f);
    }

    FlashController ctrl2(&flash, 0);
    EXPECT_FLOAT_EQ(ctrl2.load(FlashKey::MOTOR3_KD), 3.14f);
}

TEST_F(FlashControllerTest, StorePreservesOtherKeys)
{
    FlashController ctrl(&flash, 0);

    ctrl.store(FlashKey::MOTOR1_KP, 1.0f);
    ctrl.store(FlashKey::MOTOR2_KI, 2.0f);
    ctrl.store(FlashKey::MOTOR3_KD, 3.0f);

    EXPECT_FLOAT_EQ(ctrl.load(FlashKey::MOTOR1_KP), 1.0f);
    EXPECT_FLOAT_EQ(ctrl.load(FlashKey::MOTOR2_KI), 2.0f);
    EXPECT_FLOAT_EQ(ctrl.load(FlashKey::MOTOR3_KD), 3.0f);
    EXPECT_FLOAT_EQ(ctrl.load(FlashKey::MOTOR4_KP), 0.0f);
}

TEST_F(FlashControllerTest, AllMotorGainsAreIndependent)
{
    FlashController ctrl(&flash, 0);

    ctrl.store(FlashKey::MOTOR1_KP, 10.0f);
    ctrl.store(FlashKey::MOTOR2_KP, 20.0f);
    ctrl.store(FlashKey::MOTOR3_KP, 30.0f);
    ctrl.store(FlashKey::MOTOR4_KP, 40.0f);

    EXPECT_FLOAT_EQ(ctrl.load(FlashKey::MOTOR1_KP), 10.0f);
    EXPECT_FLOAT_EQ(ctrl.load(FlashKey::MOTOR2_KP), 20.0f);
    EXPECT_FLOAT_EQ(ctrl.load(FlashKey::MOTOR3_KP), 30.0f);
    EXPECT_FLOAT_EQ(ctrl.load(FlashKey::MOTOR4_KP), 40.0f);
}
