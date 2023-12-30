#ifndef MyHL_H
#define MyHL_H

#include <cstdint>

class GPIO
{
public:
    enum PORT_GROUP : uint8_t
    {
        PG_A = 0,
        PG_B = 1,
        PG_C = 2,
        PG_D = 3,
        PG_E = 4,
        PG_COUNT
    };
    enum PORT_MODE : uint8_t
    {
        PM_INPUT = 0,
        PM_OUTPUT_10MHz = 1,
        PM_OUTPUT_2MHz = 2,
        PM_OUTPUT_50MHz = 3
    };
    static constexpr uint32_t PORT_GROUP_BASE_ADDRs[] = 
    {
        0x40010800u, // A
        0X40010C00u, // B
        0x40011000u, // C
        0x40011400u, // D
        0x40011800u  // E
    };
    // Output data register offset (GPIO group base)
    static constexpr uint32_t PORT_ODR_ADDR_OFFSET = 0xCu;
    // Output data recover register address offset 用来重置端口输出
    static constexpr uint32_t PORT_BRR_ADDR_OFFSET = 0x14u;
    static void SetGPIOPortMode(PORT_GROUP group, uint32_t portIdx, PORT_MODE mode);
    static void SetGPIOPortValue(PORT_GROUP group, uint32_t portIdx, bool value);
    static uint32_t GetGPIOPortValue(PORT_GROUP group, uint32_t portIdx);
    GPIO() = delete;
};

class RCC
{
public:
    // 需要保证以下函数必须正常执行！！！
    static constexpr uint32_t RCC_BASE_ADDR = 0x40021000u;
    static constexpr uint32_t RCC_APB2ENR_REGISTER_ADDR = RCC_BASE_ADDR + 0x18u;
    static void DisableAll_RCC_GPIO();
    static void Set_RCC_GPIOGroup(GPIO::PORT_GROUP group, bool enable = true);
    RCC() = delete;
};

#endif // MyHL_H
