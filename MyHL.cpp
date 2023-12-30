#include <cstdlib>
#include "MyHL.h"

void RCC::DisableAll_RCC_GPIO()
{
    for(uint8_t groupIdx = 0; groupIdx < GPIO::PORT_GROUP::PG_COUNT; ++groupIdx)
    {
        uint32_t groupAddr = GPIO::PORT_GROUP_BASE_ADDRs[groupIdx];
        uint32_t brrAddr = groupAddr + GPIO::PORT_BRR_ADDR_OFFSET;
        uint32_t* brrPtr = reinterpret_cast<uint32_t*>(brrAddr);
        (*brrPtr) &= 0x0u;
        (*brrPtr) |= 0xFFFFu;
        RCC::Set_RCC_GPIOGroup(static_cast<GPIO::PORT_GROUP>(groupIdx), false);
    }
}

void RCC::Set_RCC_GPIOGroup(GPIO::PORT_GROUP group, bool enable)
{
    uint32_t* ptr = reinterpret_cast<uint32_t*>(RCC_APB2ENR_REGISTER_ADDR);
    uint32_t bitOffset = static_cast<uint32_t>(group) + 2;
    if (enable)
        (*ptr) |= (1u << bitOffset);
    else
        (*ptr) &= ~(1u << bitOffset);
}

constexpr uint32_t GPIO::PORT_GROUP_BASE_ADDRs[5];

void GPIO::SetGPIOPortMode(GPIO::PORT_GROUP group, uint32_t portIdx, GPIO::PORT_MODE targetMode)
{
    uint8_t mode = static_cast<uint8_t>(targetMode);
    uint8_t cnf = 0x0u;
    if (targetMode == GPIO::PORT_MODE::PM_INPUT)
    {
        // Not implement!
        std::abort();
    }
    else
    {
        cnf = 0; // 当前默认使用推挽输出
        uint32_t groupAddr = GPIO::PORT_GROUP_BASE_ADDRs[static_cast<uint32_t>(group)];
        // port idx [0, 7] : GPIOx_CRL(addr offset 0); [8, 15] : GPIOx_CRH(addr offset 0x04)
        uint32_t controlRegisterAddr = groupAddr + (portIdx > 7 ? 0x04u : 0x00u);
        uint32_t* ptr = reinterpret_cast<uint32_t*>(controlRegisterAddr);
        uint32_t portBitSetOffset = portIdx > 7 ? portIdx - 8 : portIdx;

        // 属于这个port的CNF[3,2]以及MODE[1,0]位全部置为0
        (*ptr) &= ~(0xF << (portBitSetOffset * 4));
        (*ptr) |= (((cnf << 2) | (mode)) << (portBitSetOffset * 4));
    }
}

void GPIO::SetGPIOPortValue(GPIO::PORT_GROUP group, uint32_t portIdx, bool value)
{
    uint32_t groupAddr = GPIO::PORT_GROUP_BASE_ADDRs[static_cast<uint32_t>(group)];
    uint32_t outputDataRegisterAddr = groupAddr + GPIO::PORT_ODR_ADDR_OFFSET;
    uint32_t* ptr = reinterpret_cast<uint32_t*>(outputDataRegisterAddr);
    if (value)
        (*ptr) |= (0x1 << portIdx);
    else
        (*ptr) &= ~(0x1 << portIdx);
}

uint32_t GPIO::GetGPIOPortValue(GPIO::PORT_GROUP group, uint32_t portIdx)
{
    // Not Implement!
    std::abort();
    return 0;
}


