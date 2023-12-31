#include <algorithm>
#include <cstdlib>
#include "MyHL.h"

void RCC::InitClock()
{
    // 启动外部高速时钟，注意：当前硬件外接的时钟周期是8MHz!!!8MHz!!!8MHz!!!8MHz!!!
    (*reinterpret_cast<uint32_t*>(RCC_CR_ADDR) |= (1u << 16u));
    // 检查外部时钟是否就绪
    while(((*reinterpret_cast<uint32_t*>(RCC_CR_ADDR)) & (1u << 17u)) == 0);
    // 设置低速APB与分频为2分频(这里连接的时钟频率最高不超过36MHz，而STM32最高是72MHz。因此2分频刚好可以保证不超过最大上限)
    (*reinterpret_cast<uint32_t*>(RCC_CFGR_ADDR)) = (0x4u << 8u);
    // 将PLLMUL设置为9，即假如是直连8MHz外部时钟，就可以获得一个72MHz的时钟周期(默认是2倍频)
    (*reinterpret_cast<uint32_t*>(RCC_CFGR_ADDR)) |= (0x7u << 18u);
    // 将PLLSRC设置为使用外部时钟(默认是使用2分频后的内部时钟)
    (*reinterpret_cast<uint32_t*>(RCC_CFGR_ADDR)) |= (0x1u << 16u);
    // 调整Flash的时延设置？似乎和Flash的读写正确性相关
    (*reinterpret_cast<uint32_t*>(Flash::FLASH_ACR_ADDR)) |= 0x32u;
    // 启动PLL时钟
    (*reinterpret_cast<uint32_t*>(RCC_CR_ADDR)) |= (0x1u << 24u);
    // 等待PLL时钟就绪
    while(((*reinterpret_cast<uint32_t*>(RCC_CR_ADDR)) & (1u << 25)) == 0);
    // 将PLL时钟作为系统时钟(SysClk)
    (*reinterpret_cast<uint32_t*>(RCC_CFGR_ADDR)) |= 0x2u;
    // 等待PLL时钟顺利切换成系统时钟
    while(((*reinterpret_cast<uint32_t*>(RCC_CFGR_ADDR)) & (0x3 << 2u)) != (0x2 << 2u));
}

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

void SysTick::Delay(uint32_t delayTimeInMS)
{
    if (delayTimeInMS <= 0)
        return;
    // 声明我们需要使用外部时钟源进行计时
    // 其次0bit依旧是0，即sys tick还没有enable
    // 1bit位置是0，即计数器归零之后不会出发SysTick中断
    (*reinterpret_cast<uint32_t*>(SYST_CSR_ADDR)) &= ~(4u);
    // 假设我们的外部时钟在倍频之后就是72MHz，而SysTick是经过了一次8分频，因此是9MHz
    // 不难计算，Tick 9次相当于过去了1us，9k次相当于过去了1ms
    constexpr uint32_t FAC_Us = 9u;
    constexpr uint32_t FAC_Ms = FAC_Us * 1000u;
    constexpr uint32_t MAX_COUNT_VALUE = 0xFFFFFFu;
    // 设置Reload寄存器
    (*reinterpret_cast<uint32_t*>(SYST_RVR_ADDR)) = (std::min(FAC_Ms * delayTimeInMS, MAX_COUNT_VALUE - 1));
    // 清空Current Value寄存器，准备开始重新计时
    (*reinterpret_cast<uint32_t*>(SYST_CVR_ADDR)) = 0x0u;
    // 开始计时
    (*reinterpret_cast<uint32_t*>(SYST_CSR_ADDR)) = 0x1u;
    volatile uint32_t SYST_CSR_Value = (*reinterpret_cast<uint32_t*>(SYST_CSR_ADDR));
    // 假如CSR寄存器中COUNTFLAG的标识位一直是0，说明还没有完成计数，则一直等待
    while((SYST_CSR_Value & 0x1u) && ((SYST_CSR_Value & (1u << 16)) == 0))
    {
        SYST_CSR_Value = (*reinterpret_cast<uint32_t*>(SYST_CSR_ADDR));
    }
    // 停止计数
    (*reinterpret_cast<uint32_t*>(SYST_CSR_ADDR)) = 0x0u;
    // 清空Current Value寄存器
    (*reinterpret_cast<uint32_t*>(SYST_CVR_ADDR)) = 0x0u;
}
