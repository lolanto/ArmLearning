#include <cstdint>
#include <cstdlib>
#include "MyHL.h"

int App();

int main()
{
    RCC::InitClock();
    RCC::Set_RCC_GPIOGroup(GPIO::PORT_GROUP::PG_A, true);
    GPIO::SetGPIOPortMode(GPIO::PORT_GROUP::PG_A, 1, GPIO::PORT_MODE::PM_OUTPUT_2MHz);
    GPIO::SetGPIOPortValue(GPIO::PORT_GROUP::PG_A, 1, true);
    App();
    return 0;
}

#define SCB_BASE 0xE000E008
#define SCB_HARD_FAULT_STATUS_REGISTER (*(volatile uint32_t *)(SCB_BASE + 0x2C))

extern "C" int NMIHandler();
int NMIHandler()
{
    return 0;
}

extern "C" int HardFaultHandler();
int HardFaultHandler()
{
    RCC::DisableAll_RCC_GPIO();
    RCC::Set_RCC_GPIOGroup(GPIO::PORT_GROUP::PG_A, true);
    GPIO::SetGPIOPortMode(GPIO::PORT_GROUP::PG_A, 0, GPIO::PORT_MODE::PM_OUTPUT_2MHz);
    GPIO::SetGPIOPortValue(GPIO::PORT_GROUP::PG_A, 0, true);
    return 0;
}

extern "C" int MemoryManagementFaultHandler();
int MemoryManagementFaultHandler()
{
    return 0;
}

extern "C" int BusFaultHandler();
int BusFaultHandler()
{
    return 0;
}

extern "C" int UsageFaultHandler();
int UsageFaultHandler()
{
    RCC::DisableAll_RCC_GPIO();
    RCC::Set_RCC_GPIOGroup(GPIO::PORT_GROUP::PG_A, true);
    GPIO::SetGPIOPortMode(GPIO::PORT_GROUP::PG_A, 2, GPIO::PORT_MODE::PM_OUTPUT_2MHz);
    GPIO::SetGPIOPortValue(GPIO::PORT_GROUP::PG_A, 2, true);
    return 0;
}
