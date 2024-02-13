#include <cstdint>
#include <cstdlib>
#include "MyHL.h"

int App();

int main()
{
    RCC& rcc = RCC::GetInstance();
    rcc.InitClock();
    GPIO& gpioPA1 = GPIO::GetInstance(GPIO::PORT_GROUP::PG_A, 1);
    gpioPA1.SetEnable(true);
    gpioPA1.SetPortMode(GPIO::PORT_MODE::PM_OUTPUT_2MHz, GPIO::PORT_USAGE::PU_OUTPUT_PUSH_PULL);
    gpioPA1.SetPortOutputValue(true);
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
    RCC::GetInstance().DisableAllGPIO();
    GPIO& gpioPA0 = GPIO::GetInstance(GPIO::PORT_GROUP::PG_A, 0);
    gpioPA0.SetEnable(true);
    gpioPA0.SetPortMode(GPIO::PORT_MODE::PM_OUTPUT_2MHz, GPIO::PORT_USAGE::PU_OUTPUT_PUSH_PULL);
    gpioPA0.SetPortOutputValue(true);
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
    RCC::GetInstance().DisableAllGPIO();
    GPIO& gpioPA2 = GPIO::GetInstance(GPIO::PORT_GROUP::PG_A, 2);
    gpioPA2.SetEnable(true);
    gpioPA2.SetPortMode(GPIO::PORT_MODE::PM_OUTPUT_2MHz, GPIO::PORT_USAGE::PU_OUTPUT_PUSH_PULL);
    gpioPA2.SetPortOutputValue(true);
    return 0;
}
