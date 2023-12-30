#include <cstdint>
#include "MyHL.h"

int App()
{
    RCC::Set_RCC_GPIOGroup(GPIO::PORT_GROUP::PG_C, true);
    GPIO::SetGPIOPortMode(GPIO::PORT_GROUP::PG_C, 13, GPIO::PORT_MODE::PM_OUTPUT_10MHz);
    while(1)
    {
        GPIO::SetGPIOPortValue(GPIO::PORT_GROUP::PG_C, 13, true);
        for(uint32_t i = 0; i < 400000; ++i)
        {
            __asm__ volatile("nop");
        }
        GPIO::SetGPIOPortValue(GPIO::PORT_GROUP::PG_C, 13, false);
        for(uint32_t i = 0; i < 10000; ++i)
        {
            __asm__ volatile("nop");
        }
    }

    return 0;
}