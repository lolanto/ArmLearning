#include <cstdlib>
#include <cstdint>
#include "MyHL.h"

int App()
{
    RCC::Set_RCC_GPIOGroup(GPIO::PORT_GROUP::PG_C, true);
    GPIO::SetGPIOPortMode(GPIO::PORT_GROUP::PG_C, 13, GPIO::PORT_MODE::PM_OUTPUT_10MHz);
    while(1)
    {
        GPIO::SetGPIOPortValue(GPIO::PORT_GROUP::PG_C, 13, true);
        SysTick::Delay(2000);
        GPIO::SetGPIOPortValue(GPIO::PORT_GROUP::PG_C, 13, false);
        SysTick::Delay(1000);
    }

    return 0;
}