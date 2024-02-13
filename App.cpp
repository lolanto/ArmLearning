#include <cstdlib>
#include <cstdint>
#include "MyHL.h"


int App()
{
    GPIO& gpioPC13 = GPIO::GetInstance(GPIO::PORT_GROUP::PG_C, 13);
    gpioPC13.SetEnable(true);
    gpioPC13.SetPortMode(GPIO::PORT_MODE::PM_OUTPUT_2MHz, GPIO::PORT_USAGE::PU_OUTPUT_PUSH_PULL);
    USART& usart3 = USART::GetInstance(USART::PORT::U3);
    usart3.Enable(115200);
    char c = 'b';
    while(1)
    {
        gpioPC13.SetPortOutputValue(true);
        SysTick::Delay(1000);
        gpioPC13.SetPortOutputValue(false);
        SysTick::Delay(1000);
        usart3.Send((uint8_t*)(&c), 1);
    }

    return 0;
}