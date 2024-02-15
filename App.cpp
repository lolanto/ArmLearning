#include <cstdlib>
#include <cstdint>
#include <cstring>
#include "MyHL.h"
#include "MyLibrary.h"

int App()
{
    GPIO& gpioPC13 = GPIO::GetInstance(GPIO::PORT_GROUP::PG_C, 13);
    gpioPC13.SetEnable(true);
    gpioPC13.SetPortMode(GPIO::PORT_MODE::PM_OUTPUT_2MHz, GPIO::PORT_USAGE::PU_OUTPUT_PUSH_PULL);
    USART& usart3 = USART::GetInstance(USART::PORT::U3);
    usart3.Enable(115200);
    const char* str = "Hello World!(from USART Sync)\r\n";
    while(1)
    {
        gpioPC13.SetPortOutputValue(true);
        SysTick::Delay(1000);
        gpioPC13.SetPortOutputValue(false);
        SysTick::Delay(1000);
        usart3.SendAsync((uint8_t*)(str), strlen(str));
    }

    return 0;
}

void USART3InterruptHandler()
{
    RCC::GetInstance().DisableAllGPIO();
    GPIO& gpioPA01 = GPIO::GetInstance(GPIO::PORT_GROUP::PG_A, 1);
    gpioPA01.SetEnable(true);
    gpioPA01.SetPortMode(GPIO::PORT_MODE::PM_OUTPUT_2MHz, GPIO::PORT_USAGE::PU_OUTPUT_PUSH_PULL);
    while(1)
    {
        gpioPA01.SetPortOutputValue(true);
        SysTick::Delay(1000);
        gpioPA01.SetPortOutputValue(false);
        SysTick::Delay(1000);
    }
}
