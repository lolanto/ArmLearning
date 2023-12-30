#include <cstdint>
#include "RCC.h"

#define RCC_BASE 0x40021000
#define RCC_APB2ENR_REGISTER (*(volatile uint32_t *)(RCC_BASE + 0x18))
#define RCC_APB2ENR_IOPCEN (1 << 4)

#define GPIO_PORTC_BASE 0x40011000
#define GPIO_CRH_REGISTER(x) (*(volatile uint32_t *)(x + 0x4))
#define GPIO_CHR_MODE_MASK(x) (0xF << ((x - 8) * 4))
#define GPIO_CHR_MODE_OUTPUT(x) (0x1 << ((x - 8) * 4))

#define GPIO_BLINK_PORT GPIO_PORTC_BASE
#define GPIO_BLINK_NUM 13

#define GPIO_ODR_REGISTER(x) (*(volatile uint32_t *)(x + 0xC))
#define GPIO_ODR_PIN(x) (1 << x)

int App();

int main()
{
    // RCC_APB2ENR_REGISTER |= RCC_APB2ENR_IOPCEN;
    RCC::GetInstance().EnableAPB2();

    GPIO_CRH_REGISTER(GPIO_BLINK_PORT) &= ~(GPIO_CHR_MODE_MASK(GPIO_BLINK_NUM));
    GPIO_CRH_REGISTER(GPIO_BLINK_PORT) |= GPIO_CHR_MODE_OUTPUT(GPIO_BLINK_NUM);

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
    RCC_APB2ENR_REGISTER |= RCC_APB2ENR_IOPCEN;
    GPIO_CRH_REGISTER(GPIO_BLINK_PORT) &= ~(GPIO_CHR_MODE_MASK(GPIO_BLINK_NUM));
    GPIO_CRH_REGISTER(GPIO_BLINK_PORT) |= GPIO_CHR_MODE_OUTPUT(GPIO_BLINK_NUM);
    while(1)
    {
        GPIO_ODR_REGISTER(GPIO_BLINK_PORT) &= ~GPIO_ODR_PIN(GPIO_BLINK_NUM); // Lighting
    }
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
    RCC_APB2ENR_REGISTER |= RCC_APB2ENR_IOPCEN;
    GPIO_CRH_REGISTER(GPIO_BLINK_PORT) &= ~(GPIO_CHR_MODE_MASK(GPIO_BLINK_NUM));
    GPIO_CRH_REGISTER(GPIO_BLINK_PORT) |= GPIO_CHR_MODE_OUTPUT(GPIO_BLINK_NUM);
    while(1)
    {
        GPIO_ODR_REGISTER(GPIO_BLINK_PORT) |= GPIO_ODR_PIN(GPIO_BLINK_NUM);
        for(uint32_t i = 0; i < 400000; ++i)
        {
            __asm__ volatile("nop");
        }
        GPIO_ODR_REGISTER(GPIO_BLINK_PORT) &= ~GPIO_ODR_PIN(GPIO_BLINK_NUM); // Lighting
        for(uint32_t i = 0; i < 400000; ++i)
        {
            __asm__ volatile("nop");
        }
    }
}
