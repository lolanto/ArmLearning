#include <cstdint>
#include "RCC.h"

#define RCC_BASE 0x40021000
#define RCC_APB2ENR_REGISTER (*(volatile uint32_t *)(RCC_BASE + 0x18))
#define RCC_APB2ENR_IOPCEN (1 << 4)

#define GPIO_PORTC_BASE 0x40011000
#define GPIO_CRH_REGISTER(x) (*(volatile uint32_t *)(x + 0x4))
#define GPIO_CHR_MODE_MASK(x) (0x3 << ((x - 8) * 4))
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