#include <cstdlib>
#include <cstdint>
#include "MyHL.h"

class Object
{
public:
    static Object& GetInstance()
    {
        static Object _inst;
        return _inst;
    }
    Object()
    : mValue(1000)
    {}
    uint32_t GetValue() { return mValue; }
    uint32_t ModifyValue(uint32_t x) { return mValue += x; }
private:
    uint32_t mValue;
};

int App()
{
    RCC::Set_RCC_GPIOGroup(GPIO::PORT_GROUP::PG_C, true);
    GPIO::SetGPIOPortMode(GPIO::PORT_GROUP::PG_C, 13, GPIO::PORT_MODE::PM_OUTPUT_10MHz);
    auto& obj = Object::GetInstance();
    while(1)
    {
        GPIO::SetGPIOPortValue(GPIO::PORT_GROUP::PG_C, 13, true);
        SysTick::Delay(obj.ModifyValue(1));
        GPIO::SetGPIOPortValue(GPIO::PORT_GROUP::PG_C, 13, false);
        SysTick::Delay(1000);
    }

    return 0;
}