#ifndef RCC_H
#define RCC_H

#include <cstdint>

class RCC
{
private:
    const static uint32_t RCC_BASE = 0x40021000u;
    const static uint32_t RCC_APB2ENR = RCC_BASE + 0x18u;
public:
    static RCC& GetInstance()
    {
        static RCC _inst;
        return _inst;
    }
    void EnableAPB2()
    {
        uint32_t* ptr = reinterpret_cast<uint32_t*>(RCC_APB2ENR);
        (*ptr) |= (1 << 4);
    }
public:
    RCC() = default;
};

#endif // RCC_H
