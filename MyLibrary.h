#ifndef MY_LIBRARY_H
#define MY_LIBRARY_H

#include <cstdint>
#include <charconv>

namespace MyLibrary
{
    inline void Trap(const char* errInfo)
    {
        while(true);
    }

};
extern "C" void USART1InterruptHandler();
void __attribute__((weak)) USART1InterruptHandler();
extern "C" void USART2InterruptHandler();
void __attribute__((weak)) USART2InterruptHandler();
extern "C" void USART3InterruptHandler();
void __attribute__((weak)) USART3InterruptHandler();

#endif // MY_LIBRARY_H