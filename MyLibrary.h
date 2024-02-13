#ifndef MY_LIBRARY_H
#define MY_LIBRARY_H

#include <cstdint>
#include <charconv>

namespace MyLibrary
{
    void Trap(const char* errInfo)
    {
        while(true);
    }
};

#endif // MY_LIBRARY_H