

#include <stdint.h>
#include "divider.h"

uint32_t divider_udiv(uint32_t dividend, uint16_t divisor)
{
    uint32_t result;

    result=dividend/divisor;

    return result;
}
