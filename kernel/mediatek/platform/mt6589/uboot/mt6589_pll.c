
#include <common.h>

#include <asm/arch/mt65xx.h>
#include <asm/arch/mt65xx_typedefs.h>

kal_uint32 mtk_get_bus_freq(void)
{
    return 268000; // BUS clock = MAINPLL DIV3
}