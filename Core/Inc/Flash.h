#ifndef __STM32F0xx_LL_FLASH_EX_H
#define __STM32F0xx_LL_FLASH_EX_H

#include "main.h"


#define XMEM_ALIGN_SIZE(size , align)                                        (((size) + (align) - 1) / (align))

#define XOFS(t , m)                                                         ((u32)(&(((t *)0)->m)))

#define SYSTEM_ARG_STORE_START_ADDRE                                        (0x0800F800UL)

#define FLASH_OPT_OVERTIMER         										(0x1FFFF)
#define FLASH_OPT_TRY_COUNT         										(5)

#define FLASH_KEY1 0x45670123
#define FLASH_KEY2 0xCDEF89AB

#define FLASH_PAGE_SIZE 0x800

#endif
