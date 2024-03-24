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

#define SYSTEM_ARG_STORE_START_ADDR 0x8007F00

static uint8_t ubFLASH_Unlock(void);
static uint8_t ubFLASH_Lock(void);
static uint8_t ubFlash_WaitFor_Operate(uint32_t timeOut);
static uint32_t ulGetPage(uint32_t startAddr);
static uint8_t ubFLASH_PageErase(uint32_t page);
static uint8_t ubFLASH_Program_DoubleWord(uint32_t addr, uint64_t data);
uint8_t ubFlash_Write_DoubleWord(uint32_t startAddr, uint64_t * pDat, uint16_t len);
void vFlash_Read_DoubleWord(uint32_t startAddr, uint64_t * pDat, uint16_t len);

#endif
