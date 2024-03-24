#include "main.h"
#include "Flash.h"
#include "stm32g030xx.h"
#include "stdio.h"

/* Unlock the FLASH control register access */
static uint8_t ubFLASH_Unlock(void)
{
    uint8_t sta = 0;

    if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0x00U)
    {
        /* Authorize the FLASH Registers access */
        WRITE_REG(FLASH->KEYR, FLASH_KEY1);
        WRITE_REG(FLASH->KEYR, FLASH_KEY2);

        /* verify Flash is unlock */
        if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0x00U)
        {
            sta = 1;
        }
    }

    return sta;
}


/* Lock the FLASH control register access */
static uint8_t ubFLASH_Lock(void)
{
    uint8_t sta = 1;

    /* Set the LOCK Bit to lock the FLASH Registers access */
    SET_BIT(FLASH->CR, FLASH_CR_LOCK);

    /* verify Flash is locked */
    if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0x00u)
    {
        sta = 0;
    }

    return sta;
}

/* Wait for a FLASH operation to complete */
static uint8_t ubFlash_WaitFor_Operate(uint32_t timeOut)
{
    uint32_t timer = 0;
    uint32_t error = 0;

    while ((FLASH->SR & FLASH_SR_BSY1) != 0x00U)
    {
        if ((++timer) >= timeOut)
        {
            return 1;
        }
    }

#if ( USE_STM32G0_LL_LIB_ENABLE > 0)
    /* check flash errors */
    error = (FLASH->SR & FLASH_FLAG_SR_ERROR);

    /* Clear SR register */
    FLASH->SR = FLASH_FLAG_SR_CLEAR;
#endif


#if ( USE_STM32G0_HAL_LIB_ENABLE > 0)
    /* check flash errors */
    error = (FLASH->SR & FLASH_SR_ERRORS);

    /* Clear SR register */
    FLASH->SR = FLASH_SR_CLEAR;
#endif


    if (error != 0x00U)
    {
        return 2;
    }

    timer = 0;
    while ((FLASH->SR & FLASH_SR_CFGBSY) != 0x00U)
    {
        if ((++timer) > timeOut)
        {
            return 3;
        }
    }

    return 0;
}

/* Gets the page of a given address */
static uint32_t ulGetPage(uint32_t startAddr)
{
    return ((startAddr - FLASH_BASE) / FLASH_PAGE_SIZE);
}


/* Erase the specified FLASH memory page */
static uint8_t ubFLASH_PageErase(uint32_t page)
{
    uint32_t tmp  = 0;
    uint32_t time = 0;
    uint8_t  res  = 0;

    /* Get configuration register, then clear page number */
    tmp = (FLASH->CR & ~FLASH_CR_PNB);

    /* Set page number, Page Erase bit & Start bit */
    FLASH->CR = (tmp | (FLASH_CR_STRT | (page <<  FLASH_CR_PNB_Pos) | FLASH_CR_PER));

    /* wait for BSY1 in order to be sure that flash operation is ended before allowing prefetch in flash */
    while ((FLASH->SR & FLASH_SR_BSY1) != 0x00U)
    {
        if ((++time) > FLASH_OPT_OVERTIMER)
        {
            res = 1;
            break;
        }
    }

    /* If operation is completed or interrupted, disable the Page Erase Bit */
    CLEAR_BIT(FLASH->CR, FLASH_CR_PER);

    return res;
}

/* Program double-word (64-bit) at a specified address */
/* Must EN PG bit before and DIS PG bit after */
static uint8_t ubFLASH_Program_DoubleWord(uint32_t addr, uint64_t data)
{
    uint32_t time = 0;


    /* Wait for last operation to be completed */
    while ((FLASH->SR & FLASH_SR_BSY1) != 0x00U)
    {
        if ((++time) > FLASH_OPT_OVERTIMER)
        {
            return 1;
        }
    }

    /* Set PG bit */
    SET_BIT(FLASH->CR, FLASH_CR_PG);

    /* Program first word */
    *(uint32_t *)addr = (uint32_t)data;

    /* Barrier to ensure programming is performed in 2 steps, in right order
    (independently of compiler optimization behavior) */
    __ISB();

    /* Program second word */
    *(uint32_t *)(addr + 4U) = (uint32_t)(data >> 32U);

    /* Wait for last operation to be completed */
    while ((FLASH->SR & FLASH_SR_BSY1) != 0x00U)
    {
        if ((++time) > FLASH_OPT_OVERTIMER)
        {
            return 2;
        }
    }

    return 0;
}


/* Program double-word(64-bit) at a specified address */
uint8_t ubFlash_Write_DoubleWord(uint32_t startAddr, uint64_t * pDat, uint16_t len)
{
    uint32_t page = 0, time = 0;
    uint8_t  tryCount = 0, res = 0;
    uint16_t i = 0;


FLASH_UNLOCK:
    if (ubFLASH_Unlock())
    {
        if ((++tryCount) < FLASH_OPT_TRY_COUNT)
        {
            res = ubFlash_WaitFor_Operate(FLASH_OPT_OVERTIMER);
            printf("Wait For Operate %s...%d\r\n", (res ? "Fail" : "OK"), res);
            goto FLASH_UNLOCK;
        }
        else
        {
            printf("Flash Unlock Fail...\r\n");
            return 1;
        }
    }

    page = ulGetPage(startAddr);
    tryCount = 0;


FLASH_ERASE:
    if(ubFLASH_PageErase(page))
    {
        if ((++tryCount) < FLASH_OPT_TRY_COUNT)
        {
            res = ubFlash_WaitFor_Operate(FLASH_OPT_OVERTIMER);
            printf("Wait For Operate %s...%d\r\n", (res ? "Fail" : "OK"), res);
            goto FLASH_ERASE;
        }
        else
        {
            ubFLASH_Lock();
            printf("Flash Erase Fail...\r\n");
            return 2;
        }
    }

    tryCount = 0;
    for (i = 0; i < len; ++i)
    {
        while(tryCount < FLASH_OPT_TRY_COUNT)
        {
            if(ubFLASH_Program_DoubleWord(startAddr, pDat[i]))
            {
                res = ubFlash_WaitFor_Operate(FLASH_OPT_OVERTIMER);
                printf("Wait For Operate %s...%d\r\n", (res ? "Fail" : "OK"), res);
                tryCount++;
            }
            else
            {
                startAddr += 8;
                tryCount   = 0;
                break;
            }
        }

        if (tryCount)
        {
            ubFLASH_Lock();
            printf("Write Flash Fail...\r\n");
            return 3;
        }
    }

    ubFLASH_Lock();
    printf("Write Flash OK...\r\n");
    return 0;
}

/* Read double-word (64-bit) at a specified address */
void vFlash_Read_DoubleWord(uint32_t startAddr, uint64_t * pDat, uint16_t len)
{
    uint16_t i = 0;

    for(i = 0; i < len; ++i)
    {
        *pDat++ = *(volatile uint64_t *)(startAddr + (i << 3));
    }
}
