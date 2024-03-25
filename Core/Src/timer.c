#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "timer.h"

struct timer_t timer_list[OS_TIMER_MAX];

void timer_init(void)
{
    uint8_t index;

    for (index = 0; index < OS_TIMER_MAX; index++)
    {
        timer_list[index].pfun = NULL;
    }
}

timer_id timer_creat(void(*pFunction)(void* para),
                     const unsigned int delay,
                     const unsigned int period,
                     bool run,
                     void* para)
{
    uint8_t index = 0;

    while ((timer_list[index].pfun != NULL) && (index < OS_TIMER_MAX))
    {
        index++;
    }

    //printf("time index %d\r\n",index);
    timer_list[index].pfun  = pFunction;
    timer_list[index].delay  = delay;
    timer_list[index].period  = period;
    timer_list[index].run  = run;
    timer_list[index].para = para;
    return index;
}

bool timer_delete(const timer_id index)
{
    if(index >= OS_TIMER_MAX) return false;

    if (timer_list[index].pfun == NULL)
    {
        return false;
    }
    timer_list[index].pfun   = NULL;
    timer_list[index].delay  = 0;
    timer_list[index].period = 0;
    timer_list[index].run    = false;
    timer_list[index].para   = NULL;
    return true;
}

void timer_start(const timer_id index)
{
    if(index >= OS_TIMER_MAX) return;

    timer_list[index].run    = true;
}

void timer_stop(const timer_id index)
{
    if(index >= OS_TIMER_MAX) return;

    timer_list[index].run    = false;
}

void timer_sched(void)
{
    uint8_t index;
    for (index = 0; index < OS_TIMER_MAX; index++)
    {
        if(timer_list[index].delay == 0)
        {
            if (timer_list[index].run)
            {
                (*timer_list[index].pfun)(timer_list[index].para);
                if (timer_list[index].period == 0)
                {
                    timer_delete(index);
                }
                else timer_list[index].delay = timer_list[index].period;
            }
        }
        else
        {
            timer_list[index].delay -= 1;
        }
    }
}
