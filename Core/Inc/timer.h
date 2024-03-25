#ifndef __TIMER_H__
#define __TIMER_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>

#define OS_TIMER_MAX 4

  struct timer_t
  {
    void (*pfun)(void *para);
    unsigned int delay;
    unsigned int period;
    bool run;
    void *para;
  };

  typedef uint8_t timer_id;

  void timer_init(void);
  timer_id timer_creat(void (*pFunction)(void *para),
                       const unsigned int delay,
                       const unsigned int period,
                       bool run,
                       void *para);
  bool timer_delete(const timer_id index);
  void timer_start(const timer_id index);
  void timer_stop(const timer_id index);
  void timer_sched(void);

#ifdef __cplusplus
}
#endif

#endif /* __TIMER_H__ */
