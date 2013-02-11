
#include <stdlib.h>

#include "ch.h"
#include "hal.h"

#include "chprintf.h"

#include "shellblink.hpp"

/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/


void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
  size_t n, size;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: mem\r\n");
    return;
  }
  n = chHeapStatus(NULL, &size);
  chprintf(chp, "core free memory : %u bytes\r\n", chCoreStatus());
  chprintf(chp, "heap fragments   : %u\r\n", n);
  chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[])
{
  static const char *states[] = {THD_STATE_NAMES};
  Thread *tp;

  (void)argv;
  if (argc > 0)
  {
    chprintf(chp, "Usage: threads\r\n");
    return;
  }
  chprintf(chp, "%10s %10s %10s %6s %6s %11s %7s\r\n",
           "name", "add", "stack", "prio", "refs", "state", "time");
  tp = chRegFirstThread();
  do
  {
    chprintf(chp, "%10s %.10lx %.10lx %6lu %6lu %11s %7lu\r\n",
             (uint32_t)tp->p_name, (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
             (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
             states[tp->p_state], (uint32_t)tp->p_time);
    tp = chRegNextThread(tp);
  } while (tp != NULL);
}


/*
 * Red LED blinker thread, times are in milliseconds.
 */
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (TRUE) {
    palClearPad(GPIOD, GPIOD_LED6);
    chThdSleepMilliseconds(500);
    palSetPad(GPIOD, GPIOD_LED6);
    chThdSleepMilliseconds(500);

  }
  return 0;
}

void startBlinker(void){
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
}