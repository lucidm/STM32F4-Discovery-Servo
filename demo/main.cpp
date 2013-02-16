/*
    ChibiOS/PCA9685 - Copyright (C) 2013
                 Jarek Zok <jarek.zok@fwioo.pl>

    This file is part of ChibiOS/PCA9685.

    ChibiOS/PCA9685 is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/PCA9685 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef _cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "ch.h"
#include "hal.h"

#include "shell.h"
#include "chprintf.h"


static const I2CConfig i2cconfig = {
    OPMODE_I2C,
    400000,
    FAST_DUTY_CYCLE_2,
};

#ifdef _cplusplus
}
#endif

#include <cstdlib>

#include "shellblink.hpp" //
#include <PCA9685.hpp>
#include <Servo.hpp>

Mailbox mbox;
msg_t mbox_buffer[8];
PCA9685 *pcachip;
Servo *servos[16];
Servo *servo1;
Servo *servo2;

// static WORKING_AREA(waServo1, 128);
// static msg_t SERVO1(void * arg) {
//     Mailbox* mbox = (Mailbox *)arg;
//     msg_t msg, result;
//
//     while(TRUE) {
//         result = chMBFetch(mbox, &msg, TIME_INFINITE);
//         if(result == RDY_OK) {
//             if(msg & 1)
//                 palSetPad(IOPORT2, LED1);
//             else
//                 palClearPad(IOPORT2, LED1);
//             if(msg & 2)
//                 palSetPad(IOPORT2, LED2);
//             else
//                 palClearPad(IOPORT2, LED2);
//         }
//     }
//
//     return 0;
// }

//-----------------------------------------------------------------
void cmd_moveto(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argv;
  if (argc != 2) {
    chprintf(chp, "Usage: mv [channel] [degree -90 to 90]\r\n");
    return;
  }

  //chMBPost(mbox, 0, TIME_INFINITE);
  servos[atoi(argv[0])]->moveTo(atof(argv[1]));
  chprintf(chp, "OLD = %2.2f \r\n", servos[atoi(argv[0])]->getPosition());
}

void cmd_relative(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argv;
  if (argc != 2) {
    chprintf(chp, "Usage: rel [channel] [relative degree]\r\n");
    return;
  }

  //chMBPost(mbox, 0, TIME_INFINITE);
  servos[atoi(argv[0])]->moveRelative(atof(argv[1]));
  chprintf(chp, "OLD = %2.2f \r\n", servos[atoi(argv[0])]->getPosition());
}

void cmd_reg(BaseSequentialStream *chp, int argc, char *argv[]) {

  (void)argv;
  if (argc > 1) {
    chprintf(chp, "Usage: reg [regnum]\r\n");
    return;
  }

  if (argc == 0)
  {
    chprintf(chp, "MODE1 = %u \r\n", pcachip->getRegisterValue(PCA9685_MODE1));
    chprintf(chp, "PRESCALE = %u \r\n", pcachip->getRegisterValue(PCA9685_PRESCALE));
    chprintf(chp, "STATUS = %u \r\n", pcachip->getStatus());
  }

  if (argc == 1)
  {
    uint8_t reg = atoi(argv[0]);
    chprintf(chp, "REGISTER[%u] = %u \r\n", reg, pcachip->getRegisterValue(reg));
  }
}

void cmd_freq(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argv;
  if (argc != 1) {
    chprintf(chp, "Usage: freq <freq>\r\n");
    return;
  }

  uint16_t freq = atoi(argv[0]);
  pcachip->setFreq(freq);
  chprintf(chp, "FREQ : %u \r\n", pcachip->getFreq());
  chprintf(chp, "STATUS : %u \r\n", pcachip->getStatus());
}

void cmd_pwm(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argv;
  if (argc != 3) {
    chprintf(chp, "Usage: pwm <channel> <on> <off>\r\n");
    return;
  }

  uint8_t channel = atoi(argv[0]);
  uint16_t on = atoi(argv[1]);
  uint16_t off = atoi(argv[2]);
  pcachip->setPWM(channel, on, off);
  chprintf(chp, "STATUS : %u \r\n", pcachip->getStatus());
}


void cmd_duty(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argv;
  if (argc != 2) {
    chprintf(chp, "Usage: duty <channel> <duty>\r\n");
    return;
  }

  pcachip->setChannel(atoi(argv[0]));
  pcachip->setPWM(atof(argv[1]));
  chprintf(chp, "STATUS : %u \r\n", pcachip->getStatus());
}


/*
 * assert Shell Commands to functions
 */
static const ShellCommand commands[] = {
  {"mem", cmd_mem},
  {"threads", cmd_threads},
  {"reg", cmd_reg},
  {"freq", cmd_freq},
  {"pwm", cmd_pwm},
  {"mov", cmd_moveto},
  {"rel", cmd_relative},
  {"duty", cmd_duty},
  {NULL, NULL}
};


/*
 * Shell configuration
 */

#define SHELL_WA_SIZE   THD_WA_SIZE(2048)

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SD2,
  commands
};



/*
 * Application entry point.
 */
int main(void) {
  /*
   * Shell thread
   */
  Thread *shelltp = NULL;

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Activates the serial driver 2 using the driver default configuration.
   * PA2(TX) and PA3(RX) are routed to USART2.
   */
  sdStart(&SD2, NULL);
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

  chMBInit(&mbox, mbox_buffer, 8);

  pcachip = new PCA9685(&PCA9685_DEFI2C_DRIVER, &i2cconfig, PCA9685_ADDRESS, 47, true);
  servos[0] = new Servo(pcachip, 0, 5.1, 10.2, 0.12, 60, TRUE);
  servos[1] = new Servo(pcachip, 1, 5.1, 10.2, 0.12, 60, TRUE);

  servos[0]->moveTo(0.0);
  servos[1]->moveTo(0.0);

  startBlinker();

  /*
   * Main loop, does nothing except spawn a shell when the old one was terminated
   */
  while (TRUE) {
    if (!shelltp)
      {
        shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
      }
    else if (chThdTerminated(shelltp)) {
      chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
      shelltp = NULL;           /* Triggers spawning of a new shell.        */
    }
    chThdSleepMilliseconds(1000);
  }
}
