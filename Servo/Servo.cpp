/*
    ChibiOS/Servo - Copyright (C) 2013
                 Jarek Zok <jarek.zok@fwioo.pl>

    This file is part of ChibiOS/Servo.

    ChibiOS/Servo is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/Servo is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdlib.h>

#ifdef _cplusplus
extern "C" {
#endif
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#ifdef _cplusplus
}
#endif

#include <cmath>
#include <cstdlib>


#include <PCA9685.hpp>
#include "Servo.hpp"

/**
 * Servo object constructor
 * @param PCA9685 * a PCA9685 object set for communication with servo
 * @param uint8_t channel number [0-15]
 * @param uint16_t minimum - min value of PWM setting at which pulse will take exactly 1ms
 * @param uint16_t maximum - max value of PWM setting at which pulse will take 2ms
 * @param float speed - time in seconds the servo will move in given range of degrees,
 *                      usually in seconds per 60 degrees (i.e. 0.12s/60deg). Please consult
 *                      documentation for used servo.
 * @param uint8_t sdegree - range at which servo will move for given time.
 * @param bool blocking - if TRUE, then after changing PWM value, methods as moveTo or moveRelative, will wait
 *                        for servo to settle at given position. Waiting time is relative to speed of servo and difference
 *                        between current and new position. If FALSE, servo will move and method returns immediately.
 *                        The time isn't exact value for your particular servo, it's just calculated value of time the
 *                        servo should be at position based on it's specification.
 */
Servo::Servo(PCA9685 *driver, uint8_t channel, uint16_t minimum, uint16_t maximum, float speed, uint8_t sdegree, bool blocking ) {
    this->driver = driver;
    this->channel = channel;
    this->minimum = minimum;
    this->maximum = maximum;
    this->neutral = (this->maximum - this->minimum) / 2;
    this->current = 0;
    this->speed = speed;
    this->sdegree = sdegree;
    this->blocking = blocking;
    chMtxInit(&this->flag);
}

void* Servo::operator new(size_t size) {
    return chCoreAlloc(size);
}

void Servo::operator delete(void *mem) {

}

/**
 * Move to position in degree.
 * @param float degree - degrees range from -90 to 90
 * @return old position
 */
Servo *Servo::moveTo(float degree) {
    uint16_t position = this->minimum;
    uint16_t turntime;

    chMtxLock(&this->flag);
    turntime = (abs(degree - this->current)/this->sdegree)*(this->speed * 1000);
    position += ceil(((this->maximum - this->minimum) / 180.0) * (degree + 90));
    position = position > this->maximum ? this->maximum : position < this->minimum ? this->minimum : position;
    this->driver->setPWM(this->channel, 0, position);
    this->current = degree;
    if (this->blocking) chThdSleepMicroseconds(turntime);
    chMtxUnlock();

    return this;
}

/**
 * Moves servo relative to current position.
 * @param float value - new position value relative to current position
 * @return old position
 */
Servo *Servo::moveRelative(float value) {
    uint16_t position = this->minimum;
    uint16_t turntime;

    chMtxLock(&this->flag);
    turntime = (abs((this->current + value) - this->current)/this->sdegree)*(this->speed * 1000);
    position += ceil(((this->maximum - this->minimum) / 180.0) * ((this->current + value) + 90));
    position = position > this->maximum ? this->maximum : position < this->minimum ? this->minimum : position;
    this->driver->setPWM(this->channel, 0, position);
    this->current = (this->current + value);
    if (this->blocking) chThdSleepMicroseconds(turntime);
    chMtxUnlock();

    return this;

}

/**
 * Sets servo neutral position.
 * @return old position
 */
Servo *Servo::setNeutral() {
    uint16_t turntime;

    chMtxLock(&this->flag);
    turntime = (abs(this->current)/this->sdegree)*(this->speed * 1000);
    this->current = 0;
    this->driver->setPWM(this->channel, 0, this->neutral);
    if (this->blocking) chThdSleepMicroseconds(turntime);
    chMtxUnlock();

    return this;
}

float Servo::getPosition(void) {
    return this->current;
}