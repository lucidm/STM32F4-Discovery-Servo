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

Servo::Servo(PCA9685 *driver, uint8_t channel, uint16_t minimum, uint16_t maximum) {
    this->driver = driver;
    this->channel = channel;
    this->minimum = minimum;
    this->maximum = maximum;
    this->neutral = (this->maximum - this->minimum) / 2;
}

void* Servo::operator new(size_t size) {
    return chCoreAlloc(size);
}

void Servo::operator delete(void *mem) {

}

void Servo::moveTo(float degree) {
    uint16_t position = this->minimum;

    position += ceil(((this->maximum - this->minimum) / 180.0) * (degree + 90));
    this->driver->setPWM(this->channel, 0, position);
}

void Servo::setNeutral() {
    this->driver->setPWM(this->channel, 0, this->neutral);
}