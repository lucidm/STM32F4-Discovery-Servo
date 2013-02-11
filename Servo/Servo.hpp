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
#ifndef _SERVO_HPP_
#define _SERVO_HPP_

class Servo {
    public:
        Servo(PCA9685 *driver, uint8_t channel, uint16_t minimum, uint16_t maximum);
        void* operator new(size_t size);
        void operator delete(void *mem);

        void setNeutral();
        void moveTo(float degree);



private:
    PCA9685 *driver;
    uint8_t channel;
    uint16_t minimum;
    uint16_t maximum;
    uint16_t neutral;
};

#endif