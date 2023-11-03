/*
 * Stepper.h
 *
 * Copyright (c) 2018 Adriano Zenzen
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#pragma once

#include "driver/gpio.h"
#include "shiftOut.h"

#ifdef __cplusplus
extern "C" {
#endif


#define CW 0
#define CCW 1

class Stepper {
private:
    bool inverted;
    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t enablePin;

public:
    //Recebe o pino de passo, direção, habilitação e se é para inverter o sentido do motor
    Stepper(
			uint8_t stepPinNumber,
			uint8_t dirPinNumber,
			uint8_t enablePinNumber,
			bool invert
	);

    ~Stepper();

    //Executa um passo na direção definida
    void step();

    //Define a direção de movimento do motor
    void direction(uint8_t dir);

    //Retorna a direção do motor
    uint8_t getDirection();

    //Inverte CW e CCW
    void invertDirection();

    //Habilita o motor
    void enable();

    //Desabilita o motor
    void disable();

    //Retorna verdadeiro se o motor estiver habilitado
    bool isEnabled();
};

#ifdef __cplusplus
}
#endif
