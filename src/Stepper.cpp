/*
 * Stepper.cpp
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

#include <Stepper.h>
#include "freertos/FreeRTOS.h"

//Recebe o pino de passo, direção, habilitação e se é para inverter o sentido do motor
Stepper::Stepper(
			uint8_t stepPinNumber,
			uint8_t dirPinNumber,
			uint8_t enablePinNumber,
			bool invert
	) {

    //Inicializa os pinos
    stepPin = stepPinNumber;
    dirPin = dirPinNumber;
    enablePin = enablePinNumber;

    //Desabilita o driver
    disable();

    //Configura se o motor é invertido ou não
    inverted = invert;
}

Stepper::~Stepper() {
    //Desabilita o driver
    disable();
}

//Executa um passo na direção definida
void Stepper::step() {
	shiftOut( sr_data | (0x01 << stepPin) );
    shiftOut( sr_data & ~(0x01 << stepPin) );
}

//Define a direção de movimento do motor
void Stepper::direction(uint8_t dir) {
    if (dir == CW) {
        if (!inverted) {
            shiftOut( sr_data & ~(0x01 << dirPin) );
        } else {
        	shiftOut( sr_data | (0x01 << dirPin) );
        }
    } else {
        if (!inverted) {
        	shiftOut( sr_data | (0x01 << dirPin) );
        } else {
        	shiftOut( sr_data & ~(0x01 << dirPin) );
        }
    }
}

//Retorna a direção do motor
uint8_t Stepper::getDirection() {
	if (!inverted) {
		if (sr_data & (1 << dirPin)) {
			return CCW;
		} else {
			return CW;
		}
	} else {
		if (sr_data & (1 << dirPin)) {
			return CW;
		} else {
			return CCW;
		}
	}
}

//Inverte CW e CCW
void Stepper::invertDirection() {
    inverted = !inverted;
}

//Habilita o motor
void Stepper::enable() {
    shiftOut( sr_data & ~(0x01 << enablePin) );
}

//Desabilita o motor
void Stepper::disable() {
    shiftOut( sr_data | (0x01 << enablePin) );
}

//Retorna verdadeiro se o motor estiver habilitado
bool Stepper::isEnabled() {
	return !(sr_data & (1 << enablePin));
}

