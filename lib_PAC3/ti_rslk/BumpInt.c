// BumpInt.c
// Runs on MSP432, interrupt version
// Provide low-level functions that interface bump switches on the robot.
// Daniel Valvano and Jonathan Valvano
// July 11, 2019

/* This example accompanies the book
 "Embedded Systems: Introduction to Robotics,
 Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

 Simplified BSD License (FreeBSD License)
 Copyright (c) 2019, Jonathan Valvano, All rights reserved.

 Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
 this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are
 those of the authors and should not be interpreted as representing official
 policies, either expressed or implied, of the FreeBSD Project.
 */

// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot
#include <stdint.h>
#include "msp.h"
#include "gpio.h"
#include "msp432_launchpad_board.h"
#include "FreeRTOS.h"

#define BUMP_IRQ_PRIO         ( 0xFF )
static void BumpInit_Individual_Init(uint8_t, uint8_t);

static uint8_t ports[] =
        {
        GPIO_PIN0,
          GPIO_PIN2, GPIO_PIN3, GPIO_PIN5, GPIO_PIN6, GPIO_PIN7 };
static void (*callback)(uint8_t);

// Initialize Bump sensors
// Make six Port 4 pins inputs
// Activate interface pullup
// pins 7,6,5,3,2,0
// Interrupt on falling edge (on touch)
void BumpInt_Init(void (*task)(uint8_t))
{
    // write this as part of Lab 14
    uint8_t i;
    for (i = 0; i < 6; i++)
    {
        BumpInit_Individual_Init(GPIO_PORT_P4, ports[i]);
    }
    MAP_Interrupt_setPriority(INT_PORT4, BUMP_IRQ_PRIO);
    MAP_Interrupt_enableInterrupt(INT_PORT4);
    callback = task;

}

void BumpInit_Individual_Init(uint8_t port, uint8_t pin)
{

    MAP_GPIO_setAsInputPinWithPullUpResistor(port, pin);
    MAP_GPIO_clearInterruptFlag(port, pin);
    MAP_GPIO_interruptEdgeSelect(port, pin, GPIO_HIGH_TO_LOW_TRANSITION);
    MAP_GPIO_enableInterrupt(port, pin);
}

// Read current state of 6 switches
// Returns a 6-bit positive logic result (0 to 63)
// bit 5 Bump5
// bit 4 Bump4
// bit 3 Bump3
// bit 2 Bump2
// bit 1 Bump1
// bit 0 Bump0
uint8_t BumpInt_Read(void)
{
    // write this as part of Lab 14
    uint8_t value;
    uint8_t status;
    uint8_t i;
    status = 0;

    for (i = 0; i < 6; i++)
    {
        value = ROM_GPIO_getInputPinValue(GPIO_PORT_P4, ports[i]);
        if(value == pdFALSE){
            status |= 1 << i;
        }
    }
    return status;
}

void PORT4_IRQHandler(void)
{
    // write this as part of Lab 14

    uint_fast16_t status;

    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, status);
    callback(status);

}

