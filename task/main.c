//*****************************************************************************
//
// Copyright (C) 2015 - 2016 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  Redistributions of source code must retain the above copyright
//  notice, this list of conditions and the following disclaimer.
//
//  Redistributions in binary form must reproduce the above copyright
//  notice, this list of conditions and the following disclaimer in the
//  documentation and/or other materials provided with the
//  distribution.
//
//  Neither the name of Texas Instruments Incorporated nor the names of
//  its contributors may be used to endorse or promote products derived
//  from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

// Includes standard
#include <stdio.h>
#include <stdint.h>

// Includes DriverLib
#include <driverlib.h>

// Includes board
#include "msp432_launchpad_board.h"

// Includes FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Definicion de prioridades de tareas
#define prvRED_LED_TASK_PRIORITY    2
#define prvGREEN_LED_TASK_PRIORITY  1

// Prototipos de funciones privadas
static void prvRedLedTask(void *pvParameters);
static void prvGreenLedTask(void *pvParameters);

int main(void)
{
  // Inicializacion del hardware (Clocks, GPIOs, IRQs)
  board_init();

  // Creacion de tarea RedLedTask
  xTaskCreate(prvRedLedTask,
              "RedLedTask",
              configMINIMAL_STACK_SIZE,
              NULL,
              prvRED_LED_TASK_PRIORITY,
              NULL);

  // Creacion de tarea GreenLedTask
  xTaskCreate(prvGreenLedTask,
              "GreenLedTask",
              configMINIMAL_STACK_SIZE,
              NULL,
              prvGREEN_LED_TASK_PRIORITY,
              NULL);

  // Puesta en marcha de las tareas creadas
  vTaskStartScheduler();

  // Solo llega aqui si no hay suficiente memoria para iniciar el scheduler
  return 0;
}

// Tarea RedLedTask
static void prvRedLedTask (void *pvParameters)
{
  // Calcula el tiempo de activacion del LED (en ticks)
  // a partir del tiempo en milisegundos
  static const TickType_t xBlinkOn  = pdMS_TO_TICKS(1000);

  // Calcula el tiempo de desactivacion del LED (en ticks)
  // a partir del tiempo en milisegundos
  static const TickType_t xBlinkOff = pdMS_TO_TICKS(1000);

  // La tarea se repite en un bucle infinito
  while (true)
  {
    // Enciende LED
    led_on(MSP432_LAUNCHPAD_LED_RED);

    // Bloquea la tarea durante el tiempo de on del LED
    vTaskDelay(xBlinkOn);

    // Apaga LED
    led_off(MSP432_LAUNCHPAD_LED_RED);

    // Bloquea la tarea durante el tiempo de off del LED
    vTaskDelay(xBlinkOff);
  }
}

// Tarea GreenLedTask
static void prvGreenLedTask(void *pvParameters)
{
  // variable ii declarada volatile para evitar optimizacion
  volatile uint32_t ii;

  // La tarea se repite en un bucle infinito
  while (true)
  {
    // Enciende LED
    led_on(MSP432_LAUNCHPAD_LED_GREEN);

    // Bucle for para introducir retardo (espera activa)
    for(ii=0; ii<200000; ii++)
    {
    }

    // Fuerza cambio de contexto
    taskYIELD();

    // Apaga LED
    led_off(MSP432_LAUNCHPAD_LED_GREEN);

    // Bucle for para introducir retardo (espera activa)
    for(ii=0; ii<200000; ii++)
    {
    }
  }
}
