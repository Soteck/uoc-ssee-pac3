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

// Includes board
#include "msp432_launchpad_board.h"

// Includes FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// Definicion de prioridades de tareas
#define prvSENDER_TASK_PRIORITY    3
#define prvRECEIVER_TASK_PRIORITY  1

// Prototipos de funciones privadas
static void prvSenderTask(void *pvParameters);
static void prvReceiverTask(void *pvParameters);
static void button1_interrupt(void);

// Declaracion de un semaforo binario
// para sincronizar SenderTask con la ISR
SemaphoreHandle_t xBinarySemaphore;

// Declaracion de un mutex
SemaphoreHandle_t xMutex;

// Tipo comando
typedef struct {
  // Led a activar en tarea ReceiverTask
  uint8_t led;
  // Tiempo de activacion del LED
  uint16_t blinkOnMs;
  // Flag indicador de comando nuevo
  bool nuevo;
} command_t;

// Variable comando compartida entre las
// tareas SenderTask y ReceiverTask
command_t commandVar;

int main(void)
{
  // Inicializacion del hardware (Clocks, GPIOs, IRQs)
  board_init();
  board_buttons_set_callback(MSP432_LAUNCHPAD_BUTTON_S1, button1_interrupt);

  // Inicializacion del semaforo binario
  xBinarySemaphore = xSemaphoreCreateBinary();

  // Inicializacion del mutex
  xMutex = xSemaphoreCreateMutex();

  // Inicialización de la variable de comando
  commandVar.led = MSP432_LAUNCHPAD_LED_RED;
  commandVar.blinkOnMs = 100;
  commandVar.nuevo = true;

  // Comprueba si semaforo y mutex se han creado bien
  if ((xBinarySemaphore != NULL) && (xMutex != NULL))
  {
    // Creacion de tarea SenderTask
    xTaskCreate(prvSenderTask,
                "SenderTask",
                configMINIMAL_STACK_SIZE,
                NULL,
                prvSENDER_TASK_PRIORITY,
                NULL);

    // Creacion de tarea ReceiverTask
    xTaskCreate(prvReceiverTask,
                "ReceiverTask",
                configMINIMAL_STACK_SIZE,
                NULL,
                prvRECEIVER_TASK_PRIORITY,
                NULL);

    // Puesta en marcha de las tareas creadas
    vTaskStartScheduler();
  }

  // Solo llega aqui si no hay suficiente memoria para iniciar el scheduler
  return 0;
}

// Tarea SenderTask
static void prvSenderTask(void *pvParameters)
{
  // Tiempo maximo de espera entre dos interrupciones del pulsador
  const TickType_t xMaxExpectedBlockTime = pdMS_TO_TICKS(500);

  // La tarea se repite en un bucle infinito
  while (true)
  {
    // El semaforo debe ser entregado por la ISR PORT1_IRQHandler
    // Espera un numero maximo de xMaxExpectedBlockTime ticks
    if (xSemaphoreTake(xBinarySemaphore, xMaxExpectedBlockTime) == pdPASS)
    {
      // Intenta coger el mutex, bloqueandose si no esta disponible
      xSemaphoreTake(xMutex, portMAX_DELAY);
      {
        // Escribe el nuevo comando en la variable commandVar
        if (commandVar.led == MSP432_LAUNCHPAD_LED_RED)
        {
          commandVar.blinkOnMs = 500;
          commandVar.led = MSP432_LAUNCHPAD_LED_GREEN;
        }
        else
        {
          commandVar.blinkOnMs = 100;
          commandVar.led = MSP432_LAUNCHPAD_LED_RED;
        }

        // Activa el flag para indicar hay comando nuevo
        commandVar.nuevo = true;
      }

      // Libera el mutex
      xSemaphoreGive(xMutex);
    }
  }
}

// Tarea ReceiverTask
static void prvReceiverTask(void *pvParameters)
{
  // La tarea se repite en un bucle infinito
  while (true)
  {
    // Intenta coger el mutex, bloqueandose si no esta disponible
    xSemaphoreTake(xMutex, portMAX_DELAY);
    {
      // Comprueba si hay comando nuevo
      if (commandVar.nuevo == true)
      {
        // Tiempo de activacion del LED (en ticks)
        TickType_t xBlinkOn;

        // Calcula el tiempo de activacion del LED (en ticks)
        xBlinkOn = pdMS_TO_TICKS(commandVar.blinkOnMs);

        // Desactiva el flag para indicar no hay comando nuevo
        commandVar.nuevo = false;

        // Libera el mutex
        xSemaphoreGive(xMutex);

        // Enciende LED
        led_on(commandVar.led);

        // Bloquea la tarea durante el tiempo de on del LED
        vTaskDelay(xBlinkOn);

        // Apaga LED
        led_off(commandVar.led);
      }
      else
      {
        // Si no hay comando nuevo, libera el mutex
        xSemaphoreGive(xMutex);
      }
    }
  }
}

// Rutina de Servicio a Interrupcion (ISR) del PORT1
static void button1_interrupt(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // Entrega el semaforo para desbloquear la tarea SenderTask
  xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);

  // Pasa xHigherPriorityTaskWoken en portYIELD_FROM_ISR().
  // Si xHigherPriorityTaskWoken valia pdTRUE dentro de
  // xSemaphoreGiveFromISR(), entonces al llamar a
  // portYIELD_FROM_ISR() solicitara un cambio de contexto.
  // Si xHigherPriorityTaskWoken sigue siendo pdFALSE, la llamada
  // a portYIELD_FROM_ISR() no tendra ningun efecto
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
