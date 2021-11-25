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
#define prvSENDER_TASK_PRIORITY    1
#define prvRECEIVER_TASK_PRIORITY  2

// Prototipos de funciones privadas
static void prvSenderTask(void *pvParameters);
static void prvReceiverTask(void *pvParameters);

// Declaracion de una cola de mensajes
QueueHandle_t xQueue;

// Tipo de mensaje a almacenar en la cola
typedef struct {
  // Led a activar en tarea ReceiverTask
  uint8_t led;
  // Tiempo de activacion del LED
  uint16_t blinkOnMs;
} command_t;

int main(void)
{
  // Inicializacion del hardware (Clocks, GPIOs, IRQs)
  board_init();

  // Inicializacion de la cola
  xQueue = xQueueCreate(5, sizeof(command_t));
  if(xQueue != NULL)
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
  // Periodo de tiempo entre envios consecutivos (en ticks)
  const TickType_t xPeriod = pdMS_TO_TICKS(500);

  // Resultado del envio a la cola
  BaseType_t xStatus;
  command_t commandToSend;

  // Inicializa el comando a enviar
  commandToSend.led = MSP432_LAUNCHPAD_LED_RED;
  commandToSend.blinkOnMs = 100;

  // La tarea se repite en un bucle infinito
  while (true)
  {
    // Envia un comando a la cola
    xStatus = xQueueSend(xQueue, &commandToSend, 0);

    // Comprueba si ha enviado correctamente
    if (xStatus == pdPASS)
    {
      if (commandToSend.led == MSP432_LAUNCHPAD_LED_RED)
      {
        commandToSend.blinkOnMs = 500;
        commandToSend.led = MSP432_LAUNCHPAD_LED_GREEN;
      }
      else
      {
        commandToSend.blinkOnMs = 100;
        commandToSend.led = MSP432_LAUNCHPAD_LED_RED;
      }
    }

    // Bloquea la tarea durante xPeriod
    vTaskDelay(xPeriod);
  }
}

// Tarea ReceiverTask
static void prvReceiverTask(void *pvParameters)
{
  // Tiempo de espera hasta que la cola tiene dato nuevo
  const TickType_t xTicksToWait = pdMS_TO_TICKS(100);

  // La tarea se repite en un bucle infinito
  while (true)
  {
    // Resultado de la recepcion a la cola
    BaseType_t xStatus;

    // Variable para guadar comando recibido
    command_t receivedCommand;

    // Recibir nuevo dato de la cola
    xStatus = xQueueReceive(xQueue, &receivedCommand, xTicksToWait);

    // Comprueba si ha recibido correctamente
    if (xStatus == pdPASS)
    {
      // Tiempo de activacion del LED (en ticks)
      TickType_t xBlinkOn;

      // Calcula el tiempo de activacion del LED (en ticks)
      xBlinkOn = pdMS_TO_TICKS(receivedCommand.blinkOnMs);

      // Enciende LED
      led_on(receivedCommand.led);

      // Bloquea la tarea durante el tiempo de on del LED
      vTaskDelay(xBlinkOn);

      // Apaga LED
      led_off(receivedCommand.led);
    }
  }
}
