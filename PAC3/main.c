/*
 * Copyright (C) 2017 Universitat Oberta de Catalunya - http://www.uoc.edu/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Universitat Oberta de Catalunya nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*----------------------------------------------------------------------------*/

/* Standard includes */
#include <stdlib.h>
#include <stdio.h>


/* Free-RTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "portmacro.h"
#include "timers.h"


/* MSP432 drivers includes */
#include "msp432_launchpad_board.h"
#include "BumpInt.h"
#include "driverlib.h"
#include "motor.h"


/*----------------------------------------------------------------------------*/

#define TASK_PRIORITY               ( tskIDLE_PRIORITY + 2 )
#define HEARTBEAT_TASK_PRIORITY     ( tskIDLE_PRIORITY + 1 )

#define TASK_STACK_SIZE             ( 1024 )
#define HEARTBEAT_STACK_SIZE        ( 128 )

#define HEART_BEAT_ON_MS            ( 10 )
#define HEART_BEAT_OFF_MS           ( 990 )
#define DEBOUNCING_MS               ( 20 )

#define QUEUE_SIZE                  ( 50 )

#define MOTOR_PWM               ( 50 )

/*----------------------------------------------------------------------------*/

typedef enum{
    left,
    right
} bumper_side;


typedef struct{
    bumper_side side;
    uint8_t code;
}queue_message;

// Tasks
static void HeartBeatTask(void *pvParameters);
static void SensingTask(void *pvParameters);
static void ActuationTask(void *pvParameters);

// Constantes
const TickType_t xMaxExpectedBlockTime = pdMS_TO_TICKS(500);
const TickType_t bumperDEBOUNCE_DELAY = pdMS_TO_TICKS(20);
const TickType_t minPeriod = 1;
BaseType_t xHigherPriorityTaskWoken = pdTRUE;

// callbacks & functions
static void BumperCallback(uint8_t);
static uint32_t BumperToPeriod(uint8_t);
static void BuildAndSendAction(uint8_t, bumper_side);
static void RunTimer(uint8_t, TimerHandle_t, TickType_t);
static void button1_interrupt(void);
static void button2_interrupt(void);

//Task sync tools and variables
SemaphoreHandle_t xBumperReceived;
QueueHandle_t xQueueActions;
TimerHandle_t xRMotorTimer;
TimerHandle_t xLMotorTimer;
motor_dir_e motorDirection;

/*----------------------------------------------------------------------------*/


static void HeartBeatTask(void *pvParameters){
    for(;;){
        led_toggle(MSP432_LAUNCHPAD_LED_RED);
        vTaskDelay( pdMS_TO_TICKS(HEART_BEAT_ON_MS) );
        led_toggle(MSP432_LAUNCHPAD_LED_RED);
        vTaskDelay( pdMS_TO_TICKS(HEART_BEAT_OFF_MS) );
    }
}

/**
 * Tarea de actuación. Espera activamente nuevas acciones en la cola.
 * Cuando recibe una acción, llama a la función RunTimer con el color indicado de led para que lo encienda por X tiempo
 */
static void ActuationTask(void *pvParameters) {
    //Leer queue de acciones y realizarlas
    queue_message commandToReceive;
    while(1){

        if (xQueueReceive(xQueueActions, (void *)&commandToReceive, xMaxExpectedBlockTime) == pdTRUE) {
            TickType_t ticks = pdMS_TO_TICKS(BumperToPeriod(commandToReceive.code));

            if(commandToReceive.side == left){
                RunTimer(MOTOR_LEFT, xLMotorTimer, ticks);
            }else{
                RunTimer(MOTOR_RIGHT, xRMotorTimer, ticks);
            }
        }
    }
}

/**
 * Primero asegura que el led este encendido, luego, si el timer está corriendo, aumenta en "ticks" el tiempo que va a estar corriendo.
 * Si no esta corriendo, lo arranca por ticks pasados por param
 */
static void RunTimer(motor_e motor, TimerHandle_t timer, TickType_t ticks){
    MotorStart(motor);
    if( xTimerIsTimerActive( timer ) != pdFALSE ){
        ticks = ticks + xTimerGetPeriod(timer);
        xTimerChangePeriod(timer, ticks, xMaxExpectedBlockTime );
    }else{
        xTimerChangePeriod(timer, ticks, xMaxExpectedBlockTime );
        xTimerReset(timer, xMaxExpectedBlockTime);
    }
}

/**
 * Funcion que devuelve, en milisegundos el tiempo que debe estar encendido cualquier led según el bumper
 */
static uint32_t BumperToPeriod(uint8_t bumper){
    if(bumper == 0 || bumper == 5){
        return 5000;
    }
    if(bumper == 1 || bumper == 4){
        return 3000;
    }
    return 1000;
}

/**
 * Tarea de sensor. Espera activamente mediante un semáforo que ocurra una interacción.
 * Cuando ocurre una interacción calcula el comando
 */
static void SensingTask(void *pvParameters) {
    //Esperar a que el semaforo este en verde
    //Leer sensores, resetear sensores, llenar la queue de acciones
    uint8_t lastStatus = 0;
    while(1){
        if (xSemaphoreTake(xBumperReceived, xMaxExpectedBlockTime) == pdPASS){
            vTaskDelay( bumperDEBOUNCE_DELAY );
            uint8_t status = BumpInt_Read();
            uint8_t i;
            uint8_t iMask;
            bumper_side side;
            for (i = 0; i < 6; i++)
            {
                //iMask es una máscara que usaremos mas adelante para facilmente extraer si el bumper i ha sido activado en la lectura
                iMask = 64 >> 6-i;
                if( i < 3){
                    side = right;
                }else{
                    side = left;
                }
                //Con "& iMask" extraemos el bit que nos interesa. Si el valor es mayor que 0 y distinto al anterior, enviamos la acción a la cola
                if((status & iMask) > 0 && ((lastStatus & iMask) != (status & iMask))){
                    BuildAndSendAction(i, side);
                }
            }
            lastStatus = status;
        }
    }
}

/**
 * Creamos y mandamos el comando a la cola de acciones
 */
static void BuildAndSendAction(uint8_t bumpCode, bumper_side side){
    queue_message commandToSend;
    commandToSend.side = side;
    commandToSend.code = bumpCode;
    xQueueSend(xQueueActions, (void *)&commandToSend, xMaxExpectedBlockTime);
}


static void BumperCallback(uint8_t bumperHit) {
    //Poner en verde el semaforo
    // Entrega el semaforo
    xSemaphoreGiveFromISR(xBumperReceived, &xHigherPriorityTaskWoken);
    //portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/**
 * Callback de los timers. Depende del timer terminado, apaga el led y resetea el timer
 */
void timerCallback(TimerHandle_t xTimer) {
    if ((uint32_t) pvTimerGetTimerID(xTimer) == 0) {
        MotorStop(MOTOR_LEFT);
        xTimerStop(xLMotorTimer, xMaxExpectedBlockTime);
    }else{
        MotorStop(MOTOR_RIGHT);
        xTimerStop(xRMotorTimer, xMaxExpectedBlockTime);
    }
    //Al terminar cualquier timer, forzamos una nueva lectura de los bumpers para resetear el change state
    xSemaphoreGiveFromISR(xBumperReceived, &xHigherPriorityTaskWoken);
}


/*----------------------------------------------------------------------------*/

void configureMotors(motor_dir_e direction){
    if(direction == NULL){
        if(motorDirection == MOTOR_DIR_FORWARD){
            direction = MOTOR_DIR_BACKWARD;
        }else{
            direction = MOTOR_DIR_FORWARD;
        }
    }
    motorDirection = direction;
    if(direction == MOTOR_DIR_FORWARD){
        led_on(MSP432_LAUNCHPAD_LED_BLUE);
        led_off(MSP432_LAUNCHPAD_LED_GREEN);
    }else{
        led_on(MSP432_LAUNCHPAD_LED_GREEN);
        led_off(MSP432_LAUNCHPAD_LED_BLUE);
    }
    MotorConfigure(MOTOR_LEFT, direction, MOTOR_PWM);
    MotorConfigure(MOTOR_RIGHT, direction, MOTOR_PWM);
}

void button1_interrupt(){
    configureMotors(MOTOR_DIR_FORWARD);
}

void button2_interrupt(){
    configureMotors(MOTOR_DIR_BACKWARD);
}


int main(int argc, char** argv)
{
    int32_t retVal = -1;

    // Initialize semaphores and queue
    xBumperReceived = xSemaphoreCreateBinary ();
    xQueueActions = xQueueCreate( QUEUE_SIZE, sizeof(queue_message) );
    TaskHandle_t xHandle;

    /* Initialize the board */
    board_init();
    MotorInit();
    configureMotors(MOTOR_DIR_FORWARD);
    BumpInt_Init(BumperCallback);
    board_buttons_set_callback(MSP432_LAUNCHPAD_BUTTON_S1, button1_interrupt);
    board_buttons_set_callback(MSP432_LAUNCHPAD_BUTTON_S2, button2_interrupt);

    if ( (xBumperReceived != NULL) && (xQueueActions != NULL)) {

        /* Create HeartBeat task */
        retVal = xTaskCreate(HeartBeatTask, "HeartBeatTask", HEARTBEAT_STACK_SIZE, NULL, HEARTBEAT_TASK_PRIORITY, NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }

        /* Create Actuation task */
        retVal = xTaskCreate(ActuationTask, "ActuationTask", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }

        /* Create Sensing task */
        retVal = xTaskCreate(SensingTask, "SensingTask", TASK_STACK_SIZE, NULL, TASK_PRIORITY, &xHandle );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }


        /* Timers */
        xLMotorTimer = xTimerCreate( "L motor timer", 1,  pdFALSE, (void *) 0,  timerCallback);
        xRMotorTimer = xTimerCreate( "R motor timer", 1,  pdFALSE, (void *) 1,  timerCallback);
        /* Start the task scheduler */
        vTaskStartScheduler();
    }

    return 0;
}

/*----------------------------------------------------------------------------*/
