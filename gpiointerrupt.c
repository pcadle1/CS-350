/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */

/*
 * Problem Description:
 * This code implements Project 1 for SNHU CS-350
 * as defined in the project guidelines. Using a task scheduler,
 * assess button presses every 200ms, temperature every 500ms, and
 * LED/UART every 1000ms. Buttons increase/decrease the temp set point,
 * and LED(heater) should turn on when the temp is below the set point.
 */

/*Solution:
 * A global period of 100000 microseconds is used in coordination with a timer and
 * task scheduler. Each task's elapsed time is incremented by the globabl period until
 * the corresponding increment is reached, and the task functions are executed. A state machine
 * is used to coordinate turning the heater/LED on and off when in the correct state.
 */

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Timer.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#define DISPLAY(x) UART_write(uart, &output, x);
#define TRUE 1
#define FALSE 0
#define GLOBAL_PERIOD 100000 //in microseconds
#define START_SET_POINT 20
#define NUM_TASKS 3

void buttonTask();
void tempTask();
void ledTask();

unsigned char setpoint = START_SET_POINT;
int16_t temperature = 0;
char readyTasks = FALSE;
volatile unsigned char IncrementFlag = FALSE;
volatile unsigned char DecrementFlag = FALSE;
volatile unsigned char TimerFlag = FALSE;
volatile unsigned char heat = FALSE;
volatile unsigned int totalSeconds = 0;

struct task {
    char triggered; //flag to indicate if task is ready to be executed
    unsigned long period; // Rate at which the task should tick
    unsigned long elapsedTime; // Time since task's previous tick
    void (*func)(); // Function to call for task's tick
};

struct task tasks[3] = {
    {FALSE, 200000, 200000, &buttonTask}, //check buttons every 200ms
    {FALSE, 500000, 500000, &tempTask}, //check temp every 500ms
    {FALSE, 1000000, 1000000, &ledTask} //change led every 1s and output to UART every 1s
};

//================UART DRIVER===================

// UART Global Variables
char output[64];
int bytesToSend;
// Driver Handles - Global variables
UART_Handle uart;
void initUART(void)
{
    UART_Params uartParams;

    // Init the driver
    UART_init();

    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);
    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}


//=============I2C DRIVER===========================
// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
}

sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;
// Driver Handles - Global variables
I2C_Handle i2c;
// Make sure you call initUART() before calling this function.
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))
    // Init the driver
    I2C_init();
    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL) {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }
    DISPLAY(snprintf(output, 32, "Passed\n\r"))
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses
    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;
    for (i=0; i<3; ++i) {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))
        if (I2C_transfer(i2c, &i2cTransaction)){
            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"))
    }
    if(found) {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address:%x\n\r", sensors[i].id, i2cTransaction.slaveAddress))
    }
    else {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found,contact professor\n\r"))
    }
}

int16_t readTemp(void)
{
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction)) {
        /*
        * Extract degrees C from the received data;
        * see TMP sensor datasheet
        */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;
        /*
        * If the MSB is set '1', then we have a 2's complement
        * negative value which needs to be sign extended
        */
        if (rxBuffer[0] & 0x80) {
        temperature |= 0xF000;
        }
    }
    else {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor(%d)\n\r",i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }
    return temperature;
}


//============TIMER===================
// Driver Handles - Global variables
Timer_Handle timer0;

void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
    int i = 0;
    //Mark tasks as triggered when their time interval has been reached
    for (i = 0; i < NUM_TASKS; i++){
        if (tasks[i].elapsedTime >= tasks[i].period){
            tasks[i].triggered = TRUE;
            TimerFlag = TRUE;
            tasks[i].elapsedTime = 0;
        }
        else {
            tasks[i].elapsedTime += GLOBAL_PERIOD;
        }
    }
}

void initTimer(void) {
    Timer_Params params;
    // Init the driver
    Timer_init();

    // Configure the driver
    Timer_Params_init(&params);
    params.period = 100000; // 100ms
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}

//====================================================================

//  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
void gpioButtonFxn0(uint_least8_t index)
{
    IncrementFlag = TRUE;
}

//Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
void gpioButtonFxn1(uint_least8_t index)
{
    DecrementFlag = TRUE;
}

enum Heat_States { Heat_on, Heat_off } Heat_state;

void Heat_St_Machine(int16_t temperature) {

   switch (Heat_state) { //Transitions
      case Heat_on:
         if (temperature >= setpoint){
             Heat_state = Heat_off;
             break;
         }
         Heat_state = Heat_on;
         break;

      case Heat_off:
          if (temperature < setpoint){
              Heat_state = Heat_on;
              break;
          }
          Heat_state = Heat_off;
          break;

      default:
         Heat_state = Heat_off;
         break;
   }

   switch (Heat_state) { //State actions

      case Heat_on:
         GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
         heat = TRUE;
         totalSeconds++;
         DISPLAY( snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temperature, setpoint, heat, totalSeconds));
         break;

      case Heat_off:
          GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
          heat = FALSE;
          totalSeconds++;
          DISPLAY( snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temperature, setpoint, heat, totalSeconds));
          break;

      default:
         GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
         break;
   }
}

//handles tasks associated with button presses -- incrementing or decrementing the set point
void buttonTask(){
    if (DecrementFlag){
        setpoint--;
        DecrementFlag = FALSE;
    }
    if (IncrementFlag){
        setpoint++;
        IncrementFlag = FALSE;
    }
}

//handles tasks at the temperature time interval, reading the current temp
void tempTask(){
    temperature = readTemp();
}

//handles tasks associated with LED/heater and output to UART via calling the state machine
void ledTask(){
    Heat_St_Machine(temperature);
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();
    initUART();
    initI2C();
    initTimer();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    Heat_state = Heat_off;
    while (1){
        while (!TimerFlag){}
        int i = 0;
        //execute any of the triggered tasks and reset
        for (i = 0; i < NUM_TASKS; i++){
            if (tasks[i].triggered){
                tasks[i].func();
                tasks[i].triggered = FALSE;
            }
        }
        TimerFlag = FALSE;
    }

    return (NULL);
}
