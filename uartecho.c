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
 * Problem description:
 * This code implements the details of the
 * CS-350 milestone two assignment.
 * Turn LED on when user types "on".
 * Turn LED off when user types "off".
 *
 * Solution:
 * A state machine function is implemented
 * to manage current state and handle transitions between
 * states as well as turning on/off the LED when the correct
 * user input is received. Input of 'o' represents the start of
 * a valid user input string. If next character is 'n' then turn
 * the LED on. Otherwise, if 'f', await to see if a second 'f' follows
 * to turn LED off. Reset to initial state any time invalid input
 * is entered.
 */

/*
 *  ======== uartecho.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <ctype.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

/* Driver configuration */
#include "ti_drivers_config.h"

enum LED_States { LED_init, LED_await_user_input, LED_on, LED_off } LED_state;
/*
Function that manages LED state, state transitions and actions
considers user input, turning LED on when user types "on"
and turns LED off when user types "off"
 */
void LED_st_machine(char curr_letter) {
    //State Transitions
   switch (LED_state){
      case LED_init:
          //'o' indicates beginning of valid prompt
          if (curr_letter == 'o'){
              LED_state = LED_on;
          }
         break;
      case LED_await_user_input:
          if (curr_letter == 'f'){
              LED_state = LED_off;
          }
          else{
              LED_state = LED_init;
          }
          break;
      case LED_on:
          if (curr_letter == 'f'){
              LED_state = LED_await_user_input;
          }
          //reset to init state if not starting valid prompt and light is already on
          else if (curr_letter != 'o' && GPIO_read(CONFIG_GPIO_LED_0) == 1){
              LED_state = LED_init;
          }
          break;
      case LED_off:
         //return to LED_on state in the event next char is 'n'/'f'
         if (curr_letter == 'o'){
             LED_state = LED_on;
             break;
         }
         LED_state = LED_init;
         break;
      default:
         LED_state = LED_init;
         break;
   }
   //State actions
   switch (LED_state) {
      case LED_on:
         if (curr_letter == 'n'){
             GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
         }
         break;
      case LED_off:
             GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
         break;
      default:
         break;
   }
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    char        input;
    const char  echoPrompt[] = "Echoing characters:\r\n";
    UART_Handle uart;
    UART_Params uartParams;

    /* Call driver init functions */
    GPIO_init();
    UART_init();

    /* Configure the LED pin */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    uart = UART_open(CONFIG_UART_0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }

    /* Turn on user LED to indicate successful initialization */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
    UART_write(uart, echoPrompt, sizeof(echoPrompt));
    
    LED_state = LED_init; // initializes state
    /* Loop forever echoing */
    while (1) {
        UART_read(uart, &input, 1);
        LED_st_machine(tolower(input));
        UART_write(uart, &input, 1);
    }
}
