/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef ATMEL_START_PINS_H_INCLUDED
#define ATMEL_START_PINS_H_INCLUDED

#include <hal_gpio.h>

// SAME51 has 14 pin functions

#define GPIO_PIN_FUNCTION_A 0
#define GPIO_PIN_FUNCTION_B 1
#define GPIO_PIN_FUNCTION_C 2
#define GPIO_PIN_FUNCTION_D 3
#define GPIO_PIN_FUNCTION_E 4
#define GPIO_PIN_FUNCTION_F 5
#define GPIO_PIN_FUNCTION_G 6
#define GPIO_PIN_FUNCTION_H 7
#define GPIO_PIN_FUNCTION_I 8
#define GPIO_PIN_FUNCTION_J 9
#define GPIO_PIN_FUNCTION_K 10
#define GPIO_PIN_FUNCTION_L 11
#define GPIO_PIN_FUNCTION_M 12
#define GPIO_PIN_FUNCTION_N 13

#define INV_ARM GPIO(GPIO_PORTA, 4)
#define CKT3_EN GPIO(GPIO_PORTA, 6)
#define ADC_SDA GPIO(GPIO_PORTA, 12)
#define ADC_SCL GPIO(GPIO_PORTA, 13)
#define CO2_EXH GPIO(GPIO_PORTA, 14)
#define C3P_SDA GPIO(GPIO_PORTA, 16)
#define C3P_SCL GPIO(GPIO_PORTA, 17)
#define CO2_REF GPIO(GPIO_PORTA, 18)
#define CAL_REF GPIO(GPIO_PORTA, 19)
#define CAL_LO GPIO(GPIO_PORTA, 20)
#define CAL_HI GPIO(GPIO_PORTA, 21)
#define MS_SDA GPIO(GPIO_PORTA, 22)
#define MS_SCL GPIO(GPIO_PORTA, 23)
#define PA24 GPIO(GPIO_PORTA, 24)
#define PA25 GPIO(GPIO_PORTA, 25)
#define CO2_PUMP GPIO(GPIO_PORTB, 16)
#define MM_EXH GPIO(GPIO_PORTB, 17)
#define MM_PUMP GPIO(GPIO_PORTB, 22)
#define CAL_SPR GPIO(GPIO_PORTB, 23)

#endif // ATMEL_START_PINS_H_INCLUDED
