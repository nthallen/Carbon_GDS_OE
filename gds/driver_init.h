/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef DRIVER_INIT_INCLUDED
#define DRIVER_INIT_INCLUDED

#include "atmel_start_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <hal_atomic.h>
#include <hal_delay.h>
#include <hal_gpio.h>
#include <hal_init.h>
#include <hal_io.h>
#include <hal_sleep.h>

#include <hal_i2c_m_async.h>

#include <hal_i2c_m_async.h>

#include <hal_i2c_m_async.h>
#include <hal_timer.h>
#include <hpl_tc_base.h>

#include "hal_usb_device.h"

extern struct i2c_m_async_desc PM_I2C;

extern struct i2c_m_async_desc ADC_I2C;

extern struct i2c_m_async_desc MS_I2C;
extern struct timer_descriptor TIMER_0;

void PM_I2C_PORT_init(void);
void PM_I2C_CLOCK_init(void);
void PM_I2C_init(void);

void ADC_I2C_PORT_init(void);
void ADC_I2C_CLOCK_init(void);
void ADC_I2C_init(void);

void MS_I2C_PORT_init(void);
void MS_I2C_CLOCK_init(void);
void MS_I2C_init(void);

void USB_CTRL_CLOCK_init(void);
void USB_CTRL_init(void);

/**
 * \brief Perform system initialization, initialize pins and clocks for
 * peripherals
 */
void system_init(void);

#ifdef __cplusplus
}
#endif
#endif // DRIVER_INIT_INCLUDED
