/************************************************************************/
/* 2:16 PM 4/24/2023	i2c.h Carbon_GDS
	
	Feather I2C interfaces : SCL, SDA
        ADC_I2C : PA13, PA12
        PM_I2C : PA17, PA16
        MS_I2C : PA23, PA22

 ************************************************************************/
#ifndef I2C_H_INCLUDED
#define I2C_H_INCLUDED
#include <hal_i2c_m_async.h>
#include "subbus.h"
#include "gds_pins.h"

/* 
#include <hal_atomic.h>
#include <hal_delay.h>
#include <hal_gpio.h>
#include <hal_init.h>
#include <hal_io.h>
#include <hal_sleep.h>
 */
 
extern struct i2c_m_async_desc PM_I2C;

extern struct i2c_m_async_desc ADC_I2C;

extern struct i2c_m_async_desc MS_I2C;

void PM_I2C_PORT_init(void);
void PM_I2C_CLOCK_init(void);
void PM_I2C_init(void);

void ADC_I2C_PORT_init(void);
void ADC_I2C_CLOCK_init(void);
void ADC_I2C_init(void);

void MS_I2C_PORT_init(void);
void MS_I2C_CLOCK_init(void);
void MS_I2C_init(void);

#define I2C_PM_ENABLE_DEFAULT true	// Circuit 3 Power Monitor
#define I2C_ADC_ENABLE_DEFAULT true	// All On board ADC's
#define I2C_MS_ENABLE_DEFAULT true	// On board MS8607 PTRH

// PM Defs
#define I2C_PM_BASE_ADDR 0x80
#define I2C_PM_STATUS_OFFSET 0x00
#define I2C_PM_HIGH_ADDR 0x86

#define I2C_PM_MAX_READ_LENGTH 3

// ADC Defs
#define I2C_ADC_BASE_ADDR 0x20
#define I2C_ADC_STATUS_OFFSET 0x00
#define I2C_ADC_STATUS_NREGS 1
#define I2C_ADC_ADS_OFFSET (I2C_ADC_STATUS_OFFSET+I2C_ADC_STATUS_NREGS)
#define I2C_ADC_ADS_NREGS 16
#define I2C_ADC_NREADS_NREGS 1
#define I2C_ADC_NREGS (I2C_ADC_ADS_OFFSET+I2C_ADC_ADS_NREGS+I2C_ADC_NREADS_NREGS)
#define I2C_ADC_HIGH_ADDR (I2C_ADC_BASE_ADDR+I2C_ADC_NREGS-1)

// MS Defs
#define I2C_MS_BASE_ADDR 0x60
#define I2C_MS_STATUS_OFFSET 0x00
#define I2C_MS_HIGH_ADDR 0x72

#define I2C_MS_MAX_READ_LENGTH 3

#define MS8P_I2C_ADDR 0x76
#define MSRH_I2C_ADDR 0x40

// MS8607 Pressure & Temperature Commands
#define MSP_RESET 0x1E // ADC reset command
#define MSP_ADC_READ 0x00 // ADC read command
#define MSP_CONV_D1 0x40 // ADC D1 conversion command
#define MSP_CONV_D2 0x50 // ADC D2 conversion command
#define MSP_OSR_OFFS 0x04 // ADC OSR offset (Default = 4096)
//	#define MSP_ADC_256 0x00 // ADC OSR=256	Offset 0
//	#define MSP_ADC_512 0x02 // ADC OSR=512	Offset 1
//	#define MSP_ADC_1024 0x04 // ADC OSR=1024	Offset 2
//	#define MSP_ADC_2048 0x06 // ADC OSR=2056	Offset 3
//	#define MSP_ADC_4096 0x08 // ADC OSR=4096	Offset 4
//	#define MSP_ADC_8192 0x0A // ADC OSR=8192	Offset 5
#define MSP_PROM_RD 0xA0 // Prom read command base address

// MS8607 Relative Humidity Commands
#define MSRH_RESET 0xFE // RH reset command
#define MSRH_WRITE_UREG 0xE6 // Write User Register
#define MSRH_READ_UREG 0xE7 // Read User Register
#define MSRH_MEAS_RH_HOLD 0xE5 // Measure RH (D3) Hold Master
#define MSRH_MEAS_RH 0xF5 // Measure RH (D3) No Hold Master
#define MSRH_PROM_RD 0xA0 // RH Prom read command base address


extern subbus_driver_t sb_i2c_pm;
extern subbus_driver_t sb_i2c_adc;
extern subbus_driver_t sb_i2c_ms;
void i2c_pm_enable(bool value);
void i2c_adc_enable(bool value);
void i2c_ms_enable(bool value);
extern struct i2c_m_async_desc MS_I2C;
void MS_I2C_PORT_init(void);
void MS_I2C_CLOCK_init(void);
void MS_I2C_init(void);

#endif
