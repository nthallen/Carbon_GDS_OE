/************************************************************************/
/* 2:16 PM 5/14/2023	file i2c_ms.c  
	
	gds I2C interface to on-board MS8607 PTRH measurements
	I2C ADDR: 0x76h (P&T), 0x40h (RH)
	
	NOTE: Needs RTC timer module for delays

 ************************************************************************/
#include <hpl_pm_base.h>
#include <peripheral_clk_config.h>
#include <hpl_gclk_base.h>
#include "gds_pins.h"
#include "i2c.h"
#include "subbus.h"
#include "rtc_timer.h"
 
#define pow2(X) (float)(1<<X)

struct i2c_m_async_desc MS_I2C;

static bool i2c_ms_enabled = I2C_MS_ENABLE_DEFAULT;

// Need? ***
static struct io_descriptor *MS_I2C_io;
static volatile bool I2C_txfr_complete = true;
static volatile bool I2C_error_seen = false;
/** i2c error codes are defined in hal/include/hpl_i2c_m_sync.h
 *  named I2C_ERR_* and I2C_OK
 */
static volatile int32_t I2C_error = I2C_OK;
static volatile uint8_t pm_ov_status = 0;

static void i2c_write(int16_t i2c_addr, const uint8_t *obuf, int16_t nbytes);
static void i2c_read(int16_t i2c_addr, uint8_t *ibuf, int16_t nbytes);

typedef struct {
    uint8_t cmd[1];	// cmd - Coefficient read commands
} ms8607_prom_read;

static ms8607_prom_read msp_read_coef[8] = {
  { { 0xA0 } }, // Read CRC & Manuf info (?)
  { { 0xA2 } }, // Read Coeff C1
  { { 0xA4 } }, // Read Coeff C2
  { { 0xA6 } }, // Read Coeff C3
  { { 0xA8 } }, // Read Coeff C4
  { { 0xAA } }, // Read Coeff C5
  { { 0xAC } }, // Read Coeff C6 & RH CRC
  { { 0xAE } }  // Unused 
};
static uint8_t coef_num = 0;
static uint8_t msp_ibuf[I2C_MS_MAX_READ_LENGTH];
static uint8_t msrh_ibuf[I2C_MS_MAX_READ_LENGTH];

static uint8_t msp_adc_read[1] = { MSP_ADC_READ };
static uint8_t msp_reset_cmd[1] = { MSP_RESET };
static uint8_t msp_conv_D1_osr[1];
static uint8_t msp_conv_D2_osr[1];
static uint8_t msrh_meas_rh[1] = { MSRH_MEAS_RH };

// Need to add RH values to cache ***

/* These addresses belong to the I2C_MS module
 * 0x60 R:  ST: 16b I2C Status
 * 0x61 R:  PL: 16b Compensated Pressure LSW
 * 0x62 R:  PM: 16b Compensated Pressure MSB
 * 0x63 R:  TL: 16b Compensated Temperature LSW
 * 0x64 R:  TM: 16b Compensated Temperature MSB
 * 0x65 R:  C1: 16b Pressure sensitivity | SENST1
 * 0x66 R:  C2: 16b Pressure offset | OFFT1
 * 0x67 R:  C3: 16b Temperature coeff. of pressure sensitivity | TCS
 * 0x68 R:  C4: 16b Temperature coefficient of pressure offset | TCO
 * 0x69 R:  C5: 16b Reference temperature | TREF
 * 0x6A R:  C6: 16b Temperature coefficient of the temperature | TEMPSENS
 * 0x6B R:  D1L:16b Raw Pressure LSW
 * 0x6C R:  D1M:16b Raw Pressure MSB
 * 0x6D R:  D2L:16b Raw Temperature LSW
 * 0x6E R:  D2M:16b Raw Temperature MSB
 * 0x6F RW: OSR:16b OSR select (0:256, 1:512, 2:1024, 3:2048, 4:4096, 5:8192)
 * 0x70 R:  RH: 16b MS8607 Compensated Relative Humidity measurement in %
 * 0x71 R:  RH: 16b MS8607 Relative Humidity measurement in %
 * 0x72 RW:UREG:16b MS8607 8-bit User Register (Place holder; Not implemented)
 */
static subbus_cache_word_t i2c_ms_cache[I2C_MS_HIGH_ADDR-I2C_MS_BASE_ADDR+1] = {
  { 0, 0, true,  false, false,  false, false }, // Offset 0x00: R: 16b I2C Status
  { 0, 0, true,  false, false,  false, false }, // Offset 0x01: R: Compensated Pressure LSW
  { 0, 0, true,  false, false,  false, false }, // Offset 0x02: R: Compensated Pressure MSB
  { 0, 0, true,  false, false,  false, false }, // Offset 0x03: R: Compensated Temperature LSW
  { 0, 0, true,  false, false,  false, false }, // Offset 0x04: R: Compensated Temperature MSB
  { 0, 0, true,  false, false,  false, false }, // Offset 0x05: R: C1: Pressure sensitivity | SENST1
  { 0, 0, true,  false, false,  false, false }, // Offset 0x06: R: C2: Pressure offset | OFFT1
  { 0, 0, true,  false, false,  false, false }, // Offset 0x07: R: C3: Temperature coefficient of pressure sensitivity | TCS
  { 0, 0, true,  false, false,  false, false }, // Offset 0x08: R: C4: Temperature coefficient of pressure offset | TCO
  { 0, 0, true,  false, false,  false, false }, // Offset 0x09: R: C5: Reference temperature | TREF
  { 0, 0, true,  false, false,  false, false }, // Offset 0x0A: R: C6: Temperature coefficient of the temperature | TEMPSENS
  { 0, 0, true,  false, false,  false, false }, // Offset 0x0B: R: Raw Pressure LSW
  { 0, 0, true,  false, false,  false, false }, // Offset 0x0C: R: Raw Pressure MSB
  { 0, 0, true,  false, false,  false, false }, // Offset 0x0D: R: Raw Temperature LSW
  { 0, 0, true,  false, false,  false, false }, // Offset 0x0E: R: Raw Temperature MSB
  { 4, 0, true,  false,  true,  false, false }, // Offset 0x0F: RW: OSR select (0:256, 1:512, 2:1024, 3:2048, 4:4096)
  { 0, 0, true,  false, false,  false, false }, // Offset 0x10: R: MS8607 COmpensated Relative Humidity
  { 0, 0, true,  false, false,  false, false }, // Offset 0x11: R: MS8607 Relative Humidity
  { 4, 0, true,  false,  true,  false, false }, // Offset 0x12: RW: MS8607 RH User Register (Place holder; Not implemented)
// .cache	.wvalue	.readable	.was_read	.writable	.written	.dynamic
};

/*	Need to add  RH states
 ****************************************************
 *	MS8607 Driver State Machine
 *	ms8607_init - Reset ms8607 to initialize
 *	ms8607_readcal - Read Calibration data (6)
 *	ms8607_convp - Send Convert Pressure Command
 *	ms8607_readp - Read Pressure
 *	ms8607_convt - Send Convert Temperature Command
 *	ms8607_readt - Read Temperature
 */
enum ms8607_state_t {
        ms8607_init, ms8607_init_tx, ms8607_init_delay,
        ms8607_readcal, ms8607_readcal_tx, ms8607_readcal_cache,
        ms8607_convp, ms8607_convp_tx, ms8607_convp_delay,
        ms8607_readp, ms8607_readp_tx, ms8607_readp_cache,
        ms8607_convt, ms8607_convt_tx, ms8607_convt_delay,
        ms8607_readt, ms8607_readt_tx, ms8607_readt_cache,
        };

typedef struct {
  bool enabled;
  enum ms8607_state_t state;
  uint32_t D1;		// Raw Pressure
  uint32_t D2;		// Raw Temperature
  uint16_t cal[8];
  float P; 	// Compensated Pressure
  float T; 	// Compensated Temperature
  uint32_t endtime;
  uint32_t delay;
//  uint16_t current;
} ms8607_poll_def;

static ms8607_poll_def ms8607 = {
    I2C_MS_ENABLE_DEFAULT, ms8607_init
//	, 0, 0
//	, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
//	, 0, 0
};


/**
 * poll_ms8607() is only called when I2C_txfr_complete = true
 *    and I2C bus is free
 * return true if we are relinquishing the I2C bus
 */
static bool poll_ms8607() {
  // uint32_t delay = 0x00000000;
  float dT; 	// difference between actual and measured temperature
  float OFF; 	// offset at actual temperature
  float SENS; 	// sensitivity at actual temperature
  if (!i2c_ms_enabled || !I2C_txfr_complete ) return true;
  switch (ms8607.state) {
    // Reset ms8607
    case ms8607_init:
			i2c_write(MS8P_I2C_ADDR, msp_reset_cmd, 1);  // Reset MS8607 ~ 2.8mSec
      ms8607.state = ms8607_init_tx;
      return false;

    case ms8607_init_tx:
      ms8607.endtime = rtc_current_count + ( 3 * RTC_COUNTS_PER_MSEC ); 
      ms8607.state = ms8607_init_delay;
      return false;

    case ms8607_init_delay:
      if ( rtc_current_count <= ms8607.endtime ) return false;
      ms8607.state = ms8607_readcal;
      return true;

    case ms8607_readcal:
      // need to send 7 / receive 12 bytes to get
      // 1 x 16b CRC & Manuf info (TBD)
      // 6 x 16b coefficients
			i2c_write(MS8P_I2C_ADDR, msp_read_coef[coef_num].cmd, 1);	// maybe use array for cmd instead of struct?
			ms8607.state = ms8607_readcal_tx;
      return false;

    case ms8607_readcal_tx:
      i2c_read(MS8P_I2C_ADDR, msp_ibuf, 2);
			ms8607.state = ms8607_readcal_cache;
      return false;
      
    case ms8607_readcal_cache:
      // place coeff from msp_ibuf into cache
      ms8607.cal[coef_num] = ( // update ms8607 struct
         (((uint16_t)msp_ibuf[0])<<8)
          | ((uint16_t)msp_ibuf[1]));
      if (coef_num > 0) i2c_ms_cache[coef_num + 4].cache = ms8607.cal[coef_num]; // update cache
      ms8607.state = ms8607_convp;
      if (++coef_num < 7) ms8607.state = ms8607_readcal;
      return true;

    // return loop here
    case ms8607_convp:
      msp_conv_D1_osr[0] = MSP_CONV_D1 + (2 * i2c_ms_cache[0x0F].cache); // Update CONV_D1 cmd with OSR offset
			i2c_write(MS8P_I2C_ADDR, msp_conv_D1_osr, 1); // Send Convert D1 (P)
			
			//      *** NOt sure this needs to be here in the loop ***
      //  ADC OSR=256 	560us ~1ms
      //  ADC OSR=512 	1.10ms ~2ms
      //  ADC OSR=1024	2.17ms ~3ms
      //  ADC OSR=2056	4.32ms ~5ms
      //  ADC OSR=4096	8.61ms ~9ms
      //  ADC OSR=8192	17.2ms ~18ms
      switch (i2c_ms_cache[0x0F].cache) {
        case 0:	ms8607.delay = 1 * RTC_COUNTS_PER_MSEC ; break; // 1mS
        case 1:	ms8607.delay = 2 * RTC_COUNTS_PER_MSEC ; break; // 2mS
        case 2:	ms8607.delay = 3 * RTC_COUNTS_PER_MSEC ; break; // 3mS
        case 3:	ms8607.delay = 5 * RTC_COUNTS_PER_MSEC ; break; // 5mS
        case 4:	ms8607.delay = 9 * RTC_COUNTS_PER_MSEC ; break; // 9mS
        default: ms8607.delay = 18 * RTC_COUNTS_PER_MSEC ; break; // 18mS : case 5 or default
      }
      ms8607.state = ms8607_convp_tx;
      return false;

    case ms8607_convp_tx:
      ms8607.endtime = rtc_current_count + ms8607.delay ;
      ms8607.state = ms8607_convp_delay;
      return false;

    case ms8607_convp_delay:
  	  if ( rtc_current_count <= ms8607.endtime ) return false;
      ms8607.state = ms8607_readp;
      return true;

    case ms8607_readp:
	  i2c_write(MS8P_I2C_ADDR, msp_adc_read, 1);	
      ms8607.state = ms8607_readp_tx;
      return false;

    case ms8607_readp_tx:
			i2c_read(MS8P_I2C_ADDR, msp_ibuf, 3);
      ms8607.state = ms8607_readp_cache;
      return false;

    case ms8607_readp_cache:
      i2c_ms_cache[0x0B].cache = (  // read P LSW from msp_ibuf and update cache
         (((uint16_t)msp_ibuf[1])<<8)
        | ((uint16_t)msp_ibuf[2]));
      i2c_ms_cache[0x0C].cache = (  // read P MSB from msp_ibuf and update cache
          ((uint16_t)msp_ibuf[0]));
      ms8607.D1 = (((uint32_t)i2c_ms_cache[0x0C].cache)<<16)
        | ((uint32_t)i2c_ms_cache[0x0B].cache);  // Update ms8607.D1 for P calculation
      ms8607.state = ms8607_convt;
      return true;

    case ms8607_convt:
      msp_conv_D2_osr[0] = MSP_CONV_D2 + (2 * i2c_ms_cache[0x0F].cache); // Update CONV_D1 cmd with OSR offset
			i2c_write(MS8P_I2C_ADDR, msp_conv_D2_osr, 1); // Send Convert D2 (T)
      ms8607.state = ms8607_convt_tx;
      return false;

    case ms8607_convt_tx:
      ms8607.endtime = rtc_current_count + ms8607.delay ;
      ms8607.state = ms8607_convt_delay;
      return false;

    case ms8607_convt_delay:
  	  if ( rtc_current_count <= ms8607.endtime ) return false;
      ms8607.state = ms8607_readt;
      return true;

    case ms8607_readt:
			i2c_write(MS8P_I2C_ADDR, msp_adc_read, 1);	
      ms8607.state = ms8607_readt_tx;
      return false;

    case ms8607_readt_tx:
			i2c_read(MS8P_I2C_ADDR, msp_ibuf, 3);
      ms8607.state = ms8607_readt_cache;
      return false;

    case ms8607_readt_cache:
      i2c_ms_cache[0x0D].cache = (
         (((uint16_t)msp_ibuf[1])<<8)
        | ((uint16_t)msp_ibuf[2])); // read T LSW from msp_ibuf and update cache
      i2c_ms_cache[0x0E].cache = (
          ((uint16_t)msp_ibuf[0])); // read T MSB from msp_ibuf and update cache
      ms8607.D2 = ((uint32_t)i2c_ms_cache[0x0E].cache)<<16
        | ((uint32_t)i2c_ms_cache[0x0D].cache);	// Update ms8607.D2 for T calculation

      // Perform Compensation calculations here and update cache
      dT = ((float)ms8607.D2) - ((float)(ms8607.cal[5]) * pow2(8));
      OFF = ((float)(ms8607.cal[2]) * pow2(17)) + (dT * ((float)(ms8607.cal[4]))) / pow2(6);
      SENS = ((float)(ms8607.cal[1]) * pow2(16)) + (dT * ((float)(ms8607.cal[3]))) / pow2(7);
      ms8607.T = ( 2000 + ((dT * (float)(ms8607.cal[6])) / pow2(23))) / 100;  // degC
      ms8607.P = ((((float)(ms8607.D1) * SENS) / pow2(21)) - OFF) / pow2(15) / 100; // mBar

      sb_cache_update32(i2c_ms_cache, 1, &ms8607.P);	// Update cache P
      sb_cache_update32(i2c_ms_cache, 3, &ms8607.T);	// Update cache T

      ms8607.state = ms8607_convp;	// return to perform next P reading
      return true;

    default:
      assert(false, __FILE__, __LINE__);
   }
   return true;
}


/****************************************************
 *	MS8607 RH Sensor Driver State Machine
 *	msrh_init - Reset MS8607 RH sensor to initialize
 *	msrh_convrh - Send Measure Relative Humidity Command
 *	msrh_readrh - Read Relative Humidity
 */
enum msrh_state_t {
        msrh_init, msrh_init_tx, msrh_init_delay,
        msrh_convrh, msrh_convrh_tx, msrh_convrh_delay,
        msrh_readrh, msrh_readrh_cache
        };

typedef struct {
  bool enabled;
  enum msrh_state_t state;
  uint16_t D3; 	//  Relative Humidity
  uint16_t RH; 	//  COmpensated Relative Humidity
  uint32_t endtime;
//  uint32_t delay;
//  uint16_t current;
} msrh_poll_def;

static msrh_poll_def msrh = {
    I2C_MS_ENABLE_DEFAULT, msrh_init
//	, 0, 0
//	, 0, 0
};

/**
 * poll_msrh() is only called when I2C_txfr_complete = true
 *    and I2C bus is free
 * return true if we are relinquishing the I2C bus
 */
static bool poll_msrh(void) {
  if (!i2c_ms_enabled || !I2C_txfr_complete ) return true;
  switch (msrh.state) {
  //  Reset MS8607 RH
    case msrh_init:
			i2c_write(MSRH_I2C_ADDR, msp_reset_cmd, 1);
      msrh.state = msrh_init_tx;
      return false;

    case msrh_init_tx:
      msrh.endtime = rtc_current_count + ( 15 * RTC_COUNTS_PER_MSEC ); // Reset < 15mS
      msrh.state = msrh_init_delay;
      return false;

    case msrh_init_delay:
      if ( rtc_current_count <= msrh.endtime ) return false;
      msrh.state = msrh_convrh;
      return true;

  //  Process MS8607 RH
    case msrh_convrh:
	  i2c_write(MSRH_I2C_ADDR, msrh_meas_rh, 1);	
      msrh.state = msrh_convrh_tx;
      return false;

    case msrh_convrh_tx:
      msrh.endtime = rtc_current_count + ( 16 * RTC_COUNTS_PER_MSEC ); // RHmax delay 16mS
      msrh.state = msrh_convrh_delay;
      return false;

    case msrh_convrh_delay:
  	  if ( rtc_current_count <= msrh.endtime ) return false;
      msrh.state = msrh_readrh;
      return true;

    case msrh_readrh:
			i2c_read(MSRH_I2C_ADDR, msrh_ibuf, 3);  // 3rd byte is checksum
      msrh.state = msrh_readrh_cache;
      return false;

    case msrh_readrh_cache:
      msrh.D3 = -600 + 12500 *   // convert to 12b (16b) and Calculate '% * 100'
        ((((uint16_t)msrh_ibuf[0])<<8) | ((uint16_t)(msrh_ibuf[1] & 0xFC))) / pow2(16);
      i2c_ms_cache[0x11].cache = msrh.D3;  // update cache
      msrh.RH = msrh.D3 + (18)*(20 - (uint16_t)ms8607.T); // 'Tcoeff * 100'
      i2c_ms_cache[0x10].cache = msrh.RH;  // update cache
      msrh.state = msrh_convrh;
      return true;

    default:
      assert(false, __FILE__, __LINE__);
   }
  return true;
}

// i2c functions

static void i2c_write(int16_t i2c_addr, const uint8_t *obuf, int16_t nbytes) {
  assert(I2C_txfr_complete, __FILE__, __LINE__);
  I2C_txfr_complete = false;
  i2c_m_async_set_slaveaddr(&MS_I2C, i2c_addr, I2C_M_SEVEN);
  io_write(MS_I2C_io, obuf, nbytes);
}

static void i2c_read(int16_t i2c_addr, uint8_t *ibuf, int16_t nbytes) {
  assert(I2C_txfr_complete, __FILE__, __LINE__);
  I2C_txfr_complete = false;
  i2c_m_async_set_slaveaddr(&MS_I2C, i2c_addr, I2C_M_SEVEN);
  io_read(MS_I2C_io, ibuf, nbytes);
}

void i2c_ms_enable(bool value) {
  i2c_ms_enabled = value;
}

#define I2C_INTFLAG_ERROR (1<<7)

static void I2C_async_error(struct i2c_m_async_desc *const i2c, int32_t error) {
  I2C_txfr_complete = true;
  I2C_error_seen = true;
  I2C_error = error;
  if (sb_cache_was_read(i2c_ms_cache, I2C_MS_STATUS_OFFSET)) {
    sb_cache_update(i2c_ms_cache, I2C_MS_STATUS_OFFSET, 0);
  }
  if (I2C_error >= -7 && I2C_error <= -2) {
    uint16_t val = i2c_ms_cache[I2C_MS_STATUS_OFFSET].cache;
    val |= (1 << (7+I2C_error));
    sb_cache_update(i2c_ms_cache, I2C_MS_STATUS_OFFSET, val);
  }
  if (error == I2C_ERR_BUS) {
    hri_sercomi2cm_write_STATUS_reg(MS_I2C.device.hw, SERCOM_I2CM_STATUS_BUSERR);
    hri_sercomi2cm_clear_INTFLAG_reg(MS_I2C.device.hw, I2C_INTFLAG_ERROR);
  }
}

static void I2C_txfr_completed(struct i2c_m_async_desc *const i2c) {
  I2C_txfr_complete = true;
}

static void i2c_ms_reset() {
  if (!sb_i2c_ms.initialized) {
    MS_I2C_init();
    i2c_m_async_get_io_descriptor(&MS_I2C, &MS_I2C_io);
    i2c_m_async_enable(&MS_I2C);
    i2c_m_async_register_callback(&MS_I2C, I2C_M_ASYNC_ERROR, (FUNC_PTR)I2C_async_error);
    i2c_m_async_register_callback(&MS_I2C, I2C_M_ASYNC_TX_COMPLETE, (FUNC_PTR)I2C_txfr_completed);
    i2c_m_async_register_callback(&MS_I2C, I2C_M_ASYNC_RX_COMPLETE, (FUNC_PTR)I2C_txfr_completed);

    sb_i2c_ms.initialized = true;
  }
}
//  End of I2C functions

//	MS_I2C Driver
void MS_I2C_PORT_init(void)
{
	gpio_set_pin_pull_mode(MS_SDA, GPIO_PULL_OFF);
	gpio_set_pin_function(MS_SDA, PINMUX_PA22C_SERCOM3_PAD0);

	gpio_set_pin_pull_mode(MS_SCL, GPIO_PULL_OFF);
	gpio_set_pin_function(MS_SCL, PINMUX_PA23C_SERCOM3_PAD1);
}

void MS_I2C_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_CORE, CONF_GCLK_SERCOM3_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_SLOW, CONF_GCLK_SERCOM3_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

	hri_mclk_set_APBBMASK_SERCOM3_bit(MCLK);
}

void MS_I2C_init(void)
{
	MS_I2C_CLOCK_init();
	i2c_m_async_init(&MS_I2C, SERCOM3);
	MS_I2C_PORT_init();
}
//	End of MS_I2C Driver

// Main poll loop

enum i2c_state_t {i2c_ms8607, i2c_msrh };
static enum i2c_state_t i2c_state = i2c_ms8607;

void i2c_ms_poll(void) {
	// cycle between P&T and RH
  if (!i2c_ms_enabled) return;
	  if (true) {	//  do we need condition here? 
    switch (i2c_state) {
      case i2c_ms8607:
        if (poll_ms8607()) {
          i2c_state = i2c_msrh;
        }
        break;
      case i2c_msrh:
        if (poll_msrh()) {
          i2c_state = i2c_ms8607;
        }
        break;
      default:
        assert(false, __FILE__, __LINE__);
    }
  }
}

subbus_driver_t sb_i2c_ms = {
  I2C_MS_BASE_ADDR, I2C_MS_HIGH_ADDR, // address range
  i2c_ms_cache,
  i2c_ms_reset,
  i2c_ms_poll,
  0, // Dynamic function
  false // initialized
};
