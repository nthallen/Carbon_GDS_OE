/************************************************************************/
/* 1:16 PM 5/24/2023	file i2c_adc.c Carbon_GDS
	
	Feather based Carbon Gas Deck Shield board ADS1115 Driver 
	16 Single -ended channels. Custom range per channel. 
	
	NOTE: Needs RTC timer module for delays

 ************************************************************************/
#include <utils.h>
#include <peripheral_clk_config.h>
#include <hal_init.h>
#include <hal_i2c_m_async.h>
#include <stdint.h>
#include "gds_pins.h"
#include "i2c.h"
#include "subbus.h"

struct i2c_m_async_desc ADC_I2C;

static bool i2c_enabled = I2C_ADC_ENABLE_DEFAULT;
static struct io_descriptor *I2C_io;
static volatile bool I2C_txfr_complete = true;
static volatile bool I2C_error_seen = false;
/** i2c error codes are defined in hal/include/hpl_i2c_m_sync.h
 *  named I2C_ERR_* and I2C_OK
 */
static volatile int32_t I2C_error = I2C_OK;
static volatile uint8_t pm_ov_status = 0;
#define PM_SLAVE_ADDR 0x67
#define PM_OVERFLOW 1
#define PM_UNDERFLOW 2

static void i2c_write(int16_t i2c_addr, const uint8_t *obuf, int16_t nbytes);
static void i2c_read(int16_t i2c_addr, uint8_t *ibuf, int16_t nbytes);

/**
 * These addresses belong to the I2C ADC module
 * 0x20 R:  I2C_Status
 * 0x21 R:  AIN0
 * 0x22 R:  AIN1
 * 0x23 R:  AIN2
 * 0x24 R:  AIN3
 * 0x25 R:  AIN4
 * 0x26 R:  AIN5
 * 0x27 R:  AIN6
 * 0x28 R:  AIN7
 * 0x29 R:  AIN8
 * 0x2A R:  AIN9
 * 0x2B R:  AIN10
 * 0x2C R:  AIN11
 * 0x2D R:  AIN12
 * 0x2E R:  AIN13
 * 0x2F R:  AIN14
 * 0x30 R:  AIN15
 * 0x31 R:  N_reads before conversion complete
 */
static subbus_cache_word_t i2c_adc_cache[I2C_ADC_HIGH_ADDR-I2C_ADC_BASE_ADDR+1] = {
  // Value, Wvalue, readable, was_read, writable, written, dynamic
  // I2C Status I2C_STATUS_NREGS
  { 0, 0, true,  false,  false, false, false }, // Offset 0: R: I2C Status
  // ADS registers I2C_ADS_NREGS
  { 0, 0, true,  false, false, false, false },  // AIN0
  { 0, 0, true,  false, false, false, false },  // AIN1
  { 0, 0, true,  false, false, false, false },  // AIN2
  { 0, 0, true,  false, false, false, false },  // AIN3
  { 0, 0, true,  false, false, false, false },  // AIN4
  { 0, 0, true,  false, false, false, false },  // AIN5
  { 0, 0, true,  false, false, false, false },  // AIN6
  { 0, 0, true,  false, false, false, false },  // AIN7
  { 0, 0, true,  false, false, false, false },  // AIN8
  { 0, 0, true,  false, false, false, false },  // AIN9
  { 0, 0, true,  false, false, false, false },  // AIN10
  { 0, 0, true,  false, false, false, false },  // AIN11
  { 0, 0, true,  false, false, false, false },  // AIN12
  { 0, 0, true,  false, false, false, false },  // AIN13
  { 0, 0, true,  false, false, false, false },  // AIN14
  { 0, 0, true,  false, false, false, false },  // AIN15
  { 0, 0, true,  false, false, false, false },  // N_reads before conversion complete
};

/** ADS1115 Config Register
 *	SE Temperature example: Write to config register [01] 0xC3 0x03:
 *   0x01: Address Pointer register value specifying config register
 *   0xC303:
 *     OS[15] = 1: Single Conversion
 *     MUX[14:12] = 1xx: AINx/GND => Single Ended AINx
 *     PGA[11:9] = 001: FSR = ±4.096V
 *     MODE[8] = 1: Single shot conversion
 *     DR[7:5] = 000: 8 SPS
 *     COMP_MODE[4] = 0: Default/Don't Care
 *     COMP_POL[3] = 0: Default/Don't Care
 *     COMP_LAT[2] = 0: Default/Don't Care
 *     COMP_QUE[1:0] = 11: Disable comparator and set ALERT to high impedance
 */

/*******************************************************************/
// ADS1115 Cofiguration commands: 
//	I2C Address and config register for all 16 channels
//	Must have same number of reads/device (i.e. ok to repeat differential 
//	cfg command to match number of SE reads.
//	No need to arrange channels in correct order: 
//    AIN0[0:3], AIN1[0:3], AIN2[0:3], AIN3[0:3] 
//	
/*******************************************************************/
#define ADC_NDEVS 4		// Number of ADS1115 devices on i2c chain
#define ADC_NCHS 4		// Number of  channels on each ADS1115
#define ADC_NCHANNELS (ADC_NDEVS * ADC_NCHS)	// Total number of ADC channels

// static uint8_t ads_slave_addr[ADC_NDEVS] = { 0x48, 0x49, 0x4A, 0x4B }; // 7-bit i2c address

typedef struct {
  uint8_t cfg[3];
} ch_cfg;

typedef struct {
  uint8_t adr;
  ch_cfg chan[4];
} dev_cfg;

// Don't think we need the third level nested struct, but I'm probably wrong
/* typedef struct {
  dev_cfg ads1115[4];
} adc_cfg;
 */
 
static dev_cfg ads_cfg[4] = {
  { 0x48, {
      { { 0x01, 0xC1, 0x03 } },   //  AIN0/J3: CAL_HI_HP ±6.144V
			{ { 0x01, 0xD1, 0x03 } },   //  AIN1/J4: CAL_HI_LP ±6.144V
			{ { 0x01, 0xE1, 0x03 } },   //  AIN2/J5: CAL_LO_HP ±6.144V
			{ { 0x01, 0xF1, 0x03 } } }  //  AIN3/J6: CAL_LO_LP ±6.144V
  },                            
  { 0x49, {
      { { 0x01, 0xC1, 0x03 } },   //  AIN4/J7: REF_HP ±6.144V
			{ { 0x01, 0xD1, 0x03 } },   //  AIN5/J8: REF_LP ±6.144V
			{ { 0x01, 0xE3, 0x03 } },   //  AIN6/J11: ROV1_T ±4.096V
			{ { 0x01, 0xF3, 0x03 } } }  //  AIN7/J12: ROV2_T ±4.096V
  },                            
  { 0x4A, {
      { { 0x01, 0xC3, 0x03 } },   //  AIN8: CO2_MOT_T ±4.096V
			{ { 0x01, 0xD3, 0x03 } },   //  AIN9: CO2_PUMP_T ±4.096V
			{ { 0x01, 0xE3, 0x03 } },   // AIN10: MM_MOT_T ±4.096V
			{ { 0x01, 0xF3, 0x03 } } }  // AIN11: MM_PUMP_T ±4.096V
  },                            
  { 0x4B, {
      { { 0x01, 0xC3, 0x03 } },   // AIN12/J13: ROV3_T ±4.096V
			{ { 0x01, 0xD3, 0x03 } },   // AIN13/J14: ROV4_T ±4.096V
			{ { 0x01, 0xE3, 0x03 } },   // AIN14/J15: ROV5_T ±4.096V
			{ { 0x01, 0xF3, 0x03 } } }  // AIN15/J16: ROV6_T ±4.096V
  }
};

enum ads_state_t {ads_init, ads_read_cfg,
                  ads_reg0, ads_read_adc,
                  ads_cache,
				  };
static enum ads_state_t ads_state = ads_init;
static uint16_t ads_n_reads;


static uint8_t ads_r0_prep[1] = { 0x00 };
static uint8_t ads_ibuf[2];
static int devnum = 0 ;
static int chnum = 0 ;

static bool I2C_ADC_chk_error()
{
  if (I2C_error_seen) {
    ads_state = ads_init;
    I2C_error_seen = 0;
    return true;
  }
  return false;
}
/**
 * @return true if the bus is free and available for another device
 */
static bool ads1115_poll(void) {
  if ( !i2c_enabled || !I2C_txfr_complete ) return true;  
  switch (ads_state) {
    case ads_init: // Start to convert AINx
      ads_n_reads = 0;
      i2c_write(ads_cfg[devnum].adr, ads_cfg[devnum].chan[chnum].cfg, 3);
	    if (devnum < ADC_NDEVS-1 ) {	// Start up all ADS1115 devices
          ++devnum;
	    } else {
        if (sb_cache_was_read(i2c_adc_cache, I2C_ADC_STATUS_OFFSET)) {
          sb_cache_update(i2c_adc_cache, I2C_ADC_STATUS_OFFSET, 0);
        }
        devnum = 0;
        ads_state = ads_read_cfg;
      }
      return false;
    case ads_read_cfg: // Start read from config register
      if (I2C_ADC_chk_error()) return true;
      i2c_read(ads_cfg[devnum].adr, ads_ibuf, 2);
      if (ads_ibuf[0] & 0x80) { // If high bit is set, conversion is complete
        ads_state = ads_reg0;
        return false;
      } else {
        ++ads_n_reads;  // nreads now accumulated across all ADS1115's
        ads_state = ads_read_cfg;
        return true;
      }
      return true;
    case ads_reg0: // Write pointer register to read from conversion reg[0]
      if (I2C_ADC_chk_error()) return true;
      i2c_write(ads_cfg[devnum].adr, ads_r0_prep, 1);
      ads_state = ads_read_adc;
      return false;
    case ads_read_adc: // Start read from conversion reg
      if (I2C_ADC_chk_error()) return true;
      i2c_read(ads_cfg[devnum].adr, ads_ibuf, 2);
      ads_state = ads_cache;
      return true;
    case ads_cache:
      if (I2C_ADC_chk_error()) return true;
      sb_cache_update(i2c_adc_cache, I2C_ADC_ADS_OFFSET + (devnum * 4) + chnum, 
        (ads_ibuf[0] << 8) | ads_ibuf[1]); // Save converted value
      sb_cache_update(i2c_adc_cache, I2C_ADC_ADS_OFFSET + I2C_ADC_ADS_NREGS, 
        ads_n_reads); // Save n_reads
      if (devnum < ADC_NDEVS-1 ) {
        ++devnum;
        ads_state = ads_read_cfg;
      } else {
        devnum = 0;
        if (++chnum > ADC_NCHS-1) chnum = 0;  // Do for each channel
        ads_state = ads_init;
      }
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
  i2c_m_async_set_slaveaddr(&ADC_I2C, i2c_addr, I2C_M_SEVEN);
  io_write(I2C_io, obuf, nbytes);
}

static void i2c_read(int16_t i2c_addr, uint8_t *ibuf, int16_t nbytes) {
  assert(I2C_txfr_complete, __FILE__, __LINE__);
  I2C_txfr_complete = false;
  i2c_m_async_set_slaveaddr(&ADC_I2C, i2c_addr, I2C_M_SEVEN);
  io_read(I2C_io, ibuf, nbytes);
}

void i2c_enable(bool value) {
  i2c_enabled = value;
}

#define I2C_INTFLAG_ERROR (1<<7)

static void I2C_async_error(struct i2c_m_async_desc *const i2c, int32_t error) {
  I2C_txfr_complete = true;
  I2C_error_seen = true;
  I2C_error = error;
  if (I2C_error >= -7 && I2C_error <= -2) {
    uint16_t val = i2c_adc_cache[I2C_ADC_STATUS_OFFSET].cache;
    val |= (1 << (7+I2C_error));
    sb_cache_update(i2c_adc_cache, I2C_ADC_STATUS_OFFSET, val);
  }
  if (error == I2C_ERR_BUS) {
    hri_sercomi2cm_write_STATUS_reg(ADC_I2C.device.hw, SERCOM_I2CM_STATUS_BUSERR);
    hri_sercomi2cm_clear_INTFLAG_reg(ADC_I2C.device.hw, I2C_INTFLAG_ERROR);
  }
}

static void I2C_txfr_completed(struct i2c_m_async_desc *const i2c) {
  I2C_txfr_complete = true;
}

static void i2c_adc_reset() {
  if (!sb_i2c_adc.initialized) {
    ADC_I2C_init();
    i2c_m_async_get_io_descriptor(&ADC_I2C, &I2C_io);
    i2c_m_async_enable(&ADC_I2C);
    i2c_m_async_register_callback(&ADC_I2C, I2C_M_ASYNC_ERROR, (FUNC_PTR)I2C_async_error);
    i2c_m_async_register_callback(&ADC_I2C, I2C_M_ASYNC_TX_COMPLETE, (FUNC_PTR)I2C_txfr_completed);
    i2c_m_async_register_callback(&ADC_I2C, I2C_M_ASYNC_RX_COMPLETE, (FUNC_PTR)I2C_txfr_completed);

    sb_i2c_adc.initialized = true;
  }
}
//  End of I2C functions

//	ADC_I2C Driver
void ADC_I2C_PORT_init(void)
{
	gpio_set_pin_pull_mode(ADC_SDA, GPIO_PULL_OFF);
	gpio_set_pin_function(ADC_SDA, PINMUX_PA12C_SERCOM2_PAD0);

	gpio_set_pin_pull_mode(ADC_SCL, GPIO_PULL_OFF);
	gpio_set_pin_function(ADC_SCL, PINMUX_PA13C_SERCOM2_PAD1);
}

void ADC_I2C_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM2_GCLK_ID_CORE, CONF_GCLK_SERCOM2_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM2_GCLK_ID_SLOW, CONF_GCLK_SERCOM2_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

	hri_mclk_set_APBBMASK_SERCOM2_bit(MCLK);
}

void ADC_I2C_init(void)
{
	ADC_I2C_CLOCK_init();
	i2c_m_async_init(&ADC_I2C, SERCOM2);
	ADC_I2C_PORT_init();
}
//	End of ADC_I2C Driver

// Main poll loop

enum i2c_state_t {i2c_ads1115, i2c_extra };
static enum i2c_state_t i2c_state = i2c_ads1115;

void i2c_adc_poll(void) {
  enum i2c_state_t input_state = i2c_state;
  while (i2c_enabled && I2C_txfr_complete) {
    switch (i2c_state) {
      case i2c_ads1115:
        if (ads1115_poll()) {
          i2c_state = i2c_ads1115;  // Change to = i2c_extra if extra devices
        }
        break;
      case i2c_extra:
        i2c_state = i2c_ads1115;
        break;
      default:
        assert(false, __FILE__, __LINE__);
    }
    if (i2c_state == input_state) break;
  }
}

subbus_driver_t sb_i2c_adc = {
  I2C_ADC_BASE_ADDR, I2C_ADC_HIGH_ADDR, // address range
  i2c_adc_cache,
  i2c_adc_reset,
  i2c_adc_poll,
  0, // Dynamic function
  false // initialized
};
